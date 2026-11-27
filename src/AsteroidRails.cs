using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using HarmonyLib;
using Keen.Game2.Simulation.WorldObjects.Characters;
using Keen.Game2.Simulation.WorldObjects.CubeGrids;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Game.GameSystems.ProceduralGeneration;
using Keen.VRage.Core.Game.Systems;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.CoreData;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Components;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Havok;
using Keen.VRage.Physics.Havok.Character;
using Keen.VRage.Voxels.Components;

namespace SpinProbe;

/// <summary>
/// Stage 3 of docs/asteroid-orbits-plan.md: movable asteroids on analytic orbits.
/// <para>A coasting asteroid far from players and ships is switched to a Static body (RigidBodyComponent.SetMotionType ->
/// IPhysics.SetBodyMotionType, RigidBodyComponent.cs:198-205). A static body far from every dynamic body has no Havok body
/// (HavokLargeWorldManager.cs:478-491), so it costs nothing in the step; it moves on a patched conic (Orbits.Conic) about the
/// live centre of its dominant body. Its pose is written rarely (every write re-clusters the static body,
/// UpdateMovedStaticBodyClusters): 4x a second when a player is within 30 km (remote clients interpolate the replicated
/// samples), every real_rails_write_s otherwise. The in-process client draws it at the exact analytic pose every tick
/// (<see cref="AsteroidRailsClientJobs"/>).</para>
/// <para>Wake triggers, checked 4x a second against the ANALYTIC pose: A close approach with another asteroid (both woken so
/// Havok resolves it; with nobody within 30 km the impact is solved analytically and cratered / shattered in the background,
/// stage 3.4), B a player or awake ship within max(2.5 km, 5 s at the closing speed), C below 1.15 planet radii, a planet
/// bubble within 2 km (handoff only works on dynamic bodies), D grids welded to it (assemblies never go on rails).</para>
/// <para>All motion-type changes, pose writes and teleports happen in <see cref="AsteroidRailJobs"/> at WorldUpdateBegin
/// (the slot RealRails.FlushMotion and the clipboard use for structural changes), never inside the parallel physics jobs.</para>
/// </summary>
internal static class AsteroidRails
{
    internal static bool On => Settings.RealPhysics && Settings.AsteroidRails && MovableAsteroids.On;

    private const int EvalTicks = 15;                // 0.25 s
    private const int ConsiderTicks = 60;            // rail-on candidate checks
    private const double PlayerNearForWrites = 30000;
    private const double OffscreenDistance = 30000;
    private const double ApproachMargin = 200;
    private const double LookaheadSeconds = 4;
    private const double HashCell = 4000;
    private static readonly long RecentImpactTicks = TimeSpan.FromSeconds(10).Ticks;

    internal sealed class Rail
    {
        internal VoxelPhysicsComponent Phys = null!;
        internal MovableAsteroids.State S = null!;
        internal Orbits.Conic Conic = null!;
        internal Quaternion Q0;
        internal Vector3D W;           // inertial angular velocity (constant on rails)
        internal double T0;
        internal bool Waking;
        internal string WakeReason = "";
        internal long LastWriteTick;
        internal long SinceTick;
        internal string? Key;

        internal void PoseAt(double t, out WorldTransform wt, out Vector3D vel)
        {
            Conic.Eval(t, out Vector3D pos, out vel);
            wt = new WorldTransform(pos, Quaternion.Normalize(Orbits.RotQ(W, t - T0) * Q0));
        }
    }

    private sealed class Track
    {
        internal Vector3D Pos;
        internal Quaternion Ori;
        internal Vector3D Lin, Ang;
        internal long Tick;
        internal int Streak;
    }

    private static readonly Dictionary<VoxelPhysicsComponent, Rail> Rails = new(ReferenceEqualityComparer.Instance);
    private static readonly Dictionary<VoxelPhysicsComponent, Track> Tracks = new(ReferenceEqualityComparer.Instance);
    /// <summary>Railed asteroids by procedural id, for the in-process client (same id on both copies).</summary>
    internal static readonly ConcurrentDictionary<string, Rail> ById = new();
    private static readonly object Gate = new();

    internal static long RailedTotal, Woken, OffscreenImpacts, Handoffs, DespawnedShards;
    private static readonly Dictionary<string, long> WakeReasons = new();
    private static int _errors;

    // ---------------------------------------------------------------------------------------------------------------
    // Movers (players and awake ships), in GLOBAL coordinates
    // ---------------------------------------------------------------------------------------------------------------

    private static readonly ConcurrentDictionary<DEntity, (Vector3D p, Vector3D v, long tick, bool player)> Movers = new();

    /// <summary>Bubble-local server pose/velocity -> global (Bubbles.ToGlobal; velocity as BubbleHandoff.ConvertEntityToGlobal
    /// :554-571: rotate (v + ω×r) by the spin angle and add the bubble's orbital velocity).</summary>
    // Reads: BubbleHandoff.ResolveServerBubble / OrbitalVelocity; Bubbles.ToGlobal.
    private static void ToGlobal(DEntity e, in Vector3D p, in Vector3D v, out Vector3D gp, out Vector3D gv)
    {
        gp = p; gv = v;
        if (!Bubbles.Ready) return;
        Bubbles.Bubble? b = Settings.RealBubbleHandoff ? BubbleHandoff.ResolveServerBubble(e, in p) : Bubbles.BubbleAt(p);
        if (b == null || b.IsIdentity) return;
        double t = Bubbles.T;
        gp = Bubbles.ToGlobal(b, new WorldTransform(p), t).Position;
        Vector3D omega = b.SpinAxis * b.SpinRate;
        gv = Orbits.Rot(v + Vector3D.Cross(omega, p - b.C0), b.SpinAxis, b.SpinRate * t) + BubbleHandoff.OrbitalVelocity(b, t);
    }

    internal static void NoteMover(DEntity e, in Vector3D p, in Vector3D v, bool player)
    {
        if ((Profiler.Ticks + (e.GetHashCode() & 7)) % 8 != 0) return;
        ToGlobal(e, p, v, out Vector3D gp, out Vector3D gv);
        Movers[e] = (gp, gv, Profiler.Ticks, player);
    }

    private static List<(Vector3D p, Vector3D v, bool player)> SnapshotMovers()
    {
        var list = new List<(Vector3D, Vector3D, bool)>(Movers.Count + 1);
        long now = Profiler.Ticks;
        foreach (var kv in Movers)
        {
            if (now - kv.Value.tick > 120) { Movers.TryRemove(kv.Key, out _); continue; }
            list.Add((kv.Value.p, kv.Value.v, kv.Value.player));
        }
        // Listen-host spectator / active camera: treat as a player mover so asteroids wake into full Dynamic physics
        // when the player flies up to them in spectator (instead of staying Static / railed right in front of the camera).
        if (GlobalRender.HaveCamera)
        {
            Vector3D camPos = GlobalRender.LocalCamera.Position;
            if (Bubbles.Ready)
            {
                Bubbles.Bubble? cb = GlobalRender.CameraBubble;
                if (cb != null && !cb.IsIdentity) camPos = Bubbles.ToGlobal(cb, GlobalRender.LocalCamera, Bubbles.T).Position;
            }
            list.Add((camPos, Vector3D.Zero, true));
        }
        return list;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Hooks from MovableAsteroids / AsteroidShatter
    // ---------------------------------------------------------------------------------------------------------------

    /// <summary>MovableAsteroidJobs.Watch: records the awake asteroid's live state; returns true when it is on rails (the
    /// caller then skips drift / promotion logic for this tick).</summary>
    internal static bool Observe(VoxelPhysicsComponent physics, MovableAsteroids.State s, in WorldTransform wt, in RigidBodyData rb)
    {
        if (!On) return false;
        lock (Gate)
        {
            if (Rails.TryGetValue(physics, out Rail? r) && !r.Waking) return true;
            if (!Tracks.TryGetValue(physics, out Track? tr)) Tracks[physics] = tr = new Track();
            tr.Pos = wt.Position; tr.Ori = wt.Orientation; tr.Lin = rb.LinearVelocity; tr.Ang = rb.AngularVelocity; tr.Tick = Profiler.Ticks;
            return r != null;
        }
    }

    internal static bool IsRailed(VoxelPhysicsComponent physics)
    {
        lock (Gate) return Rails.TryGetValue(physics, out Rail? r) && !r.Waking;
    }

    /// <summary>Analytic pose / velocity of a railed asteroid now (false when awake).</summary>
    internal static bool TryLivePose(VoxelPhysicsComponent physics, out WorldTransform wt, out Vector3 lin, out Vector3 ang)
    {
        wt = default; lin = default; ang = default;
        Rail? r;
        lock (Gate) if (!Rails.TryGetValue(physics, out r) || r.Waking) return false;
        r.PoseAt(Orbits.Now, out wt, out Vector3D v);
        lin = (Vector3)v; ang = (Vector3)r.W;
        return true;
    }

    /// <summary>Storage corner frame (VoxelStorageComponentBase.ComputeCenteredTransform = world * StorageOffset, :414-417)
    /// from the analytic pose when railed.</summary>
    internal static WorldTransform LiveCentered(VoxelPhysicsComponent physics, VoxelStorageComponentBase storage)
    {
        if (TryLivePose(physics, out WorldTransform wt, out _, out _)) return wt * new WorldTransform(storage.StorageOffset);
        return storage.ComputeCenteredTransform();
    }

    /// <summary>Velocity change on a railed asteroid (spall recoil): re-anchor its conic at now.</summary>
    internal static void Kick(VoxelPhysicsComponent physics, Vector3 dv)
    {
        lock (Gate)
        {
            if (!Rails.TryGetValue(physics, out Rail? r)) return;
            double t = Orbits.Now;
            r.Conic.Eval(t, out Vector3D p, out Vector3D v);
            r.Conic = Orbits.Anchor(p, v + (Vector3D)dv, t);
        }
    }

    internal static void RequestWake(VoxelPhysicsComponent physics, string why)
    {
        lock (Gate) if (Rails.TryGetValue(physics, out Rail? r) && !r.Waking) { r.WakeReason = why; _wakeRequests.Add(physics); }
    }
    private static readonly HashSet<VoxelPhysicsComponent> _wakeRequests = new(ReferenceEqualityComparer.Instance);

    internal static void Forget(VoxelPhysicsComponent physics)
    {
        lock (Gate)
        {
            if (Rails.Remove(physics, out Rail? r) && r.Key != null) ById.TryRemove(r.Key, out _);
            Tracks.Remove(physics);
        }
    }

    internal static string? KeyOf(VoxelPhysicsComponent physics, MovableAsteroids.State s)
    {
        if (s.IdKey != null) return s.IdKey;
        try
        {
            var id = physics.Entity?.TryGet<ProceduralVoxelEntityComponent>()?.ID;
            if (id != null) s.IdKey = id.Value.ToString();
        }
        catch { }
        return s.IdKey;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Scheduler (WorldUpdateBegin, once per tick)
    // ---------------------------------------------------------------------------------------------------------------

    private static long _lastTick = -1;

    internal static void Tick(Session session)
    {
        long tick = Profiler.Ticks;
        if (tick == _lastTick) return;
        _lastTick = tick;
        if (!On || !Orbits.ClockReady) return;
        try
        {
            DrainWakeRequests();
            Land();
            if (tick % EvalTicks == 0) Evaluate(session, tick);
            if (tick % 600 == 0) Report();
            AsteroidStore.MaybeFlush(tick);
        }
        catch (Exception ex) { if (Interlocked.Increment(ref _errors) <= 8) ProbeLog.Write("arails: tick error: " + ex); }
    }

    private static void DrainWakeRequests()
    {
        List<VoxelPhysicsComponent>? req = null;
        lock (Gate)
        {
            if (_wakeRequests.Count == 0) return;
            req = _wakeRequests.ToList();
            _wakeRequests.Clear();
        }
        foreach (var p in req)
        {
            Rail? r;
            lock (Gate) if (!Rails.TryGetValue(p, out r) || r.Waking) continue;
            Wake(r, r.WakeReason);
        }
    }

    /// <summary>Woken asteroids whose body is dynamic again: teleport to the analytic pose with the analytic velocity
    /// (EntityTransformFunctions.Teleport sets Teleported so clients reset their motion history, as RealRails.Land does).</summary>
    // Reads: VRage.Core.Game/Keen/VRage/Core/Game/Data/EntityTransformFunctions.cs:57 Teleport.
    private static void Land()
    {
        List<Rail>? landing = null;
        lock (Gate)
            foreach (Rail r in Rails.Values)
                if (r.Waking) (landing ??= new()).Add(r);
        if (landing == null) return;
        double t = Orbits.Now;
        foreach (Rail r in landing)
        {
            try
            {
                if (r.Phys.Entity == null || r.Phys.DEntity.IsEmpty) { Forget(r.Phys); continue; }
                if (!r.Phys.Data.Has<DynamicTag>()) continue;   // motion change still queued
                r.PoseAt(t, out WorldTransform wt, out Vector3D v);
                var landedVel = new RigidBodyData { LinearVelocity = (Vector3)v, AngularVelocity = (Vector3)r.W };
                r.Phys.Data.Teleport(in wt);
                r.Phys.Data.Set(landedVel);
                MovableAsteroids.FlushDeferredChunkWelds(r.Phys, r.S, in wt, in landedVel);
                lock (Gate)
                {
                    Rails.Remove(r.Phys);
                    if (r.Key != null) ById.TryRemove(r.Key, out _);
                    if (Tracks.TryGetValue(r.Phys, out Track? tr)) { tr.Streak = 0; tr.Pos = wt.Position; tr.Lin = v; tr.Tick = Profiler.Ticks; }
                }
                r.S.InitialLinVel = (Vector3)v;   // dV for promotion is measured from here
                ProbeLog.Write($"arails: {r.S.Name} OFF rails ({r.WakeReason}) at {wt.Position.X:0},{wt.Position.Y:0},{wt.Position.Z:0} v={v.Length():0.0} m/s after {(Profiler.Ticks - r.SinceTick) / 60.0:0} s");
            }
            catch (Exception ex) { if (Interlocked.Increment(ref _errors) <= 8) ProbeLog.Write("arails: land error: " + ex.Message); }
        }
    }

    private struct Entry
    {
        internal VoxelPhysicsComponent Phys;
        internal MovableAsteroids.State S;
        internal Rail? Rail;
        internal Vector3D Pos, Vel;
        internal double Radius, Mass;
    }

    internal static double EqRadius(MovableAsteroids.State s) =>
        s.IsShard ? s.Radius : Math.Cbrt(3.0 * Math.Max(1.0, s.Mass / Math.Max(300.0, s.AvgDensity)) / (4.0 * Math.PI));

    private static void Evaluate(Session session, long tick)
    {
        double t = Orbits.Now;
        var movers = SnapshotMovers();
        (VoxelPhysicsComponent comp, MovableAsteroids.State state)[] live;
        lock (MovableAsteroids.LiveServer) live = MovableAsteroids.LiveServer.ToArray();

        var entries = new List<Entry>(live.Length);
        lock (Gate)
        {
            foreach (var (comp, s) in live)
            {
                if (comp.Entity == null || comp.DEntity.IsEmpty || s.IsClient) continue;
                var e = new Entry { Phys = comp, S = s, Radius = EqRadius(s), Mass = Math.Max(1.0, s.Mass) };
                if (Rails.TryGetValue(comp, out Rail? r) && !r.Waking)
                {
                    e.Rail = r;
                    r.Conic.Eval(t, out e.Pos, out e.Vel);
                }
                else if (Tracks.TryGetValue(comp, out Track? tr))
                {
                    double age = (tick - tr.Tick) / 60.0;
                    e.Pos = tr.Pos + tr.Lin * age; e.Vel = tr.Lin;
                }
                else continue;
                entries.Add(e);
            }
        }

        // Pair tests first (trigger A / off-screen impacts) so a rail-on candidate about to meet a neighbour is held.
        var hold = new HashSet<VoxelPhysicsComponent>(ReferenceEqualityComparer.Instance);
        PairPass(entries, movers, t, hold);

        foreach (Entry e in entries)
        {
            try
            {
                if (e.Rail != null) Railed(e, movers, t, tick);
                else if ((tick + (e.Phys.GetHashCode() & 63)) % ConsiderTicks < EvalTicks && !hold.Contains(e.Phys)) Consider(e, movers, t, tick);
            }
            catch (Exception ex) { if (Interlocked.Increment(ref _errors) <= 8) ProbeLog.Write("arails: eval error: " + ex.Message); }
        }

        // Stage 1.1 pile-up containment for off-screen shatters: shards are spawned manual so they replicate cleanly,
        // so untouched shards (!PlayerTouched) despawn once no player is within BeltDespawnRange.
        if (tick % 120 == 0 && movers.Any(m => m.player))
        {
            long nowUtc = DateTime.UtcNow.Ticks;
            foreach (Entry e in entries)
            {
                if (!e.S.IsShard || e.S.PlayerTouched || e.S.Grids.Count > 0) continue;
                if (nowUtc < e.S.ShatterImmuneUntilTicks) continue;
                if (NearestPlayer(movers, e.Pos) < Settings.BeltDespawnRange) continue;
                try
                {
                    if (e.Phys.Entity != null)
                    {
                        Forget(e.Phys);
                        session.MarkEntityForClose(e.Phys.Entity, true);
                        Interlocked.Increment(ref DespawnedShards);
                    }
                }
                catch { }
            }
        }

        OrbitalBelts.Tick(session, movers, t, tick);
    }

    private static double WakeDistance(in Vector3D p, in Vector3D v, List<(Vector3D p, Vector3D v, bool player)> movers, out double nearestPlayer, out bool hit)
    {
        nearestPlayer = double.MaxValue; hit = false;
        double worst = double.MaxValue;
        foreach (var m in movers)
        {
            double d = (m.p - p).Length();
            if (m.player && d < nearestPlayer) nearestPlayer = d;
            double wake = Math.Max(Settings.AsteroidWakeDistance, (m.v - v).Length() * 5.0 + 500);
            if (d < wake) hit = true;
            worst = Math.Min(worst, d / wake);   // < 1 = inside the wake distance
        }
        return worst;
    }

    private static string? EnvironmentWake(in Vector3D p, in Vector3D v, double t)
    {
        Vector3D pAhead = p + v * 5.0;
        double tAhead = t + 5.0;
        foreach (Orbits.Body b in Orbits.Bodies)
        {
            if (b.Radius <= 0) continue;
            if ((p - Orbits.CentreAt(b, t)).Length() < b.Radius * 1.15 ||
                (pAhead - Orbits.CentreAt(b, tAhead)).Length() < b.Radius * 1.15)
                return $"C: below 1.15 R of {b.Name}";
        }
        if (RealRails.NearBubble(p, t, 2000, out string? which) || RealRails.NearBubble(pAhead, tAhead, 2000, out which))
            return $"near bubble {which}";
        return null;
    }

    private static void Railed(Entry e, List<(Vector3D p, Vector3D v, bool player)> movers, double t, long tick)
    {
        Rail r = e.Rail!;
        if (Orbits.Patch(ref r.Conic, t)) { Interlocked.Increment(ref Handoffs); r.Conic.Eval(t, out e.Pos, out e.Vel); }
        WakeDistance(e.Pos, e.Vel, movers, out double nearestPlayer, out bool moverNear);
        string? why = moverNear ? "B: player or ship near" : EnvironmentWake(e.Pos, e.Vel, t);
        if (why == null && e.S.Grids.Count > 0) why = "D: grid welded";
        if (why != null) { Wake(r, why); return; }

        int interval = nearestPlayer < PlayerNearForWrites
            ? Math.Max(1, (int)(Settings.AsteroidRailWriteNearSeconds * 60))
            : Math.Max(60, (int)(Settings.RealRailsWriteSeconds * 60));
        if (tick - r.LastWriteTick >= interval)
        {
            r.LastWriteTick = tick;
            r.PoseAt(t, out WorldTransform wt, out _);
            // Reads: EntityTransformFunctions.cs:44 SetWorldTransform (a static body's pose write teleports it,
            // HavokSessionComponent CollectDataForStaticBodies).
            r.Phys.Data.SetWorldTransform(in wt);
        }
    }

    private static void Consider(Entry e, List<(Vector3D p, Vector3D v, bool player)> movers, double t, long tick)
    {
        MovableAsteroids.State s = e.S;
        long nowUtc = DateTime.UtcNow.Ticks;
        string? why = null;
        if (s.Grids.Count > 0) why = "D: grids welded";
        else if (s.Welds.Count > 0 || s.DeferredChunkWelds.Count > 0) why = "chunk bodies present";
        else if (!s.MassApplied) why = "mass pending";
        else if (!e.Phys.Data.Has<Keen.VRage.Physics.Havok.BodyProxy>()) why = "no physics body";
        else if (nowUtc < s.ImpactImmuneUntilTicks || nowUtc < s.ShatterImmuneUntilTicks || s.ShardGroupUntilTicks != 0) why = "fresh shard / shatter";
        else if (nowUtc - s.LastImpactTicks < RecentImpactTicks) why = "recent impact";
        else if (!RealPhysics.InGlobalSpace(e.Phys.DEntity, e.Pos)) why = "inside a bubble";
        if (why == null)
        {
            double ratio = WakeDistance(e.Pos, e.Vel, movers, out _, out _);
            if (ratio < 1.3) why = "B: player or ship near";
        }
        why ??= EnvironmentWake(e.Pos, e.Vel, t);
        Orbits.Conic? conic = null;
        if (why == null)
        {
            // Unlike ships (RealRails.GlobalRejection), drifting/falling asteroids far outside any planet bubble are still
            // railed on universal-variable Kepler trajectories — EnvironmentWake wakes them 5 s / 2 km before bubble entry.
            conic = Orbits.Anchor(e.Pos, e.Vel, t);
        }
        Track? tr;
        lock (Gate) Tracks.TryGetValue(e.Phys, out tr);
        if (tr == null) return;
        if (why != null) { tr.Streak = 0; return; }
        if (++tr.Streak < 3) return;   // ~3 s of quiet coasting

        var rail = new Rail
        {
            Phys = e.Phys, S = s, Conic = conic!, Q0 = tr.Ori, W = tr.Ang, T0 = t, LastWriteTick = tick, SinceTick = tick,
            Key = KeyOf(e.Phys, s),
        };
        try
        {
            // Reads: VRage.Physics/Keen/VRage/Physics/Components/RigidBodyComponent.cs:198-205 SetMotionType (protected,
            // publicized) -> IPhysics.SetBodyMotionType; queued by Havok (HavokSessionComponent _motionTypeChanges).
            ((RigidBodyComponent)e.Phys).SetMotionType(BodyArgs.Motion.Static);
        }
        catch (Exception ex) { ProbeLog.Write($"arails: {s.Name} SetMotionType(Static) failed: {ex.Message}"); return; }
        lock (Gate)
        {
            Rails[e.Phys] = rail;
            if (rail.Key != null) ById[rail.Key] = rail;
        }
        Interlocked.Increment(ref RailedTotal);
        Orbits.Apsides(conic!, out double peri, out double apo);
        ProbeLog.Write($"arails: {s.Name} ON rails about {conic!.Parent?.Name ?? "nothing (straight line)"} at {e.Pos.X:0},{e.Pos.Y:0},{e.Pos.Z:0} " +
                       $"v={e.Vel.Length():0.0} m/s r={e.Radius:0} m" + (conic.Parent != null ? $" peri {peri / 1000:0.0} km apo {(double.IsInfinity(apo) ? "unbound" : $"{apo / 1000:0.0} km")}" : ""));
    }

    private static void Wake(Rail r, string why)
    {
        if (r.Waking) return;
        r.Waking = true;
        r.WakeReason = why;
        try
        {
            // Place at the exact analytic pose and velocity BEFORE switching Static -> Dynamic so Havok allocates the
            // dynamic body in the right spot with the right velocity on tick 0 (instead of stepping 1 tick at the stale pose
            // with pre-rail RigidBodyData before Land() runs).
            if (r.Phys.Entity != null && !r.Phys.DEntity.IsEmpty)
            {
                r.PoseAt(Orbits.Now, out WorldTransform wt, out Vector3D v);
                r.Phys.Data.Teleport(in wt);
                r.Phys.Data.Set(new RigidBodyData { LinearVelocity = (Vector3)v, AngularVelocity = (Vector3)r.W });
                if (r.Phys.Data.Has<Keen.VRage.Physics.Havok.BodyProxy>() ||
                    r.Phys.Data.Has<Keen.VRage.Physics.Havok.WorldManager.HavokLargeWorldManager.StaticData>())
                    ((RigidBodyComponent)r.Phys).SetMotionType(BodyArgs.Motion.Dynamic);
            }
        }
        catch (Exception ex) { ProbeLog.Write($"arails: {r.S.Name} SetMotionType(Dynamic) failed: {ex.Message}"); }
        Interlocked.Increment(ref Woken);
        string key = why.Length > 2 && why[1] == ':' ? why[..1] : why;
        lock (WakeReasons) WakeReasons[key] = (WakeReasons.TryGetValue(key, out long n) ? n : 0) + 1;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Trigger A and stage 3.4 off-screen impacts
    // ---------------------------------------------------------------------------------------------------------------

    private static (long, long, long) Cell(in Vector3D p) => ((long)Math.Floor(p.X / HashCell), (long)Math.Floor(p.Y / HashCell), (long)Math.Floor(p.Z / HashCell));

    private static void PairPass(List<Entry> entries, List<(Vector3D p, Vector3D v, bool player)> movers, double t, HashSet<VoxelPhysicsComponent> hold)
    {
        if (entries.Count < 2) return;
        var grid = new Dictionary<(long, long, long), List<int>>();
        for (int i = 0; i < entries.Count; i++)
        {
            var c = Cell(entries[i].Pos);
            if (!grid.TryGetValue(c, out var l)) grid[c] = l = new List<int>(2);
            l.Add(i);
        }
        var done = new HashSet<(int, int)>();
        double dtEval = EvalTicks / 60.0;
        for (int i = 0; i < entries.Count; i++)
        {
            var (cx, cy, cz) = Cell(entries[i].Pos);
            for (long x = cx - 1; x <= cx + 1; x++)
                for (long y = cy - 1; y <= cy + 1; y++)
                    for (long z = cz - 1; z <= cz + 1; z++)
                    {
                        if (!grid.TryGetValue((x, y, z), out var l)) continue;
                        foreach (int j in l)
                        {
                            if (j <= i || !done.Add((i, j))) continue;
                            Entry a = entries[i], b = entries[j];
                            Vector3D d = b.Pos - a.Pos, w = b.Vel - a.Vel;
                            double ww = w.LengthSquared();
                            double tc = ww > 1e-9 ? Math.Clamp(-Vector3D.Dot(d, w) / ww, 0, LookaheadSeconds) : 0;
                            double dmin = (d + w * tc).Length();
                            double reach = a.Radius + b.Radius;
                            if (dmin > reach + ApproachMargin) continue;
                            // Hold even when both are currently awake so neither rails and immediately ping-pongs back off.
                            hold.Add(a.Phys); hold.Add(b.Phys);
                            if (a.Rail == null && b.Rail == null) continue;   // Havok handles awake pairs

                            bool watched = NearestPlayer(movers, a.Pos) < OffscreenDistance || NearestPlayer(movers, b.Pos) < OffscreenDistance;
                            bool awake = a.Rail == null || b.Rail == null;
                            if (watched || awake || a.S.Grids.Count > 0 || b.S.Grids.Count > 0)
                            {
                                lock (Gate)
                                {
                                    if (a.Rail != null) Wake(a.Rail, "A: close approach with " + b.S.Name);
                                    if (b.Rail != null) Wake(b.Rail, "A: close approach with " + a.S.Name);
                                }
                                continue;
                            }
                            // Off-screen: resolve analytically when contact falls inside this evaluation step.
                            double qa = ww, qb = Vector3D.Dot(d, w), qc = d.LengthSquared() - reach * reach;
                            if (qc <= 0) { Resolve(a, b, t); continue; }      // already touching
                            double disc = qb * qb - qa * qc;
                            if (qa < 1e-9 || disc < 0) continue;
                            double tContact = (-qb - Math.Sqrt(disc)) / qa;
                            if (tContact >= 0 && tContact <= dtEval) Resolve(a, b, t + tContact);
                        }
                    }
        }
    }

    private static double NearestPlayer(List<(Vector3D p, Vector3D v, bool player)> movers, in Vector3D p)
    {
        double best = double.MaxValue;
        foreach (var m in movers) if (m.player) best = Math.Min(best, (m.p - p).Length());
        return best;
    }

    /// <summary>
    /// Stage 3.4: two railed asteroids meet with nobody within 30 km. Rigid-sphere impulse with restitution 0.2 along the
    /// line of centres, both conics re-anchored at the contact time with the post-impact velocities, and the voxel result
    /// applied in the background: a crater (AsteroidImpacts.CarveLocal, replicated) or, for e = E/M >= Q*, the Voronoi
    /// shatter worker (AsteroidShatter.TryQueueLocal) - the same regimes as a watched impact.
    /// </summary>
    // Source: none (plugin-only) - J = -(1+e) m_eff v_n, E = ½ m_eff v_n².
    private static void Resolve(Entry a, Entry b, double tc)
    {
        if (a.Rail == null || b.Rail == null) return;
        a.Rail.Conic.Eval(tc, out Vector3D pa, out Vector3D va);
        b.Rail.Conic.Eval(tc, out Vector3D pb, out Vector3D vb);
        Vector3D n = pb - pa;
        double dist = n.Length();
        if (dist < 1e-3) return;
        n /= dist;
        double vn = Vector3D.Dot(vb - va, n);
        if (vn >= 0) return;   // separating
        double mEff = a.Mass * b.Mass / (a.Mass + b.Mass);
        const double restitution = 0.2;
        double J = -(1 + restitution) * mEff * vn;
        Vector3D va2 = va - n * (J / a.Mass), vb2 = vb + n * (J / b.Mass);
        double energy = 0.5 * mEff * vn * vn;
        lock (Gate)
        {
            a.Rail.Conic = Orbits.Anchor(pa, va2, tc);
            b.Rail.Conic = Orbits.Anchor(pb, vb2, tc);
        }
        Interlocked.Increment(ref OffscreenImpacts);
        long now = DateTime.UtcNow.Ticks;
        a.S.LastImpactTicks = now; b.S.LastImpactTicks = now;
        ProbeLog.Write($"arails: OFF-SCREEN impact {a.S.Name} x {b.S.Name}: v_n={-vn:0.0} m/s, E={energy / 1e6:0.00} MJ, " +
                       $"dv {(va2 - va).Length():0.00} / {(vb2 - vb).Length():0.00} m/s");
        if (!Settings.AsteroidImpacts || -vn < 10.0) return;   // bounce regime (AsteroidImpactJobs.MinImpactSpeed)
        Damage(a, n * a.Radius, -n, energy, -vn, b);
        Damage(b, -n * b.Radius, n, energy, -vn, a);
    }

    private static void Damage(Entry target, Vector3D relHit, Vector3D inwardWorld, double energy, double speed, Entry other)
    {
        try
        {
            var storage = target.Phys._storage;
            if (storage?.Storage == null || target.Rail == null) return;
            target.Rail.PoseAt(Orbits.Now, out WorldTransform pose, out _);
            WorldTransform centered = pose * new WorldTransform(storage.StorageOffset);
            WorldTransform toStorage = WorldTransform.GetRelativeTransform(WorldTransform.Identity, centered);
            // Hit point = its offset from the body centre at contact, applied to the body as it is now.
            Vector3 hitLocal = (Vector3)WorldTransform.Transform(pose.Position + relHit, in toStorage);
            Vector3 inwardLocal = Vector3.Normalize(Vector3.Transform((Vector3)inwardWorld, toStorage.Orientation));
            double bulk = target.S.AvgDensity > 0 ? target.S.AvgDensity : Settings.AsteroidDensity;
            double e = energy / Math.Max(1.0, target.S.Mass);
            double q = AsteroidShatter.QStar(bulk);
            if (Settings.AsteroidShatter && e >= q &&
                AsteroidShatter.TryQueueLocal(target.Phys, target.S, hitLocal, inwardLocal, energy, e, q, bulk, (float)speed, "off-screen " + other.S.Name))
                return;
            int hardness = bulk < 1500 ? 2 : bulk > 4500 ? 7 : 5;
            double otherBulk = other.S.AvgDensity > 0 ? other.S.AvgDensity : Settings.AsteroidDensity;
            AsteroidImpactJobs.CraterSize(energy, bulk, hardness, otherBulk, out float radius, out float depth);
            AsteroidImpactJobs.CarveLocal(target.Phys.Entity, storage, target.S, hitLocal, inwardLocal, radius, depth, energy, (float)speed,
                "rock", bulk, hardness, otherBulk, DateTime.UtcNow.Ticks, "off-screen " + other.S.Name);
        }
        catch (Exception ex) { if (Interlocked.Increment(ref _errors) <= 8) ProbeLog.Write("arails: off-screen damage failed: " + ex.Message); }
    }

    private static void Report()
    {
        int railed, tracked;
        lock (Gate) { railed = Rails.Count; tracked = Tracks.Count; }
        string reasons;
        lock (WakeReasons) reasons = string.Join(", ", WakeReasons.Select(kv => $"{kv.Key} {kv.Value}"));
        ProbeLog.Write($"arails: {railed} on rails / {tracked} tracked; railed {RailedTotal}, woken {Woken} ({reasons}), SOI handoffs {Handoffs}, " +
                       $"off-screen impacts {OffscreenImpacts}, despawned shards {DespawnedShards}, movers {Movers.Count}; belts: {OrbitalBelts.Stats()}");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Saves (stage 3.1): a railed or moving asteroid keeps its velocity across save / load (vanilla never saves it:
    // VoxelPhysicsComponent.Serialize :185-189 hands RigidBodyComponent.Serialize a throw-away object builder).
    // ---------------------------------------------------------------------------------------------------------------

    // Patches: VRage.Voxels/Keen/VRage/Voxels/Components/VoxelPhysicsComponent.cs  Serialize(VoxelPhysicsComponentObjectBuilder) (:185)
    [HarmonyPatch(typeof(VoxelPhysicsComponent), "Serialize")]
    internal static class SavePatch
    {
        [HarmonyPostfix]
        private static void Postfix(VoxelPhysicsComponent __instance)
        {
            try
            {
                if (!MovableAsteroids.States.TryGetValue(__instance, out MovableAsteroids.State? s) || s.IsClient) return;
                string? key = KeyOf(__instance, s);
                if (key == null) return;
                // Match key MUST be the entity's serialized WorldTransform.Position (for a railed rock, vanilla saves its
                // last written static pose, which can lag hundreds of metres behind the analytic pose), while storing the
                // live analytic velocity so TryVelocity matches on reload.
                Vector3D savedPos = __instance.Data.GetWorldTransform().Position;
                Vector3 lin, ang;
                if (!TryLivePose(__instance, out _, out lin, out ang))
                {
                    RigidBodyData rb = __instance.Data.TryGet<RigidBodyData>(out RigidBodyData v) ? v : default;
                    lin = rb.LinearVelocity; ang = rb.AngularVelocity;
                }
                AsteroidStore.Record(key, savedPos, lin, ang, urgent: true);
            }
            catch { }
        }
    }
}

/// <summary>
/// Sidecar velocity store and catalog delta store (stages 3.1 / 4.2), one JSON file per save name next to the plugin:
/// velocities of saved movable asteroids (vanilla drops them) and which belt-catalog asteroids became permanent or were
/// destroyed.
/// </summary>
internal static class AsteroidStore
{
    internal sealed class Entry
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public float VX { get; set; }
        public float VY { get; set; }
        public float VZ { get; set; }
        public float WX { get; set; }
        public float WY { get; set; }
        public float WZ { get; set; }
    }

    internal sealed class Data
    {
        public Dictionary<string, Entry> Velocities { get; set; } = new();
        public HashSet<int> CatalogPersistent { get; set; } = new();
        public HashSet<int> CatalogDestroyed { get; set; } = new();
    }

    private static Data? _data;
    private static volatile bool _dirty;
    private static volatile bool _urgentFlush;
    private static long _lastFlush;
    private static readonly object Gate = new();

    private static string FilePath
    {
        get
        {
            string dir = Path.Combine(Path.GetDirectoryName(typeof(AsteroidStore).Assembly.Location) ?? ".", "asteroid-store");
            Directory.CreateDirectory(dir);
            string name = string.Concat(Settings.SaveName.Split(Path.GetInvalidFileNameChars()));
            return Path.Combine(dir, name + ".json");
        }
    }

    internal static Data Get
    {
        get
        {
            lock (Gate)
            {
                if (_data != null) return _data;
                try
                {
                    string p = FilePath;
                    _data = File.Exists(p) ? JsonSerializer.Deserialize<Data>(File.ReadAllText(p)) ?? new Data() : new Data();
                    ProbeLog.Write($"astore: loaded {p}: {_data.Velocities.Count} velocities, {_data.CatalogPersistent.Count} persistent / {_data.CatalogDestroyed.Count} destroyed catalog asteroids");
                }
                catch (Exception e) { ProbeLog.Write("astore: load failed: " + e.Message); _data = new Data(); }
                return _data;
            }
        }
    }

    internal static void Record(string key, in Vector3D p, in Vector3 v, in Vector3 w, bool urgent = false)
    {
        lock (Gate)
        {
            Get.Velocities[key] = new Entry { X = p.X, Y = p.Y, Z = p.Z, VX = v.X, VY = v.Y, VZ = v.Z, WX = w.X, WY = w.Y, WZ = w.Z };
            _dirty = true;
            if (urgent) _urgentFlush = true;
        }
    }

    /// <summary>The stored velocity of a saved asteroid, if it was saved at (about) this position.</summary>
    internal static bool TryVelocity(string key, in Vector3D at, out Vector3 v, out Vector3 w)
    {
        v = default; w = default;
        lock (Gate)
        {
            if (!Get.Velocities.TryGetValue(key, out Entry? e)) return false;
            if ((new Vector3D(e.X, e.Y, e.Z) - at).Length() > 50) return false;   // a different save state
            v = new Vector3(e.VX, e.VY, e.VZ); w = new Vector3(e.WX, e.WY, e.WZ);
            return true;
        }
    }

    internal static void MarkCatalog(int id, bool destroyed)
    {
        lock (Gate)
        {
            if (destroyed) Get.CatalogDestroyed.Add(id); else Get.CatalogPersistent.Add(id);
            _dirty = true;
        }
    }

    internal static bool CatalogSkipped(int id)
    {
        lock (Gate) return Get.CatalogPersistent.Contains(id) || Get.CatalogDestroyed.Contains(id);
    }

    internal static bool CatalogPersistent(int id)
    {
        lock (Gate) return Get.CatalogPersistent.Contains(id);
    }

    internal static void MaybeFlush(long tick)
    {
        if (!_dirty) return;
        if (!_urgentFlush && tick - _lastFlush < 600) return;
        _lastFlush = tick;
        FlushNow();
    }

    internal static void FlushNow()
    {
        if (!_dirty) return;
        try
        {
            string json;
            lock (Gate)
            {
                if (!_dirty) return;
                json = JsonSerializer.Serialize(Get);
                _dirty = false;
                _urgentFlush = false;
            }
            File.WriteAllText(FilePath, json);
        }
        catch (Exception e) { ProbeLog.Write("astore: write failed: " + e.Message); }
    }
}

/// <summary>Server jobs for asteroid rails: mover tracking (parallel, read-only) and the once-per-tick scheduler at
/// WorldUpdateBegin.</summary>
internal static class AsteroidRailJobs
{
    private static readonly object Gate = new();

    [PhysicsUpdate.OnCollectMassProperties]
    [MustHave(typeof(CharacterInfo))]
    internal static void TrackCharacters(DEntity e, in WorldTransform wt, Optional<RigidBodyData> rb)
    {
        if (!AsteroidRails.On) return;
        AsteroidRails.NoteMover(e, wt.Position, rb.HasValue ? (Vector3D)rb.Value.LinearVelocity : Vector3D.Zero, true);
    }

    [PhysicsUpdate.OnCollectMassProperties]
    [MustHave(typeof(DynamicTag))]
    [MustHave(typeof(ActiveTag))]
    internal static void TrackGrids(DEntity e, CubeGridComponent grid, in WorldTransform wt, in RigidBodyData rb)
    {
        if (!AsteroidRails.On) return;
        AsteroidRails.NoteMover(e, wt.Position, rb.LinearVelocity, false);
    }

    // Reads: RealRails.FlushMotion / AsteroidShatterJobs.Flush - one WorldUpdateBegin job per planet; Tick dedupes per tick.
    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(InstanceBind<VoxelPlanetComponent>))]
    internal static void Schedule(VoxelPlanetComponent planet)
    {
        if (planet?.Entity == null || !Monitor.TryEnter(Gate)) return;
        try { AsteroidRails.Tick(planet.Entity.GetSession()); }
        finally { Monitor.Exit(Gate); }
    }

    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(InstanceBind<CharacterComponent>))]
    internal static void ScheduleCharacter(CharacterComponent character)
    {
        if (character?.Entity == null || !Monitor.TryEnter(Gate)) return;
        try { AsteroidRails.Tick(character.Entity.GetSession()); }
        finally { Monitor.Exit(Gate); }
    }
}

/// <summary>
/// Stage 3.2, in-process client: a railed asteroid's client copy is drawn at the exact analytic pose every client tick
/// (orbit clock = Bubbles.T, the clock that also places the planets), instead of jumping between the server's sparse
/// static-pose samples. Matched to the server rail by the procedural entity id, which the client copy shares. Remote
/// clients get the server's samples (4 Hz within 30 km of a player), interpolated by the kinematic client copy.
/// </summary>
// Reads: MovableAsteroidClientJobs.Follow (the chunk-follow job reads this same WorldTransform next tick).
internal static class AsteroidRailsClientJobs
{
    internal static long Overrides;

    [PhysicsUpdate.OnCollectMassProperties]
    internal static void FollowRail(DEntity entity, VoxelPhysicsComponent physics, EntityData<WorldTransform> transforms)
    {
        // Disabled: writing WorldTransform here fought Keen's MotionStateApplicationSystem (which interpolates the
        // server's 4 Hz static samples onto the kinematic client copy), causing railed asteroids to visually snap back
        // and forth. Keen's own interpolation smoothly drives railed rocks at 4-30 km, and any rock closer than
        // max(2.5 km, 5 s * closing speed) to the player, ship or spectator camera wakes into full Dynamic physics.
    }
}
