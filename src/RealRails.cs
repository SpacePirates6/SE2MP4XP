using System;
using System.Collections.Generic;
using System.Threading;
using HarmonyLib;
using Keen.Game2.Simulation.GameSystems.RangedAffectGenerators.Atmosphere;
using Keen.Game2.Simulation.WorldObjects.CubeGrids.Physicss;
using Keen.Game2.Simulation.WorldObjects.Movement;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Havok;
using Keen.VRage.Physics.Havok.Character;
using Keen.VRage.Physics.Havok.MotionSharing;

namespace SpinProbe;

/// <summary>
/// Phase 7: orbits on rails (docs/realistic-physics-plan.md). A coasting grid with no player within
/// real_rails_distance, out of the atmosphere, not thrusting and not welded, on a bound orbit that clears the planet, is
/// switched to Static (GridPhysicsComponent.SetGridMotion, :466) and moved analytically (universal-variable Kepler) instead
/// of being stepped by Havok. A static body far from every dynamic body has no Havok body at all
/// (HavokLargeWorldManager.cs:478-491), so it costs nothing in the physics step and its world is freed. Its pose is written
/// every real_rails_write_s (default 5 s) so clients and the scene subset keep seeing it (SyncMotionState sends static bodies Reliable,
/// ServerSessionComponent.cs:35-73). When a player comes near, it goes back to Dynamic, and once the body is dynamic again
/// (DynamicTag; the motion change is queued, HavokSessionComponent.cs:1012-1017) it is teleported to the analytic pose
/// with the analytic velocity. Saves: GridPhysicsComponent.Serialize (:155-160) is patched so a railed grid is saved as
/// Dynamic with its rail velocity, so a save never contains a frozen "station" in orbit.
/// Rotating frame: inside the spinning bubble the state is converted to the planet's inertial frame before propagation
/// and back after (x_in = R(Ωt)(x_l - C), v_in = R(Ωt)(v_l + Ω×(x_l - C))).
/// </summary>
internal static class RealRails
{
    private const double TickDt = 1.0 / 60.0;
    private const int CheckInterval = 60;   // candidate / release checks, ticks
    // Pose writes while on rails. Every write of a static body's WorldTransform makes Havok re-cluster it
    // (UpdateMovedStaticBodyClusters), so it is written rarely; release checks use the analytic pose, not the written one.
    private static int WriteInterval => Math.Max(CheckInterval, (int)(Settings.RealRailsWriteSeconds * 60));

    // SetGridMotion sets the replicated MotionTypeProperty and Data.Set<RigidBodyData> (GridPhysicsComponent.cs:466-477);
    // it is not called from the parallel physics jobs but queued and run at the start of the next tick, the same slot the
    // clipboard paste uses for its structural changes (AsteroidClipboard.PasteJobPlanet).
    private static readonly System.Collections.Concurrent.ConcurrentQueue<(GridPhysicsComponent grid, bool isStatic)> MotionQueue = new();
    private static readonly object FlushGate = new();

    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(Keen.VRage.DCS.CoreData.InstanceBind<Keen.VRage.Voxels.Components.VoxelPlanetComponent>))]
    internal static void FlushMotion(in WorldTransform planet)
    {
        if (MotionQueue.IsEmpty || !System.Threading.Monitor.TryEnter(FlushGate)) return; // one planet job drains it
        try
        {
            while (MotionQueue.TryDequeue(out var m))
            {
                try { m.grid.SetGridMotion(m.isStatic); }
                catch (Exception ex) { if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("rails: SetGridMotion error: " + ex); }
            }
        }
        finally { System.Threading.Monitor.Exit(FlushGate); }
    }

    internal sealed class Rail
    {
        internal GridPhysicsComponent Grid = null!;
        internal Vector3D Centre;     // planet centre (server/local coords)
        internal double Mu;           // m³/s² (0 = straight line)
        internal Vector3D Omega;      // frame angular velocity (0 = non-rotating)
        internal long T0;             // epoch, server ticks
        internal Vector3D R0, V0;     // inertial state relative to Centre at T0 (inertial axes = local axes at T0)
        internal Quaternion Q0;       // local orientation at T0
        internal Vector3D WInertial;  // inertial angular velocity (constant on rails)
        internal long LastWrite;
        internal Vector3D LastVelocity, LastAngular; // local, at the last write (used for saves)
        internal bool Releasing;
        // Source: none (plugin-only). Stage 3.1: in global space the rail is a patched conic about the dominant body's LIVE
        // centre (Orbits.Conic) on the orbit clock, so grids orbiting the moving moon (or the home planet going round the
        // star) can be railed too; the rotating-frame fields above are then unused.
        internal Orbits.Conic? Conic;
        internal double OrbitT0;
    }

    internal static readonly Dictionary<DEntity, Rail> Rails = new();
    internal static readonly object Gate = new();
    private static readonly Dictionary<DEntity, (long tick, int streak)> Candidates = new();
    private static readonly Dictionary<DEntity, (Vector3D p, long tick)> Players = new();
    internal static readonly System.Collections.Concurrent.ConcurrentDictionary<Vector3D, double> PlanetG0 = new();
    internal static float GravityMultiplier = 1f;
    internal static long OnRails, Released, Rejected;
    private static int _errors;

    private static long Now => Profiler.Ticks;

    // ---------- players ----------

    [PhysicsUpdate.OnCollectMassProperties]
    [MustHave(typeof(CharacterInfo))]
    internal static void TrackPlayers(DEntity e, in WorldTransform wt)
    {
        if (!RealPhysics.On || !Settings.RealRails) return;
        lock (Gate) Players[e] = (wt.Position, Now);
    }

    private static double NearestPlayer(in Vector3D p)
    {
        double best = double.MaxValue;
        long now = Now;
        lock (Gate)
        {
            List<DEntity>? stale = null;
            foreach (var kv in Players)
            {
                if (now - kv.Value.tick > 300) { (stale ??= new()).Add(kv.Key); continue; }
                double d = (kv.Value.p - p).Length();
                if (d < best) best = d;
            }
            if (stale != null) foreach (var s in stale) Players.Remove(s);
        }
        return best;
    }

    // ---------- dominant body ----------

    /// <summary>The body whose gravity governs the orbit. Membership decides the coordinates: inside a bubble (server
    /// membership, BubbleHandoff) that planet alone, at its local origin. In global space the strongest pull wins; an
    /// orbiting planet's pull there comes from MovingPlanetGravity toward its moving centre, which a fixed-centre Kepler
    /// orbit cannot follow, so <paramref name="moving"/> is set and the caller does not rail it (a patched-conic orbit
    /// around a moving moon would be the next step).</summary>
    private static bool Dominant(DEntity entity, in Vector3D p, out Vector3D centre, out double mu, out double surface, out double range, out Vector3D omega, out double bubbleRadius, out bool moving)
    {
        centre = default; mu = 0; surface = 0; range = 0; omega = default; bubbleRadius = 0; moving = false;
        var planets = RealPhysics.Planets;
        bool global = RealPhysics.InGlobalSpace(entity, in p);
        if (!global)
        {
            foreach (var (c, r) in planets)
            {
                double bubble = r * Settings.FrameRadiusFactor;
                if ((p - c).LengthSquared() < bubble * bubble) return Planet(entity, p, c, r, out centre, out mu, out surface, out range, out omega, out bubbleRadius);
            }
        }
        double bestA = 0; bool found = false; double t = Bubbles.T;
        foreach (var (c, r) in planets)
        {
            double g0 = G0(c, r);
            if (g0 <= 0) continue;
            Bubbles.Bubble? mb = global ? RealPhysics.MovingBubbleOf(c) : null;
            Vector3D cNow = mb != null ? Bubbles.Centre(mb, t) : c;
            double d = (p - cNow).Length();
            if (d > r * Settings.RealGravityRange) continue;
            double a = g0 * r * r / (d * d);
            if (a > bestA)
            {
                bestA = a;
                found = Planet(entity, p, c, r, out centre, out mu, out surface, out range, out omega, out bubbleRadius);
                moving = mb != null;
                centre = cNow;
            }
        }
        return found || true; // no planet: free coasting (mu = 0)
    }

    private static double G0(in Vector3D c, double r)
    {
        return RealPhysics.TryGetG0(c, r, out float g) ? g : 0;
    }

    private static bool Planet(DEntity entity, in Vector3D p, Vector3D c, double r, out Vector3D centre, out double mu, out double surface, out double range, out Vector3D omega, out double bubbleRadius)
    {
        centre = c; surface = r; range = r * Settings.RealGravityRange; omega = default; mu = 0; bubbleRadius = r * Settings.FrameRadiusFactor;
        double g0 = G0(c, r);
        if (g0 > 0) mu = g0 * GravityMultiplier * r * r;
        // Only apply rotating-frame omega if the ship itself is INSIDE this planet's rotating bubble!
        // Outside the bubble (after BubbleHandoff moves the ship to inertial global space), omega = 0.
        if (RealPhysics.SpinningFrameAt(entity, in p, out Vector3D fc, out Vector3D w) && (fc - c).LengthSquared() < 4.0) omega = w;
        return true;
    }

    private static readonly Dictionary<DEntity, (string why, long tick)> LastRejection = new();

    private static void NoteRejection(DEntity e, string why, in Vector3D p, double speed)
    {
        long now = Now;
        lock (Gate)
        {
            if (LastRejection.TryGetValue(e, out var last) && (last.why == why || now - last.tick < 600)) return;
            LastRejection[e] = (why, now);
            if (LastRejection.Count > 256) LastRejection.Clear();
        }
        ProbeLog.Write($"rails: {e} not railed: {why} (at {p}, {speed:0.0} m/s)");
    }

    // ---------- patched-conic checks (global space) ----------

    /// <summary>Why a global-space conic may not be railed (null = fine): it must clear its parent's surface, and its parent's
    /// bubble boundary must not be crossed (handoff converts frames on a dynamic body only). Other bubbles (e.g. the moon's
    /// while orbiting the home planet) are watched on rails by <see cref="NearBubble"/>.</summary>
    // Source: none (plugin-only).
    internal static string? GlobalRejection(Orbits.Conic conic, in Vector3D p, double t)
    {
        if (NearBubble(p, t, 1500.0, out string? which)) return "near bubble boundary " + which;
        if (conic.Parent == null) return null;
        Orbits.Apsides(conic, out double peri, out double apo);
        if (conic.Parent.Radius > 0 && peri < conic.Parent.Radius * 1.2) return $"periapsis {peri / 1000:0.0} km too low about {conic.Parent.Name}";
        double bubble = conic.Parent.Frame?.Radius ?? 0;
        if (bubble > 0 && peri < bubble + 500 && apo > bubble - 500) return $"orbit crosses {conic.Parent.Name} bubble boundary";
        return null;
    }

    /// <summary>True when <paramref name="p"/> (global) is within <paramref name="margin"/> of any planet bubble's sphere.</summary>
    internal static bool NearBubble(in Vector3D p, double t, double margin, out string? which)
    {
        which = null;
        foreach (Bubbles.Bubble b in Bubbles.All)
        {
            if (b.Radius <= 0) continue;
            if ((p - Bubbles.Centre(b, t)).Length() < b.Radius + margin) { which = b.Name; return true; }
        }
        return false;
    }

    // ---------- going on rails ----------

    /// <summary>Candidate check for awake dynamic grids, once a second per grid.</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [WhenSimulated]
    [MustHave(typeof(DynamicTag))]
    [MustHave(typeof(ActiveTag))]
    [DoesNotHave(typeof(KinematicTag))]
    [DoesNotHave(typeof(TransformDriven))]
    internal static void Consider(DEntity e, GridPhysicsComponent grid, in WorldTransform wt, in RigidBodyData rb,
        Optional<SharedMotionData> shared, Optional<ActiveThrustData> thrust, Optional<AirData> air)
    {
        try
        {
            if (!RealPhysics.On || !Settings.RealRails) return;
            long now = Now;
            if ((now + e.GetHashCode()) % CheckInterval != 0) return;
            lock (Gate) if (Rails.ContainsKey(e)) return;
            string? why = null;
            if (shared.HasValue) why = "welded";
            else if (thrust.HasValue && !Vector3.IsZero(thrust.Value.ComputedThrustPerFrame)) why = "thrusting";
            else if (air.HasValue && air.Value.Density > 0f) why = "in atmosphere";
            else if (NearestPlayer(wt.Position) < Settings.RealRailsDistance) why = "player near";
            // Stage 3.1: global space -> patched conic about the live dominant body (moving moons included).
            bool global = RealPhysics.InGlobalSpace(e, wt.Position);
            if (global)
            {
                if (why == null && !Orbits.ClockReady) why = "orbit clock not running yet";
                Orbits.Conic? conic = null;
                if (why == null)
                {
                    double tNow = Orbits.Now;
                    conic = Orbits.Anchor(wt.Position, (Vector3D)rb.LinearVelocity, tNow);
                    why = GlobalRejection(conic, wt.Position, tNow);
                }
                if (why != null)
                {
                    lock (Gate) Candidates.Remove(e);
                    if (why != "thrusting" && why != "in atmosphere") NoteRejection(e, why, wt.Position, rb.LinearVelocity.Length());
                    return;
                }
                int gs;
                lock (Gate)
                {
                    gs = Candidates.TryGetValue(e, out var g0) ? g0.streak + 1 : 1;
                    Candidates[e] = (now, gs);
                }
                if (gs < 3) return;
                var grail = new Rail
                {
                    Grid = grid, Conic = conic, OrbitT0 = conic!.T0, T0 = now, Q0 = wt.Orientation,
                    WInertial = (Vector3D)rb.AngularVelocity, LastWrite = now,
                    LastVelocity = rb.LinearVelocity, LastAngular = rb.AngularVelocity,
                };
                lock (Gate) { Rails[e] = grail; Candidates.Remove(e); }
                MotionQueue.Enqueue((grid, true));
                Interlocked.Increment(ref OnRails);
                ProbeLog.Write($"rails: {e} ON rails (patched conic about {conic.Parent?.Name ?? "nothing"}) at {wt.Position} v={rb.LinearVelocity.Length():0.0} m/s");
                return;
            }
            Dominant(e, wt.Position, out Vector3D c, out double mu, out double surface, out double range, out Vector3D omega, out double bubbleRadius, out bool moving);
            Vector3D r0 = wt.Position - c;
            Vector3D v0 = (Vector3D)rb.LinearVelocity + Vector3D.Cross(omega, r0);
            if (why == null && moving) why = "orbiting a moving moon in global space";
            if (why == null && bubbleRadius > 0 && Math.Abs(r0.Length() - bubbleRadius) < 1500.0)
                why = "near bubble boundary";
            if (why == null && mu > 0)
            {
                double rl = r0.Length(), v2 = v0.LengthSquared();
                double energy = v2 / 2 - mu / rl;
                if (energy >= 0) why = "unbound";
                else
                {
                    double a = -mu / (2 * energy);
                    Vector3D h = Vector3D.Cross(r0, v0);
                    double e2 = 1 - h.LengthSquared() / (mu * a);
                    double ecc = Math.Sqrt(Math.Max(0, e2));
                    double peri = a * (1 - ecc), apo = a * (1 + ecc);
                    if (peri < surface * 1.2) why = $"periapsis {peri / 1000:0.0} km too low";
                    else if (apo > range * 0.95) why = "apoapsis beyond gravity range";
                    else if (bubbleRadius > 0 && peri < bubbleRadius + 500 && apo > bubbleRadius - 500)
                        why = "orbit crosses bubble boundary";
                }
            }
            if (why != null)
            {
                lock (Gate) Candidates.Remove(e);
                // Test aid: say why a coasting grid out of the air is not railed (once per reason change, at most every 10 s).
                if (why != "thrusting" && why != "in atmosphere") NoteRejection(e, why, wt.Position, rb.LinearVelocity.Length());
                return;
            }
            int streak;
            lock (Gate)
            {
                streak = Candidates.TryGetValue(e, out var c0) ? c0.streak + 1 : 1;
                Candidates[e] = (now, streak);
            }
            if (streak < 3) return; // stable for ~3 s
            var rail = new Rail
            {
                Grid = grid, Centre = c, Mu = mu, Omega = omega, T0 = now, R0 = r0, V0 = v0, Q0 = wt.Orientation,
                WInertial = (Vector3D)rb.AngularVelocity + omega, LastWrite = now,
                LastVelocity = rb.LinearVelocity, LastAngular = rb.AngularVelocity,
            };
            lock (Gate) { Rails[e] = rail; Candidates.Remove(e); }
            MotionQueue.Enqueue((grid, true));
            Interlocked.Increment(ref OnRails);
            ProbeLog.Write($"rails: {e} ON rails at {wt.Position} v={rb.LinearVelocity.Length():0.0} m/s mu={mu:0.###E+0} " +
                           $"r={r0.Length() / 1000:0.0} km frame={(omega.LengthSquared() > 0 ? "rotating" : "inertial")}");
        }
        catch (Exception ex)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("rails: Consider error: " + ex);
        }
    }

    // ---------- while on rails ----------

    /// <summary>Static railed grids: write the analytic pose twice a second; release when a player comes near.</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [DoesNotHave(typeof(DynamicTag))]
    internal static void Drive(DEntity e, GridPhysicsComponent grid, EntityData<WorldTransform> transforms)
    {
        Rail? rail;
        lock (Gate) if (!Rails.TryGetValue(e, out rail)) return;
        try
        {
            long now = Now;
            if (rail.Releasing || (now + e.GetHashCode()) % CheckInterval != 0) return;
            if (rail.Conic != null) Orbits.Patch(ref rail.Conic, Orbits.Now);   // SOI entry / exit
            Pose(rail, now, out WorldTransform wt, out Vector3D v, out Vector3D w);
            rail.LastVelocity = v; rail.LastAngular = w; // saves use these
            bool release = NearestPlayer(wt.Position) < Settings.RealRailsDistance * 0.8;
            if (!release && rail.Conic != null && NearBubble(wt.Position, Orbits.Now, 1000.0, out string? bubbleName))
            {
                release = true;
                ProbeLog.Write($"rails: {e} approaching bubble {bubbleName} - released for handoff");
            }
            if (!release && now - rail.LastWrite >= WriteInterval)
            {
                rail.LastWrite = now;
                transforms.WriteTo(e, wt);
            }
            if (release)
            {
                rail.Releasing = true;
                MotionQueue.Enqueue((grid, false));
                ProbeLog.Write($"rails: {e} releasing (player within {Settings.RealRailsDistance * 0.8:0} m) at {wt.Position} v={v.Length():0.0} m/s");
            }
        }
        catch (Exception ex)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("rails: Drive error: " + ex);
        }
    }

    /// <summary>Once a released grid is dynamic again, teleport it to the analytic pose with the analytic velocity
    /// (EntityTransformFunctions.Teleport sets Teleported, which makes clients reset their motion history,
    /// MotionStateApplicationSystem.cs:88-91).</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [MustHave(typeof(DynamicTag))]
    internal static void Land(DEntity e, GridPhysicsComponent grid, EntityData<WorldTransform> transforms, Writable<RigidBodyData> body,
        EntityData<Keen.VRage.Core.Game.Data.Teleported> teleported)
    {
        Rail? rail;
        lock (Gate) if (!Rails.TryGetValue(e, out rail)) return;
        try
        {
            if (!rail.Releasing) return; // still being switched to static
            Pose(rail, Now, out WorldTransform wt, out Vector3D v, out Vector3D w);
            transforms.WriteTo(e, wt);
            teleported.WriteTo(e, new Keen.VRage.Core.Game.Data.Teleported { Value = true });
            ref RigidBodyData rb = ref body.WritableValue;
            rb.LinearVelocity = (Vector3)v;
            rb.AngularVelocity = (Vector3)w;
            lock (Gate) Rails.Remove(e);
            Interlocked.Increment(ref Released);
            ProbeLog.Write($"rails: {e} OFF rails at {wt.Position} v={v.Length():0.0} m/s");
        }
        catch (Exception ex)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("rails: Land error: " + ex);
        }
    }

    // ---------- saves ----------

    [HarmonyPatch(typeof(GridPhysicsComponent), "Serialize")]
    internal static class SavePatch
    {
        [HarmonyPostfix]
        private static void Postfix(GridPhysicsComponent __instance, GridPhysicsObjectBuilder ob)
        {
            try
            {
                Rail? rail;
                lock (Gate) if (!Rails.TryGetValue(__instance.DEntity, out rail)) return;
                ob.MotionType = Keen.VRage.Physics.Data.BodyArgs.Motion.Dynamic;
                ob.LinearVelocity = (Vector3)rail.LastVelocity;
                ob.AngularVelocity = (Vector3)rail.LastAngular;
            }
            catch { }
        }
    }

    // ---------- propagation ----------

    private static Vector3D Rot(in Vector3D v, in Vector3D omega, double t)
    {
        double w = omega.Length();
        if (w < 1e-12) return v;
        Vector3D axis = omega / w; double a = w * t;
        double c = Math.Cos(a), s = Math.Sin(a);
        return v * c + Vector3D.Cross(axis, v) * s + axis * (Vector3D.Dot(axis, v) * (1 - c));
    }

    private static Quaternion RotQ(in Vector3D omega, double t)
    {
        double w = omega.Length();
        if (w < 1e-12) return Quaternion.Identity;
        return Quaternion.CreateFromAxisAngle((Vector3)(omega / w), (float)(w * t));
    }

    internal static void Pose(Rail rail, long tick, out WorldTransform wt, out Vector3D vLocal, out Vector3D wLocal)
    {
        if (rail.Conic != null)
        {
            double tn = Orbits.Now;
            rail.Conic.Eval(tn, out Vector3D pos, out Vector3D vel);
            vLocal = vel;
            wLocal = rail.WInertial;
            wt = new WorldTransform(pos, Quaternion.Normalize(Orbits.RotQ(rail.WInertial, tn - rail.OrbitT0) * rail.Q0));
            return;
        }
        double t = (tick - rail.T0) * TickDt;
        Vector3D r, v;
        if (rail.Mu <= 0) { r = rail.R0 + rail.V0 * t; v = rail.V0; }
        else Kepler(rail.R0, rail.V0, rail.Mu, t, out r, out v);
        // inertial -> local (rotating) frame
        Vector3D rl = Rot(r, rail.Omega, -t);
        vLocal = Rot(v, rail.Omega, -t) - Vector3D.Cross(rail.Omega, rl);
        wLocal = Rot(rail.WInertial, rail.Omega, -t) - rail.Omega;
        Quaternion q = RotQ(rail.Omega, -t) * RotQ(rail.WInertial, t) * rail.Q0;
        wt = new WorldTransform(rail.Centre + rl, Quaternion.Normalize(q));
    }

    /// <summary>Universal-variable Kepler propagation (H. Curtis, Orbital Mechanics for Engineering Students, alg. 3.3/3.4).</summary>
    internal static void Kepler(in Vector3D R0, in Vector3D V0, double mu, double dt, out Vector3D R, out Vector3D V)
    {
        double r0 = R0.Length(), v0 = V0.Length();
        double vr0 = Vector3D.Dot(R0, V0) / r0;
        double alpha = 2.0 / r0 - v0 * v0 / mu;
        double sq = Math.Sqrt(mu);
        double x = sq * Math.Abs(alpha) * dt;
        for (int i = 0; i < 60; i++)
        {
            double z = alpha * x * x;
            double C = StumpffC(z), S = StumpffS(z);
            double F = r0 * vr0 / sq * x * x * C + (1 - alpha * r0) * x * x * x * S + r0 * x - sq * dt;
            double dF = r0 * vr0 / sq * x * (1 - z * S) + (1 - alpha * r0) * x * x * C + r0;
            double step = F / dF;
            x -= step;
            if (Math.Abs(step) < 1e-9) break;
        }
        double zz = alpha * x * x, Cz = StumpffC(zz), Sz = StumpffS(zz);
        double f = 1 - x * x / r0 * Cz;
        double g = dt - x * x * x / sq * Sz;
        R = R0 * f + V0 * g;
        double r = R.Length();
        double fd = sq / (r * r0) * (zz * Sz - 1) * x;
        double gd = 1 - x * x / r * Cz;
        V = R0 * fd + V0 * gd;
    }

    private static double StumpffS(double z)
    {
        if (z > 1e-8) { double s = Math.Sqrt(z); return (s - Math.Sin(s)) / (s * s * s); }
        if (z < -1e-8) { double s = Math.Sqrt(-z); return (Math.Sinh(s) - s) / (s * s * s); }
        return 1.0 / 6.0;
    }

    private static double StumpffC(double z)
    {
        if (z > 1e-8) return (1 - Math.Cos(Math.Sqrt(z))) / z;
        if (z < -1e-8) return (Math.Cosh(Math.Sqrt(-z)) - 1) / -z;
        return 0.5;
    }
}
