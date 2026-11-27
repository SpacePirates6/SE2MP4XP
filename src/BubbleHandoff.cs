using System;
using System.Collections.Concurrent;
using System.Threading;
using Keen.Game2.Simulation.WorldObjects.CubeGrids;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Components;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Havok;
using Keen.VRage.Physics.Havok.Character;

namespace SpinProbe;

/// <summary>
/// Atomic Bubble-Boundary Handoff for ships (with connected rotor/hinge/piston subgrids) and airborne characters.
///
/// Key architecture guarantees (fixing client/server ID isolation, multiplayer compatibility, and thread safety):
/// 1. Server and Client state tables are COMPLETELY SEPARATE (ServerStates vs ClientStates):
///    - Server DEntity IDs are NEVER looked up by client rendering code (in single-player host, client and server
///      are separate Scenes whose DEntity IDs overlap!).
///    - Client tracking works identically in single-player AND remote multiplayer purely from replicated poses:
///      it maintains +/-250 m hysteresis AND detects the server's atomic handoff jump (matching ToGlobal/ToLocal
///      within tolerance) so every subgrid switches render frame on the exact replicated snapshot!
/// 2. Thread safety:
///    - Parallel physics jobs (CheckGridHandoff / CheckCharacterHandoff) are READ-ONLY and only enqueue pending
///      group handoffs into HandoffQueue.
///    - Actual pose/velocity/Teleported writes execute SINGLE-THREADED in [WorldUpdateBeginJobGroup] (FlushHandoffs),
///      before PhysicsUpdate runs — identical to RealRails.FlushMotion — so cross-entity writes never race!
/// </summary>
internal static class BubbleHandoff
{
    internal const double Band = 250.0; // hysteresis half-width (m)

    private sealed class ServerState
    {
        internal Bubbles.Bubble? Bubble; // null = global inertial space
        internal Vector3D LastPos;
        internal long LastTick;
        internal bool Pending;
    }

    private sealed class ClientState
    {
        internal Bubbles.Bubble? Bubble;
        internal Vector3D LastPos;
        internal double LastT;
    }

    private sealed class PendingHandoff
    {
        internal Entity[] Targets = Array.Empty<Entity>();
        internal DEntity PrimaryId;
        internal Bubbles.Bubble? FromBubble;
        internal Bubbles.Bubble? ToBubble;
        internal double RadiusAtCross;
        internal string Kind = "ship";
    }

    // Strictly server-side tracking (keyed by SERVER DEntity only)
    private static readonly ConcurrentDictionary<DEntity, ServerState> ServerStates = new();
    private static readonly ConcurrentQueue<PendingHandoff> HandoffQueue = new();
    private static readonly object FlushGate = new();

    // Strictly client-side tracking (keyed by CLIENT DEntity only; works in both SP host and remote MP)
    private static readonly ConcurrentDictionary<DEntity, ClientState> ClientStates = new();

    internal static long HandoffCount;
    private static int _errors;

    // ----------------------------------------------------------------------------------------------------------------
    // Server-side membership lookup (ONLY called from server jobs, e.g. RealPhysics.SpinningFrameAt / RealRails)
    // ----------------------------------------------------------------------------------------------------------------

    internal static Bubbles.Bubble? ResolveServerBubble(DEntity serverEntity, in Vector3D pos)
    {
        if (ServerStates.TryGetValue(serverEntity, out ServerState? st))
            return st.Bubble;
        return Bubbles.BubbleAt(pos);
    }

    // ----------------------------------------------------------------------------------------------------------------
    // Client-side membership lookup (ONLY called from client GlobalRenderJobs & SpectatorHandoff; MP-safe)
    // ----------------------------------------------------------------------------------------------------------------

    // The client never decides a crossing on its own inside the hysteresis band: the server converts the entity (pose
    // jump) a tick or more before the client sees it, so a client-side hysteresis test switched the frame while the
    // replicated pose was still in the old coordinates (flicker of kilometres for a few frames, subgrids switching on
    // different frames). The frame switches only when the replicated pose itself jumps to the converted pose. A position
    // that can only be explained by the other frame (well past the band) is a fallback for a missed jump.
    private const double ClientFallbackMargin = 2000.0;
    private static long _lastClientPrune;

    internal static Bubbles.Bubble? ResolveClientBubble(DEntity clientEntity, in Vector3D pos)
    {
        if (!Bubbles.Ready) return null;
        double t = Bubbles.T;
        MaybePruneClient(t);

        if (!ClientStates.TryGetValue(clientEntity, out ClientState? st))
        {
            Bubbles.Bubble? init = Bubbles.BubbleAt(pos);
            ClientStates[clientEntity] = new ClientState { Bubble = init, LastPos = pos, LastT = t };
            return init;
        }

        Vector3D prevPos = st.LastPos;
        double moved = (pos - prevPos).Length();
        Bubbles.Bubble? cur = st.Bubble;

        if (moved > 5.0)
        {
            // Server handoff: the new pose is the old pose converted to the other frame (plus this frame's own motion).
            // Accept the frame whose conversion explains the new pose much better than plain motion does.
            if (cur != null)
            {
                if (!cur.IsIdentity)
                {
                    double viaConvert = (pos - Bubbles.ToGlobal(cur, new WorldTransform(prevPos), t).Position).Length();
                    if (viaConvert < 0.5 * moved) cur = null;
                }
            }
            else
            {
                var all = Bubbles.All;
                for (int i = 0; i < all.Count; i++)
                {
                    var b = all[i];
                    if (b.IsIdentity) continue;
                    double viaConvert = (pos - Bubbles.ToLocal(b, new WorldTransform(prevPos), t).Position).Length();
                    if (viaConvert < 0.5 * moved) { cur = b; break; }
                }
            }
            if (moved > 5000.0 && cur == st.Bubble) cur = Bubbles.BubbleAt(pos); // unrelated teleport (respawn, paste)
        }

        // Fallback for a missed jump: a pose that is only possible in the other frame.
        if (cur != null)
        {
            if ((pos - cur.C0).Length() > cur.Radius + Band + ClientFallbackMargin) cur = null;
        }
        else
        {
            var all = Bubbles.All;
            for (int i = 0; i < all.Count; i++)
            {
                var b = all[i];
                if (!b.IsIdentity && (pos - Bubbles.Centre(b, t)).Length() < b.Radius - Band - ClientFallbackMargin) { cur = b; break; }
            }
        }

        st.Bubble = cur;
        st.LastPos = pos;
        st.LastT = t;
        return cur;
    }

    /// <summary>Drops client entries not seen for 30 s (deleted or streamed-out entities), at most every 10 s.</summary>
    private static void MaybePruneClient(double t)
    {
        long now = (long)t;
        long last = Interlocked.Read(ref _lastClientPrune);
        if (now - last < 10 || Interlocked.CompareExchange(ref _lastClientPrune, now, last) != last) return;
        foreach (var kv in ClientStates)
            if (t - kv.Value.LastT > 30.0) ClientStates.TryRemove(kv.Key, out _);
    }

    /// <summary>
    /// Client-only camera bubble resolver: when the camera is near a bubble boundary, matches the nearest
    /// client-tracked entity (e.g. cockpit/ship being piloted) so the camera and ship switch frames together.
    /// Never reads ServerStates.
    /// </summary>
    internal static bool TryResolveClientCameraBubble(in Vector3D cameraPos, out Bubbles.Bubble? bubble)
    {
        bubble = null;
        if (!Bubbles.Ready || ClientStates.IsEmpty) return false;
        bool nearBoundary = false;
        double t = Bubbles.T;
        var all = Bubbles.All;
        for (int i = 0; i < all.Count; i++)
        {
            var b = all[i];
            if (b.IsIdentity) continue;
            if (Math.Abs((cameraPos - b.C0).Length() - b.Radius) < 1500.0 ||
                Math.Abs((cameraPos - Bubbles.Centre(b, t)).Length() - b.Radius) < 1500.0)
            {
                nearBoundary = true;
                break;
            }
        }
        if (!nearBoundary) return false;

        double bestDistSq = 150.0 * 150.0;
        bool found = false;
        foreach (var kv in ClientStates)
        {
            ClientState st = kv.Value;
            if (t - st.LastT > 1.0) continue;
            double dSq = (st.LastPos - cameraPos).LengthSquared();
            if (dSq < bestDistSq)
            {
                bestDistSq = dSq;
                bubble = st.Bubble;
                found = true;
            }
        }
        return found;
    }

    // ----------------------------------------------------------------------------------------------------------------
    // Math helpers
    // ----------------------------------------------------------------------------------------------------------------

    internal static Vector3D OrbitalVelocity(Bubbles.Bubble b, double t)
    {
        if (b.OrbitAround == null) return Vector3D.Zero;
        Vector3D parentVel = OrbitalVelocity(b.OrbitAround, t);
        Vector3D rel = Bubbles.Centre(b, t) - Bubbles.Centre(b.OrbitAround, t);
        return parentVel + Vector3D.Cross(b.OrbitAxis * b.OrbitRate, rel);
    }

    private static Vector3D RotVec(in Vector3D v, in Vector3D axis, double a)
    {
        if (a == 0.0) return v;
        double c = Math.Cos(a), s = Math.Sin(a);
        return v * c + Vector3D.Cross(axis, v) * s + axis * (Vector3D.Dot(axis, v) * (1.0 - c));
    }

    // ----------------------------------------------------------------------------------------------------------------
    // Server-side READ-ONLY boundary detectors (parallel safe) + single-threaded FlushHandoffs (WorldUpdateBegin)
    // ----------------------------------------------------------------------------------------------------------------

    [PhysicsUpdate.OnCollectMassProperties]
    [WhenSimulated]
    [MustHave(typeof(DynamicTag))]
    [MustHave(typeof(ActiveTag))]
    [DoesNotHave(typeof(KinematicTag))]
    [DoesNotHave(typeof(TransformDriven))]
    [DoesNotHave(typeof(CharacterInfo))]
    internal static void CheckGridHandoff(DEntity entity, CubeGridComponent grid, in WorldTransform wt)
    {
        try
        {
            if (!Settings.RealPhysics || !Settings.RealBubbleHandoff || !Bubbles.Ready) return;

            ShipAssembly.GroupInfo? group = ShipAssembly.GetGroup(grid);
            if (group != null)
            {
                if (group.HasStaticMember) return;
                if (group.Primary != entity) return;
            }

            Vector3D refPos = group != null ? group.WorldCoM : wt.Position;
            long tick = Profiler.Ticks;

            if (!ServerStates.TryGetValue(entity, out ServerState? state))
            {
                Bubbles.Bubble? initB = Bubbles.BubbleAt(refPos);
                SetServerGroupMembership(entity, group, initB, refPos, tick);
                return;
            }

            if (state.Pending) return;

            if ((refPos - state.LastPos).LengthSquared() > 5000.0 * 5000.0)
            {
                Bubbles.Bubble? syncB = Bubbles.BubbleAt(refPos);
                SetServerGroupMembership(entity, group, syncB, refPos, tick);
                return;
            }

            state.LastPos = refPos;
            state.LastTick = tick;

            double t = Bubbles.T;
            Bubbles.Bubble? cur = state.Bubble;

            if (cur != null)
            {
                double dist = (refPos - cur.C0).Length();
                if (dist <= cur.Radius + Band) return;

                state.Pending = true;
                HandoffQueue.Enqueue(new PendingHandoff
                {
                    Targets = CollectTargets(grid.Entity, group),
                    PrimaryId = entity,
                    FromBubble = cur,
                    ToBubble = null,
                    RadiusAtCross = dist,
                    Kind = "ship",
                });
            }
            else
            {
                var all = Bubbles.All;
                for (int i = 0; i < all.Count; i++)
                {
                    Bubbles.Bubble b = all[i];
                    double dist = (refPos - Bubbles.Centre(b, t)).Length();
                    if (dist >= b.Radius - Band) continue;

                    state.Pending = true;
                    HandoffQueue.Enqueue(new PendingHandoff
                    {
                        Targets = CollectTargets(grid.Entity, group),
                        PrimaryId = entity,
                        FromBubble = null,
                        ToBubble = b,
                        RadiusAtCross = dist,
                        Kind = "ship",
                    });
                    return;
                }
            }
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("handoff: CheckGridHandoff error: " + e);
        }
    }

    [PhysicsUpdate.OnCollectMassProperties]
    [WhenSimulated]
    [MustHave(typeof(DynamicTag))]
    [MustHave(typeof(CharacterInfo))]
    internal static void CheckCharacterHandoff(DEntity entity, Keen.VRage.Physics.Components.CharacterPhysicsComponent charComp, in WorldTransform wt)
    {
        try
        {
            if (!Settings.RealPhysics || !Settings.RealBubbleHandoff || !Bubbles.Ready) return;
            Vector3D pos = wt.Position;
            long tick = Profiler.Ticks;

            if (!ServerStates.TryGetValue(entity, out ServerState? state))
            {
                ServerStates[entity] = new ServerState { Bubble = Bubbles.BubbleAt(pos), LastPos = pos, LastTick = tick };
                return;
            }
            if (state.Pending) return;
            if ((pos - state.LastPos).LengthSquared() > 5000.0 * 5000.0)
            {
                state.Bubble = Bubbles.BubbleAt(pos);
                state.LastPos = pos;
                state.LastTick = tick;
                return;
            }
            state.LastPos = pos;
            state.LastTick = tick;

            double t = Bubbles.T;
            Bubbles.Bubble? cur = state.Bubble;

            if (cur != null)
            {
                double dist = (pos - cur.C0).Length();
                if (dist <= cur.Radius + Band) return;

                state.Pending = true;
                HandoffQueue.Enqueue(new PendingHandoff
                {
                    Targets = new[] { charComp.Entity },
                    PrimaryId = entity,
                    FromBubble = cur,
                    ToBubble = null,
                    RadiusAtCross = dist,
                    Kind = "char",
                });
            }
            else
            {
                var all = Bubbles.All;
                for (int i = 0; i < all.Count; i++)
                {
                    Bubbles.Bubble b = all[i];
                    double dist = (pos - Bubbles.Centre(b, t)).Length();
                    if (dist >= b.Radius - Band) continue;

                    state.Pending = true;
                    HandoffQueue.Enqueue(new PendingHandoff
                    {
                        Targets = new[] { charComp.Entity },
                        PrimaryId = entity,
                        FromBubble = null,
                        ToBubble = b,
                        RadiusAtCross = dist,
                        Kind = "char",
                    });
                    return;
                }
            }
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("handoff: CheckCharacterHandoff error: " + e);
        }
    }

    /// <summary>Other awake dynamic bodies (floating objects, loose voxel chunks). Without this the client switched them
    /// to the other frame past the band while the server kept them in the old coordinates, so they were drawn off by the
    /// planet's spin angle for as long as they stayed out there. Bodies in a shared-motion (welded) group are left alone:
    /// the group has to move as one and only grids carry the group lookup.</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [WhenSimulated]
    [MustHave(typeof(DynamicTag))]
    [MustHave(typeof(ActiveTag))]
    [DoesNotHave(typeof(KinematicTag))]
    [DoesNotHave(typeof(TransformDriven))]
    [DoesNotHave(typeof(CharacterInfo))]
    [DoesNotHave(typeof(CubeGridComponent))]
    internal static void CheckBodyHandoff(DEntity entity, Keen.VRage.Physics.Components.RigidBodyComponent body, in WorldTransform wt,
        Optional<Keen.VRage.Physics.Havok.MotionSharing.SharedMotionData> shared)
    {
        try
        {
            if (!Settings.RealPhysics || !Settings.RealBubbleHandoff || !Bubbles.Ready || shared.HasValue) return;
            Vector3D pos = wt.Position;
            long tick = Profiler.Ticks;
            if (!ServerStates.TryGetValue(entity, out ServerState? state))
            {
                ServerStates[entity] = new ServerState { Bubble = Bubbles.BubbleAt(pos), LastPos = pos, LastTick = tick };
                return;
            }
            if (state.Pending) return;
            if ((pos - state.LastPos).LengthSquared() > 5000.0 * 5000.0)
            {
                state.Bubble = Bubbles.BubbleAt(pos);
                state.LastPos = pos; state.LastTick = tick;
                return;
            }
            state.LastPos = pos; state.LastTick = tick;
            double t = Bubbles.T;
            Bubbles.Bubble? cur = state.Bubble;
            if (cur != null)
            {
                double dist = (pos - cur.C0).Length();
                if (dist <= cur.Radius + Band) return;
                state.Pending = true;
                HandoffQueue.Enqueue(new PendingHandoff { Targets = new[] { body.Entity }, PrimaryId = entity, FromBubble = cur, RadiusAtCross = dist, Kind = "body" });
                return;
            }
            var all = Bubbles.All;
            for (int i = 0; i < all.Count; i++)
            {
                Bubbles.Bubble b = all[i];
                double dist = (pos - Bubbles.Centre(b, t)).Length();
                if (dist >= b.Radius - Band) continue;
                state.Pending = true;
                HandoffQueue.Enqueue(new PendingHandoff { Targets = new[] { body.Entity }, PrimaryId = entity, ToBubble = b, RadiusAtCross = dist, Kind = "body" });
                return;
            }
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("handoff: CheckBodyHandoff error: " + e);
        }
    }

    /// <summary>
    /// Single-threaded atomic handoff executor at the very start of the server tick (WorldUpdateBeginJobGroup,
    /// same slot as RealRails.FlushMotion). Zero races with parallel physics jobs!
    /// </summary>
    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(Keen.VRage.DCS.CoreData.InstanceBind<Keen.VRage.Voxels.Components.VoxelPlanetComponent>))]
    internal static void FlushHandoffs(in WorldTransform planet)
    {
        if (HandoffQueue.IsEmpty || !Monitor.TryEnter(FlushGate)) return;
        try
        {
            double t = Bubbles.T;
            long tick = Profiler.Ticks;

            while (HandoffQueue.TryDequeue(out PendingHandoff? h))
            {
                try
                {
                    if (h.FromBubble != null && h.ToBubble == null)
                    {
                        if (!h.FromBubble.IsIdentity)
                            for (int i = 0; i < h.Targets.Length; i++)
                                ConvertEntityToGlobal(h.Targets[i], h.FromBubble, t);
                    }
                    else if (h.FromBubble == null && h.ToBubble != null)
                    {
                        if (!h.ToBubble.IsIdentity)
                            for (int i = 0; i < h.Targets.Length; i++)
                                ConvertEntityToLocal(h.Targets[i], h.ToBubble, t);
                    }

                    for (int i = 0; i < h.Targets.Length; i++)
                    {
                        Entity e = h.Targets[i];
                        if (e == null) continue;
                        Vector3D newPos = e.Data.TryGet<WorldTransform>(out WorldTransform nwt) ? nwt.Position : Vector3D.Zero;
                        ServerStates[e.DEntity] = new ServerState
                        {
                            Bubble = h.ToBubble,
                            LastPos = newPos,
                            LastTick = tick,
                            Pending = false,
                        };
                    }

                    long n = Interlocked.Increment(ref HandoffCount);
                    string fromName = h.FromBubble?.Name ?? "global";
                    string toName = h.ToBubble?.Name ?? "global";
                    double rad = (h.FromBubble ?? h.ToBubble)?.Radius ?? 0;
                    ProbeLog.Write($"handoff {h.Kind} #{n}: {fromName} -> {toName} ({h.Targets.Length} entity) at r={h.RadiusAtCross:0}/{rad:0} t={t:0.0}s");
                }
                catch (Exception ex)
                {
                    if (ServerStates.TryGetValue(h.PrimaryId, out var st)) st.Pending = false;
                    if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("handoff: FlushHandoffs error: " + ex);
                }
            }
        }
        finally
        {
            Monitor.Exit(FlushGate);
        }
    }

    private static Entity[] CollectTargets(Entity primary, ShipAssembly.GroupInfo? group)
    {
        if (group == null) return new[] { primary };
        lock (group.Sync)
        {
            var arr = new Entity[group.Members.Length];
            for (int i = 0; i < group.Members.Length; i++)
                arr[i] = group.Members[i].Entity;
            return arr;
        }
    }

    private static void SetServerGroupMembership(DEntity primary, ShipAssembly.GroupInfo? group, Bubbles.Bubble? b, in Vector3D refPos, long tick)
    {
        if (group == null)
        {
            ServerStates[primary] = new ServerState { Bubble = b, LastPos = refPos, LastTick = tick };
            return;
        }
        lock (group.Sync)
        {
            for (int i = 0; i < group.Members.Length; i++)
            {
                DEntity id = group.Members[i].DEntity;
                ServerStates[id] = new ServerState { Bubble = b, LastPos = refPos, LastTick = tick };
            }
        }
    }

    private static void ConvertEntityToGlobal(Entity e, Bubbles.Bubble b, double t)
    {
        if (e == null || !e.Data.TryGet<WorldTransform>(out WorldTransform wtLocal)) return;
        WorldTransform wtGlobal = Bubbles.ToGlobal(b, wtLocal, t);
        e.Data.Set(wtGlobal);
        e.Data.Set(new Teleported { Value = true });

        if (e.Data.TryGet<RigidBodyData>(out RigidBodyData rb))
        {
            Vector3D omega = b.SpinAxis * b.SpinRate;
            double theta = b.SpinRate * t;
            Vector3D rLocal = wtLocal.Position - b.C0;
            Vector3D vInLocal = (Vector3D)rb.LinearVelocity + Vector3D.Cross(omega, rLocal);
            Vector3D vGlobal = RotVec(vInLocal, b.SpinAxis, theta) + OrbitalVelocity(b, t);
            Vector3D wGlobal = RotVec((Vector3D)rb.AngularVelocity + omega, b.SpinAxis, theta);

            rb.LinearVelocity = (Vector3)vGlobal;
            rb.AngularVelocity = (Vector3)wGlobal;
            e.Data.Set(rb);
        }
    }

    private static void ConvertEntityToLocal(Entity e, Bubbles.Bubble b, double t)
    {
        if (e == null || !e.Data.TryGet<WorldTransform>(out WorldTransform wtGlobal)) return;
        WorldTransform wtLocal = Bubbles.ToLocal(b, wtGlobal, t);
        e.Data.Set(wtLocal);
        e.Data.Set(new Teleported { Value = true });

        if (e.Data.TryGet<RigidBodyData>(out RigidBodyData rb))
        {
            Vector3D omega = b.SpinAxis * b.SpinRate;
            double theta = b.SpinRate * t;
            Vector3D vRel = RotVec((Vector3D)rb.LinearVelocity - OrbitalVelocity(b, t), b.SpinAxis, -theta);
            Vector3D rLocal = wtLocal.Position - b.C0;
            Vector3D vLocal = vRel - Vector3D.Cross(omega, rLocal);
            Vector3D wLocal = RotVec((Vector3D)rb.AngularVelocity, b.SpinAxis, -theta) - omega;

            rb.LinearVelocity = (Vector3)vLocal;
            rb.AngularVelocity = (Vector3)wLocal;
            e.Data.Set(rb);
        }
    }
}
