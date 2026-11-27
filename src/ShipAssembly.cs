using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using Keen.Game2.Simulation.WorldObjects.CubeGrids;
using Keen.Game2.Simulation.WorldObjects.CubeGrids.GridGraphs;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Components;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Data;

namespace SpinProbe;

/// <summary>
/// Per-tick cached mechanical ship assembly queries (parent grid + rotor/hinge/piston/connector subgrids).
///
/// Crash-safe & zero-alloc design:
/// - Reads GridGraphHandler._graphs -> GridGraph._nodes.Keys directly (O(N), zero Satsuma graph DFS, zero iterator
///   allocations) with a hard cap of 64 members.
/// - Caches BOTH connected groups AND solo/negative results for 30 ticks (0.5 s) so a shattered/split grid never
///   re-scans on every tick.
/// - Per-tick WorldCoM / mass refresh updates the existing GroupInfo in-place under its own lock.
/// </summary>
internal static class ShipAssembly
{
    private const long TopologyRefreshTicks = 30;
    private const int MaxGroupMembers = 64;

    internal sealed class GroupInfo
    {
        internal readonly object Sync = new();
        internal long TopologyTick;
        internal long MassTick = -1;
        internal bool IsSolo;
        internal DEntity Primary;
        internal float TotalMass;
        internal float PrimaryMass;
        internal Vector3D WorldCoM;
        internal bool HasStaticMember;
        internal Member[] Members = Array.Empty<Member>();
    }

    internal struct Member
    {
        internal Entity Entity;
        internal DEntity DEntity;
        internal CubeGridComponent Grid;
        internal WorldTransform Transform;
        internal RigidBodyMassProperties Mass;
        internal bool IsDynamic;
    }

    private static readonly ConcurrentDictionary<DEntity, GroupInfo> Cache = new();
    private static readonly object TopologyLock = new();
    private static readonly List<Member> ScratchList = new(16);
    private static readonly HashSet<DEntity> ScratchSeen = new();
    private static long _lastPruneTick;

    /// <summary>
    /// Returns the cached whole-ship mechanical group for <paramref name="grid"/> for the current server tick,
    /// or null if the grid is standalone (no connected subgrids).
    /// </summary>
    internal static GroupInfo? GetGroup(CubeGridComponent? grid)
    {
        if (grid == null) return null;
        GridGraphHandler? handler = grid.GridGraphHandler;
        if (handler == null) return null;
        try
        {
            var graphs = handler._graphs;
            if (graphs == null || graphs.Count == 0)
            {
                Cache.TryRemove(grid.DEntity, out _);
                return null;
            }

            long tick = Profiler.Ticks;
            DEntity selfId = grid.DEntity;

            if (Cache.TryGetValue(selfId, out GroupInfo? cached))
            {
                if (tick - cached.TopologyTick < TopologyRefreshTicks)
                {
                    if (cached.IsSolo) return null;
                    if (cached.MassTick != tick) RefreshMassInPlace(cached, tick);
                    return cached;
                }
            }

            GroupInfo? rebuilt = RebuildTopology(grid, handler, tick);
            MaybePrune(tick);
            return rebuilt != null && !rebuilt.IsSolo ? rebuilt : null;
        }
        catch
        {
            return null;
        }
    }

    private static GroupInfo? RebuildTopology(CubeGridComponent grid, GridGraphHandler handler, long tick)
    {
        lock (TopologyLock)
        {
            DEntity selfId = grid.DEntity;
            if (Cache.TryGetValue(selfId, out GroupInfo? existing) && tick - existing.TopologyTick < TopologyRefreshTicks)
            {
                if (!existing.IsSolo && existing.MassTick != tick) RefreshMassInPlace(existing, tick);
                return existing;
            }

            ScratchList.Clear();
            ScratchSeen.Clear();

            var graphs = handler._graphs;
            if (graphs != null && graphs.Count > 0)
            {
                foreach (var kv in graphs)
                {
                    GridGraph? gg = kv.Value;
                    if (gg == null) continue;
                    foreach (Entity e in gg.GetEntities(grid.Entity))
                    {
                        if (e == null || !ScratchSeen.Add(e.DEntity)) continue;
                        CubeGridComponent? cg = e.TryGet<CubeGridComponent>();
                        if (cg == null) continue;
                        ScratchList.Add(new Member
                        {
                            Entity = e,
                            DEntity = e.DEntity,
                            Grid = cg,
                        });
                        if (ScratchList.Count >= MaxGroupMembers) break;
                    }
                    if (ScratchList.Count >= MaxGroupMembers) break;
                }
            }

            if (ScratchList.Count <= 1)
            {
                // Cache negative (solo) result for 30 ticks so broken/dangling graphs don't re-scan every tick!
                GroupInfo solo = existing ?? new GroupInfo();
                lock (solo.Sync)
                {
                    solo.IsSolo = true;
                    solo.TopologyTick = tick;
                    solo.MassTick = tick;
                    solo.Members = Array.Empty<Member>();
                }
                Cache[selfId] = solo;
                return solo;
            }

            GroupInfo info = existing ?? new GroupInfo();
            lock (info.Sync)
            {
                info.IsSolo = false;
                if (info.Members.Length != ScratchList.Count)
                    info.Members = new Member[ScratchList.Count];
                for (int i = 0; i < ScratchList.Count; i++)
                    info.Members[i] = ScratchList[i];
                info.TopologyTick = tick;
                RefreshMassLocked(info, tick);
            }

            for (int i = 0; i < info.Members.Length; i++)
                Cache[info.Members[i].DEntity] = info;

            return info;
        }
    }

    private static void RefreshMassInPlace(GroupInfo info, long tick)
    {
        lock (info.Sync)
        {
            if (info.MassTick == tick || info.IsSolo) return;
            RefreshMassLocked(info, tick);
        }
    }

    private static void RefreshMassLocked(GroupInfo info, long tick)
    {
        float totalMass = 0f;
        float bestMass = -1f;
        DEntity primary = info.Members.Length > 0 ? info.Members[0].DEntity : default;
        Vector3D weightedCom = Vector3D.Zero;
        bool hasStatic = false;

        for (int i = 0; i < info.Members.Length; i++)
        {
            ref Member m = ref info.Members[i];
            Entity e = m.Entity;
            if (e == null || !e.Data.TryGet<WorldTransform>(out WorldTransform wt))
            {
                m.IsDynamic = false;
                continue;
            }
            m.Transform = wt;
            bool isDyn = e.Data.Has<DynamicTag>() && !e.Data.Has<KinematicTag>();
            bool hasMass = e.Data.TryGet<RigidBodyMassProperties>(out RigidBodyMassProperties mp);
            m.Mass = mp;

            if (!isDyn || !hasMass || mp.InvMass <= 0f || !float.IsFinite(mp.InvMass))
            {
                m.IsDynamic = false;
                hasStatic = true;
            }
            else
            {
                m.IsDynamic = true;
                float kg = 1f / mp.InvMass;
                if (kg > 0f && float.IsFinite(kg))
                {
                    Vector3D comLocal = mp.CenterOfMass;
                    Vector3D comWorld = WorldTransform.Transform(in comLocal, in wt);
                    weightedCom += comWorld * kg;
                    totalMass += kg;
                    if (kg > bestMass || (kg == bestMass && m.DEntity.GetHashCode() < primary.GetHashCode()))
                    {
                        bestMass = kg;
                        primary = m.DEntity;
                    }
                }
            }
        }

        info.Primary = primary;
        info.TotalMass = totalMass;
        info.PrimaryMass = Math.Max(bestMass, 0f);
        info.WorldCoM = totalMass > 1e-3f ? weightedCom / totalMass : (info.Members.Length > 0 ? info.Members[0].Transform.Position : Vector3D.Zero);
        info.HasStaticMember = hasStatic;
        info.MassTick = tick;
    }

    internal static Vector3 GetWholeShipLocalCoM(CubeGridComponent? grid, in WorldTransform wt, in RigidBodyMassProperties mass, out GroupInfo? group)
    {
        group = GetGroup(grid);
        return GetLocalCoMFromGroup(group, in wt, in mass);
    }

    internal static Vector3 GetLocalCoMFromGroup(GroupInfo? group, in WorldTransform wt, in RigidBodyMassProperties mass)
    {
        if (group == null || group.IsSolo || group.TotalMass <= 1e-3f) return mass.CenterOfMass;
        Vector3D rel = WorldTransform.TransformInv(group.WorldCoM, wt);
        return (Vector3)rel;
    }

    internal static bool IsConstrainedSubgrid(DEntity entity, GroupInfo? group)
    {
        return group != null && !group.IsSolo && group.Members.Length > 1 && group.Primary != entity;
    }

    private static void MaybePrune(long tick)
    {
        if (tick - _lastPruneTick < 600) return;
        _lastPruneTick = tick;
        if (Cache.Count < 128) return;
        foreach (var kv in Cache)
            if (tick - kv.Value.MassTick > 600)
                Cache.TryRemove(kv.Key, out _);
    }
}
