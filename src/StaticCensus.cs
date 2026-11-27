using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.CoreData;
using Keen.VRage.Physics.Havok.WorldManager;

namespace SpinProbe;

/// <summary>
/// Stage 1.3: which static bodies have their WorldTransform written every server tick. Run 93's profile showed
/// HavokLargeWorldManager.UpdateMovedStaticBodyClusters at 0.65 ms/tick, 38 calls/tick. That job is
/// [PhysicsUpdate.OnUpdateStaticBodyClusters][OnChanged(WorldTransform)] over static bodies (HavokLargeWorldManager.cs:1295-1299),
/// so 38 static bodies are moved every tick. Each call ends in UpdateStaticEntityCluster(entity, ...) (:651; also reached from
/// UpdateMutatedStaticAabbBodyClusters :1286), which this counts per entity and names (entity, or the owner its InstanceBind
/// points at, e.g. a planet's physics chunk).
/// </summary>
internal static class StaticCensus
{
    private const string ServerThread = "In-Process Server Thread";
    private static readonly Dictionary<DEntity, (string name, int calls)> Counts = new();
    private static long _windowStartTick = -1;

    // Patches: VRage.Physics/Keen/VRage/Physics/Havok/WorldManager/HavokLargeWorldManager.cs  UpdateStaticEntityCluster (:651)
    [HarmonyPatch(typeof(HavokLargeWorldManager), "UpdateStaticEntityCluster")]
    internal static class Patch
    {
        [HarmonyPrefix]
        private static void Prefix(HavokLargeWorldManager __instance, DEntity entity)
        {
            if (Thread.CurrentThread.Name != ServerThread) return;
            lock (Counts)
            {
                if (Counts.TryGetValue(entity, out var e)) { Counts[entity] = (e.name, e.calls + 1); return; }
                Counts[entity] = (Name(__instance, entity), 1);
            }
        }
    }

    // Reads: HavokLargeWorldManager.GetContext (:1084, private, publicized); Entity.TryGetFromDataEntity; InstanceBind<Entity>.
    private static string Name(HavokLargeWorldManager manager, DEntity entity)
    {
        try
        {
            DEntityContext ctx = manager.GetContext(entity);
            Entity? e = Entity.TryGetFromDataEntity(ctx);
            if (e != null) return e.ToString() ?? entity.ToString();
            if (ctx.TryGet<InstanceBind<Entity>>(out var bind) && bind.Instance != null) return "part of " + bind.Instance;
        }
        catch { }
        return entity.ToString();
    }

    /// <summary>Called from Profiler.Report (server thread, every 2 s); logs every 10 s.</summary>
    internal static void MaybeReport(long ticks)
    {
        if (_windowStartTick < 0) { _windowStartTick = ticks; return; }
        long span = ticks - _windowStartTick;
        if (span < 600) return;
        List<(string name, int calls)> rows;
        lock (Counts)
        {
            rows = Counts.Values.ToList();
            Counts.Clear();
        }
        _windowStartTick = ticks;
        if (rows.Count == 0) return;
        double total = rows.Sum(r => r.calls) / (double)span;
        // Group by name (a planet's many chunks collapse into one row with an entity count).
        var grouped = rows.GroupBy(r => r.name)
            .Select(g => (name: g.Key, entities: g.Count(), perTick: g.Sum(r => r.calls) / (double)span))
            .OrderByDescending(g => g.perTick).Take(10);
        ProbeLog.Write($"census: {total:0.0} static-cluster updates/tick from {rows.Count} static entities over {span} ticks: " +
                       string.Join("; ", grouped.Select(g => $"{g.name} x{g.entities} = {g.perTick:0.0}/tick")));
    }
}
