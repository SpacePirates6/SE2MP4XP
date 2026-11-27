using System;
using System.Diagnostics;
using System.Threading;
using HarmonyLib;
using Keen.VRage.Voxels.Client.Clipmap;

namespace SpinProbe;

/// <summary>
/// Observation only (no behaviour change): voxel terrain cell churn per second, to tell clipmap-side causes of the terrain
/// "blink" (cells really deleted / re-created) from render-side ones (cells exist but are not drawn).
/// Hypotheses and sources: renderer study 2026-11-25 (H1 frozen-clipmap wipe VoxelClipmap.cs:1300-1372 - no
/// "Detected frozen clipmap" in any test log; H2 settings re-mesh VoxelClipmap.cs:1394-1398 - presets only at startup;
/// H3 ring churn VoxelClipmapRing.cs:151-260/379-400; H4 empty meshes VoxelClipmapRing.cs:430-487; H5 stalled batches
/// VoxelClipmap.cs:2394-2405; H7 render culling).
/// Patched (all in VRage.Voxels.Client\...\Clipmap): VoxelClipmap.CreateCell :2329, DeleteCell :2350, InvalidateAll :1274,
/// InvalidateAllInternal :1740, EndBatch :2394 (return value); VoxelCell.ApplyTransition :239, SetForDeletion :429;
/// VoxelClipmapRing.AddCell :367, RemoveCell :379, UpdateCellData :430 (guide == null -> empty), UpdateCellRender :465.
/// </summary>
internal static class ClipmapDiag
{
    private static long _create, _delete, _invalidateAll, _invalidateAllInternal, _endBatchTrue, _endBatchFalse;
    private static long _transition, _setForDeletion, _ringAdd, _ringRemove, _cellData, _cellDataEmpty, _cellRender;
    private static readonly Stopwatch Clock = Stopwatch.StartNew();
    private static long _lastReportMs;
    private static bool _loggedInvalidateStack;

    internal static void Install(Harmony h)
    {
        void P(Type t, string name, string patch, bool postfix = false)
        {
            try
            {
                var m = AccessTools.Method(t, name);
                if (m == null) { ProbeLog.Write($"clipdiag: {t.Name}.{name} NOT FOUND"); return; }
                var hm = new HarmonyMethod(typeof(ClipmapDiag), patch);
                if (postfix) h.Patch(m, postfix: hm); else h.Patch(m, prefix: hm);
            }
            catch (Exception e) { ProbeLog.Write($"clipdiag: {t.Name}.{name} failed: {e.Message}"); }
        }
        P(typeof(VoxelClipmap), "CreateCell", nameof(Create));
        P(typeof(VoxelClipmap), "DeleteCell", nameof(Delete));
        P(typeof(VoxelClipmap), "InvalidateAll", nameof(InvalidateAll));
        P(typeof(VoxelClipmap), "InvalidateAllInternal", nameof(InvalidateAllInternal));
        P(typeof(VoxelClipmap), "EndBatch", nameof(EndBatch), postfix: true);
        P(typeof(VoxelCell), "ApplyTransition", nameof(Transition));
        P(typeof(VoxelCell), "SetForDeletion", nameof(SetForDeletion));
        P(typeof(VoxelClipmapRing), "AddCell", nameof(RingAdd));
        P(typeof(VoxelClipmapRing), "RemoveCell", nameof(RingRemove));
        P(typeof(VoxelClipmapRing), "UpdateCellData", nameof(CellData));
        P(typeof(VoxelClipmapRing), "UpdateCellRender", nameof(CellRender));
        ProbeLog.Write("clipdiag: installed");
    }

    private static void Create() => Interlocked.Increment(ref _create);
    private static void Delete() => Interlocked.Increment(ref _delete);
    private static void InvalidateAll()
    {
        Interlocked.Increment(ref _invalidateAll);
        if (!_loggedInvalidateStack) { _loggedInvalidateStack = true; ProbeLog.Write("clipdiag: InvalidateAll called from\n" + Environment.StackTrace); }
    }
    private static void InvalidateAllInternal() => Interlocked.Increment(ref _invalidateAllInternal);
    private static void EndBatch(bool __result) { if (__result) Interlocked.Increment(ref _endBatchTrue); else Interlocked.Increment(ref _endBatchFalse); }
    private static void Transition() => Interlocked.Increment(ref _transition);
    private static void SetForDeletion() => Interlocked.Increment(ref _setForDeletion);
    private static void RingAdd() => Interlocked.Increment(ref _ringAdd);
    private static void RingRemove() => Interlocked.Increment(ref _ringRemove);
    private static void CellData(object __1) { Interlocked.Increment(ref _cellData); if (__1 == null) Interlocked.Increment(ref _cellDataEmpty); }
    private static void CellRender() => Interlocked.Increment(ref _cellRender);

    /// <summary>Called from the main-thread frame hook; writes one line per second.</summary>
    internal static void Tick()
    {
        long now = Clock.ElapsedMilliseconds;
        if (now - _lastReportMs < 1000) return;
        _lastReportMs = now;
        ProbeLog.Write($"clipdiag/s: create={X(ref _create)} delete={X(ref _delete)} transition={X(ref _transition)} setForDeletion={X(ref _setForDeletion)} " +
                       $"ringAdd={X(ref _ringAdd)} ringRemove={X(ref _ringRemove)} cellData={X(ref _cellData)} (empty {X(ref _cellDataEmpty)}) cellRender={X(ref _cellRender)} " +
                       $"endBatch ok/blocked={X(ref _endBatchTrue)}/{X(ref _endBatchFalse)} invalidateAll={X(ref _invalidateAll)} invalidateAllInternal={X(ref _invalidateAllInternal)}");
    }

    private static long X(ref long v) => Interlocked.Exchange(ref v, 0);
}
