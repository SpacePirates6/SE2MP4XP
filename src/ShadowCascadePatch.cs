using System;
using System.Reflection;
using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Render.Contracts;

namespace SpinProbe;

/// <summary>
/// Bubbles mode: keep reused directional shadow cascades pinned to the rotating planet, with no extra rendering.
/// <para>Why: CascadeShadowsContext.FlushUpdates (VRage.Render12\...\LightingStage\CascadeShadowsContext.cs:92-130) re-renders only
/// ShadowSettings.CascadesUpdateCount cascades per frame (default 5 of 6, CascadesAlwaysUpdated 4) and samples the others from a
/// shadow map rendered on an earlier frame (priority :292-311). Exact in vanilla - terrain never moves in world space. Under
/// global rendering the planet turns every frame (3 deg/s = ~53 m/frame at the surface), so the two far cascades, which take
/// turns, hold shadows where the terrain was one frame ago and snap when re-rendered: far shadows shimmer at 30 Hz.</para>
/// <para>Fix: a cascade is sampled only through its ViewTransform / InvViewTransform / ViewProjection
/// (Cascade.cs GetCascadeSetupRelativeToView). Geometry in a bubble moves rigidly: g_prev = g_now * inv(F(t_now)) * F(t_prev),
/// F = the bubble's local-to-global frame (row vectors, as MatrixD). So before FlushUpdates, every cascade's matrices get that
/// frame's planet motion D = inv(F_now) * F_prev: a reused map then lines up with the terrain as drawn now. Cascades that
/// FlushUpdates re-renders are recomputed from scratch (UpdateViewSetupFull), so the correction is harmless for them. The
/// light direction of a reused map is off by one frame of rotation (0.05 deg) - the same staleness vanilla accepts.</para>
/// <para>Frame time: the render thread gets the camera through SettingsManager.SetCameraParameters (:676) before
/// DrawContextManager calls FlushUpdates (DrawContextManager.cs:153). The game thread records which t / bubble each global
/// camera pose was converted with (GlobalRender.CameraPatch converts it); the render thread looks its pose up, so the
/// correction uses exactly the t the terrain of that frame was drawn at.</para>
/// </summary>
internal static class ShadowCascadePatch
{
    private static readonly Type? ContextType = AccessTools.TypeByName("Keen.VRage.Render12.LightingStage.CascadeShadowsContext");
    private static readonly Type? CascadeType = AccessTools.TypeByName("Keen.VRage.Render12.LightingStage.Cascade");
    private static readonly Type? SettingsType = AccessTools.TypeByName("Keen.VRage.Render12.Core.Systems.SettingsManager");
    private static readonly AccessTools.FieldRef<object, Array>? Cascades =
        ContextType == null ? null : AccessTools.FieldRefAccess<Array>(ContextType, "_cascades");
    private static readonly PropertyInfo? ViewP = CascadeType == null ? null : AccessTools.Property(CascadeType, "ViewTransform");
    private static readonly PropertyInfo? InvViewP = CascadeType == null ? null : AccessTools.Property(CascadeType, "InvViewTransform");
    private static readonly PropertyInfo? ViewProjP = CascadeType == null ? null : AccessTools.Property(CascadeType, "ViewProjection");

    // Game thread -> render thread: global camera position -> (t, bubble) it was converted with. Small ring, newest wins.
    private struct Entry { public Vector3D Pos; public double T; public Bubbles.Bubble? B; public bool Used; }
    private static readonly Entry[] Ring = new Entry[16];
    private static int _ringNext;
    private static readonly object RingLock = new();

    // Render thread state.
    private static double _renderT = double.NaN;
    private static Bubbles.Bubble? _renderB;
    private static double _prevT = double.NaN;
    private static Bubbles.Bubble? _prevB;
    private static long _applied, _missed;
    private static bool _logged;

    internal static void Install(Harmony h)
    {
        if (ContextType == null || CascadeType == null || SettingsType == null || Cascades == null || ViewP == null || InvViewP == null || ViewProjP == null)
        {
            ProbeLog.Write("shadows: cascade reprojection NOT installed (renderer types/members not found)");
            return;
        }
        h.Patch(AccessTools.Method(typeof(RenderSettings), nameof(RenderSettings.SetCameraParameters)),
            postfix: new HarmonyMethod(typeof(ShadowCascadePatch), nameof(GameCameraPostfix)));
        h.Patch(AccessTools.Method(SettingsType, "SetCameraParameters"),
            prefix: new HarmonyMethod(typeof(ShadowCascadePatch), nameof(RenderCameraPrefix)));
        h.Patch(AccessTools.Method(ContextType, "FlushUpdates"),
            prefix: new HarmonyMethod(typeof(ShadowCascadePatch), nameof(FlushPrefix)));
        ProbeLog.Write("shadows: cascade reprojection installed");
    }

    /// <summary>Game thread, after GlobalRender.CameraPatch converted the camera: remember which t / bubble it used.</summary>
    private static void GameCameraPostfix(RenderSettings __instance)
    {
        if (!Bubbles.Ready || !GlobalRender.HaveCamera) return;
        // Only reproject shadow cascades when the camera is actually inside a rotating planet bubble.
        // In global space (e.g. at an asteroid within 4*R of a planet), nearby asteroids/grids do NOT
        // rotate with the planet, so rotating reused cascades by planet spin causes 30 Hz shadow flicker.
        Bubbles.Bubble? b = GlobalRender.CameraBubble;
        lock (RingLock)
        {
            Ring[_ringNext] = new Entry { Pos = __instance.CameraTransform.Position, T = Bubbles.T, B = b, Used = true };
            _ringNext = (_ringNext + 1) % Ring.Length;
        }
    }

    /// <summary>Render thread: the camera of the frame being drawn. Find the t / bubble it was converted with.</summary>
    private static void RenderCameraPrefix(in WorldTransform cameraTransform)
    {
        lock (RingLock)
        {
            for (int k = 1; k <= Ring.Length; k++)
            {
                Entry e = Ring[(_ringNext - k + Ring.Length) % Ring.Length];
                if (e.Used && e.Pos == cameraTransform.Position) { _renderT = e.T; _renderB = e.B; return; }
            }
        }
        _missed++;
    }

    /// <summary>Render thread, before the cascades are scheduled: move every cascade with the planet by this frame's motion.</summary>
    private static void FlushPrefix(object __instance)
    {
        double t = _renderT;
        Bubbles.Bubble? b = _renderB;
        if (double.IsNaN(t) || b == null || b.IsIdentity || b != _prevB || double.IsNaN(_prevT))
        {
            _prevT = t; _prevB = b;
            return;
        }
        if (t == _prevT) return;
        MatrixD fNow = Bubbles.ToGlobal(b, WorldTransform.Identity, t);
        MatrixD fPrev = Bubbles.ToGlobal(b, WorldTransform.Identity, _prevT);
        MatrixD d = MatrixD.Invert(in fNow) * fPrev;      // world now -> world when the previous frame was drawn
        MatrixD dInv = MatrixD.Invert(in fPrev) * fNow;
        _prevT = t;
        Array cascades = Cascades!(__instance);
        foreach (object c in cascades)
        {
            MatrixD v = (MatrixD)ViewP!.GetValue(c)!;
            MatrixD iv = (MatrixD)InvViewP!.GetValue(c)!;
            MatrixD vp = (MatrixD)ViewProjP!.GetValue(c)!;
            ViewP.SetValue(c, d * v);
            InvViewP.SetValue(c, iv * dInv);
            ViewProjP.SetValue(c, d * vp);
        }
        _applied++;
        if (!_logged) { _logged = true; ProbeLog.Write($"shadows: reprojecting {cascades.Length} cascades with the planet (first at t={t:0.000}s)"); }
        if (_applied % 3600 == 0) ProbeLog.Write($"shadows: reprojected {_applied} frames, camera lookups missed {_missed}");
    }
}
