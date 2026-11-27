using System;
using HarmonyLib;
using Keen.Game2.Client.GameSystems.CameraSystems;
using Keen.Game2.Client.GameSystems.GPS;
using Keen.Game2.Simulation.GameSystems.GPS;
using Keen.VRage.Core;
using Keen.VRage.Library.Mathematics;

namespace SpinProbe;

/// <summary>
/// HUD GPS / antenna / ore markers under planet bubbles.
/// Source: Game2.Client\GameSystems\GPS\GPSMarkerRenderSessionComponent.cs:100-338 DrawHUDMarkers projects every
/// marker with CameraComponent.WorldToProjected(in pos) using the game-side (bubble-local or spectator) camera,
/// and measures distance against GPSMarkerHelpers.GetAnchorPosition.
/// Under global rendering, each marker lives in its own bubble (or global space) and is seen from GlobalRender.CameraBubble:
/// convert marker -> global -> camera's frame so markers stay glued to rotating planets, orbiting moons, and fixed stars
/// from both the ground and space, with true global distances.
/// </summary>
internal static class HudMarkers
{
    [ThreadStatic] private static bool _inDraw;
    [ThreadStatic] private static Vector3D _anchorGlobal;
    [ThreadStatic] private static bool _haveAnchor;

    private static Vector3D ToGlobalPos(in Vector3D pos, double t)
    {
        Bubbles.Bubble? b = Bubbles.BubbleAt(pos);
        if (b == null || b.IsIdentity) return pos;
        return Bubbles.ToGlobal(b, new WorldTransform(pos), t).Position;
    }

    [HarmonyPatch(typeof(GPSMarkerRenderSessionComponent), "DrawHUDMarkers")]
    internal static class DrawScopePatch
    {
        [HarmonyPrefix]
        private static void Prefix()
        {
            _inDraw = true;
            _haveAnchor = false;
        }

        [HarmonyFinalizer]
        private static void Finalizer()
        {
            _inDraw = false;
            _haveAnchor = false;
        }
    }

    [HarmonyPatch(typeof(GPSMarkerHelpers), nameof(GPSMarkerHelpers.GetAnchorPosition))]
    internal static class AnchorPatch
    {
        [HarmonyPostfix]
        private static void Postfix(Vector3D __result)
        {
            if (!_inDraw || !Bubbles.Ready || MapRender.Open) return;
            Bubbles.Bubble? camB = GlobalRender.CameraBubble;
            double t = Bubbles.T;
            // Anchor is either the possessed spectator or the character; if it matches the camera, use CameraBubble
            // (so hysteresis at the spectator boundary agrees), otherwise classify by radius.
            Bubbles.Bubble? ab = (GlobalRender.HaveCamera && (GlobalRender.LocalCamera.Position - __result).LengthSquared() < 25.0)
                ? camB
                : Bubbles.BubbleAt(__result);
            _anchorGlobal = (ab != null && !ab.IsIdentity) ? Bubbles.ToGlobal(ab, new WorldTransform(__result), t).Position : __result;
            _haveAnchor = true;
        }
    }

    [HarmonyPatch(typeof(CameraComponent), nameof(CameraComponent.WorldToProjected))]
    internal static class ProjectPatch
    {
        [HarmonyPrefix]
        private static void Prefix(ref Vector3D worldPosition)
        {
            if (!_inDraw || !Bubbles.Ready || MapRender.Open) return;
            double t = Bubbles.T;
            Vector3D globalPos = ToGlobalPos(worldPosition, t);
            Bubbles.Bubble? camB = GlobalRender.CameraBubble;
            worldPosition = (camB != null && !camB.IsIdentity)
                ? Bubbles.ToLocal(camB, new WorldTransform(globalPos), t).Position
                : globalPos;
        }
    }

    [HarmonyPatch(typeof(MarkerBunch), nameof(MarkerBunch.Add), new[] { typeof(GPSMarker), typeof(double) })]
    internal static class DistancePatch
    {
        [HarmonyPrefix]
        private static void Prefix(GPSMarker marker, ref double distance)
        {
            if (!_inDraw || !_haveAnchor || !Bubbles.Ready || MapRender.Open) return;
            Vector3D globalPos = ToGlobalPos(marker.Position, Bubbles.T);
            distance = Vector3D.Distance(in _anchorGlobal, in globalPos);
        }
    }
}
