using System;
using HarmonyLib;
using Keen.Game2.Client.WorldObjects.ColonizationMap;
using Keen.Game2.Simulation.GameSystems.Discoveries.Discoverables;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Library.Mathematics;

namespace SpinProbe;

/// <summary>
/// Colonization map markers under planet bubbles (companion to MapRender.cs, which spins/moves the planet previews).
/// <para>Vanilla (Game2.Client ColonizationMap\MapMarkersRenderComponent.cs): every marker drawn on the map - GPS markers
/// (BunchMarkers :616-660), bunch averages, fast-travel lanes (DrawFastTravelLanes :662-700) and the player / screen-position
/// queries (TryGetMapScreenPosition :542 via the 1-arg BiasPositionAlongPlanetRadius :527) - goes through the private
/// BiasPositionAlongPlanetRadius(position, closestPlanet, closestPlanetPosition) (:514), then WorldToMapPosition. The
/// position is the marker's game (bubble-local) position and closestPlanetPosition the planet's SAVED location
/// (PrepareMarkersClosestPlanets :770), so markers stayed put while MapRender turned and moved the preview under them.</para>
/// <para>Fix: in that one function, take the marker to its live global position (Bubbles.ToGlobal at Bubbles.T - the same t
/// and maths MapRender uses for the preview) and bias it against the planet's live centre. Both then go through the
/// unchanged vanilla bias and WorldToMapPosition, exactly like the preview, so the marker rides the spinning, orbiting
/// planet. The reverse (MapToWorldPosition, :1512 - map point back to a world point) gets global -> local.</para>
/// </summary>
internal static class MapMarkers
{
    private static int _errors;
    private static long _converted;
    private static bool _logged;

    private static Bubbles.Bubble? BubbleOf(PlanetDiscoverable p)
    {
        Vector3D loc = p.Details.Location;
        double tol = Math.Max(1000.0, p.Radius * 0.1);
        foreach (Bubbles.Bubble b in Bubbles.All)
            if ((b.C0 - loc).Length() < tol) return b;
        return null;
    }

    [HarmonyPatch(typeof(MapMarkersRenderComponent), "BiasPositionAlongPlanetRadius",
        new[] { typeof(Vector3D), typeof(PlanetDiscoverable), typeof(Vector3D) })]
    internal static class BiasPatch
    {
        [HarmonyPrefix]
        private static void Prefix(ref Vector3D position, PlanetDiscoverable closestPlanet, ref Vector3D closestPlanetPosition)
        {
            try
            {
                if (!Bubbles.Ready || closestPlanet == null) return;
                Bubbles.Bubble? b = BubbleOf(closestPlanet);
                if (b == null || b.IsIdentity) return;
                double t = Bubbles.T;
                // Only a marker that lives in this planet's grid is in its local frame.
                if (Bubbles.BubbleAt(position) == b)
                    position = Bubbles.ToGlobal(b, new WorldTransform(position), t).Position;
                closestPlanetPosition = Bubbles.Centre(b, t);
                _converted++;
                if (!_logged && _converted > 0)
                {
                    _logged = true;
                    ProbeLog.Write($"map markers: biasing against live planet {b.Name} (t={t:0.0}s)");
                }
            }
            catch (Exception e) { if (_errors++ < 5) ProbeLog.Write("map markers: bias failed: " + e); }
        }
    }

    [HarmonyPatch(typeof(ColonizationMapSessionComponent), nameof(ColonizationMapSessionComponent.MapToWorldPosition),
        new[] { typeof(Vector3D), typeof(Vector3D), typeof(Quaternion), typeof(Vector3D), typeof(float) })]
    internal static class MapToWorldPatch
    {
        [HarmonyPostfix]
        private static void Postfix(ref Vector3D __result)
        {
            try
            {
                if (!Bubbles.Ready || !MapRender.Open) return;
                double t = Bubbles.T;
                foreach (Bubbles.Bubble b in Bubbles.All)
                {
                    if (b.IsIdentity) continue;
                    if ((__result - Bubbles.Centre(b, t)).LengthSquared() < b.Radius * b.Radius)
                    {
                        __result = Bubbles.ToLocal(b, new WorldTransform(__result), t).Position;
                        return;
                    }
                }
            }
            catch (Exception e) { if (_errors++ < 5) ProbeLog.Write("map markers: map->world failed: " + e); }
        }
    }
}
