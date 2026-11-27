using System;
using System.Collections.Generic;
using HarmonyLib;
using Keen.Game2.Client.WorldObjects.ColonizationMap;
using Keen.Game2.Simulation.GameSystems.Discoveries;
using Keen.Game2.Simulation.GameSystems.Discoveries.Discoverables;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Render;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Render.Utils;

namespace SpinProbe;

/// <summary>
/// The colonization map (the M screen) under planet bubbles.
/// <para>Vanilla (Game2.Client ColonizationMapSessionComponent, installed build): UpdateMapPosition [UpdateDataJob, MustHave VisualizeTag]
/// puts the map entity at the observed entity's WorldTransform position with orientation <c>Orientation</c> (built from
/// ColonizationMapConfiguration.SkyboxOrientation so the map lines up with the skybox). Planet previews are hierarchy children
/// placed by UpdateDiscoveries at WorldToMapPosition(Details.Location, ...) - the SAVED planet location, flattened onto the map
/// plane (WorldToMapPosition :1525) - with identity orientation; moons are pushed out with
/// MapMarkersRenderComponent.TranslatePointAlongPlanetRadius.</para>
/// <para>Under global rendering the map entity sits at the player's bubble-local position, so GlobalRender rotated the map AND
/// the map camera with the planet: the whole map turned against the fixed skybox. Fix: while the map is open, the map
/// hierarchy and the camera are drawn with a pure translation (local + Offset, where Offset carries the anchor to its true
/// global position) - no rotation - so the map keeps the skybox orientation. Game-side everything stays local, so map
/// interaction raycasts and label projection (MapObjectRenderComponent) are unchanged.</para>
/// <para>On the map itself: each planet preview is moved to its planet's live centre (orbiting moons move) and turned by its
/// current spin angle, and orbits are drawn as rings in the Map mesh layer (MeshBuilder.Type.Map, the layer the map's own
/// DebugDrawSectors uses; factory created like UI3DSessionComponent.cs:349-351).</para>
/// </summary>
internal static class MapRender
{
    internal static ColonizationMapSessionComponent? Map;
    /// <summary>Map open this frame; set/cleared on the game thread before the render-transform jobs run.</summary>
    internal static volatile bool Open;
    /// <summary>Translation from the map's bubble-local frame to global, for the map hierarchy and the map camera.</summary>
    internal static Vector3D Offset;
    private static HashSet<DEntity> _members = new();
    private static MeshBuilder? _lines;
    private static int _errors;
    private static bool _logged;

    private const int OrbitSegments = 128;

    internal static bool IsMember(DEntity e) => Open && _members.Contains(e);

    internal static WorldTransform Translate(in WorldTransform local)
    {
        WorldTransform g = local;
        g.Position = local.Position + Offset;
        return g;
    }

    // ---------- bubble maths (same as Bubbles.ToGlobal / Centre; Rodrigues, double) ----------
    private static Vector3D Rot(in Vector3D v, in Vector3D axis, double a)
    {
        double c = Math.Cos(a), s = Math.Sin(a);
        return v * c + Vector3D.Cross(axis, v) * s + axis * (Vector3D.Dot(axis, v) * (1 - c));
    }

    private static Vector3D Centre(Bubbles.Bubble b, double t)
    {
        if (b.OrbitAround == null) return b.C0;
        return Centre(b.OrbitAround, t) + Rot(b.C0 - b.OrbitAround.C0, b.OrbitAxis, b.OrbitRate * t);
    }

    private static Bubbles.Bubble? BubbleOf(PlanetDiscoverable p)
    {
        Vector3D loc = p.Details.Location;
        double tol = Math.Max(1000.0, p.Radius * 0.1);
        foreach (Bubbles.Bubble b in Bubbles.All)
            if ((b.C0 - loc).Length() < tol) return b;
        return null;
    }

    /// <summary>Per frame while the map is open (postfix of UpdateMapPosition, which only runs with VisualizeTag).</summary>
    private static void Update(ColonizationMapSessionComponent map)
    {
        Map = map;
        Entity? mapEntity = map._mapEntity;
        if (!map.IsVisible || mapEntity == null || !Bubbles.Ready) { Close(); return; }

        double t = Bubbles.T;
        Vector3D anchor = mapEntity.Data.GetWorldTransform().Position;
        Bubbles.Bubble? ab = Bubbles.BubbleAt(anchor);
        Offset = ab == null || ab.IsIdentity ? Vector3D.Zero : Bubbles.ToGlobal(ab, new WorldTransform(anchor), t).Position - anchor;

        // Map hierarchy: the map entity and every descendant (planet previews, star, volumes, markers).
        var members = new HashSet<DEntity>();
        Collect(mapEntity, members);
        _members = members;
        Open = true;

        Vector3D mapOrigin = map._sectorsSession.MapWorldPosition;
        float mapScale = map._sectorsSession.MapWorldScale;
        Quaternion mapOrientation = map.Orientation;
        Vector3D mapGlobal = anchor + Offset;

        // Planet previews: live centre + spin.
        var planetPos = new Dictionary<PlanetDiscoverable, Vector3D>();
        foreach (var (discovery, preview) in map._discoveredPlanets)
            if (discovery.Discoverable is PlanetDiscoverable p)
            {
                Bubbles.Bubble? b = BubbleOf(p);
                planetPos[p] = b != null ? Centre(b, t) : p.Details.Location;
            }

        _lines ??= new RenderMeshBuilderFactory("SpinProbeMapOrbits").CreateMeshBuilder(MeshBuilder.Type.Map, null, 2048, "SpinProbeMapOrbits");
        // Vertices are float and relative to the batch entity (MeshBuilder has update support by default): put the entity at
        // the map and store map-relative vertices (SetPrimitiveOffset subtracts it in AddLine). Run map1: with neither set,
        // the rings were absolute ~300 km floats and the thick-line ribbon (MeshBuilder.AddLine gradient overload builds its
        // width from normalize(midpoint), i.e. assumes camera-relative input) produced beams across the whole screen.
        _lines.SetPrimitiveOffset(mapGlobal);
        _lines.UpdateEntityTransform(new WorldTransform(mapGlobal));

        foreach (var (discovery, preview) in map._discoveredPlanets)
        {
            if (discovery.Discoverable is not PlanetDiscoverable p) continue;
            Bubbles.Bubble? b = BubbleOf(p);
            if (b == null) continue;
            PlanetDiscoverable? parent = null;
            Vector3D parentPos = default;
            if (p.IsMoon)
            {
                parent = map.FindClosestNonMoonPlanet(p.Details.Location, out _);
                if (parent != null && !planetPos.TryGetValue(parent, out parentPos)) parentPos = parent.Details.Location;
            }

            Vector3D MapPoint(Vector3D world)
            {
                if (parent != null)
                    world = MapMarkersRenderComponent.TranslatePointAlongPlanetRadius(parent, parentPos, world,
                        map._configuration.PlanetScale, map.PlanetInfluenceScale, map.MoonBiasScale);
                return ColonizationMapSessionComponent.WorldToMapPosition(world, anchor, Quaternion.Identity, mapOrigin, mapScale) - anchor;
            }

            Vector3D v = MapPoint(planetPos[p]);
            double spin = b.SpinRate * t % (2 * Math.PI);
            Quaternion q = Quaternion.CreateFromAxisAngle((Vector3)b.SpinAxis, (float)spin);
            preview.Entity.Get<ChildTransformComponent>().ChildTransform = new RelativeTransform((Vector3)v, q);

            // Orbit ring: the moon's centre over one full revolution, mapped exactly like the preview.
            if (b.OrbitAround != null && b.OrbitRate != 0)
            {
                Vector3D centre = Centre(b.OrbitAround, t);
                Vector3D arm = b.C0 - b.OrbitAround.C0;
                Vector3D prev = default;
                for (int i = 0; i <= OrbitSegments; i++)
                {
                    Vector3D world = centre + Rot(arm, b.OrbitAxis, 2 * Math.PI * i / OrbitSegments);
                    Vector3D pt = mapGlobal + mapOrientation * (Vector3)MapPoint(world);
                    if (i > 0) _lines.AddLine(prev, pt, new ColorSRGB(120, 200, 255, 200), depth: false);   // plain GPU line, no ribbon
                    prev = pt;
                }
            }
        }
        // Source: none (plugin-only). Stage 4.2: each orbital belt as an inner and an outer ring about its parent's live
        // centre (plain map position, WorldToMapPosition as above, no moon bias), plus the stage 4.1 star's apparent orbit
        // about the home planet (OrbitalBelts.MapRings).
        foreach (var (centre, up, radius) in OrbitalBelts.MapRings(t))
        {
            Vector3D a = Vector3D.Normalize(Vector3D.Cross(up, Math.Abs(up.Y) < 0.9 ? new Vector3D(0, 1, 0) : new Vector3D(1, 0, 0))) * radius;
            Vector3D prev = default;
            for (int i = 0; i <= OrbitSegments; i++)
            {
                Vector3D world = centre + Rot(a, up, 2 * Math.PI * i / OrbitSegments);
                Vector3D local = ColonizationMapSessionComponent.WorldToMapPosition(world, anchor, Quaternion.Identity, mapOrigin, mapScale) - anchor;
                Vector3D pt = mapGlobal + mapOrientation * (Vector3)local;
                if (i > 0) _lines.AddLine(prev, pt, new ColorSRGB(200, 170, 110, 160), depth: false);
                prev = pt;
            }
        }
        _lines.Commit();

        if (!_logged)
        {
            _logged = true;
            ProbeLog.Write($"map: open, {members.Count} map entities, {planetPos.Count} planet previews, offset {Offset.X:0},{Offset.Y:0},{Offset.Z:0}");
        }
    }

    private static void Collect(Entity e, HashSet<DEntity> into)
    {
        into.Add(e.DEntity);
        var children = e.TryGet<HierarchyComponent>()?.Children;
        if (children == null) return;
        foreach (Entity c in children) Collect(c, into);
    }

    private static void Close()
    {
        if (!Open) return;
        Open = false;
        Offset = Vector3D.Zero;
        _members = new HashSet<DEntity>();
        _lines?.Commit();      // nothing added since the last commit: clears the rings
        _logged = false;
    }

    [HarmonyPatch(typeof(ColonizationMapSessionComponent), "UpdateMapPosition")]
    internal static class UpdatePatch
    {
        [HarmonyPostfix]
        private static void Postfix(ColonizationMapSessionComponent __instance)
        {
            try { Update(__instance); }
            catch (Exception e) { if (_errors++ < 5) ProbeLog.Write("map: update failed: " + e); }
        }
    }

    /// <summary>
    /// Map markers (installed Game2.Client MapMarkersRenderComponent): GPS markers (:574, :598), fast-travel points (:618, :627)
    /// and TryGetMapScreenPosition (:440) - which MapTabViewModel (:366) calls with the player's position for "You are here" -
    /// all place points through this one private WorldToMapPosition(point, anchor) (:388). The points are the game's
    /// bubble-local positions, so on the map they stayed put while the planet preview spins. Convert the POINT to its true
    /// global position; the ANCHOR stays local (the map entity is at the local anchor and only translated at render time).
    /// The planet-radius bias is applied before this, radially about the planet's saved centre: for a spin-only planet that
    /// commutes with the spin about the same centre, so the result is exact.
    /// </summary>
    [HarmonyPatch(typeof(MapMarkersRenderComponent), "WorldToMapPosition", new[] { typeof(Vector3D), typeof(Vector3D) })]
    internal static class MarkerPatch
    {
        [HarmonyPrefix]
        private static void Prefix(ref Vector3D pointWorldPosition)
        {
            try
            {
                if (!Bubbles.Ready) return;
                Bubbles.Bubble? b = Bubbles.BubbleAt(pointWorldPosition);
                if (b == null || b.IsIdentity) return;
                pointWorldPosition = Bubbles.ToGlobal(b, new WorldTransform(pointWorldPosition), Bubbles.T).Position;
            }
            catch (Exception e) { if (_errors++ < 5) ProbeLog.Write("map: marker conversion failed: " + e); }
        }
    }

    [HarmonyPatch(typeof(ColonizationMapSessionComponent), nameof(ColonizationMapSessionComponent.HideMap))]
    internal static class HidePatch
    {
        [HarmonyPostfix]
        private static void Postfix()
        {
            try { Close(); }
            catch (Exception e) { if (_errors++ < 5) ProbeLog.Write("map: close failed: " + e); }
        }
    }
}
