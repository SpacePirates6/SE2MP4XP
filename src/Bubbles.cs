using System;
using System.Collections.Generic;
using System.Diagnostics;
using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Render.SessionComponents;
using Keen.VRage.Voxels.Client.Components;
using Keen.VRage.Voxels.Components;

namespace SpinProbe;

/// <summary>
/// Planet bubbles (docs/ENGINE-ARCHITECTURE.md Part F). Each planet owns a grid space; the planet is fixed in it, and the
/// grid space moves through inertial global space: x_global = F_p(t)(x_local) = Rspin_p(t)(x_local - C0_p) + C_p(t), with
/// C_p(t) = O + Rorbit(phi(t))(C0_p - O) for an orbiting body. All frames coincide with global space at t = 0, so the save
/// needs no migration and the server keeps simulating every bubble in its local coordinates exactly like vanilla.
/// <para>Client side: everything is rendered in the VIEWER's bubble (the bubble the camera is in). Entities in that bubble
/// need no work at all. Other bubbles are placed at F_v(t)^-1 · F_q(t) every frame, and the sun direction and skybox are
/// rotated by Rspin_v(t)^-1 - which is what a rotating planet looks like from its surface.</para>
/// </summary>
internal static class Bubbles
{
    internal sealed class Bubble
    {
        internal string Name = "";
        internal Vector3D C0;            // centre at t = 0 (= its local origin)
        internal double Radius;          // bubble radius (planet radius * factor)
        internal Vector3D SpinAxis = new(0, 1, 0);
        internal double SpinRate;        // rad/s
        internal Bubble? OrbitAround;    // orbit centre = that bubble's centre at t
        internal Vector3D OrbitAxis = new(0, 1, 0);
        internal double OrbitRate;       // rad/s
        internal readonly List<VoxelRenderComponent> Renders = new();
        internal bool Placed;
        internal bool IsIdentity => SpinRate == 0 && OrbitAround == null;
    }

    internal static readonly List<Bubble> All = new();
    internal static readonly Dictionary<object, Bubble> ClipmapBubble = new(ReferenceEqualityComparer.Instance);
    internal static readonly Dictionary<object, VoxelRenderComponent> ClipmapRender = new(ReferenceEqualityComparer.Instance);
    internal static readonly Dictionary<object, WorldTransform> InitialCentred = new(ReferenceEqualityComparer.Instance);
    internal static readonly Dictionary<object, (Bubble b, VoxelRenderComponent r)> RootDictOwner = new(ReferenceEqualityComparer.Instance);
    private static readonly List<VoxelRenderComponent> Pending = new();
    private static readonly Stopwatch Clock = new();
    private static bool _built;
    private static Bubble? _viewer;
    private static Quaternion _skyBase;
    private static bool _skyBaseSet;
    private static double _lastSkyAngle = double.NaN;
    private static long _frames;

    /// <summary>
    /// Bubble time, sampled ONCE per client frame (FrameClock job at the start of the frame). Run 20: every conversion read
    /// the stopwatch itself; within a frame the camera, render roots and terrain were converted milliseconds apart, and at
    /// 538 m/s surface speed that put them metres apart, varying per frame (ground flicker, popping trees, a grid seen
    /// sinking into the planet).
    /// </summary>
    internal static double T => _frameT;
    private static double _frameT;
    private static long _frameId;
    private static long _lastTickedFrameId = -1;
    private static volatile bool _hasPending;
    internal static void SampleFrameTime()
    {
        if (Clock.IsRunning) _frameT = Clock.Elapsed.TotalSeconds;
        System.Threading.Interlocked.Increment(ref _frameId);
    }

    // ---------- frame math (double precision, Rodrigues) ----------
    private static Vector3D Rot(in Vector3D v, in Vector3D axis, double a)
    {
        double c = Math.Cos(a), s = Math.Sin(a);
        return v * c + Vector3D.Cross(axis, v) * s + axis * (Vector3D.Dot(axis, v) * (1 - c));
    }

    internal static Vector3D Centre(Bubble b, double t)
    {
        if (b.OrbitAround == null) return b.C0;
        Vector3D o = Centre(b.OrbitAround, t);
        return o + Rot(b.C0 - b.OrbitAround.C0, b.OrbitAxis, b.OrbitRate * t);
    }

    /// <summary>Local → global for a pose in bubble b (null = global space).</summary>
    internal static WorldTransform ToGlobal(Bubble? b, in WorldTransform local, double t)
    {
        if (b == null) return local;
        double a = b.SpinRate * t;
        WorldTransform g = local;
        g.Position = Rot(local.Position - b.C0, b.SpinAxis, a) + Centre(b, t);
        g.Orientation = Quaternion.Normalize(Quaternion.CreateFromAxisAngle((Vector3)b.SpinAxis, (float)a) * local.Orientation);
        return g;
    }

    /// <summary>Global → local for bubble b.</summary>
    internal static WorldTransform ToLocal(Bubble? b, in WorldTransform global, double t)
    {
        if (b == null) return global;
        double a = b.SpinRate * t;
        WorldTransform l = global;
        l.Position = Rot(global.Position - Centre(b, t), b.SpinAxis, -a) + b.C0;
        l.Orientation = Quaternion.Normalize(Quaternion.CreateFromAxisAngle((Vector3)b.SpinAxis, (float)-a) * global.Orientation);
        return l;
    }

    /// <summary>The planet's local (centred) orientation, i.e. the local orientation of every terrain root of bubble b.</summary>
    internal static bool LocalOrientation(Bubble b, out Quaternion q)
    {
        q = Quaternion.Identity;
        foreach (VoxelRenderComponent r in b.Renders)
            if (InitialCentred.TryGetValue(r, out WorldTransform c)) { q = c.Orientation; return true; }
        return false;
    }

    internal static Bubble? BubbleAt(in Vector3D localPos)
    {
        foreach (Bubble b in All)
            if ((localPos - b.C0).LengthSquared() < b.Radius * b.Radius) return b;
        return null;
    }

    // ---------- discovery ----------

    /// <summary>
    /// Client planet renderers. Source: VRage.Voxels.Client\...\Components\VoxelRenderComponent.cs:126-138 - the explicit
    /// IInSceneListener.OnAddedToScene builds the clipmap and ends with <c>this.UpdateSession.RegisterClipmap(this)</c>
    /// (VoxelRenderUpdateSessionComponent.RegisterClipmap). Run 14 tried to patch "OnAddedToScene" by name, which does not
    /// exist (explicit interface implementation), so no planet was ever registered.
    /// :424 _voxelStorageComponent; :267-277 UpdatePosition -> _clipmap.NotifyMoved / _lowResClipmap.NotifyMoved.
    /// </summary>
    [HarmonyPatch(typeof(VoxelRenderUpdateSessionComponent), "RegisterClipmap")]
    internal static class RenderAddedPatch
    {
        [HarmonyPostfix]
        private static void Postfix(VoxelRenderComponent __0)
        {
            lock (Pending) Pending.Add(__0);
            _hasPending = true;
            // Stage 4.2: hide stationary client-only procedural asteroid placeholders in global space (outside all planet
            // exclusion zones) so only real orbiting server-spawned asteroids are drawn there — not just inside the two
            // narrow belt rings, so gaps (96-111 km, 153-239 km, >360 km) never show motionless placeholders either.
            try
            {
                if (OrbitalBelts.On && Settings.BeltsSuppressVanilla && __0._voxelStorageComponent != null &&
                    __0._voxelStorageComponent is not VoxelPlanetComponent)
                {
                    string name = __0._voxelStorageComponent.Entity?.ToString() ?? "";
                    if (name.Contains("ProceduralAsteroid", StringComparison.OrdinalIgnoreCase) &&
                        name.Contains("_ClientComposition", StringComparison.Ordinal))
                    {
                        Vector3D pos = __0._voxelStorageComponent.Data.GetWorldTransform().Position;
                        bool insidePlanetBubble = false;
                        if (Ready)
                        {
                            double t = T;
                            foreach (Bubble b in All)
                            {
                                double guard = b.Radius * 1.0125; // matches MovableAsteroids 1.62 R exclusion
                                if ((pos - b.C0).LengthSquared() < guard * guard ||
                                    (pos - Centre(b, t)).LengthSquared() < guard * guard)
                                {
                                    insidePlanetBubble = true;
                                    break;
                                }
                            }
                        }
                        if (!insidePlanetBubble)
                        {
                            __0.SetVisible(false);
                            if (__0._lowResClipmap != null) __0._lowResClipmap.Visible = false;
                        }
                    }
                }
            }
            catch { }
        }
    }

    [HarmonyPatch(typeof(VoxelRenderUpdateSessionComponent), "UnregisterClipmap")]
    internal static class RenderRemovedPatch
    {
        [HarmonyPostfix]
        private static void Postfix(VoxelRenderComponent __0)
        {
            lock (Pending) Pending.Remove(__0);
            Adopted.Remove(__0);
            foreach (Bubble b in All) b.Renders.Remove(__0);
            if (__0._clipmap != null)
            {
                ClipmapBubble.Remove(__0._clipmap);
                ClipmapRender.Remove(__0._clipmap);
                RootDictOwner.Remove(__0._clipmap.RootEntityProvider._rootEntities);
                ClipmapCameraPatch.Remove(__0._clipmap);
            }
            if (__0._lowResClipmap != null)
            {
                ClipmapBubble.Remove(__0._lowResClipmap);
                ClipmapRender.Remove(__0._lowResClipmap);
                RootDictOwner.Remove(__0._lowResClipmap.RootEntityProvider._rootEntities);
                ClipmapCameraPatch.Remove(__0._lowResClipmap);
            }
        }
    }

    private static void Build()
    {
        // Planets: VoxelPlanetComponent storages among the client renderers (boulders / stamps / asteroids are skipped).
        var planets = new List<(VoxelRenderComponent r, VoxelPlanetComponent p, Vector3D c)>();
        lock (Pending)
        {
            foreach (VoxelRenderComponent r in Pending)
                if (r._voxelStorageComponent is VoxelPlanetComponent p)
                    planets.Add((r, p, p.Data.GetWorldTransform().Position));
            foreach (var (r0, p0, c0) in planets) ProbeLog.Write($"bubbles: planet {c0} R={p0.Radius:0} saved orientation {p0.Data.GetWorldTransform().Orientation} centred={p0.ComputeCenteredTransform().Position} storageOffset={p0.StorageOffset}");
        }
        if (planets.Count == 0) return;
        _built = true;
        foreach (var (r, p, c) in planets)
        {
            WorldTransform initCentred = p.ComputeCenteredTransform();
            InitialCentred[r] = initCentred;
            Bubble? existing = All.Find(b => (b.C0 - c).LengthSquared() < 1.0);
            if (existing != null)
            {
                existing.Renders.Add(r);
                if (r._clipmap != null) { ClipmapBubble[r._clipmap] = existing; ClipmapRender[r._clipmap] = r; RootDictOwner[r._clipmap.RootEntityProvider._rootEntities] = (existing, r); }
                if (r._lowResClipmap != null) { ClipmapBubble[r._lowResClipmap] = existing; ClipmapRender[r._lowResClipmap] = r; RootDictOwner[r._lowResClipmap.RootEntityProvider._rootEntities] = (existing, r); }
                continue;
            }
            var b = new Bubble { Name = $"planet@{c.X:0},{c.Y:0},{c.Z:0} R={p.Radius:0}", C0 = c, Radius = p.Radius * Settings.FrameRadiusFactor };
            b.Renders.Add(r);
            All.Add(b);
            if (r._clipmap != null) { ClipmapBubble[r._clipmap] = b; ClipmapRender[r._clipmap] = r; RootDictOwner[r._clipmap.RootEntityProvider._rootEntities] = (b, r); }
            if (r._lowResClipmap != null) { ClipmapBubble[r._lowResClipmap] = b; ClipmapRender[r._lowResClipmap] = r; RootDictOwner[r._lowResClipmap.RootEntityProvider._rootEntities] = (b, r); }
        }
        ProbeLog.Write($"bubbles: {All.Count} planet bubble(s) from {planets.Count} client renderer(s)");
        AdoptVoxelRenderers();
    }

    /// <summary>
    /// Boulders and voxel stamps are separate voxel entities: PlanetEnvironmentSectorObjectBuilder.ModifyObjectTransform
    /// (VRage.Voxels\...\PlanetEnvironmentSectorObjectBuilder.cs:408-447) spawns any prefab with a VoxelOctreeStorageComponent as a
    /// root entity, rendered by its own VoxelRenderComponent/clipmap (VoxelRenderComponent.cs:267-277). Run 25: they stayed at
    /// their local pose (fixed in global space) while the camera swept past -> "boulders flying by". Every voxel renderer whose
    /// storage centre lies inside a bubble belongs to that bubble; its clipmap moves are converted by ClipmapMovePatch and it
    /// is re-posed every frame with the planet.
    /// </summary>
    private static readonly HashSet<VoxelRenderComponent> Adopted = new(ReferenceEqualityComparer.Instance);
    private static void AdoptVoxelRenderers()
    {
        _hasPending = false;
        int n = 0;
        lock (Pending)
        {
            foreach (VoxelRenderComponent r in Pending)
            {
                if (r._voxelStorageComponent is VoxelPlanetComponent || Adopted.Contains(r)) continue;
                // Never adopt a movable asteroid as a static planetary boulder.
                var vp = r._voxelStorageComponent?.Entity?.TryGet<VoxelPhysicsComponent>();
                if (vp != null && MovableAsteroids.States.TryGetValue(vp, out _)) continue;
                Vector3D c = r._voxelStorageComponent.Data.GetWorldTransform().Position;
                Bubble? b = BubbleAt(c);
                if (b == null) continue;
                Adopted.Add(r);
                b.Renders.Add(r);
                if (r._clipmap != null) { ClipmapBubble[r._clipmap] = b; ClipmapRender[r._clipmap] = r; RootDictOwner[r._clipmap.RootEntityProvider._rootEntities] = (b, r); }
                if (r._lowResClipmap != null) { ClipmapBubble[r._lowResClipmap] = b; ClipmapRender[r._lowResClipmap] = r; RootDictOwner[r._lowResClipmap.RootEntityProvider._rootEntities] = (b, r); }
                n++;
            }
        }
        if (n > 0) ProbeLog.Write($"bubbles: adopted {n} non-planet voxel renderer(s) (boulders / stamps); total adopted {Adopted.Count}");
    }

    private static void Configure(in Vector3D camera)
    {
        // Spin the planet the camera is on; send the nearest other planet into orbit around it.
        Bubble? home = BubbleAt(camera);
        if (home == null)
        {
            double best = double.MaxValue;
            foreach (Bubble b in All) { double d = (b.C0 - camera).Length() - b.Radius; if (d < best) { best = d; home = b; } }
        }
        if (home == null) return;
        home.SpinRate = Settings.SpinDegreesPerSecond * Math.PI / 180.0;
        Bubble? moon = null; double nearest = double.MaxValue;
        foreach (Bubble b in All)
        {
            if (b == home) continue;
            double d = (b.C0 - home.C0).Length();
            if (d < nearest) { nearest = d; moon = b; }
        }
        if (moon != null && Settings.OrbitPeriodSeconds > 0)
        {
            moon.OrbitAround = home;
            moon.OrbitAxis = home.SpinAxis;
            moon.OrbitRate = 2 * Math.PI / Settings.OrbitPeriodSeconds;
        }
        ProbeLog.Write($"bubbles: home {home.Name} spins {Settings.SpinDegreesPerSecond} deg/s (day {360.0 / Math.Max(Settings.SpinDegreesPerSecond, 1e-9) / 60:0.0} min); " +
                       (moon != null ? $"orbiter {moon.Name} at {nearest / 1000:0} km, period {Settings.OrbitPeriodSeconds:0} s" : "no orbiter"));
        // Source: none (plugin-only). Stage 4.1: optional star; the home bubble orbits it (added after the moon search so
        // the star is never taken for the moon).
        try { Orbits.ConfigureStar(home); } catch (Exception e) { ProbeLog.Write("orbits: star configure failed: " + e.Message); }
        Clock.Start();
    }

    // ---------- per-frame client update ----------

    /// <summary>
    /// Runs every client frame after the engine pushes the sun to the renderer.
    /// Source: VRage.Render\...\SessionComponents\SunRenderSessionComponent.cs:22-40 UpdateSun copies ISun.GetSunDirection()
    /// (azimuth/elevation in degrees, Game2.Simulation\...\Sun\SunSessionComponent.cs:153-156 / VRage.Core.Game\...\SunHelper.cs:18-42)
    /// into RenderSettings.SunlightSettings. The camera is RenderSettings.CameraTransform (read the same way by
    /// VRage.Voxels.Client\...\VoxelRenderUpdateSessionComponent.cs:148). The skybox orientation is
    /// EnvironmentSettings.SkyboxOrientation (VRage.Render\...\Data\EnvironmentSettings.cs:368), set exactly like
    /// Game2.Client\...\ColonizationMapSessionComponent.cs:951-956 sets the skybox brightness.
    /// </summary>
    /// <summary>
    /// Called once per client frame from GlobalRenderJobs.MoveClipmaps ([BubbleClipmapMoveJob], ordered after
    /// RenderSubmissionBegin and strictly before VoxelRenderUpdate.OnUpdateClipmap).
    /// </summary>
    internal static void TickFrame()
    {
        long id = System.Threading.Volatile.Read(ref _frameId);
        if (System.Threading.Interlocked.Exchange(ref _lastTickedFrameId, id) == id) return;
        try { Frame(); }
        catch (Exception e) { if (_frames++ < 3) ProbeLog.Write("bubbles: frame error " + e); }
    }

    private static void Frame()
    {
        // Global rendering: global space (sun, stars) is never touched. Every bubble's planet is placed at its global pose
        // F_q(t); every render root in a moving bubble is pushed to F_q(t) · local by GlobalRenderJobs; the render camera is
        // converted by GlobalRender.CameraPatch. Sun direction and skybox are the engine's own, unmodified.
        if (!GlobalRender.HaveCamera) return;
        WorldTransform camera = GlobalRender.LocalCamera;
        if (!_built)
        {
            Build();
            if (!_built) return;
            Configure(camera.Position);
        }
        double t = T;
        _frames++;

        if (_hasPending) AdoptVoxelRenderers();
        Bubble? viewer = BubbleAt(camera.Position);
        if (viewer != _viewer)
        {
            ProbeLog.Write($"bubbles: camera now in {(viewer?.Name ?? "global space")}");
            _viewer = viewer;
        }

        // Planets + adopted boulders: every moving bubble's terrain at its global pose (VoxelClipmap.NotifyMoved, VoxelClipmap.cs:2338).
        InFrame = true;
        try {
        foreach (Bubble b in All)
        {
            if (b.IsIdentity) continue;
            foreach (VoxelRenderComponent r in b.Renders)
            {
                // Local pose in: ClipmapMovePatch converts every NotifyMoved of a bubble planet to its global pose.
                // Use cached initial centred transform for planets so a later client-side shift doesn't offset terrain by 60 km.
                WorldTransform localPose = InitialCentred.TryGetValue(r, out WorldTransform cached) ? cached : r._voxelStorageComponent.ComputeCenteredTransform();
                WorldTransform pose = localPose;
                r._clipmap?.NotifyMoved(ref pose);
                pose = localPose;
                r._lowResClipmap?.NotifyMoved(ref pose);
                RenderDiag.ClipmapMoves++;
            }
        }
        } finally { InFrame = false; }
        RenderDiag.EndFrame();

        if (_frames == 10 || _frames % 600 == 0)
        {
            ProbeLog.Write("renderdiag: " + RenderDiag.Take() + $" engineClipmapMoves={ClipmapMovePatch.TakeForeign()} clipmapCameraRewrites={System.Threading.Interlocked.Exchange(ref ClipmapCameraPatch.Rewritten, 0)} clipmapMovesSuppressed={System.Threading.Interlocked.Exchange(ref ClipmapCameraPatch.Suppressed, 0)}");
            var info = new System.Text.StringBuilder();
            foreach (Bubble b in All)
            {
                if (b.IsIdentity) continue;
                Vector3D c = ToGlobal(b, new WorldTransform(b.C0), t).Position;
                info.Append($" [{b.Name} centre {c.X:0},{c.Y:0},{c.Z:0} spin {b.SpinRate * t * 180 / Math.PI:0.00}deg]");
            }
            WorldTransform cg = ToGlobal(viewer, camera, t);
            ProbeLog.Write($"bubbles: env clouds={PlanetEnvRender.CloudConstantsStamped} wind={PlanetEnvRender.WindRotated} grass={PlanetEnvRender.GrassLocalised}");
            ProbeLog.Write($"bubbles: t={t:0.0}s camera in {(viewer?.Name ?? "global")} local {camera.Position.X:0},{camera.Position.Y:0},{camera.Position.Z:0} " +
                           $"-> global {cg.Position.X:0},{cg.Position.Y:0},{cg.Position.Z:0}{info}");
            if (viewer != null && viewer.Renders.Count > 0)
            {
                var pr = viewer.Renders[0];
                var cm = pr._clipmap;
                if (cm != null)
                {
                    WorldTransform centred = InitialCentred.TryGetValue(pr, out WorldTransform initC) ? initC : pr._voxelStorageComponent.ComputeCenteredTransform();
                    WorldTransform liveCentred = pr._voxelStorageComponent.ComputeCenteredTransform();
                    WorldTransform wtNow = pr._voxelStorageComponent.Data.GetWorldTransform();
                    WorldTransform baseG = ToGlobal(viewer, centred, t);
                    Vector3I camStorage = pr._voxelStorageComponent.WorldToStoragePosition(camera.Position);
                    int rootCount = 0;
                    double bestDist = double.MaxValue;
                    Vector3I bestKey = default;
                    Vector3D bestPos = default;
                    var roots = cm.RootEntityProvider._rootEntities;
                    lock (roots)
                    {
                        rootCount = roots.Count;
                        foreach (var pair in roots)
                        {
                            WorldTransform wt = baseG * new WorldTransform(pair.Key * 1000);
                            double d = (wt.Position - cg.Position).Length();
                            if (d < bestDist) { bestDist = d; bestKey = pair.Key; bestPos = wt.Position; }
                        }
                    }
                    ProbeLog.Write($"planetdiag: type={pr._voxelStorageComponent.GetType().Name} wtNow={wtNow.Position.X:0},{wtNow.Position.Y:0},{wtNow.Position.Z:0} " +
                                   $"parts={cm._parts.Count} roots={rootCount} lastPos={cm._lastPosition} camStorage={camStorage} " +
                                   $"usedCentred={centred.Position.X:0},{centred.Position.Y:0},{centred.Position.Z:0} liveCentred={liveCentred.Position.X:0},{liveCentred.Position.Y:0},{liveCentred.Position.Z:0} " +
                                   $"nearestRoot key={bestKey} pos={bestPos.X:0},{bestPos.Y:0},{bestPos.Z:0} distToGlobalCam={bestDist:0.0}m visible={cm.Visible}");
                }
            }
        }
    }

    internal static bool Ready => _built && Clock.IsRunning;
    [ThreadStatic] internal static bool InFrame;
}

/// <summary>
/// Owns every terrain pose. Source: VRage.Voxels.Client\...\Clipmap\VoxelClipmap.cs:2338 NotifyMoved(ref WorldTransform) ->
/// RootEntityProvider.UpdateTransform + Move event. Callers pass the planet's LOCAL centred transform
/// (VoxelStorageComponentBase.ComputeCenteredTransform): the engine's own VoxelRenderComponent.UpdatePosition
/// (Components\VoxelRenderComponent.cs:267-277, [OnChanged(WorldTransform)]) and Bubbles.Frame. Run 24 showed the terrain
/// alternating between its global pose and its local pose ~60 km away: the engine re-sent the local pose whenever the
/// client planet's WorldTransform was touched. Converting here makes every caller agree.
/// </summary>
[HarmonyPatch(typeof(Keen.VRage.Voxels.Client.Clipmap.VoxelClipmap), nameof(Keen.VRage.Voxels.Client.Clipmap.VoxelClipmap.NotifyMoved))]
internal static class ClipmapMovePatch
{
    private static long _foreign;
    internal static long TakeForeign() { long n = _foreign; _foreign = 0; return n; }

    [HarmonyPrefix]
    private static void Prefix(Keen.VRage.Voxels.Client.Clipmap.VoxelClipmap __instance, ref WorldTransform transform)
    {
        if (!Bubbles.Ready || !Bubbles.ClipmapBubble.TryGetValue(__instance, out Bubbles.Bubble? b) || b.IsIdentity) return;
        if (!Bubbles.InFrame)
        {
            _foreign++;
            if (Bubbles.ClipmapRender.TryGetValue(__instance, out var r) && Bubbles.InitialCentred.TryGetValue(r, out WorldTransform initC))
                transform = initC;
        }
        transform = Bubbles.ToGlobal(b, transform, Bubbles.T);
    }
}

/// <summary>
/// The clipmap's LOD camera, expressed against the clipmap's CURRENT pose.
/// <para>Source: VRage.Voxels.Client\...\Clipmap\VoxelClipmap.cs:1505-1572 UpdateIfMoved(in WorldTransform view, ...) converts the
/// camera to storage space with <c>WorldTransform.TransformInv(view.Position, this._localToWorld)</c> (:1528) and derives the ring
/// boxes from it (CanSkipUpdate :1575-1598, VoxelClipmapRing.Update). _localToWorld is set by NotifyMoved through the Move event
/// (:2338, UpdateWorldTransform :303-306). The view comes from RenderSettings.CameraTransform
/// (VoxelRenderUpdateSessionComponent.cs:148), set by GlobalRender.CameraPatch at a different point in the frame than the
/// terrain pose, so under rotation the derived storage-space camera jumped by up to one frame of rotation (~50 m at 3 deg/s).
/// Run 28 measured the result: 70-170 terrain cells created/deleted and 2,000-6,000 ring removals per second with the player
/// standing still (vanilla, run 26: zero), the client at ~6 sim ticks/s, cells fading out and in.</para>
/// <para>For a clipmap in the camera's own bubble, the camera's position relative to the planet is known exactly
/// (GlobalRender.LocalCamera against the planet's local centred transform). Rebuild the view from the clipmap's current
/// _localToWorld so TransformInv returns exactly that, whatever the frame ordering.</para>
/// </summary>
// Run 29: patching VoxelClipmap.UpdateIfMoved itself fails (MonoMod: "Unexpected null" regenerating the method body), so the
// camera is rewritten one level up, in the only caller that hands it in: VRage.Voxels.Client\...\Components// VoxelRenderUpdateSessionComponent.cs:204-217 TryUpdate(VoxelClipmap clipmap, bool moveUpdate, in WorldTransform camera, bool
// loadingPhase) -> clipmap.Update(in camera, ...) -> UpdateIfMoved(in view, ...).
[HarmonyPatch(typeof(Keen.VRage.Voxels.Client.Components.VoxelRenderUpdateSessionComponent), "TryUpdate")]
internal static class ClipmapCameraPatch
{
    internal static long Rewritten;
    internal static long Suppressed;
    private static readonly Dictionary<object, Vector3D> LastLocalPos = new(ReferenceEqualityComparer.Instance);

    internal static void Remove(object clipmap)
    {
        lock (LastLocalPos) LastLocalPos.Remove(clipmap);
    }

    [HarmonyPrefix]
    private static bool Prefix(Keen.VRage.Voxels.Client.Clipmap.VoxelClipmap clipmap, ref bool moveUpdate, ref WorldTransform camera, ref bool __result)
    {
        // Hidden clipmaps (e.g. suppressed deep-space _ClientComposition placeholders) must not run VoxelClipmap.Update
        // every frame — otherwise they churn 10,000+ ringRemove/sec and starve the voxel workers and in-process server.
        if (!clipmap.Visible)
        {
            __result = false;
            return false;
        }
        if (!Bubbles.Ready || !GlobalRender.HaveCamera) return true;
        if (!Bubbles.ClipmapBubble.TryGetValue(clipmap, out Bubbles.Bubble? b) || b.IsIdentity)
        {
            // Global-space moving asteroids: throttle moveUpdate for distant clipmaps (>1.5 km) so 100+ orbiting rocks
            // moving at 500-800 m/s don't trigger 2,000+ ringRemove/sec LOD rebuilds every frame in spectator.
            if (moveUpdate && clipmap.LastCameraPosition != null)
            {
                Vector3D rel = camera.Position - clipmap._localToWorld.Position;
                double distSq = rel.LengthSquared();
                if (distSq > 1500.0 * 1500.0)
                {
                    double step = Math.Max(64.0, Math.Sqrt(distSq) * 0.10);
                    bool relMoved;
                    lock (LastLocalPos)
                    {
                        if (!LastLocalPos.TryGetValue(clipmap, out Vector3D prevRel) ||
                            Vector3D.DistanceSquared(in rel, in prevRel) >= step * step)
                        {
                            LastLocalPos[clipmap] = rel;
                            relMoved = true;
                        }
                        else relMoved = false;
                    }
                    if (!relMoved)
                    {
                        moveUpdate = false;
                        clipmap.LastCameraPosition = camera;
                        Suppressed++;
                    }
                }
            }
            return true;
        }
        if (!Bubbles.ClipmapRender.TryGetValue(clipmap, out var render)) return true;
        WorldTransform localCamera = GlobalRender.LocalCamera;
        if (Bubbles.BubbleAt(localCamera.Position) != b) return true;          // camera not in this planet's grid

        // Rebuild view in double precision against the clipmap's current _localToWorld (already updated by BubbleClipmapMoveJob).
        WorldTransform centredLocal = Bubbles.InitialCentred.TryGetValue(render, out WorldTransform initC) ? initC : render._voxelStorageComponent.ComputeCenteredTransform();
        WorldTransform cameraInPlanet = WorldTransform.GetRelativeTransform(localCamera, centredLocal);
        camera = clipmap._localToWorld * cameraInPlanet;
        Rewritten++;

        // CheckNeedsMove in VoxelRenderUpdateSessionComponent compares GLOBAL camera positions (~52 m/frame at 3 deg/s),
        // so it passes moveUpdate=true every frame. At LOD < CellBits, VoxelClipmap.CanSkipUpdate() always returns false
        // (VoxelClipmap.cs:1581-1585), forcing a full BeginBatch/EndBatch every frame even when standing still.
        // Suppress moveUpdate unless the LOCAL camera actually moved past this clipmap's LOD threshold, and keep
        // LastCameraPosition synced to the current global camera so NeedMoveUpdate() also stays false.
        if (moveUpdate && clipmap.LastCameraPosition != null)
        {
            double thresholdSq = Keen.VRage.Voxels.Client.Clipmap.VoxelClipmap.LodDistanceSquared((int)clipmap.GetCurrentLodIndex());
            bool localMoved;
            lock (LastLocalPos)
            {
                if (!LastLocalPos.TryGetValue(clipmap, out Vector3D prev) ||
                    Vector3D.DistanceSquared(in localCamera.Position, in prev) >= thresholdSq)
                {
                    LastLocalPos[clipmap] = localCamera.Position;
                    localMoved = true;
                }
                else
                {
                    localMoved = false;
                }
            }
            if (!localMoved)
            {
                moveUpdate = false;
                clipmap.LastCameraPosition = camera;
                Suppressed++;
            }
        }
        else if (moveUpdate)
        {
            lock (LastLocalPos) LastLocalPos[clipmap] = localCamera.Position;
        }
        return true;
    }
}
