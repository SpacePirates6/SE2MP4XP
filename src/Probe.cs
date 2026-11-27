using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using HarmonyLib;
using Keen.Game2;
using Keen.Game2.Simulation.Replication;
using Keen.Game2.Simulation.RuntimeSystems;
using Keen.VRage.Core.EngineComponents;
using Keen.VRage.Core.Game.Systems;
using Keen.VRage.Core.Plugins;
using Keen.VRage.Library.Filesystem.StorageManagers;

namespace SpinProbe;

// SpinProbe: a throwaway test plugin for two questions (docs/ENGINE-ARCHITECTURE.md Part D, D6):
//   T1 "OEM shape": can a plugin add its own DCS jobs through PluginHost.OnBeforeEngineInstantiated plus an
//      ISessionConfigurator engine component, the way Keen's water system does?
//   E1: do voxel chunk bodies created Kinematic + TransformDriven and driven by WorldTransform writes get real
//      velocity (DriveBodyTo), so that a character standing on them is carried?
// Every engine reference cites its source in the decompile at D:\SE2MPNEW\Se2Decomp (build Stable_VS4.1, 2026-11-10).

public sealed class ProbePlugin : IPlugin, IDisposable
{
    private readonly Harmony _harmony = new("spacepirates6.spinprobe");
    internal static Harmony HarmonyInstance = null!;

    // Source: VRage.Core\Keen\VRage\Core\Plugins\PluginHost.cs:115-131 - PluginHost.Add calls
    // Activator.CreateInstance(type, host) for the first IPlugin type; an exception here takes the process down,
    // so the whole body is guarded.
    public ProbePlugin(PluginHost host)
    {
        try
        {
            HarmonyInstance = _harmony;
            ProbeLog.Write($"SpinProbe loaded; mode={Settings.Mode} vanilla={Settings.Vanilla} dps={Settings.DegreesPerSecond} save='{Settings.SaveName}' seconds={Settings.Seconds}");

            // Source: PluginHost.cs:30 `public event Action<EngineBuilder> OnBeforeEngineInstantiated;`, raised from
            // SpaceEngineers2\Keen\Game2\GameApp.cs:320. EngineBuilder.Add<T>(ComponentBuildInfo) (EngineBuilder.cs:93)
            // adds an engine component, which is how WaterSessionServerConfiguratorComponent gets into the engine.
            host.OnBeforeEngineInstantiated += builder =>
            {
                try
                {
                    builder.Add<ProbeConfigurator>();
                    ProbeLog.Write("T1: ProbeConfigurator added to the engine builder");
                }
                catch (Exception e) { ProbeLog.Write("T1: adding configurator failed: " + e); }
            };

            if (!Settings.Vanilla && Settings.Mode != "bubbles")
                Patch(typeof(ChunkPatch));
            if (Settings.Mode == "bubbles")
            {
                // Bubbles: the server stays vanilla (every bubble is simulated in its own local grid); only the client
                // view is changed. See Bubbles.cs.
                Patch(typeof(BubbleChunkSyncPatch));
                Patch(typeof(Bubbles.RenderAddedPatch));
                Patch(typeof(Bubbles.RenderRemovedPatch));
                Patch(typeof(GlobalRender.CameraPatch));
                Patch(typeof(SpectatorHandoff.UpdatePatch));
                Patch(typeof(HudMarkers.DrawScopePatch));
                Patch(typeof(HudMarkers.AnchorPatch));
                Patch(typeof(HudMarkers.ProjectPatch));
                Patch(typeof(HudMarkers.DistancePatch));
                Patch(typeof(MapRender.UpdatePatch));
                Patch(typeof(MapRender.HidePatch));
                Patch(typeof(MapRender.MarkerPatch));
                Patch(typeof(MapMarkers.BiasPatch));
                Patch(typeof(MapMarkers.MapToWorldPatch));
                Patch(typeof(RootChoicePatch));
                Patch(typeof(ClipmapMovePatch));
                Patch(typeof(ClipmapCameraPatch));
                Patch(typeof(RootUpdateLockPatch));
                Patch(typeof(VoxelCellSpherizePatch));
                try { ShadowCascadePatch.Install(_harmony); } catch (Exception e) { ProbeLog.Write("shadows: cascade reprojection install failed: " + e); }
                // Clouds turn with the planet, grass / tree wind in the planet's frame (PlanetEnvRender.cs).
                Patch(typeof(CloudShaderPatch));
                Patch(typeof(CloudShellPlanetPatch));
                Patch(typeof(CloudConstantsPatch));
                Patch(typeof(WindConvertPatch));
                Patch(typeof(GrassLocalPatch));
                if (Settings.ParticleFix)
                {
                    Patch(typeof(ParticleCameraDeltaPatch));
                    Patch(typeof(ParticleGravityPatch));
                }
                else ProbeLog.Write("particles: planet-frame particle patches OFF (particle_fix=0)");
                if (Settings.GlobalTrigger)
                {
                    Patch(typeof(BubbleObservers.ClientTriggerAddPatch));
                    Patch(typeof(BubbleObservers.ClientTriggerRemovePatch));
                    Patch(typeof(BubbleObservers.ClientTriggerUpdatePatch));
                }
                if (Settings.SceneSubset)
                {
                    Patch(typeof(BubbleObservers.EnforcePatch));
                    Patch(typeof(BubbleObservers.ExitPatch));
                    Patch(typeof(BubbleObservers.ReconcilePatch));
                    Patch(typeof(BubbleObservers.MaintainPatch));
                    Patch(typeof(SubsetFlushGuard.FlushPatch));
                    Patch(typeof(SubsetFlushGuard.CommitPatch));
                    Patch(typeof(SubsetChildTags.CountPatch));
                    Patch(typeof(SubsetChildTags.InitialPatch));
                    Patch(typeof(SubsetChildTags.ChildAddedPatch));
                    Patch(typeof(SubsetChildTags.ReparentPatch));
                    Patch(typeof(SubsetEntityCloneSkipFix));
                }
                Patch(typeof(RenderClock.GameSide));
                Patch(typeof(RenderClock.RenderSide));
                Patch(typeof(RenderClock.DrawBegin));
                try { RenderDiag.Install(_harmony); } catch (Exception e) { ProbeLog.Write("renderdiag install failed: " + e.Message); }
            }
            if (!Settings.Vanilla && Settings.Mode == "frame")
            {
                try { FrameStubs.Install(_harmony); } catch (Exception e) { ProbeLog.Write("frame stubs install failed: " + e); }
                Patch(typeof(FrameCreationPatch));
                Patch(typeof(FrameStepPatch));
            }
            Patch(typeof(AutoRun));
            if (Settings.MovableAsteroidMax > 0)
            {
                Patch(typeof(MovableAsteroids.InitPatch));
                Patch(typeof(MovableAsteroids.RemovePatch));
                Patch(typeof(MovableAsteroids.CreateChunkPatch));
                Patch(typeof(MovableAsteroids.DestroyChunkPatch));
                Patch(typeof(MovableAsteroids.VoxelTriggerFilterPatch));
                Patch(typeof(MovableAsteroids.ChunkPredictFilterPatch));
                Patch(typeof(MovableAsteroidGrids.PlacePatch));
                Patch(typeof(MovableAsteroidPlacement.VoxelMergePatch));
                Patch(typeof(MovableAsteroidPlacement.AlignSpeedPatch));
                Patch(typeof(MovableAsteroidGrids.GridAddedPatch));
                if (Settings.AsteroidImpacts)
                {
                    Patch(typeof(VoxelDestructionRefreshPatch));
                    ProbeLog.Write("asteroid: density-based voxel impacts ON");
                }
                Patch(typeof(MovableAsteroids.AutoPromoteGatePatch));
                if (Settings.AsteroidShatter || Settings.OrbitalBelts) Patch(typeof(AsteroidShatter.SkipSaveStoragePatch));
                if (Settings.RealPhysics) Patch(typeof(AsteroidRails.SavePatch));
                Patch(typeof(AsteroidClipboard.PreviewVoxelAddSkipPatch));
                Patch(typeof(AsteroidClipboard.PreviewVoxelRemoveSkipPatch));
                Patch(typeof(AsteroidClipboard.HotKeyCopyPatch));
                Patch(typeof(AsteroidClipboard.HotKeyCutPatch));
                Patch(typeof(AsteroidClipboard.HotKeyPastePatch));
                // Client playback of replicated motion interpolates only when this is on; it is a plain bool that only the
                // debug menu sets (VRage.Multiplayer MotionInterpolationSettings.Interpolate, default false), so moving
                // asteroids (and every other replicated body) otherwise snap to each new sample.
                Keen.VRage.Multiplayer.Replications.MotionInterpolationSettings.Instance.Interpolate = true;
                ProbeLog.Write("asteroid: client motion interpolation ON");
            }
            if (Settings.ForceCreative)
            {
                Keen.Game2.Simulation.GameFeatureConfig.SurvivalSpectator = true;
                Patch(typeof(DebugCreativeToolsPatch));
                Patch(typeof(CreativeProgressionPatch));
                Patch(typeof(CreativeColonizationPatch));
            }
            if (Settings.SkipTutorial)
            {
                Patch(typeof(SkipTutorialMissionPatch));
                Patch(typeof(SkipTutorialClientUiPatch));
                Patch(typeof(SkipTutorialOverlayPatch));
                try { SkipTutorialActions.Install(_harmony); } catch (Exception e) { ProbeLog.Write("tutorial: action guards failed: " + e.Message); }
            }
            if (Settings.RealPhysics)
            {
                ProbeLog.Write("real: realistic physics ON");
                Patch(typeof(RealPhysics.PlanetRegistryPatch));
                Patch(typeof(RealPhysics.LimitsPatch));
                Patch(typeof(RealPhysics.GravityRangePatch));
                Patch(typeof(RealPhysics.GravityInitLog));
                Patch(typeof(RealPhysics.GravityFalloffPatch));
                if (Settings.RealFlightComputer && Settings.RealRcs) Patch(typeof(FlightComputerJobs.KeepRcsFiringPatch));
                if (Settings.RealRails) Patch(typeof(RealRails.SavePatch));
                try { RealExtrapolation.Install(_harmony); } catch (Exception e) { ProbeLog.Write("real: extrapolation install failed: " + e.Message); }
            }

            if (Settings.StaticCensus) Patch(typeof(StaticCensus.Patch));
            if (Orbits.StarOn) Patch(typeof(Orbits.SunPatch));
            if (Settings.ClipDiag)
                try { ClipmapDiag.Install(_harmony); } catch (Exception e) { ProbeLog.Write("clipdiag install failed: " + e.Message); }
            try { Profiler.Install(_harmony); } catch (Exception e) { ProbeLog.Write("profiler install failed: " + e); }
        }
        catch (Exception e)
        {
            ProbeLog.Write("init failed: " + e);
        }
    }

    private void Patch(Type t)
    {
        try { _harmony.CreateClassProcessor(t).Patch(); ProbeLog.Write("patched " + t.Name); }
        catch (Exception e) { ProbeLog.Write($"patch {t.Name} failed: {e}"); }
    }

    public void Dispose()
    {
        try { AsteroidStore.FlushNow(); } catch { }
        try { _harmony.UnpatchAll(_harmony.Id); } catch { }
    }
}

/// <summary>
/// The OEM extension point. Source: Game2.Simulation\...\Water\WaterSessionServerConfiguratorComponent.cs - an
/// <c>EngineComponent, ISessionConfigurator</c> whose <c>ConfigureSession</c> calls
/// <c>sessionBuilder.SceneBuilder.AddJobsFromAssembly(...)</c>. WorldSessionComponent.InitAsync calls ConfigureSession on
/// every ISessionConfigurator in the engine scene (Game2.Game\...\WorldSessionComponent.cs:312-319).
/// </summary>
public sealed class ProbeConfigurator : EngineComponent, ISessionConfigurator
{
    public void ConfigureSession(SessionBuilder sessionBuilder)
    {
        try
        {
            ProbeLog.Write($"T1: ConfigureSession called, IsServer={sessionBuilder.IsServer}");
            // All game assemblies are loaded by the time a session is configured.
            if (Settings.AuditClient)
                try { Profiler.InstallAll(ProbePlugin.HarmonyInstance); } catch (Exception e) { ProbeLog.Write("client audit install failed: " + e.Message); }
            if (!sessionBuilder.IsServer)
            {
                // Client session: global rendering jobs (GlobalRender.cs) run after the engine's render-transform jobs.
                if (Settings.Mode == "bubbles")
                {
                    sessionBuilder.SceneBuilder.AddJobsFromType(typeof(GlobalRenderJobs));
                    ProbeLog.Write("bubbles: GlobalRenderJobs registered in the client session");
                }
                if (Settings.MovableAsteroidMax > 0)
                {
                    sessionBuilder.SceneBuilder.AddJobsFromType(typeof(MovableAsteroidClientJobs));
                    ProbeLog.Write("asteroid: client chunk-follow job registered");
                    if (AsteroidRails.On)
                    {
                        sessionBuilder.SceneBuilder.AddJobsFromType(typeof(AsteroidRailsClientJobs));
                        ProbeLog.Write("arails: client analytic-pose job registered");
                    }
                }
                if (Settings.RealPhysics && Settings.RealFlightComputer && Settings.RealRcs)
                {
                    sessionBuilder.SceneBuilder.AddJobsFromType(typeof(RcsFlameJobs));
                    ProbeLog.Write("real: client RCS flame job registered");
                }
                return;
            }
            // Source: VRage.DCS\...\Builders\SceneBuilder.cs:103 AddJobsFromType(Type, includeNestedTypes, includeJobParams).
            sessionBuilder.SceneBuilder.AddJobsFromType(typeof(ProbeJobs));
            if (Settings.MovableAsteroidMax > 0)
            {
                sessionBuilder.SceneBuilder.AddJobsFromType(typeof(MovableAsteroidJobs));
                sessionBuilder.SceneBuilder.AddJobsFromType(typeof(MovableAsteroidControl));
                sessionBuilder.SceneBuilder.AddJobsFromType(typeof(AsteroidClipboard));
                if (Settings.AsteroidImpacts)
                    sessionBuilder.SceneBuilder.AddJobsFromType(typeof(AsteroidImpactJobs));
                if (Settings.AsteroidImpacts && Settings.AsteroidShatter)
                    sessionBuilder.SceneBuilder.AddJobsFromType(typeof(AsteroidShatterJobs));
                if (AsteroidRails.On)
                {
                    sessionBuilder.SceneBuilder.AddJobsFromType(typeof(AsteroidRailJobs));
                    ProbeLog.Write($"arails: asteroid rails registered (wake {Settings.AsteroidWakeDistance:0} m, belts={Settings.OrbitalBelts}, star={(Orbits.StarOn ? $"{Settings.StarDistance / 1000:0} km / {Settings.StarPeriod:0} s" : "off")})");
                }
            }
            sessionBuilder.SceneBuilder.AddJobsFromType(typeof(FrameJobs));
            if (Settings.RealPhysics)
            {
                sessionBuilder.SceneBuilder.AddJobsFromType(typeof(RealPhysicsJobs));
                if (Settings.RealFlightComputer) sessionBuilder.SceneBuilder.AddJobsFromType(typeof(FlightComputerJobs));
                if (Settings.RealBubbleHandoff) sessionBuilder.SceneBuilder.AddJobsFromType(typeof(BubbleHandoff));
                if (Settings.RealRails) sessionBuilder.SceneBuilder.AddJobsFromType(typeof(RealRails));
                ProbeLog.Write($"real: physics jobs registered (fc={Settings.RealFlightComputer} rcs={Settings.RealRcs} aeroLift={Settings.RealAeroLift} handoff={Settings.RealBubbleHandoff})");
            }
            if (Settings.SceneSubset && Settings.Mode == "bubbles") sessionBuilder.SceneBuilder.AddJobsFromType(typeof(BubbleObserverJobs));
            if (Settings.Audit)
                try { Profiler.InstallAll(ProbePlugin.HarmonyInstance); } catch (Exception e) { ProbeLog.Write("audit install failed: " + e); }
            ProbeLog.Write("T1: ProbeJobs registered in the server session");
        }
        catch (Exception e)
        {
            ProbeLog.Write("T1: ConfigureSession failed: " + e);
        }
    }
}

/// <summary>
/// Read from spinprobe.ini next to the DLL (key=value), because Steam does not pass environment variables to the game.
/// Keys: dps, save, seconds, vanilla (1 = no rotation, no chunk change: baseline), keep_destruction, keep_threshold.
/// </summary>
internal static class Settings
{
    private static readonly Dictionary<string, string> Ini = LoadIni();
    internal static readonly double DegreesPerSecond = Num("dps", 0.02);
    internal static readonly string SaveName = Str("save", "Welcome To Verdure 1 47");
    internal static readonly double Seconds = Num("seconds", 90);
    internal static readonly bool Vanilla = Str("vanilla", "0") == "1";
    internal static readonly bool KeepDestructionLayer = Str("keep_destruction", "0") == "1";
    internal static readonly bool NoChunkGroup = Str("no_chunk_group", "0") == "1";
    internal static readonly string Mode = Str("mode", "kinematic");
    internal static readonly bool Audit = Str("audit", "0") == "1";
    internal static readonly bool AuditClient = Str("audit_client", "0") == "1";
    internal static readonly bool ClipDiag = Str("clipdiag", "1") == "1";
    internal static readonly double FrameRadiusFactor = Num("frame_radius", 1.25);
    // Movable asteroids (MovableAsteroids.cs): procedural asteroids up to this size in metres get one dynamic body. 0 = off.
    internal static readonly double MovableAsteroidMax = Num("movable_asteroid_max", 0);
    internal static readonly double AsteroidDensity = Num("asteroid_density", 2700);
    internal static readonly double AsteroidDriftSpeed = Num("asteroid_drift_speed", 3.5);
    internal static readonly double AsteroidSpinRate = Num("asteroid_spin_rate", 0.10);
    // Natural procedural asteroids outside planet bubbles spawn with real prograde Keplerian belt orbits instead of tiny random drift.
    internal static readonly bool AsteroidBeltOrbits = Num("asteroid_belt_orbits", 1) != 0;
    internal static readonly bool AsteroidImpacts = Num("asteroid_impacts", 1) != 0;
    // Source: none (plugin-only). Stage 1 pile-up containment (docs/asteroid-orbits-plan.md): asteroids a player never touched
    // may still be promoted to saved ("manual") ones, up to this many per session. 0 = player-caused promotions only.
    internal static readonly int AsteroidBackgroundPromotions = (int)Num("asteroid_background_promotions", 0);
    // Stage 2: real Voronoi shattering replaces fissure trenches when E/M >= Q*. 0 = old crater + fissure behaviour.
    internal static readonly bool AsteroidShatter = Num("asteroid_shatter", 1) != 0;
    // Stage 1.3: count which static bodies have their WorldTransform written every tick (UpdateStaticEntityCluster callers).
    internal static readonly bool StaticCensus = Num("static_census", 1) != 0;
    // Stage 3 (AsteroidRails.cs): coasting movable asteroids on analytic patched-conic orbits (needs real_physics=1).
    internal static readonly bool AsteroidRails = Num("asteroid_rails", 1) != 0;
    internal static readonly double AsteroidWakeDistance = Num("asteroid_wake_distance", 2500);
    internal static readonly double AsteroidRailWriteNearSeconds = Num("asteroid_rail_write_near_s", 0.25);
    // Stage 4.1 (Orbits.cs): a virtual star; the home planet orbits it at star_distance metres every star_period seconds.
    // 0 = no star (default).
    internal static readonly double StarDistance = Num("star_distance", 0);
    internal static readonly double StarPeriod = Num("star_period", 14400);
    // Stage 4.2 (OrbitalBelts.cs): deterministic orbiting asteroid belts (radii in planet radii for the home belt).
    internal static readonly bool OrbitalBelts = Num("orbital_belts", 0) != 0;
    internal static readonly double BeltInner = Num("belt_inner", 1.5);
    internal static readonly double BeltOuter = Num("belt_outer", 4.5);
    internal static readonly int BeltHomeCount = (int)Num("belt_home_count", 300);
    internal static readonly int BeltStarCount = (int)Num("belt_star_count", 500);
    internal static readonly double BeltSpawnRange = Num("belt_spawn_range", 250000);
    internal static readonly double BeltDespawnRange = Num("belt_despawn_range", 280000);
    internal static readonly bool BeltsSuppressVanilla = Num("belts_suppress_vanilla", 1) != 0;
    // Isolation switches for the run 61/65 GPU hangs (DXGI_ERROR_DEVICE_HUNG). Both default on.
    internal static readonly bool ParticleFix = Num("particle_fix", 1) != 0;
    internal static readonly bool ClientKinematicAsteroids = Num("client_kinematic", 1) != 0;
    // Bubble observers (BubbleObservers.cs): client asteroid trigger at the true global position; SceneSubset enforcement.
    internal static readonly bool GlobalTrigger = Num("global_trigger", 1) != 0;
    internal static readonly bool SceneSubset = Num("scene_subset", 1) != 0;
    internal static readonly double SpinDegreesPerSecond = Num("spin_dps", 0.5);
    internal static readonly double OrbitPeriodSeconds = Num("orbit_period", 300);
    internal static readonly float SkyStepDegrees = (float)Num("sky_step", 0);
    internal static readonly bool AutoActivation = Str("activation", "always") == "auto";
    internal static readonly bool RaiseTeleportThreshold = Str("keep_threshold", "0") != "1";
    internal static readonly bool ForceCreative = Num("force_creative", 1) != 0;
    internal static readonly bool SkipTutorial = Num("skip_tutorial", 1) != 0;
    // Realistic physics (RealPhysics.cs, docs/realistic-physics-plan.md). real_physics=1 turns the whole set on; each
    // feature has its own switch so it can be isolated.
    internal static readonly bool RealPhysics = Num("real_physics", 0) != 0;
    internal static readonly double RealSpeedLimit = Num("real_speed_limit", 10000);
    internal static readonly double RealAngularLimit = Num("real_angular_limit", 100);
    internal static readonly double RealAngularDamping = Num("real_angular_damping", 0);
    internal static readonly bool RealGravity = Num("real_gravity", 1) != 0;
    internal static readonly double RealGravityRange = Num("real_gravity_range", 12);   // x surface radius
    internal static readonly bool RealRotatingFrame = Num("real_rotating_frame", 1) != 0;
    internal static readonly bool RealGyroscopic = Num("real_gyroscopic", 1) != 0;
    internal static readonly bool RealGyroFree = Num("real_gyro_free", 1) != 0;
    internal static readonly bool RealThrustTorque = Num("real_thrust_torque", 1) != 0;
    internal static readonly bool RealFlightComputer = Num("real_flight_computer", 1) != 0;
    internal static readonly bool RealRcs = Num("real_rcs", 1) != 0; // note: RCS torque shows no flames and uses no power/fuel (one thrust value per axis in VoluntaryThrustData)
    internal static readonly bool RealDrag = Num("real_drag", 1) != 0;
    internal static readonly double RealDragCd = Num("real_drag_cd", 1.0);
    internal static readonly bool RealAeroLift = Num("real_aero_lift", 1) != 0;
    internal static readonly bool RealAeroTorque = Num("real_aero_torque", 1) != 0;
    internal static readonly bool RealBubbleHandoff = Num("real_bubble_handoff", 1) != 0;
    internal static readonly bool RealRails = Num("real_rails", 1) != 0;
    internal static readonly double RealRailsDistance = Num("real_rails_distance", 3000);
    internal static readonly double RealRailsWriteSeconds = Num("real_rails_write_s", 5);
    internal static readonly double RealMaxExtrapolation = Num("real_max_extrapolation", 0.5);

    private static Dictionary<string, string> LoadIni()
    {
        var d = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        try
        {
            string path = Path.Combine(Path.GetDirectoryName(typeof(Settings).Assembly.Location) ?? ".", "spinprobe.ini");
            if (File.Exists(path))
                foreach (string line in File.ReadAllLines(path))
                {
                    int eq = line.IndexOf('=');
                    if (eq > 0 && !line.TrimStart().StartsWith("#")) d[line[..eq].Trim()] = line[(eq + 1)..].Trim();
                }
        }
        catch { }
        return d;
    }

    private static string Str(string key, string fallback) => Ini.TryGetValue(key, out string? v) && v.Length > 0 ? v : fallback;

    private static double Num(string key, double fallback) =>
        double.TryParse(Str(key, ""), System.Globalization.NumberStyles.Float,
            System.Globalization.CultureInfo.InvariantCulture, out double v) ? v : fallback;
}

/// <summary>
/// Loads the test save once the main menu is up, and exits the game when the probe is done.
/// Source: SpaceEngineers2\Keen\Game2\GameAppComponent.cs:1115 `private void GameUpdate()` (the engine-scene AppUpdate job,
/// called every frame on the main thread); :74 `MainMenu`; :489 `LoadGameAsync(WorldLoadArgs)`; :223 `ExitGame()`.
/// The same load call is what SE2MP's BackgroundServerHost.StartFromSave uses (SE2MP.Net\BackgroundServerHost.cs:524-546).
/// </summary>
[HarmonyPatch(typeof(GameAppComponent), "GameUpdate")]
internal static class AutoRun
{
    private static readonly Stopwatch Clock = Stopwatch.StartNew();
    private static long _menuSince = -1;
    private static bool _loadIssued;
    private static bool _exitIssued;
    internal static volatile bool Done;

    [HarmonyPostfix]
    private static void After(GameAppComponent __instance)
    {
        try
        {
            Profiler.MainThreadId = Environment.CurrentManagedThreadId;
            if (Settings.ClipDiag) ClipmapDiag.Tick();
            AsteroidClipboard.SetInputManager(__instance.Scene.Single<Keen.VRage.Input.EngineComponents.InputEngineComponent>());
            AsteroidClipboard.ClientTick();
            if (!_loadIssued)
            {
                if (__instance.MainMenu == null) { _menuSince = -1; return; }
                if (_menuSince < 0) { _menuSince = Clock.ElapsedMilliseconds; return; }
                if (Clock.ElapsedMilliseconds - _menuSince < 3000) return;
                _loadIssued = true;
                // Source: Game2.Simulation\...\RuntimeSystems\WorldLoadArgs.cs (SaveId, SkipDebugMenuWarning,
                // SkipNewerSaveWarning); VRage.Library\...\StorageManagers\ContainerId.cs:36 ContainerId(string name).
                // "scenario:<SaveName>" starts a fresh world from a shipped scenario instead of a user save.
                // Source: VRage.Core\...\Worlds\DefaultWorldsComponent.cs:120-137 TryFindDefaultWorldContainerId(name) matches the
                // world's SaveName (GameData\Vanilla\Worlds\Survival = "Welcome To Verdure"); same lookup as
                // SE2MP.Net\BackgroundServerHost.cs:590-620 BuildWorldLoadArgs.
                ContainerId saveId;
                if (Settings.SaveName.StartsWith("scenario:", StringComparison.OrdinalIgnoreCase))
                {
                    string key = Settings.SaveName.Substring("scenario:".Length);
                    ContainerId? found = __instance.Scene.Single<Keen.VRage.Core.Worlds.DefaultWorldsComponent>()?.TryFindDefaultWorldContainerId(key);
                    if (found == null) { ProbeLog.Write($"autorun: scenario '{key}' not found"); return; }
                    saveId = found.Value;
                }
                else saveId = new ContainerId(Settings.SaveName);
                var args = new WorldLoadArgs
                {
                    SaveId = saveId,
                    SkipDebugMenuWarning = true,
                    SkipNewerSaveWarning = true
                };
                ProbeLog.Write($"autorun: main menu up, loading save '{Settings.SaveName}'");
                _ = __instance.LoadGameAsync(args);
                return;
            }

            if (Done && !_exitIssued)
            {
                _exitIssued = true;
                ProbeLog.Write("autorun: probe finished, exiting the game");
                __instance.ExitGame();
            }
        }
        catch (Exception e)
        {
            ProbeLog.Write("autorun failed: " + e);
            _loadIssued = true;
        }
    }
}

internal static class ProbeLog
{
    private static readonly object Gate = new();
    internal static readonly string FilePath = Path.Combine(
        Path.GetDirectoryName(typeof(ProbeLog).Assembly.Location) ?? ".", $"spinprobe-{Environment.ProcessId}.log");

    internal static void Write(string message)
    {
        string line = DateTime.Now.ToString("HH:mm:ss.fff") + "  " + message;
        try { lock (Gate) File.AppendAllText(FilePath, line + Environment.NewLine); } catch { }
    }
}

/// <summary>Marks the point in the tick after which Havok results are expressed at ThetaEcs + Delta.
/// Source: VRage.Physics\...\Havok\HavokSessionComponent.cs:1851 StepWorlds (StepCollide / StepSolve), once per server tick.</summary>
[HarmonyPatch(typeof(Keen.VRage.Physics.Havok.HavokSessionComponent), "StepWorlds")]
internal static class FrameStepPatch
{
    [HarmonyPostfix]
    private static void Postfix() => Frame.SteppedThisTick = true;
}
