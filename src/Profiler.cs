using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Threading;
using HarmonyLib;

namespace SpinProbe;

/// <summary>
/// Server tick timing plus per-job timing for a list of suspect engine jobs, to find what makes a spinning planet
/// expensive (run 2 ran at roughly 3 server ticks per second).
/// <para>Tick: Keen.Game2.InProcessServerWorldTickComponent.TickServerInternal (SpaceEngineers2.dll; the server thread's
/// per-tick entry, seen in the run-1 stack: InProcessServerThread.TickUntilDisposed -> TickServerInternal ->
/// WorldTickComponent.Tick -> Session.Update -> Scene.Tick).</para>
/// <para>Jobs: every DCS job is entered through its generated <c>&lt;Job&gt;_InvocationStub(byte**, int, Scene, object)</c>,
/// called by Scene.RunJob through a function pointer (VRage.DCS\...\Scenes\Scene.cs:2445). Stubs are not inlined, so they
/// patch cleanly (Scene.RunJob itself could not be patched in run 3: MonoMod failed on one of its locals).</para>
/// </summary>
internal static class Profiler
{
    private const string ServerThread = "In-Process Server Thread";

    private static readonly (string Anchor, string Stub)[] Suspects =
    {
        // Source: VRage.Physics\...\Havok\HavokSessionComponent.cs - StepWorlds (:1851), the driven-body collectors (:1748, :1811), static collector (:1726).
        ("Keen.VRage.Physics.Havok.HavokSessionComponent, VRage.Physics", "StepWorlds_InvocationStub"),
        ("Keen.VRage.Physics.Havok.HavokSessionComponent, VRage.Physics", "CollectChangesForTransform_InvocationStub"),
        ("Keen.VRage.Physics.Havok.HavokSessionComponent, VRage.Physics", "CollectDataForTransformDrivenKinematicBodies_InvocationStub"),
        ("Keen.VRage.Physics.Havok.HavokSessionComponent, VRage.Physics", "CollectDataForStaticBodies_InvocationStub"),
        ("Keen.VRage.Physics.Havok.HavokSessionComponent, VRage.Physics", "UpdateDataFromDynamicBodies_InvocationStub"),
        // Source: VRage.Physics\...\Havok\WorldManager\HavokLargeWorldManager.cs:1187 (moving bodies vs clusters), :1248 (moved statics).
        ("Keen.VRage.Physics.Havok.WorldManager.HavokLargeWorldManager, VRage.Physics", "UpdateMovingBodyClusters_InvocationStub"),
        ("Keen.VRage.Physics.Havok.WorldManager.HavokLargeWorldManager, VRage.Physics", "UpdateMovedStaticBodyClusters_InvocationStub"),
        // Source: VRage.Core.Game\...\SpatialTrigger\SpatialTriggerSystemSessionComponent.cs:536-555 (attached triggers re-test on move).
        ("Keen.VRage.Core.Game.GameSystems.SpatialTrigger.SpatialTriggerSystemSessionComponent, VRage.Core.Game", "AttachedEntityTransformChanged_InvocationStub"),
        // Source: VRage.Water\...\Components\WaterDomainComponent.cs:4868 DomainMoved.
        ("Keen.VRage.Water.Components.WaterDomainComponent, VRage.Water", "DomainMoved_InvocationStub"),
        // Source: VRage.Voxels\...\Components\VoxelPhysicsComponent.cs:227 OnProcessSectors.
        ("Keen.VRage.Voxels.Components.VoxelPhysicsComponent, VRage.Voxels", "OnProcessSectors_InvocationStub"),
        // Source: Game2.Game\...\SessionComponents\ServerSessionComponent.cs:42-82 SyncMotionState (nested type).
        ("Keen.Game2.Game.SessionComponents.ServerSessionComponent, Game2.Game", "SyncMotionState_InvocationStub"),
        // The probe's own job.
        ("SpinProbe.ProbeJobs, ChildlessParents", "DrivePlanets_InvocationStub"),
    };

    private static readonly Dictionary<string, long> Totals = new();
    private static readonly Dictionary<string, int> Calls = new();
    private static readonly Stopwatch Wall = Stopwatch.StartNew();
    private static long _windowStartTimestamp = Stopwatch.GetTimestamp();
    private static long _windowStartTick;
    private static long _tickTotal;
    private static long _tickMax;
    internal static long Ticks;

    internal static void Install(Harmony harmony)
    {
        var tickType = AccessTools.TypeByName("Keen.Game2.InProcessServerWorldTickComponent");
        var tick = tickType == null ? null : AccessTools.Method(tickType, "TickServerInternal");
        if (tick != null)
        {
            harmony.Patch(tick, new HarmonyMethod(typeof(Profiler), nameof(TickPrefix)), new HarmonyMethod(typeof(Profiler), nameof(TickPostfix)));
            ProbeLog.Write("profiler: timing InProcessServerWorldTickComponent.TickServerInternal");
        }
        else ProbeLog.Write("profiler: TickServerInternal not found");

        // Split StepWorlds (HavokSessionComponent.cs:1851): NativeSession->StepCollide, BreakHandler.CollectManifolds,
        // NativeSession->StepSolve. Managed wrappers: VRage.Physics\...\Havok\HkSession.cs:295 StepCollide, :309 StepSolve;
        // BreakHandler\HavokBreakHandler.cs:123 CollectManifolds.
        foreach (var (typeName, method) in new[] {
            ("Keen.VRage.Physics.Havok.HkSession, VRage.Physics", "StepCollide"),
            ("Keen.VRage.Physics.Havok.HkSession, VRage.Physics", "StepSolve"),
            ("Keen.VRage.Physics.Havok.BreakHandler.HavokBreakHandler, VRage.Physics", "CollectManifolds") })
        {
            try
            {
                var m = AccessTools.Method(Type.GetType(typeName), method);
                harmony.Patch(m, new HarmonyMethod(typeof(Profiler), nameof(JobPrefix)), new HarmonyMethod(typeof(Profiler), nameof(JobPostfix)));
                ProbeLog.Write($"profiler: {method} patched");
            }
            catch (Exception e) { ProbeLog.Write($"profiler: {method} failed: {e.Message}"); }
        }

        foreach (var (anchor, stub) in Suspects)
        {
            try
            {
                Type? t = Type.GetType(anchor);
                if (t == null) { ProbeLog.Write($"profiler: type {anchor} not found"); continue; }
                int n = 0;
                foreach (Type candidate in t.Assembly.GetTypes())
                foreach (MethodInfo m in candidate.GetMethods(BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.DeclaredOnly))
                {
                    if (m.Name != stub) continue;
                    harmony.Patch(m, new HarmonyMethod(typeof(Profiler), nameof(JobPrefix)), new HarmonyMethod(typeof(Profiler), nameof(JobPostfix)));
                    n++;
                }
                ProbeLog.Write($"profiler: {stub} patched x{n}");
            }
            catch (Exception e) { ProbeLog.Write($"profiler: {stub} failed: {e.Message}"); }
        }
    }


    private static bool _allInstalled;

    /// <summary>
    /// Audit mode: time EVERY DCS job on the server thread, plus the large-world bookkeeping calls that run inside jobs.
    /// Jobs: every static method named *_InvocationStub (the generated/hand-written job entry points called by
    /// Scene.RunJob, VRage.DCS\...\Scenes\Scene.cs:2445) in the game's simulation assemblies.
    /// Bookkeeping: VRage.Physics\...\Havok\WorldManager\HavokLargeWorldManager.cs - AllocateBody :120, DestroyBody :470/:476,
    /// UpdateStaticEntityCluster :651, UpdateAddedEntityCluster :718, MigrateBodies :986, AddStaticEntityToCluster :1105/:1111,
    /// CreateCluster :1135, RemoveCluster :1155, ReorderClusters :1307; HavokWorldHelper.cs AllocateBody :41, DestroyBody :577,
    /// MigrateBodies :660.
    /// </summary>
    internal static void InstallAll(Harmony harmony)
    {
        if (_allInstalled) return;
        _allInstalled = true;
        var pre = new HarmonyMethod(typeof(Profiler), nameof(JobPrefix));
        var post = new HarmonyMethod(typeof(Profiler), nameof(JobPostfix));
        var sw = Stopwatch.StartNew();
        int n = 0, failed = 0;
        foreach (Assembly a in AppDomain.CurrentDomain.GetAssemblies())
        {
            string an = a.GetName().Name ?? "";
            if (!(an == "VRage.Physics" || an == "VRage.Core.Game" || an == "VRage.Voxels" || an == "VRage.Game" ||
                  an == "VRage.Water" || an == "Game2.Simulation" || an == "Game2.Game" || an == "VRage.AI" ||
                  an == "VRage.Voxels.Client" || an == "VRage.Game.Client" || an == "Game2.Client" || an == "VRage.Render" ||
                  an == "VRage.Render12" || an == "VRage.Core" || an == "SpaceEngineers2" || an == "VRage.Water.Client" ||
                  an == "ChildlessParents")) continue;
            Type[] types;
            try { types = a.GetTypes(); } catch (ReflectionTypeLoadException e) { types = e.Types.Where(t => t != null).ToArray()!; }
            foreach (Type t in types)
            foreach (MethodInfo m in t.GetMethods(BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.DeclaredOnly))
            {
                if (!m.Name.EndsWith("_InvocationStub") || m.GetParameters().Length != 4) continue;
                try { harmony.Patch(m, pre, post); n++; } catch { failed++; }
            }
        }
        ProbeLog.Write($"profiler: audit mode - {n} job stubs timed ({failed} failed) in {sw.ElapsedMilliseconds} ms");

        var lwm = Type.GetType("Keen.VRage.Physics.Havok.WorldManager.HavokLargeWorldManager, VRage.Physics");
        var helper = Type.GetType("Keen.VRage.Physics.Havok.WorldManager.HavokWorldHelper, VRage.Physics");
        string[] lwmNames = { "AllocateBody", "DestroyBody", "UpdateStaticEntityCluster", "UpdateAddedEntityCluster", "MigrateBodies",
                              "AddStaticEntityToCluster", "CreateCluster", "RemoveCluster", "ReorderClusters" };
        string[] helperNames = { "AllocateBody", "DestroyBody", "MigrateBodies" };
        foreach (var (type, names) in new[] { (lwm, lwmNames), (helper, helperNames) })
        {
            if (type == null) continue;
            foreach (MethodInfo m in type.GetMethods(BindingFlags.Instance | BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.DeclaredOnly))
            {
                if (Array.IndexOf(names, m.Name) < 0) continue;
                try { harmony.Patch(m, pre, post); ProbeLog.Write($"profiler: bookkeeping {type.Name}.{m.Name}({m.GetParameters().Length})"); }
                catch (Exception e) { ProbeLog.Write($"profiler: bookkeeping {type.Name}.{m.Name} failed: {e.Message}"); }
            }
        }
    }

    private static void TickPrefix(out long __state) => __state = Stopwatch.GetTimestamp();

    private static void TickPostfix(long __state)
    {
        long dt = Stopwatch.GetTimestamp() - __state;
        _tickTotal += dt;
        if (dt > _tickMax) _tickMax = dt;
        Ticks++;
        if (Frame.Active && Ticks % 120 == 0)
            ProbeLog.Write($"FRAME t={Ticks / 60.0:0.0}s theta={Frame.ThetaEcs * 180 / Math.PI:0.0000}deg refreshed/120ticks={FrameJobs.TakeRefreshed()} " +
                           $"creations={FrameCreationPatch.Converted} rows: {FrameStubs.Stats()}");
        Frame.EndTick();
        if (Ticks % 120 == 0) Report();
        if (Ticks >= (long)(Settings.Seconds * 60) && !AutoRun.Done)
        {
            ProbeLog.Write($"probe: {Settings.Seconds} s of server simulation done");
            Report();
            AutoRun.Done = true;
        }
    }

    /// <summary>Main (client + engine) thread id, captured from GameAppComponent.GameUpdate (AutoRun). With audit_client=1 jobs
    /// on that thread are timed too, keyed "C:" - the listen-host client frame, which run 28-30 showed is the bottleneck
    /// (server ticks ~5 ms, but only ~7 ticks/s wall).</summary>
    internal static int MainThreadId = -1;

    private static void JobPrefix(out long __state)
    {
        var t = Thread.CurrentThread;
        __state = (t.Name == ServerThread || (Settings.AuditClient && t.ManagedThreadId == MainThreadId)) ? Stopwatch.GetTimestamp() : 0;
    }

    private static void JobPostfix(MethodBase __originalMethod, long __state)
    {
        if (__state == 0) return;
        long dt = Stopwatch.GetTimestamp() - __state;
        string name = (Thread.CurrentThread.Name == ServerThread ? "" : "C:") + __originalMethod.DeclaringType?.Name + "." + __originalMethod.Name.Replace("_InvocationStub", "");
        lock (Totals)
        {
            Totals[name] = Totals.TryGetValue(name, out long t) ? t + dt : dt;
            Calls[name] = Calls.TryGetValue(name, out int c) ? c + 1 : 1;
        }
    }

    private static double Ms(long stopwatchTicks) => stopwatchTicks * 1000.0 / Stopwatch.Frequency;

    private static int _benchWindows;

    /// <summary>
    /// Stage 1.4 benchmark line (every 5th report = 10 s): physics and replication cost against the number of live server
    /// movable asteroids, so the 50-100 asteroid run gives measured µs/asteroid and MB/asteroid instead of estimates.
    /// Called inside Report's lock, before the window's Totals are cleared.
    /// </summary>
    // Source: none (plugin-only). Keys are the Profiler's own (HkSession.StepCollide/StepSolve split of
    // HavokSessionComponent.StepWorlds, ServerSessionComponent SyncMotionState, SpatialTriggerSystemSessionComponent
    // AttachedEntityTransformChanged).
    private static void AsteroidBench(long ticks)
    {
        if (!MovableAsteroids.On || ++_benchWindows % 5 != 0) return;
        double Sum(string contains)
        {
            long t = 0;
            foreach (var p in Totals) if (p.Key.Contains(contains) && !p.Key.StartsWith("C:")) t += p.Value;
            return Ms(t) / ticks;
        }
        int n, welds = 0, touched = 0, promoted = 0, shards = 0;
        lock (MovableAsteroids.LiveServer)
        {
            n = MovableAsteroids.LiveServer.Count;
            foreach (var (_, s) in MovableAsteroids.LiveServer)
            {
                welds += s.Welds.Count;
                if (s.PlayerTouched) touched++;
                if (s.Promoted) promoted++;
                if (s.IsShard) shards++;
            }
        }
        double step = Sum("StepWorlds"), collide = Sum("StepCollide"), solve = Sum("StepSolve");
        double sync = Sum("SyncMotionState"), trig = Sum("AttachedEntityTransformChanged");
        double perAst = n > 0 ? (step + sync + trig) * 1000.0 / n : 0;
        var proc = Process.GetCurrentProcess();
        ProbeLog.Write($"asteroid bench: {n} server movable ({welds} welded chunk bodies, {touched} player-touched, {promoted} saved, {shards} shards); " +
                       $"StepWorlds {step:0.000} (collide {collide:0.000}, solve {solve:0.000}) + SyncMotionState {sync:0.000} + triggers {trig:0.000} ms/tick " +
                       $"= {perAst:0.0} µs/tick per asteroid; managed {GC.GetTotalMemory(false) / 1048576.0:0} MB, private {proc.PrivateMemorySize64 / 1048576.0:0} MB, " +
                       $"working set {proc.WorkingSet64 / 1048576.0:0} MB; promotions {MovableAsteroids.Promotions} (background {MovableAsteroids.BackgroundPromoted}), " +
                       $"shatter queued {AsteroidShatter.Queued} / split {AsteroidShatter.Shatters} / spall {AsteroidShatter.Spalls} / shards {AsteroidShatter.ShardsSpawned} / failed {AsteroidShatter.Failed}");
    }

    internal static void Report()
    {
        lock (Totals)
        {
            long now = Stopwatch.GetTimestamp();
            long ticks = Math.Max(Ticks - _windowStartTick, 1);
            double wall = Ms(now - _windowStartTimestamp);
            ProbeLog.Write($"PROFILE t={Ticks / 60.0:0.0}s: {ticks} server ticks in {wall:0} ms wall = {ticks * 1000.0 / Math.Max(wall, 1):0.0} ticks/s; " +
                           $"server tick avg {Ms(_tickTotal) / ticks:0.00} ms, max {Ms(_tickMax):0.0} ms");
            foreach (var pair in Totals.OrderByDescending(p => p.Value).Take(30))
                ProbeLog.Write($"PROFILE   {Ms(pair.Value) / ticks,8:0.000} ms/tick  x{Calls[pair.Key] / (double)ticks:0.0}  {pair.Key}");
            try { AsteroidBench(ticks); } catch { }
            if (Settings.StaticCensus) try { StaticCensus.MaybeReport(Ticks); } catch { }
            Totals.Clear();
            Calls.Clear();
            _windowStartTimestamp = now;
            _windowStartTick = Ticks;
            _tickTotal = 0;
            _tickMax = 0;
        }
    }
}
