using System;
using System.Threading;
using HarmonyLib;

namespace SpinProbe;

/// <summary>
/// Diagnostics for render flicker under global rendering (run 19: ground sometimes missing, trees blinking, a grid shown
/// falling through the planet). Hypothesis: the engine's own render-transform job sometimes runs after ours in a frame,
/// so its LOCAL pose wins for entities that changed that frame.
/// <para>Patched (timing/order only, no behaviour change):
///  VRage.Game.Client\...\Render\RootEntityRenderComponent.cs:113 UpdateTransform_InvocationStub and :151
///  UpdateObservedTransform_InvocationStub (the engine pushing a changed entity's pose to its RootEntity);
///  our GlobalRenderJobs stubs (generated container) PushRoot / PushObservedRoot.</para>
/// </summary>
internal static class RenderDiag
{
    private static long _seq;
    private static long _engineLast, _oursLast, _oursFirst;
    internal static long EngineAfterOursFrames, Frames, EngineCalls, OursCalls, EngineRows;
    internal static long RootsPushed, ClipmapMoves;
    internal static int RenderThreadId = -1;

    internal static void Install(Harmony h)
    {
        var engineType = typeof(Keen.VRage.Game.Client.Render.RootEntityRenderComponent);
        foreach (string n in new[] { "UpdateTransform_InvocationStub", "UpdateObservedTransform_InvocationStub" })
        {
            var m = AccessTools.Method(engineType, n);
            if (m != null) h.Patch(m, postfix: new HarmonyMethod(typeof(RenderDiag), nameof(EnginePost)));
            ProbeLog.Write($"renderdiag: engine {n} {(m != null ? "hooked" : "NOT FOUND")}");
        }
        foreach (Type t in typeof(RenderDiag).Assembly.GetTypes())
        foreach (var m in t.GetMethods(System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.DeclaredOnly))
            if (m.Name == "PushRoot_InvocationStub" || m.Name == "PushObservedRoot_InvocationStub")
            {
                h.Patch(m, prefix: new HarmonyMethod(typeof(RenderDiag), nameof(OursPre)));
                ProbeLog.Write($"renderdiag: ours {t.Name}.{m.Name} hooked");
            }
    }

    private static void EnginePost(int __1)
    {
        _engineLast = Interlocked.Increment(ref _seq);
        EngineCalls++;
        EngineRows += __1;
    }

    private static void OursPre()
    {
        long s = Interlocked.Increment(ref _seq);
        if (_oursFirst == 0) _oursFirst = s;
        _oursLast = s;
        OursCalls++;
    }

    /// <summary>Called once per client frame from Bubbles.Frame.</summary>
    internal static void EndFrame()
    {
        Frames++;
        if (_engineLast > _oursLast && _oursLast != 0) EngineAfterOursFrames++;
        _oursFirst = 0;
    }

    internal static string Take()
    {
        string s = $"frames={Frames} engineAfterOurs={EngineAfterOursFrames} engineStubCalls={EngineCalls} engineRows={EngineRows} " +
                   $"ourStubCalls={OursCalls} rootsPushed={RootsPushed} clipmapMoves={ClipmapMoves} renderThread={RenderThreadId} frameThread={Environment.CurrentManagedThreadId}";
        Frames = EngineAfterOursFrames = EngineCalls = OursCalls = EngineRows = RootsPushed = ClipmapMoves = 0;
        return s;
    }
}
