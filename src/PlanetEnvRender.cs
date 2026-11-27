using System;
using System.IO;
using System.Runtime.CompilerServices;
using System.Text;
using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.Core.Render;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Library.Utils;
using Keen.VRage.Render.Data;
using Keen.VRage.Render12.Core;
using Keen.VRage.Render12.LightingStage;
using Keen.VRage.Render12.Resources.Shaders;
using Keen.VRage.Render12.SceneSystem.Components;
using Keen.VRage.Render12.SceneSystem.GPU;

namespace SpinProbe;

/// <summary>
/// Planet environment effects under global rendering: clouds, grass wind and tree wind. The engine builds these in
/// "world" space with no planet orientation; the game feeds them planet-local data (the client ECS is bubble-local),
/// while the planet itself is drawn at its rotating global pose. Without these patches the cloud shell stays fixed in
/// global space while the ground turns under it (3.2 km/s at 3 deg/s), and the grass / tree wind direction - a
/// planet-local vector - is applied as a world vector, so once the planet has turned it points into or out of the
/// ground and the grass bend normalize(up + windDir * w) (Scene\GrassRenderingVertex.hlsl:340) flips.
/// </summary>
internal static class PlanetEnvRender
{
    internal static int CloudConstantsStamped, WindRotated, GrassLocalised;

    /// <summary>Spinning bubble whose clouds / wind are being drawn, and its current angle (radians, about SpinAxis).</summary>
    internal static bool SpinOf(Bubbles.Bubble? b, out double theta)
    {
        theta = 0;
        if (b == null || b.SpinRate == 0) return false;
        const double twoPi = 2.0 * Math.PI;
        // Wrap into [0, 2*pi) (strictly non-negative) so packing (float)theta into signed int HasValidProjection
        // never sets the sign bit (which made `HasValidProjection > 0` false for 50% of every planet rotation).
        double raw = (b.SpinRate * RenderClock.T) % twoPi;
        if (raw < 0) raw += twoPi;
        theta = raw;
        return true;
    }

    internal static Bubbles.Bubble[] Snapshot()
    {
        try { lock (Bubbles.All) return Bubbles.All.ToArray(); }
        catch { return Array.Empty<Bubbles.Bubble>(); }
    }

    private sealed class BoxedPos { internal Vector3D C0; }
    private static readonly ConditionalWeakTable<PlanetEnvironmentEntityComponent, BoxedPos> SavedPlanetC0 = new();

    internal static Vector3D InitialPlanetPos(PlanetEnvironmentEntityComponent planet) =>
        SavedPlanetC0.GetValue(planet, p => new BoxedPos { C0 = p.WorldPosition }).C0;

    /// <summary>
    /// Keeps orbiting planets' (e.g. the moon's) PlanetEnvironmentEntityComponent.WorldPosition synced to their live
    /// orbital centre before PlanetEnvironmentGroup.OnBeginDraw sorts, culls, and uploads atmosphere/cloud/spherization CBVs.
    /// Without this, the moon's atmosphere, clouds, occluder, and terrain spherization stay stuck at C0 up to ~400 km away.
    /// </summary>
    internal static void UpdateOrbitingPlanetEnvironments()
    {
        if (!Bubbles.Ready) return;
        try
        {
            var mgr = CoreSystems.PlanetEnvironments;
            if (mgr == null) return;
            var planets = mgr.Planets;
            var bubbles = Snapshot();
            double t = RenderClock.T;
            for (int i = 0; i < planets.Length; i++)
            {
                PlanetEnvironmentEntityComponent? planet = planets[i];
                if (planet == null) continue;
                Vector3D c0 = InitialPlanetPos(planet);
                foreach (Bubbles.Bubble b in bubbles)
                {
                    if (b.OrbitAround == null) continue;
                    if ((b.C0 - c0).LengthSquared() > 1000.0 * 1000.0) continue;
                    Vector3D liveC = Bubbles.Centre(b, t);
                    planet.WorldPosition = liveC;
                    planet.Data.GetWritePtr<PlanetEnvironmentEntityComponent.PlanetData>().WorldPosition = liveC;
                    break;
                }
            }
        }
        catch { }
    }

    internal static Vector3 RotDir(in Vector3 v, in Vector3D axis, double a)
    {
        double c = Math.Cos(a), s = Math.Sin(a);
        Vector3D d = v;
        return (Vector3)(d * c + Vector3D.Cross(axis, d) * s + axis * (Vector3D.Dot(axis, d) * (1 - c)));
    }
}

/// <summary>
/// Bubble time of the frame the RENDER thread is drawing. Run 49: reading Bubbles.T on the render thread made the clouds
/// jitter - Bubbles.T is resampled on the game thread each frame (GlobalRenderJobs.FrameClock) while the render thread
/// may still be drawing the previous frame, so the cloud angle was sometimes a frame ahead of the terrain (~50 m at 3 deg/s).
/// The camera pairs the two exactly: the game thread converts the camera with Bubbles.T (GlobalRender.CameraPatch) and
/// sends it through RenderSettings.SetCameraParameters; the render thread applies that same frame's camera in
/// SettingsManager.SetCameraParameters (Render12\Core\Systems\SettingsManager.cs:676) before it draws. The global camera
/// position, bit for bit, is the key.
/// </summary>
internal static class RenderClock
{
    private const int N = 64;
    private static readonly Vector3D[] Pos = new Vector3D[N];
    private static readonly double[] Time = new double[N];
    private static int _next;
    private static double _t = double.NaN;
    private static double _lastDrawnT = double.NaN;
    private static double _deltaT;
    internal static int Hits, Misses;

    /// <summary>Render-frame bubble time; falls back to Bubbles.T until the first camera has been matched.</summary>
    internal static double T => double.IsNaN(_t) ? Bubbles.T : _t;

    /// <summary>Exact bubble-time delta between the previous drawn frame and the current drawn frame.</summary>
    internal static double DeltaT => _deltaT;

    internal static void BeginDraw()
    {
        double cur = T;
        _deltaT = double.IsNaN(_lastDrawnT) ? 0.0 : cur - _lastDrawnT;
        _lastDrawnT = cur;
    }

    internal static void Record(in Vector3D globalCamera, double t)
    {
        lock (Pos) { Pos[_next] = globalCamera; Time[_next] = t; _next = (_next + 1) % N; }
    }

    internal static void Apply(in Vector3D globalCamera)
    {
        lock (Pos)
        {
            for (int k = 1; k <= N; k++)
            {
                int i = (_next - k + N) % N;
                if (Pos[i].X == globalCamera.X && Pos[i].Y == globalCamera.Y && Pos[i].Z == globalCamera.Z) { _t = Time[i]; Hits++; return; }
            }
        }
        Misses++;
    }

    /// <summary>Game side, after GlobalRender.CameraPatch (a prefix that may skip the original): the pose it sent.
    /// RenderSettings.cs:727-736 stores CameraTransform in both the vanilla body and CameraPatch.</summary>
    [HarmonyPatch(typeof(Keen.VRage.Render.Contracts.RenderSettings), nameof(Keen.VRage.Render.Contracts.RenderSettings.SetCameraParameters))]
    internal static class GameSide
    {
        [HarmonyPostfix]
        private static void Postfix(Keen.VRage.Render.Contracts.RenderSettings __instance)
        {
            if (Bubbles.Ready) Record(__instance.CameraTransform.Position, Bubbles.T);
        }
    }

    [HarmonyPatch(typeof(Keen.VRage.Render12.Core.Systems.SettingsManager), nameof(Keen.VRage.Render12.Core.Systems.SettingsManager.SetCameraParameters))]
    internal static class RenderSide
    {
        [HarmonyPostfix]
        private static void Postfix(in WorldTransform cameraTransform) => Apply(cameraTransform.Position);
    }

    [HarmonyPatch(typeof(Keen.VRage.Render12.Core.Systems.CommonResources.PlanetEnvironmentGroup), nameof(Keen.VRage.Render12.Core.Systems.CommonResources.PlanetEnvironmentGroup.OnBeginDraw))]
    internal static class DrawBegin
    {
        [HarmonyPrefix]
        private static void Prefix()
        {
            BeginDraw();
            PlanetEnvRender.UpdateOrbitingPlanetEnvironments();
        }
    }
}

// ------------------------------------------------------------------------------------------------------------------
// Clouds
// ------------------------------------------------------------------------------------------------------------------

/// <summary>
/// Shader text. VRage.Render12\Resources\Shaders\ShaderFileReaderManager.cs:35 CreateStreamReader is the only reader of
/// shader source; ShaderFileCacheManager.cs:166-167 reads it to the end and ShaderFileCache.cs:174 checksums the text,
/// and the compiled-shader cache key is built from those checksums (ShaderManager.cs:362-363, BuildValidationData
/// :695-708), so changed text compiles fresh (ShaderManager.cs:385-425) into this instance's own Temp\ShaderCache.
/// <para>Cloud\CloudGeneration.hlsli:99 GetCloudDensity(clouds, weatherMapIndex, position, ...) is the single entry for
/// cloud density: position is planet-centred in world axes (CloudTrace.hlsli:165-172, CloudShadows.hlsl:72-77) and
/// both the noise (SampleNoiseA/B :24-44) and the weather map (GetWeather(normalize(position)) :104) are sampled from
/// it. Rotating it into the planet's frame first makes the whole cloud shell turn with the planet; the vanilla wind
/// drift (RotateVector(.., AnimationFrameTime * Cloud_.LocalWind)) still runs on top, in the planet frame.</para>
/// <para>Utils.hlsli:12 RotateVector(v, x) is a rotation by -x about +Y; the plugin's planet rotation is +theta about
/// +Y (Bubbles.Rot, Rodrigues), so planet-local = RotateVector(global, theta).</para>
/// <para>Temporal reprojection (CloudAccumulate.hlsl:199, CloudComposite.hlsl:214) predicts last frame's position with
/// RotateVector(p, dt * LocalWind); with the planet also turning by omega * dt it must be dt * (LocalWind + omega).
/// `clouds` is declared in both functions (CloudAccumulate.hlsl:114, CloudComposite.hlsl:77).</para>
/// <para>theta and omega travel in CloudConstants._padding (Lighting\CloudData.hlsli:28 int2 _padding; C# twin
/// Render12\LightingStage\CloudConstants.cs Vector2I _padding), which is 0 for every planet we do not touch, i.e.
/// identity rotation and vanilla reprojection.</para>
/// </summary>
// ShaderFileReaderManager.CreateStreamReader (2 lines) was patched first: "patched" was logged but it never ran (run 45),
// the JIT inlined it into ShaderFileCacheManager.GetFileCache. ShaderFileCache.Create (ShaderFileCache.cs:108) receives the
// same text straight after the read (ShaderFileCacheManager.cs:166-167) and is not inlinable (regex, loops).
[HarmonyPatch(typeof(ShaderFileCache), nameof(ShaderFileCache.Create))]
internal static class CloudShaderPatch
{
    private const string Tag = "/* SpinProbe */";
    private static int _seen;

    [HarmonyPrefix]
    private static void Prefix(ref string content, ShaderFileHandle shaderHandle)
    {
        string name;
        // Run 48: the engine passes lower-case relative paths ("lighting\ircache\ircacheclearpool.hlsl").
        try { name = Path.GetFileName(shaderHandle.RelativePath.Replace((char)92, '/')).ToLowerInvariant(); }
        catch (Exception e) { ProbeLog.Write("cloudshader: handle path failed: " + e.Message); return; }
        if (name == "cloudgeneration.hlsli") name = "CloudGeneration.hlsli";
        else if (name == "cloudaccumulate.hlsl") name = "CloudAccumulate.hlsl";
        else if (name == "cloudcomposite.hlsl") name = "CloudComposite.hlsl";
        else if (name == "grassrenderingvertex.hlsl") name = "GrassRenderingVertex.hlsl";
        else if (name == "particleemitterprocess.hlsl" && Settings.ParticleFix) name = "ParticleEmitterProcess.hlsl";
        else return;
        try
        {
            content = Rewrite(name, content, out string what);
            ProbeLog.Write($"cloudshader: {name} ({shaderHandle.RelativePath}): {what}");
        }
        catch (Exception e)
        {
            ProbeLog.Write($"cloudshader: {name} rewrite FAILED, vanilla text kept: {e.Message}");
        }
    }

    /// <summary>
    /// Scene\GrassRenderingVertex.hlsl. After WindConvertPatch the projected directions are GLOBAL (correct for the bend,
    /// :181 windOffset / :340 adjustedNormal - trees fixed in run 49), but the grass also uses them against planet-local
    /// data: the vanilla sway phase dot(windDir, instancePosition) in CalculateWindOffsetSE1 (:81-91, instancePosition is
    /// root-space = planet-local) and the scroll sign sign(..ProjectedDirection) against the planet-local tiled position
    /// (:175-188). With a global direction both sweep as the planet turns (phase rate ~ omega * |p| - grass storm, run 49).
    /// Those uses get the direction rotated back to planet-local by the angle WindConvertPatch stores in
    /// HasValidProjection (Common\Frame.hlsli:368, only ever tested "> 0", :112). Local = RotateVector-style yaw by +theta
    /// (rotation by -theta about +Y), the inverse of the plugin's +theta about +Y.
    /// </summary>
    private static string RewriteGrass(string text, out string what)
    {
        const string fn = "float3 CalculateWindOffsetSE1(float3 windDir, float3 instancePosition, TimeSpan frameTime)";
        int f = text.IndexOf(fn, StringComparison.Ordinal);
        if (f < 0) throw new InvalidOperationException("CalculateWindOffsetSE1 not found");
        string helper = Tag + @"
float3 SpinProbeLocal(float3 d, WindSettings ws)
{
    float a = asfloat(ws.HasValidProjection);
    float c = cos(a), s = sin(a);
    return float3(c * d.x - s * d.z, d.y, s * d.x + c * d.z);
}

";
        text = text.Insert(f, helper);
        // Pass windSettings into CalculateWindOffsetSE1 so PreviousWindSettings_ uses its own frame's angle too.
        text = text.Replace(fn, "float3 CalculateWindOffsetSE1(float3 windDir, float3 instancePosition, TimeSpan frameTime, WindSettings windSettings)");
        const string phase = "1, dot(windDir, instancePosition));";
        if (CountOf(text, phase) != 1) throw new InvalidOperationException("SE1 phase not found exactly once");
        text = text.Replace(phase, "1, dot(SpinProbeLocal(windDir, windSettings), instancePosition));");
        // Update both CalculateWindOffsetSE1 call sites inside CalculateWindOffset:
        // - line 113 multiplies by global surfaceTBN, so rotate the returned vector to local first to avoid double-rotating by 2*theta
        // - line 115 adds directly in global space, so keep global windDir
        const string callNew = "mul(CalculateWindOffsetSE1(windDir, instancePosition, frameTime), surfaceTBN)";
        if (CountOf(text, callNew) != 1) throw new InvalidOperationException("SE1 new-wind call not found exactly once");
        text = text.Replace(callNew, "mul(SpinProbeLocal(CalculateWindOffsetSE1(windDir, instancePosition, frameTime, windSettings), windSettings), surfaceTBN)");
        const string callOld = "CalculateWindOffsetSE1(windDir, instancePosition, frameTime) * windStrength;";
        if (CountOf(text, callOld) != 1) throw new InvalidOperationException("SE1 fallback call not found exactly once");
        text = text.Replace(callOld, "CalculateWindOffsetSE1(windDir, instancePosition, frameTime, windSettings) * windStrength;");

        // Keep grass cluster/scale lookup (ComputePlanetTangentCoords) in planet-local space so blades don't pop/resize as the planet spins.
        const string tangentTbn = "float3x3 tbn = CreateTBN(ViewToWorld(UpVectorView));";
        if (CountOf(text, tangentTbn) != 1) throw new InvalidOperationException("ComputePlanetTangentCoords TBN not found exactly once");
        text = text.Replace(tangentTbn, "float3x3 tbn = CreateTBN(SpinProbeLocal(ViewToWorld(UpVectorView), WindSettings_));");

        // Keep both wind-movement scroll signs AND normal-wave scroll directions in planet-local space.
        int dirs = 0;
        foreach (string ws in new[] { "WindSettings_", "PreviousWindSettings_" })
            foreach (string dir in new[] { "CurrentProjectedDirection", "NextProjectedDirection" })
            {
                string o = $"sign({ws}.{dir})";
                int n = CountOf(text, o);
                if (n != 1) throw new InvalidOperationException($"{o} found {n} times");
                text = text.Replace(o, $"sign(SpinProbeLocal({ws}.{dir}, {ws}))");
                dirs += n;
            }
        foreach (string dir in new[] { "CurrentProjectedDirection", "NextProjectedDirection" })
        {
            string o = $"WindSettings_.{dir} * WindSettings_.NormalWaveUVOffset";
            int n = CountOf(text, o);
            if (n != 1) throw new InvalidOperationException($"{o} found {n} times");
            text = text.Replace(o, $"SpinProbeLocal(WindSettings_.{dir}, WindSettings_) * WindSettings_.NormalWaveUVOffset");
            dirs += n;
        }
        what = $"grass sway phase, tangent coords, and {dirs} scroll/normal-wave directions use the planet-local frame";
        return text;
    }

    private static int CountOf(string text, string s)
    {
        int n = 0;
        for (int i = text.IndexOf(s, StringComparison.Ordinal); i >= 0; i = text.IndexOf(s, i + 1, StringComparison.Ordinal)) n++;
        return n;
    }

    private static string RewriteParticles(string text, out string what)
    {
        // ParticleEmitterProcess.hlsl:372-379 ComputePositionDelta advances each world-space emitter's SpaceOrigin by:
        //   paused (dt==0): Camera_.PositionDelta.xyz
        //   active:         Camera_.PositionDelta.xyz - (Translation(proxy.Transform) - Translation(proxy.PreviousTransform)) * emitter.MotionInheritance
        // Inside a spinning planet bubble, BOTH Camera_.PositionDelta.xyz AND (proxy.Transform - proxy.PreviousTransform)
        // contain +spinDelta (Camera_._padding, stamped by ParticleCameraDeltaPatch). Previously subtracting Camera_._padding
        // from Camera_.PositionDelta.xyz alone left (-spinDelta * MotionInheritance) uncancelled, flinging thruster flames
        // and any motion-inheriting emitter sideways! Subtracting Camera_._padding * (1.f - emitter.MotionInheritance) on
        // the active line cancels spinDelta identically for every MotionInheritance in [0, 1].
        const string fn = "float3 ComputePositionDelta(";
        int f = text.IndexOf(fn, StringComparison.Ordinal);
        if (f < 0) throw new InvalidOperationException("ComputePositionDelta not found");
        int end = text.IndexOf('}', f);
        if (end < 0) throw new InvalidOperationException("ComputePositionDelta body end not found");
        string body = text.Substring(f, end - f + 1);
        const string pausedSite = "return Camera_.PositionDelta.xyz;";
        const string activePrefix = "return Camera_.PositionDelta.xyz -";
        int nPaused = CountOf(body, pausedSite);
        int nActive = CountOf(body, activePrefix);
        if (nPaused != 1 || nActive != 1)
            throw new InvalidOperationException($"unexpected ComputePositionDelta layout (paused={nPaused}, active={nActive})");
        string patched = body
            .Replace(pausedSite, $"return (Camera_.PositionDelta.xyz - Camera_._padding {Tag});")
            .Replace(activePrefix, $"return (Camera_.PositionDelta.xyz - Camera_._padding * (1.f - emitter.MotionInheritance) {Tag}) -");
        what = "ComputePositionDelta cancels planet spin with MotionInheritance factor (paused + active)";
        return text.Substring(0, f) + patched + text.Substring(end + 1);
    }

    private static string Rewrite(string name, string text, out string what)
    {
        if (text.Contains(Tag)) { what = "already patched"; return text; }
        if (name == "GrassRenderingVertex.hlsl") return RewriteGrass(text, out what);
        if (name == "ParticleEmitterProcess.hlsl") return RewriteParticles(text, out what);
        if (name == "CloudGeneration.hlsli")
        {
            int f = text.IndexOf("float4 GetCloudDensity(CloudConstants clouds", StringComparison.Ordinal);
            if (f < 0) throw new InvalidOperationException("GetCloudDensity not found");
            if (text.IndexOf("float4 GetCloudDensity(CloudConstants clouds", f + 1, StringComparison.Ordinal) >= 0)
                throw new InvalidOperationException("GetCloudDensity found twice");
            int brace = text.IndexOf('{', f);
            if (brace < 0) throw new InvalidOperationException("GetCloudDensity body not found");
            what = "GetCloudDensity rotates position into the planet frame";
            return text.Insert(brace + 1, "\n    " + Tag + " position = RotateVector(position, asfloat(clouds._padding.x));\n");
        }
        const string old = "Engine_.AnimationFrameTimeDelta * Cloud_.LocalWind)";
        const string neu = "Engine_.AnimationFrameTimeDelta * Cloud_.LocalWind + asfloat(clouds._padding.y)) " + Tag;
        int first = text.IndexOf(old, StringComparison.Ordinal);
        if (first < 0 || text.IndexOf(old, first + 1, StringComparison.Ordinal) >= 0)
            throw new InvalidOperationException("reprojection line not found exactly once");
        what = "reprojection includes the planet spin";
        return text.Replace(old, neu);
    }
}

/// <summary>
/// Which planet the next CloudConstants are built for. Every CloudConstants.FromDefinition call is immediately preceded
/// by CloudsManager.GetCloudShell(planet) on the same thread: PlanetEnvironmentGroup.cs:207/209 (shadow setup),
/// :356/:367 (per-planet setups read by the cloud shaders as PlanetEnvironments[i].Clouds), CloudWeatherMapJob.cs:83/92.
/// </summary>
[HarmonyPatch(typeof(CloudsManager), nameof(CloudsManager.GetCloudShell))]
internal static class CloudShellPlanetPatch
{
    [ThreadStatic] internal static PlanetEnvironmentEntityComponent? Last;

    [HarmonyPrefix]
    private static void Prefix(PlanetEnvironmentEntityComponent planetEnvironmentEntity) => Last = planetEnvironmentEntity;
}

/// <summary>Render12\LightingStage\CloudConstants.cs FromDefinition: stamp theta / omega into _padding for a spinning planet.
/// The planet is matched by the cloud shell's centre (PlanetEnvironmentEntityComponent.WorldPosition, set once from the
/// planet's saved position, PlanetEnvironmentRenderComponent.cs:112) against the bubble's C0.</summary>
[HarmonyPatch(typeof(CloudConstants), nameof(CloudConstants.FromDefinition))]
internal static class CloudConstantsPatch
{
    [HarmonyPostfix]
    private static void Postfix(ref CloudConstants __result)
    {
        PlanetEnvironmentEntityComponent? planet = CloudShellPlanetPatch.Last;
        CloudShellPlanetPatch.Last = null;
        if (planet == null || !Bubbles.Ready) return;
        Vector3D c = PlanetEnvRender.InitialPlanetPos(planet);
        foreach (Bubbles.Bubble b in PlanetEnvRender.Snapshot())
        {
            if ((b.C0 - c).LengthSquared() > 1000.0 * 1000.0) continue;
            if (!PlanetEnvRender.SpinOf(b, out double theta) || Math.Abs(Math.Abs(b.SpinAxis.Y) - 1) > 1e-6) return;
            double sign = Math.Sign(b.SpinAxis.Y);   // RotateVector only turns about world Y
            __result._padding = new Vector2I(BitConverter.SingleToInt32Bits((float)(theta * sign)),
                                             BitConverter.SingleToInt32Bits((float)(b.SpinRate * RenderClock.DeltaT * sign)));
            PlanetEnvRender.CloudConstantsStamped++;
            return;
        }
    }
}

// ------------------------------------------------------------------------------------------------------------------
// Grass / tree / particle wind
// ------------------------------------------------------------------------------------------------------------------

/// <summary>
/// VRage.Render\Data\WindSettings.cs Convert(IGPUDataConvertor) builds the GPU wind (GPUImprint) from WindAzimuth and the
/// grass projection info that Game2.Client\GameSystems\Render\PlanetaryGrassComponent.cs:59-122 computes from the
/// camera's (bubble-local) position relative to the planet: CurrentProjectedDirection / NextProjectedDirection =
/// ComputeProjectionBasis(axis) * windVector, CurrentDirection = DirectionInTangentPlane(UpAxis, azimuth). Shaders use
/// the projected directions as WORLD bend directions (Scene\GrassRenderingVertex.hlsl:181 windOffset, :340 adjustedNormal;
/// Geometry\Materials\PBRFloraVertex.hlsl:55 SpeedTree windDirection) and the projection AXES only to pick the plane
/// the wind texture is sampled in (CalculateProjectionUV, :96-103) against TiledWorldPosition.
/// <para>So: the three directions are rotated into global (where the ground actually is), the axes stay planet-local
/// and the grass TiledWorldPosition is made planet-local too (GrassLocalPatch), so the wind pattern stays fixed to the
/// ground exactly as in vanilla. Convert is untouched; its single Write of the imprint goes through this wrapper.</para>
/// </summary>
[HarmonyPatch(typeof(WindSettings), nameof(WindSettings.Convert))]
internal static class WindConvertPatch
{
    [HarmonyPrefix]
    private static void Prefix(ref IGPUDataConvertor dataConvertor)
    {
        if (!Bubbles.Ready || !GlobalRender.HaveCamera) return;
        Bubbles.Bubble? b = Bubbles.BubbleAt(GlobalRender.LocalCamera.Position);
        if (!PlanetEnvRender.SpinOf(b, out double theta)) return;
        dataConvertor = new RotatingConvertor(dataConvertor, b!.SpinAxis, theta);
    }

    private sealed class RotatingConvertor : IGPUDataConvertor
    {
        private readonly IGPUDataConvertor _inner;
        private readonly Vector3D _axis;
        private readonly double _theta;
        internal RotatingConvertor(IGPUDataConvertor inner, Vector3D axis, double theta) { _inner = inner; _axis = axis; _theta = theta; }

        public int GetTextureId(ResourceHandle? handle, PixelValue? defaultValue) => _inner.GetTextureId(handle, defaultValue);
        public int GetTexture3DId(ResourceHandle? handle, PixelValue? defaultValue) => _inner.GetTexture3DId(handle, defaultValue);
        public int GetCubeTextureId(ResourceHandle? handle, PixelValue? defaultValue) => _inner.GetCubeTextureId(handle, defaultValue);
        public GPUBufferId GetBufferId(ResourceHandle handle) => _inner.GetBufferId(handle);
        public int GetWindAnimationId(Guid windAnimationGuid) => _inner.GetWindAnimationId(windAnimationGuid);
        public TimeSpan GetSystemTimeSpanValue(SystemTimeSpanValueEnum valueType) => _inner.GetSystemTimeSpanValue(valueType);

        public void Write<T>(in T data) where T : unmanaged
        {
            if (typeof(T) != typeof(WindSettings.GPUImprint)) { _inner.Write(in data); return; }
            WindSettings.GPUImprint w = Unsafe.As<T, WindSettings.GPUImprint>(ref Unsafe.AsRef(in data));
            w.CurrentProjectedDirection = PlanetEnvRender.RotDir(w.CurrentProjectedDirection, _axis, _theta);
            w.NextProjectedDirection = PlanetEnvRender.RotDir(w.NextProjectedDirection, _axis, _theta);
            w.CurrentDirection = PlanetEnvRender.RotDir(w.CurrentDirection, _axis, _theta);
            // The angle for GrassRenderingVertex.hlsl (CloudShaderPatch.RewriteGrass). HasValidProjection is only tested
            // "> 0"; wrap into [0, 2*pi) so sign bit 31 is always 0 (positive int), and force bit 0 on so 0.0 stays > 0.
            if (w.HasValidProjection != 0 && Math.Abs(Math.Abs(_axis.Y) - 1) < 1e-6)
            {
                const double twoPi = 2.0 * Math.PI;
                double a = (_theta * Math.Sign(_axis.Y)) % twoPi;
                if (a < 0) a += twoPi;
                w.HasValidProjection = (int)(BitConverter.SingleToUInt32Bits((float)a) | 1u);
            }
            PlanetEnvRender.WindRotated++;
            _inner.Write(in w);
        }
    }
}

/// <summary>
/// Render12\SceneSystem\Components\GrassEntityComponent.cs:158-169 OnGrassUpdated, same body, except TiledWorldPosition is
/// taken from the planet-LOCAL position. The shader adds the instance's root-space offset to it
/// (GrassRenderingVertex.hlsl:167) and projects the sum on the planet-local projection axis, so it must be planet-local
/// for the wind pattern to be continuous and fixed to the ground. The root's global pose is our own push
/// (RootUpdateLockPatch: ToGlobal(bubble, centredLocal * offset)), so its rotation is exactly
/// rootOrientation * conj(planet's local orientation); spin-only bubbles keep their centre at C0.
/// </summary>
[HarmonyPatch(typeof(GrassEntityComponent), nameof(GrassEntityComponent.OnGrassUpdated))]
internal static class GrassLocalPatch
{
    [HarmonyPrefix]
    private static bool Prefix(in GrassEntityComponent.GPUGrassLocationData grassLocationData, in GrassEntityComponent.GrassModelData grassModelData,
        in GrassEntityComponent.GrassVisualData grassVisualData, in RelativeTransform transform, in ChildComponent.RootData root,
        ReadOnlyEntityData<WorldTransform> rootTransforms)
    {
        if (!Bubbles.Ready) return true;
        try
        {
            WorldTransform rootT = rootTransforms.ReadFrom(root.Root);
            WorldTransform world = rootT * transform;
            if (!TryLocal(rootT, world.Position, out Vector3D p)) return true;
            CoreSystems.GPUScene.GrassEntityData.ScheduleUpdate(grassLocationData.Index, new GPUGrass
            {
                ModelId = grassModelData.ModelId,
                Opacity = grassVisualData.Opacity,
                LODMultiplier = 1f / (float)Math.Pow(2.0, grassModelData.Lod),
                TiledWorldPosition = new Vector3((float)Math.IEEERemainder(p.X, 5000.0), (float)Math.IEEERemainder(p.Y, 5000.0), (float)Math.IEEERemainder(p.Z, 5000.0))
            });
            PlanetEnvRender.GrassLocalised++;
            return false;
        }
        catch { return true; }
    }

    private static bool TryLocal(in WorldTransform rootGlobal, in Vector3D pGlobal, out Vector3D pLocal)
    {
        pLocal = pGlobal;
        double t = RenderClock.T;
        foreach (Bubbles.Bubble b in PlanetEnvRender.Snapshot())
        {
            if (b.IsIdentity) continue;
            Vector3D c = Bubbles.Centre(b, t);
            if ((pGlobal - c).LengthSquared() > b.Radius * b.Radius) continue;
            if (b.SpinRate == 0) { pLocal = pGlobal - c + b.C0; return true; }      // orbit only: translation
            if (!Bubbles.LocalOrientation(b, out Quaternion ql)) return false;
            Quaternion rot = Quaternion.Normalize(rootGlobal.Orientation * Quaternion.Conjugate(ql));
            Vector3 d = (Vector3)(pGlobal - b.C0);
            pLocal = (Vector3D)Vector3.Transform(d, Quaternion.Conjugate(rot)) + b.C0;
            return true;
        }
        return false;
    }
}

// ------------------------------------------------------------------------------------------------------------------
// Particles (planet-frame camera delta + rotated gravity)
// ------------------------------------------------------------------------------------------------------------------

/// <summary>
/// Stamps the planet-spin component of Camera_.PositionDelta into unused CameraSettings._padding (float3) so
/// ParticleEmitterProcess.hlsl's ComputePositionDelta can subtract it, keeping world-space particles (thrusters,
/// drill dust, grinder/welder sparks, smoke) in the spinning planet's frame instead of shooting sideways.
/// Source: VRage.Render12\Primitives\Frame\CameraSettings.cs:37-72 CreateCameraSettings.
/// </summary>
[HarmonyPatch(typeof(Keen.VRage.Render12.Primitives.Frame.CameraSettings), "CreateCameraSettings")]
internal static class ParticleCameraDeltaPatch
{
    [HarmonyPostfix]
    private static void Postfix(in Keen.VRage.Render12.Primitives.RenderView view, ref Keen.VRage.Render12.Primitives.Frame.CameraSettings __result)
    {
        if (!Bubbles.Ready || MapRender.Open) return;
        Bubbles.Bubble? b = GlobalRender.CameraBubble;
        if (b == null || b.IsIdentity) return;
        double dt = RenderClock.DeltaT;
        if (dt == 0.0) return;
        double tNow = RenderClock.T;
        double tPrev = tNow - dt;
        Vector3D prevPos = view.LastFrameCameraPositionWithCuts;
        Vector3D cPrev = Bubbles.Centre(b, tPrev);
        Vector3D cNow = Bubbles.Centre(b, tNow);
        Vector3D relPrev = prevPos - cPrev;
        Vector3D carried = cNow + relPrev;
        if (b.SpinRate != 0)
        {
            double a = b.SpinRate * dt;
            double c = Math.Cos(a), s = Math.Sin(a);
            Vector3D axis = b.SpinAxis;
            Vector3D rotRel = relPrev * c + Vector3D.Cross(axis, relPrev) * s + axis * (Vector3D.Dot(axis, relPrev) * (1.0 - c));
            carried = cNow + rotRel;
        }
        Vector3D spinDelta = carried - prevPos;
        __result._padding = (Vector3)spinDelta;
    }
}

/// <summary>
/// Rotates particle emitter gravity from bubble-local into global space when packing GPUInstanceController.PackedGravity.
/// Restores the struct field in Postfix so the ECS EffectControllerData is never permanently mutated.
/// Source: VRage.Render12\SceneSystem\Components\BaseParticleEffectEntityComponent.cs:641-678 BuildGPUInstanceController.
/// </summary>
[HarmonyPatch(typeof(BaseParticleEffectEntityComponent), "BuildGPUInstanceController")]
internal static class ParticleGravityPatch
{
    [HarmonyPrefix]
    private static void Prefix(ref BaseParticleEffectEntityComponent.EffectControllerData controllerData, out (bool rotated, Vector3 orig) __state)
    {
        __state = default;
        if (!Bubbles.Ready || MapRender.Open || controllerData.GravityVector == Vector3.Zero) return;
        Bubbles.Bubble? b = GlobalRender.CameraBubble;
        if (!PlanetEnvRender.SpinOf(b, out double theta)) return;
        __state = (true, controllerData.GravityVector);
        controllerData.GravityVector = PlanetEnvRender.RotDir(controllerData.GravityVector, b!.SpinAxis, theta);
    }

    [HarmonyPostfix]
    private static void Postfix(ref BaseParticleEffectEntityComponent.EffectControllerData controllerData, (bool rotated, Vector3 orig) __state)
    {
        if (__state.rotated)
            controllerData.GravityVector = __state.orig;
    }
}
