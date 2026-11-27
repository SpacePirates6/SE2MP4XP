using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading;
using HarmonyLib;
using Keen.Game2.Simulation.GameSystems.EntityConfigurators.Planets;
using Keen.Game2.Simulation.GameSystems.Physicss;
using Keen.Game2.Simulation.GameSystems.RangedAffectGenerators.Atmosphere;
using Keen.Game2.Simulation.GameSystems.RangedAffectGenerators.Gravity;
using Keen.Game2.Simulation.GameSystems.RangedAffectGenerators.Wind;
using Keen.Game2.Simulation.WorldObjects.CubeBlocks.Movement;
using Keen.Game2.Simulation.WorldObjects.CubeGrids;
using Keen.Game2.Simulation.WorldObjects.CubeGrids.Movement;
using Keen.Game2.Simulation.WorldObjects.Movement;
using Keen.Game2.Simulation.WorldObjects.Shared.Movement;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Game.GameSystems.Gravity;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Character;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Havok;
using Keen.VRage.Physics.Havok.Character;
using Keen.VRage.Physics.Havok.MotionSharing;
using Keen.VRage.Voxels.Components;

namespace SpinProbe;

/// <summary>
/// Realistic physics (docs/realistic-physics-plan.md). Every engine reference is to the decompile at D:\SE2MPNEW\Se2Decomp.
/// Phases implemented here: 1 limits, 2 inverse-square gravity, 3 rotating-frame forces, 4 gyro free-spin + per-thruster
/// torque, 5 gyroscopic (tumbling) torque, 6 quadratic air drag. Rails (phase 7) live in RealRails.cs.
/// </summary>
internal static class RealPhysics
{
    internal static bool On => Settings.RealPhysics;
    private const float Dt = 1f / 60f; // fixed server step: Game2.Simulation UpdateTime, physics StepFrequency 60

    // ---------- planet registry (centre, surface radius), server + client copies ----------

    private static readonly object PlanetLock = new();
    private static readonly List<(Vector3D c, double r)> PlanetList = new();
    internal static volatile (Vector3D c, double r)[] Planets = Array.Empty<(Vector3D, double)>();

    [HarmonyPatch(typeof(VoxelPhysicsComponent), "Init", new[] { typeof(VoxelPhysicsComponentObjectBuilder) })]
    internal static class PlanetRegistryPatch
    {
        [HarmonyPostfix]
        private static void Postfix(VoxelPhysicsComponent __instance)
        {
            try
            {
                if (__instance.Entity?.TryGet<VoxelPlanetComponent>() is not VoxelPlanetComponent planet) return;
                Vector3D c = __instance.Data.GetWorldTransform().Position;
                lock (PlanetLock)
                {
                    if (PlanetList.Exists(x => (x.c - c).LengthSquared() < 1.0)) return;
                    PlanetList.Add((c, planet.Radius));
                    Planets = PlanetList.ToArray();
                }
                ProbeLog.Write($"real: planet at {c} R={planet.Radius:0} m registered");
            }
            catch (Exception e) { ProbeLog.Write("real: planet registry failed: " + e.Message); }
        }
    }

    /// <summary>Surface gravity per planet centre (quantised to 1 m), recorded at GravityGeneratorComponent.Init so
    /// planets sharing the same radius (e.g. two 60 km worlds) never overwrite each other.</summary>
    internal static readonly System.Collections.Concurrent.ConcurrentDictionary<(long x, long y, long z), float> G0ByPlanet = new();
    /// <summary>Fallback surface gravity per planet radius if centre lookup misses.</summary>
    internal static readonly System.Collections.Concurrent.ConcurrentDictionary<long, float> G0ByRadius = new();

    private static (long x, long y, long z) QuantizeCenter(in Vector3D c) =>
        ((long)Math.Round(c.X), (long)Math.Round(c.Y), (long)Math.Round(c.Z));

    internal static void RecordPlanetG0(in Vector3D centre, float g0)
    {
        G0ByPlanet[QuantizeCenter(centre)] = g0;
        if (PlanetAt(centre, out _, out Vector3D pc))
            RealRails.PlanetG0[pc] = g0;
    }

    internal static bool TryGetG0(in Vector3D planetCentre, double planetRadius, out float g0)
    {
        if (RealRails.PlanetG0.TryGetValue(planetCentre, out double rg) && rg > 0) { g0 = (float)rg; return true; }
        if (G0ByPlanet.TryGetValue(QuantizeCenter(planetCentre), out g0) && g0 > 0) return true;
        foreach (var kv in G0ByPlanet)
        {
            double dx = kv.Key.x - planetCentre.X, dy = kv.Key.y - planetCentre.Y, dz = kv.Key.z - planetCentre.Z;
            if (dx * dx + dy * dy + dz * dz < 16.0) { g0 = kv.Value; return true; }
        }
        return G0ByRadius.TryGetValue((long)Math.Round(planetRadius), out g0) && g0 > 0;
    }

    /// <summary>The orbiting (moving) bubble whose local origin is this planet's centre, if any.</summary>
    internal static Bubbles.Bubble? MovingBubbleOf(in Vector3D planetCentre)
    {
        var all = Bubbles.All;
        for (int i = 0; i < all.Count; i++)
        {
            var b = all[i];
            if (b.OrbitAround != null && (b.C0 - planetCentre).LengthSquared() < 4.0) return b;
        }
        return null;
    }

    /// <summary>True when this server entity is simulated in global (inertial) coordinates, outside every bubble.</summary>
    internal static bool InGlobalSpace(DEntity entity, in Vector3D p)
    {
        if (!Bubbles.Ready) return false;
        Bubbles.Bubble? b = Settings.RealBubbleHandoff ? BubbleHandoff.ResolveServerBubble(entity, in p) : Bubbles.BubbleAt(p);
        return b == null;
    }

    /// <summary>
    /// Gravity of orbiting planets for a body in global coordinates. A planet's gravity generator sits at the planet's
    /// server position, which is its bubble's local origin C0 (Bubbles.cs: x_global = Rspin(x_local - C0) + C(t)). Inside
    /// the bubble that is right; in global space the planet is at C(t), so the engine would pull toward where the moon was
    /// at t = 0. GravityFalloffPatch therefore returns 0 for an orbiting planet's generator outside its bubble, and this
    /// adds the same inverse-square pull toward C(t) instead (times GravityMultiplier, which the engine applies to its own
    /// gravity but not to a velocity change written here).
    /// </summary>
    internal static Vector3D MovingPlanetGravity(in Vector3D p)
    {
        Vector3D a = Vector3D.Zero;
        if (!Settings.RealGravity || !Bubbles.Ready) return a;
        double t = Bubbles.T;
        foreach (var (c, r) in Planets)
        {
            Bubbles.Bubble? b = MovingBubbleOf(c);
            if (b == null || !TryGetG0(c, r, out float g0)) continue;
            Vector3D cNow = Bubbles.Centre(b, t);
            Vector3D d = cNow - p;
            double dist = d.Length();
            if (dist < 1e-3 || dist > r * 1.05 * Settings.RealGravityRange) continue;
            double mag = dist >= r ? g0 * r * r / (dist * dist) : g0 * dist / r;
            a += d / dist * (mag * RealRails.GravityMultiplier);
        }
        return a;
    }

    private static bool PlanetAt(in Vector3D generatorPos, out double radius) => PlanetAt(generatorPos, out radius, out _);

    private static bool PlanetAt(in Vector3D generatorPos, out double radius, out Vector3D centre)
    {
        foreach (var (c, r) in Planets)
            if ((c - generatorPos).LengthSquared() < 4.0) { radius = r; centre = c; return true; }
        radius = 0; centre = default; return false;
    }

    /// <summary>The spinning bubble (rotating frame) containing <paramref name="p"/>, from the client-built bubbles
    /// (Bubbles.cs; same process as the in-process server).</summary>
    internal static bool SpinningFrameAt(in Vector3D p, out Vector3D centre, out Vector3D omega)
    {
        centre = default; omega = default;
        if (!Settings.RealRotatingFrame) return false;
        try
        {
            var all = Bubbles.All;
            for (int i = 0; i < all.Count; i++)
            {
                var b = all[i];
                if (b.SpinRate == 0) continue;
                if ((p - b.C0).LengthSquared() > b.Radius * b.Radius) continue;
                centre = b.C0; omega = b.SpinAxis * b.SpinRate; return true;
            }
        }
        catch { }
        return false;
    }

    /// <summary>Hysteresis-aware spinning frame lookup for a server entity, agreeing with BubbleHandoff across the +/-250m band.</summary>
    internal static bool SpinningFrameAt(DEntity entity, in Vector3D p, out Vector3D centre, out Vector3D omega)
    {
        centre = default; omega = default;
        if (!Settings.RealRotatingFrame) return false;
        try
        {
            Bubbles.Bubble? b = Settings.RealBubbleHandoff
                ? BubbleHandoff.ResolveServerBubble(entity, in p)
                : Bubbles.BubbleAt(p);
            if (b == null || b.SpinRate == 0) return false;
            centre = b.C0;
            omega = b.SpinAxis * b.SpinRate;
            return true;
        }
        catch { return false; }
    }

    // ---------- statistics (logged every 10 s) ----------

    internal static long ForceBodies, CoriolisMaxMilli, DragMaxMilli, GyroscopicBodies, ThrustTorqueGrids, GyroFreeGrids, GravityCalls, MovingGravityMaxMilli;
    internal static float AirDensitySeen;
    private static long _lastLog;

    // Spin check: the fastest-spinning body of the last 10 s with its rotational energy ½Σ Iᵢωᵢ² and |L| (principal frame).
    // A free body in vacuum must keep both constant; if the energy creeps up, the gyroscopic term is counted twice
    // (once by Havok, once by GyroscopicDelta) and real_gyroscopic=0 is the fix.
    private static readonly object SpinGate = new();
    private static DEntity _spinEntity;
    private static double _spinW, _spinKe, _spinL;

    // Gravity check: how fast one body in vacuum actually gains speed along its gravity, divided by the gravity the engine
    // gave it (CustomGravity). Rails assume Havok scales rigid-body gravity by GravityMultiplier natively
    // (HavokSessionComponent.GravityMultiplier -> NativeSession->SetGravityMultiplier, not visible in managed code), so for
    // an unpowered body in free fall this should read the multiplier (2). Thrust, contact or drag spoil a sample.
    private static DEntity _ffEntity;
    private static long _ffTick;
    private static Vector3 _ffVel, _ffGrav;
    private static double _ffRatio = double.NaN;
    private static int _ffSamples;

    internal static void SampleFreeFall(DEntity e, in Vector3 vel, in Vector3 grav)
    {
        if (grav.LengthSquared() < 0.25f) return;
        long now = Profiler.Ticks;
        lock (SpinGate)
        {
            if (_ffEntity.IsEmpty || now - _ffTick > 120)
            {
                _ffEntity = e; _ffTick = now; _ffVel = vel; _ffGrav = grav;
                return;
            }
            if (e != _ffEntity || now - _ffTick < 60) return;
            double dt = (now - _ffTick) / 60.0;
            Vector3 gHat = Vector3.Normalize(_ffGrav);
            double measured = Vector3.Dot(vel - _ffVel, gHat) / dt;
            _ffRatio = measured / _ffGrav.Length();
            _ffSamples++;
            _ffTick = now; _ffVel = vel; _ffGrav = grav;
        }
    }

    internal static void RecordSpin(DEntity e, in RigidBodyMassProperties mass, in Vector3 omegaLocal)
    {
        double w = omegaLocal.Length();
        if (w < 0.3) return;
        Vector3 inv = mass.InvInertiaTensor;
        if (!(inv.X > 0 && inv.Y > 0 && inv.Z > 0)) return;
        Vector3 wp = mass.InvInertiaMajorAxisRotation * omegaLocal;
        double ix = 1.0 / inv.X, iy = 1.0 / inv.Y, iz = 1.0 / inv.Z;
        double ke = 0.5 * (ix * wp.X * wp.X + iy * wp.Y * wp.Y + iz * wp.Z * wp.Z);
        double l = Math.Sqrt(ix * ix * wp.X * wp.X + iy * iy * wp.Y * wp.Y + iz * iz * wp.Z * wp.Z);
        lock (SpinGate)
        {
            // keep following the same body while it still spins, so its energy can be compared across log lines
            if (e == _spinEntity || w > _spinW) { _spinEntity = e; _spinW = w; _spinKe = ke; _spinL = l; }
        }
    }

    internal static void MaybeLog()
    {
        long now = Environment.TickCount64;
        if (now - Interlocked.Read(ref _lastLog) < 10000) return;
        Interlocked.Exchange(ref _lastLog, now);
        long aeroCount = Interlocked.Exchange(ref Aerodynamics.EvalBodies, 0);
        long aeroTicks = Interlocked.Exchange(ref Aerodynamics.EvalStopwatchTicks, 0);
        long buildTicks = Interlocked.Exchange(ref Aerodynamics.BuildStopwatchTicks, 0);
        double tickToUs = 1_000_000.0 / System.Diagnostics.Stopwatch.Frequency;
        double avgEvalUs = aeroCount > 0 ? (aeroTicks * tickToUs) / aeroCount : 0.0;
        double buildMs = buildTicks * tickToUs / 1000.0;
        ProbeLog.Write($"real: 10s bodies={Interlocked.Exchange(ref ForceBodies, 0)} coriolisMax={Interlocked.Exchange(ref CoriolisMaxMilli, 0) / 1000.0:0.000} m/s2 " +
                       $"dragMax={Interlocked.Exchange(ref DragMaxMilli, 0) / 1000.0:0.000} m/s2 liftMax={Interlocked.Exchange(ref Aerodynamics.LiftMaxMilli, 0) / 1000.0:0.000} m/s2 " +
                       $"air={AirDensitySeen:0.000} kg/m3 aero={aeroCount} ({avgEvalUs:0.0} us/eval, build={buildMs:0.00} ms) occluded={Interlocked.Exchange(ref Aerodynamics.SubgridsOccluded, 0)} " +
                       $"profiles={Interlocked.Read(ref Aerodynamics.ProfilesBuilt)} gyroscopic={Interlocked.Exchange(ref GyroscopicBodies, 0)} " +
                       $"thrustTorque={Interlocked.Exchange(ref ThrustTorqueGrids, 0)} fc={Interlocked.Exchange(ref FlightComputerJobs.SolvedGrids, 0)} " +
                       $"rcs={Interlocked.Exchange(ref FlightComputerJobs.RcsAssistedGrids, 0)} rcsThrusters={Interlocked.Exchange(ref FlightComputerJobs.RcsThrusters, 0)} rcsLit={FlightComputerJobs.RcsActive.Count} handoffs={Interlocked.Read(ref BubbleHandoff.HandoffCount)} " +
                       $"gyroFree={Interlocked.Exchange(ref GyroFreeGrids, 0)} gravityCalls={Interlocked.Exchange(ref GravityCalls, 0)} " +
                       $"movingGravityMax={Interlocked.Exchange(ref MovingGravityMaxMilli, 0) / 1000.0:0.000} m/s2");
        lock (SpinGate)
        {
            if (_spinW > 0)
                ProbeLog.Write($"real: spin {_spinEntity} w={_spinW:0.000} rad/s energy={_spinKe:0.###E+0} J |L|={_spinL:0.###E+0} kg m2/s (a free body in vacuum keeps both constant)");
            _spinW = 0;
            if (_ffSamples > 0)
                ProbeLog.Write($"real: gravity check {_ffEntity} measured/engine gravity = {_ffRatio:0.00} over {_ffSamples} s " +
                               $"(GravityMultiplier={RealRails.GravityMultiplier}; only an unpowered body in free fall in vacuum counts)");
            _ffSamples = 0;
        }
    }

    private static void Max(ref long slot, double value)
    {
        long v = (long)(value * 1000);
        long cur;
        while (v > (cur = Interlocked.Read(ref slot)) && Interlocked.CompareExchange(ref slot, v, cur) != cur) { }
    }

    // ---------- phase 1: limits ----------

    /// <summary>PhysicsSessionConfiguration.Init(builder) copies the .def values (PhysicsSessionConfiguration.cs:160-168);
    /// WorldSessionComponent passes them into SceneSettings (:493-499) and into the VelocityLimitProvider (:296), which also
    /// clamps gravity (GravityGeneratorComponent ComputeLimitedGravity). Setters are private, so set through reflection.</summary>
    [HarmonyPatch(typeof(PhysicsSessionConfiguration), nameof(PhysicsSessionConfiguration.Init))]
    internal static class LimitsPatch
    {
        private static void Set(object o, string prop, float v)
        {
            PropertyInfo p = AccessTools.Property(o.GetType(), prop);
            object? old = p.GetValue(o);
            p.SetValue(o, v);
            ProbeLog.Write($"real: {prop} {old} -> {v}");
        }

        [HarmonyPostfix]
        private static void Postfix(PhysicsSessionConfiguration __instance)
        {
            if (!On) return;
            try
            {
                Set(__instance, "MaximumSpeedLinear", (float)Settings.RealSpeedLimit);
                Set(__instance, "MaximumCharacterSpeedLinear", (float)Settings.RealSpeedLimit);
                Set(__instance, "MaximumSpeedAngular", (float)Settings.RealAngularLimit);
                Set(__instance, "AngularDamping", (float)Settings.RealAngularDamping);
                RealRails.GravityMultiplier = __instance.GravityMultiplier;
                ProbeLog.Write($"real: GravityMultiplier left at {__instance.GravityMultiplier} (scales every body's gravity uniformly)");
            }
            catch (Exception e) { ProbeLog.Write("real: limits patch failed: " + e); }
        }
    }

    // ---------- phase 2: inverse-square gravity ----------

    /// <summary>Planet gravity generators get a much larger AffectDistance so gravity reaches orbit.
    /// GravityGeneratorComponent.Init (:81-108) calls GravityGeneratorProcessorComponent.UpdateOB first, which sets
    /// AffectDistance = min(R × default factor, MaxAffectDistance) (GravityGeneratorProcessorComponent.cs:35-42), and only
    /// then base.Init builds the trigger sphere from it (:408). A prefix on Init is therefore overwritten (run 74); the range
    /// is raised in a postfix on UpdateOB instead, which also bypasses the MaxAffectDistance clamp.</summary>
    [HarmonyPatch(typeof(GravityGeneratorProcessorComponent), nameof(GravityGeneratorProcessorComponent.UpdateOB))]
    internal static class GravityRangePatch
    {
        [HarmonyPostfix]
        private static void Postfix(GravityGeneratorObjectBuilder ob, float planetRadius)
        {
            if (!On || !Settings.RealGravity) return;
            try
            {
                if (ob.GravityShape != GravityShapeType.Spherical || ob.AccelerationDistance < 1000) return; // planets only
                G0ByRadius[(long)Math.Round(planetRadius)] = ob.GravitationalAcceleration;
                float range = (float)(ob.AccelerationDistance * Settings.RealGravityRange);
                if (range > ob.AffectDistance) ob.AffectDistance = range;
            }
            catch (Exception e) { ProbeLog.Write("real: gravity range patch failed: " + e.Message); }
        }
    }

    /// <summary>Logs the values the trigger is actually built from (after UpdateOB and our postfix) and records
    /// surface gravity keyed by the planet's exact world centre.</summary>
    [HarmonyPatch(typeof(GravityGeneratorComponent), "Init", new[] { typeof(GravityGeneratorObjectBuilder) })]
    internal static class GravityInitLog
    {
        [HarmonyPostfix]
        private static void Postfix(GravityGeneratorComponent __instance, GravityGeneratorObjectBuilder ob)
        {
            if (!On || !Settings.RealGravity) return;
            try
            {
                if (ob.GravityShape == GravityShapeType.Spherical && ob.AccelerationDistance >= 1000 && __instance?.Entity != null)
                {
                    Vector3D pos = __instance.Data.GetWorldTransform().Position;
                    RecordPlanetG0(pos, ob.GravitationalAcceleration);
                }
            }
            catch { }
            ProbeLog.Write($"real: gravity generator g={ob.GravitationalAcceleration} accelDist={ob.AccelerationDistance:0} " +
                           $"affect={ob.AffectDistance / 1000:0.#} km falloff={ob.FallOffPower} shape={ob.GravityShape}");
        }
    }

    /// <summary>Replaces the linear falloff (GravityGeneratorComponent.cs:509-535) for planets:
    /// g0·(R/r)² outside the surface, g0·r/R inside (uniform sphere), where R is the planet's surface radius. Bodies inside
    /// ANOTHER planet's bubble feel only that planet: the bubble frame free-falls with its planet (Bubbles.cs), so the
    /// other planet's pull is already accounted for by the frame. The radial part of the centrifugal force of the spinning
    /// frame is subtracted here so every body (characters and resting grids too) feels the same effective gravity.</summary>
    [HarmonyPatch(typeof(GravityGeneratorComponent), "CalculateGravitationalAccelerationMagnitude")]
    internal static class GravityFalloffPatch
    {
        [HarmonyPrefix]
        private static bool Prefix(WorldTransform affectedEntityWorldTransform, WorldTransform gravityGeneratorWorldTransform,
            GravityGeneratorComponent.GravityGeneratorData gravityGeneratorData, ref float __result)
        {
            if (!On || !Settings.RealGravity) return true;
            if (gravityGeneratorData.GravityShape != GravityShapeType.Spherical || gravityGeneratorData.AccelerationDistance < 1000) return true;
            try
            {
                Interlocked.Increment(ref GravityCalls);
                Vector3D p = affectedEntityWorldTransform.Position, g = gravityGeneratorWorldTransform.Position;
                if (PlanetAt(g, out double R, out Vector3D pc))
                {
                    if (!RealRails.PlanetG0.ContainsKey(pc))
                        RealRails.PlanetG0[pc] = gravityGeneratorData.GravitationalAcceleration;
                }
                else R = gravityGeneratorData.AccelerationDistance;
                // An orbiting planet's generator is only right for bodies in its own bubble (local coordinates); bodies in
                // global space get its pull from MovingPlanetGravity, toward where the planet is now.
                Bubbles.Bubble? moving = MovingBubbleOf(g);
                if (moving != null && (p - moving.C0).Length() > moving.Radius + BubbleHandoff.Band) { __result = 0f; return false; }
                foreach (var (c, r) in Planets)
                {
                    if ((c - g).LengthSquared() < 4.0) continue;
                    double bubble = r * Settings.FrameRadiusFactor;
                    if ((p - c).LengthSquared() < bubble * bubble) { __result = 0f; return false; }
                }
                double d = (p - g).Length();
                double g0 = gravityGeneratorData.GravitationalAcceleration;
                double mag = d >= R ? g0 * (R * R) / (d * d) : g0 * d / R;
                // The spinning frame's centrifugal term is applied whole in the force jobs, not folded in here: the engine
                // scales rigid-body gravity by GravityMultiplier natively but hands characters CustomGravity unscaled
                // (HavokSessionComponent.CollectCharacterGravity :318-330), so no single correction here fits both.
                __result = (float)mag;
                return false;
            }
            catch { return true; }
        }
    }

    // ---------- phase 4: free-spinning gyros (dampeners off) ----------
    // (job in RealPhysicsJobs.GyroFree)

    // ---------- shared math ----------

    /// <summary>One implicit (Newton) step of the gyroscopic term ω×Iω in the principal frame (E. Catto, "Numerical
    /// Methods", GDC 2015). Returns the change of body-local angular velocity, or zero if the inertia is degenerate.</summary>
    internal static Vector3 GyroscopicDelta(in RigidBodyMassProperties mass, in Vector3 omegaLocal)
    {
        Vector3 inv = mass.InvInertiaTensor;
        if (!(inv.X > 0 && inv.Y > 0 && inv.Z > 0) || float.IsInfinity(inv.X + inv.Y + inv.Z)) return Vector3.Zero;
        // principal frame: p = Inv(q)·local; back: Transform(p, q)  (RigidBodyDataFunctions.ApplyAngularImpulseLocal)
        Vector3 wp = mass.InvInertiaMajorAxisRotation * omegaLocal;
        double ix = 1.0 / inv.X, iy = 1.0 / inv.Y, iz = 1.0 / inv.Z;
        double wx = wp.X, wy = wp.Y, wz = wp.Z;
        double lx = ix * wx, ly = iy * wy, lz = iz * wz;            // L = Iω
        double h = Dt;
        // f = h (ω × L)
        double fx = h * (wy * lz - wz * ly), fy = h * (wz * lx - wx * lz), fz = h * (wx * ly - wy * lx);
        // J = I + h (skew(ω) I - skew(L))
        double a00 = ix, a01 = h * (-wz * iy + lz), a02 = h * (wy * iz - ly);
        double a10 = h * (wz * ix - lz), a11 = iy, a12 = h * (-wx * iz + lx);
        double a20 = h * (-wy * ix + ly), a21 = h * (wx * iy - lx), a22 = iz;
        double det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20);
        if (Math.Abs(det) < 1e-12) return Vector3.Zero;
        // Δ = -J⁻¹ f (Cramer)
        double dx = -((fx * (a11 * a22 - a12 * a21) - a01 * (fy * a22 - a12 * fz) + a02 * (fy * a21 - a11 * fz)) / det);
        double dy = -((a00 * (fy * a22 - a12 * fz) - fx * (a10 * a22 - a12 * a20) + a02 * (a10 * fz - fy * a20)) / det);
        double dz = -((a00 * (a11 * fz - fy * a21) - a01 * (a10 * fz - fy * a20) + fx * (a10 * a21 - a11 * a20)) / det);
        if (double.IsNaN(dx + dy + dz)) return Vector3.Zero;
        return Vector3.Transform(new Vector3((float)dx, (float)dy, (float)dz), mass.InertiaMajorAxisRotation);
    }
}

/// <summary>Server-side force jobs. They run in PhysicsUpdate.OnCollectMassProperties, the slot this plugin already uses
/// for its other per-tick writes (ProbeJobs.cs): RigidBodyData written here is collected in the same step by
/// HavokSessionComponent.CollectChangesForVelocities (OnChanged(RigidBodyData)). Writable&lt;T&gt; only marks the data
/// changed when WritableValue is touched, so bodies we leave alone are not re-sent to clients.</summary>
internal static class RealPhysicsJobs
{
    private const float Dt = 1f / 60f;
    private static int _errors;

    /// <summary>Coriolis + tangential centrifugal (rotating frame), cached shape-aware aerodynamics (drag, lift, CoP
    /// torque, rotational damping, closed-bay subgrid occlusion), and gyroscopic torque for awake dynamic grids.</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [WhenSimulated]
    [MustHave(typeof(DynamicTag))]
    [MustHave(typeof(ActiveTag))]
    [DoesNotHave(typeof(KinematicTag))]
    [DoesNotHave(typeof(TransformDriven))]
    [DoesNotHave(typeof(CharacterInfo))]
    internal static void ApplyRealForcesGrid(DEntity entity, CubeGridComponent grid, in WorldTransform wt, in RigidBodyMassProperties mass,
        Optional<AirData> air, Optional<BoundingBoxData> bounds, Optional<SharedMotionData> shared, Optional<CustomGravity> gravity, Writable<RigidBodyData> body)
    {
        ApplyRealForcesCore(entity, grid, in wt, in mass, air, bounds, shared, gravity, body);
    }

    /// <summary>Same forces for awake, non-grid, non-character dynamic bodies (movable asteroids, floating debris).</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [WhenSimulated]
    [MustHave(typeof(DynamicTag))]
    [MustHave(typeof(ActiveTag))]
    [DoesNotHave(typeof(CubeGridComponent))]
    [DoesNotHave(typeof(KinematicTag))]
    [DoesNotHave(typeof(TransformDriven))]
    [DoesNotHave(typeof(CharacterInfo))]
    internal static void ApplyRealForcesBody(DEntity entity, in WorldTransform wt, in RigidBodyMassProperties mass,
        Optional<AirData> air, Optional<BoundingBoxData> bounds, Optional<SharedMotionData> shared, Optional<CustomGravity> gravity, Writable<RigidBodyData> body)
    {
        ApplyRealForcesCore(entity, null, in wt, in mass, air, bounds, shared, gravity, body);
    }

    private static void ApplyRealForcesCore(DEntity entity, CubeGridComponent? grid, in WorldTransform wt, in RigidBodyMassProperties mass,
        Optional<AirData> air, Optional<BoundingBoxData> bounds, Optional<SharedMotionData> shared, Optional<CustomGravity> gravity, Writable<RigidBodyData> body)
    {
        try
        {
            if (!RealPhysics.On || mass.InvMass <= 0f || !float.IsFinite(mass.InvMass)) return;
            RigidBodyData rb = body.Value;
            if (!rb.LinearVelocity.IsValid() || !rb.AngularVelocity.IsValid()) return;

            // Kill absurd Havok penetration ejection spikes (> speed limit) on shattered crash debris so they
            // don't sweep 100+ meters per tick through voxel terrain and stall Havok StepCollide / CollectManifolds!
            float vLenSq = rb.LinearVelocity.LengthSquared();
            float hardCap = (float)Settings.RealSpeedLimit;
            if (vLenSq > hardCap * hardCap * 1.05f)
            {
                float vLen = MathF.Sqrt(vLenSq);
                body.WritableValue.LinearVelocity = rb.LinearVelocity * (hardCap / vLen);
                rb = body.Value;
            }

            Vector3 dv = Vector3.Zero, dwLocal = Vector3.Zero;
            Interlocked.Increment(ref RealPhysics.ForceBodies);
            if (gravity.HasValue && !(air.HasValue && air.Value.Density > 0f))
                RealPhysics.SampleFreeFall(entity, rb.LinearVelocity, gravity.Value.Value);

            // Fetch ShipAssembly group ONCE per body per tick (reused by Aero and Gyroscopic!)
            ShipAssembly.GroupInfo? group = grid != null ? ShipAssembly.GetGroup(grid) : null;

            // phase 3: rotating frame: Coriolis + the whole centrifugal term (see GravityFalloffPatch for why it is not in
            // the gravity magnitude).
            if (RealPhysics.SpinningFrameAt(entity, wt.Position, out Vector3D centre, out Vector3D omega))
            {
                Vector3D v = rb.LinearVelocity;
                Vector3D r = wt.Position - centre;
                Vector3D coriolis = -2.0 * Vector3D.Cross(omega, v);
                Vector3D centrifugal = -Vector3D.Cross(omega, Vector3D.Cross(omega, r));
                Vector3D a = coriolis + centrifugal;
                RealPhysicsMax(ref RealPhysics.CoriolisMaxMilli, coriolis.Length());
                dv += (Vector3)(a * Dt);
            }
            else if (RealPhysics.InGlobalSpace(entity, wt.Position))
            {
                // orbiting planets pull toward where they are now (see RealPhysics.MovingPlanetGravity); stage 4.1 adds the
                // star (Orbits.StarGravity, zero when there is no star).
                Vector3D g = RealPhysics.MovingPlanetGravity(wt.Position) + Orbits.StarGravity(wt.Position);
                if (g.LengthSquared() > 1e-10)
                {
                    RealPhysicsMax(ref RealPhysics.MovingGravityMaxMilli, g.Length());
                    dv += (Vector3)(g * Dt);
                }
            }

            // phase 6: cached shape-aware aerodynamics (drag, lift, CoP restoring torque, rotational damping, and
            // closed-bay subgrid occlusion). Zero per-tick block loops!
            if (Settings.RealDrag && air.HasValue && air.Value.Density > 0f)
            {
                RealPhysics.AirDensitySeen = air.Value.Density;
                Aerodynamics.Evaluate(entity, grid, group, in wt, in mass, in rb, bounds, air.Value.Density, Dt,
                    out Vector3 aeroDv, out Vector3 aeroDwLocal, out float dragAccel);
                if (dragAccel > 0f) RealPhysicsMax(ref RealPhysics.DragMaxMilli, dragAccel);
                dv += aeroDv;
                dwLocal += aeroDwLocal;
            }

            // phase 5: gyroscopic torque (tumbling). Skip welded groups AND constrained non-primary subgrids
            // (rotors/hinges/pistons): applying independent free-body tumbling to constrained subgrids fights Havok joints.
            if (!shared.HasValue && rb.AngularVelocity.LengthSquared() > 0.09f)
                RealPhysics.RecordSpin(entity, in mass, WorldTransform.TransformDirectionInv(rb.AngularVelocity, wt));

            if (Settings.RealGyroscopic && !shared.HasValue && !ShipAssembly.IsConstrainedSubgrid(entity, group))
            {
                Vector3 w = rb.AngularVelocity;
                if (w.LengthSquared() > 1e-4f)
                {
                    Vector3 wl = WorldTransform.TransformDirectionInv(w, wt);
                    Vector3 gyroDw = RealPhysics.GyroscopicDelta(in mass, in wl);
                    if (!Vector3.IsZero(gyroDw))
                    {
                        dwLocal += gyroDw;
                        Interlocked.Increment(ref RealPhysics.GyroscopicBodies);
                    }
                }
            }

            if (Vector3.IsZero(dv) && Vector3.IsZero(dwLocal)) return;
            ref RigidBodyData wr = ref body.WritableValue;
            wr.LinearVelocity += dv;
            if (!Vector3.IsZero(dwLocal)) wr.AngularVelocity += WorldTransform.TransformDirection(dwLocal, wt);
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("real: ApplyRealForces error: " + e);
        }
        finally { RealPhysics.MaybeLog(); }
    }

    /// <summary>Airborne characters get the rotating-frame forces too (the controller integrates its own velocity, so only
    /// when not standing on something: CharacterPhysicsState.Supported, HavokSessionComponent.UpdatePhysicsStateFromCharacters).</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [WhenSimulated]
    [MustHave(typeof(DynamicTag))]
    [MustHave(typeof(CharacterInfo))]
    internal static void ApplyCharacterFrameForces(DEntity entity, in WorldTransform wt, in CharacterPhysicsState state, Writable<RigidBodyData> body)
    {
        try
        {
            if (!RealPhysics.On || state.Supported) return;
            if (!RealPhysics.SpinningFrameAt(entity, wt.Position, out Vector3D centre, out Vector3D omega))
            {
                // global space: orbiting planets' pull toward where they are now (RealPhysics.MovingPlanetGravity)
                if (!RealPhysics.InGlobalSpace(entity, wt.Position)) return;
                Vector3D g = RealPhysics.MovingPlanetGravity(wt.Position) + Orbits.StarGravity(wt.Position);
                if (g.LengthSquared() > 1e-10) body.WritableValue.LinearVelocity += (Vector3)(g * Dt);
                return;
            }
            Vector3D v = body.Value.LinearVelocity;
            Vector3D r = wt.Position - centre;
            Vector3D a = -2.0 * Vector3D.Cross(omega, v);
            Vector3D cf = -Vector3D.Cross(omega, Vector3D.Cross(omega, r));
            a += cf;
            if (a.LengthSquared() < 1e-8) return;
            body.WritableValue.LinearVelocity += (Vector3)(a * Dt);
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("real: ApplyCharacterFrameForces error: " + e);
        }
    }

    /// <summary>Phase 4a: with dampeners off (no DampeningData, the same tag thrusters read: ThrustComponent.ComputeThrust),
    /// gyros only act on the axes the pilot is commanding. Runs in the empty vanilla slot between the torque computation
    /// (ControlRotationComputation) and its application (OnApplyAngularImpulse), GridGyroscopesComponent.cs:634-650.
    /// Vanilla ComputeTorqueAngular (:85-130) works on the whole error vector: it scales all three axes by the tightest
    /// per-axis torque budget and eases out on the total error, so zeroing axes afterwards (run 74) let a free spin on one
    /// axis throttle steering on the others, and zeroing impulse components leaks across axes for a non-diagonal inertia.
    /// The torque is recomputed here with the same steps on the commanded axes only; the error is masked in angular
    /// velocity before the inertia is applied, so the velocity change stays on those axes.</summary>
    [GridGyroscopesComponent.OnComputeRotation.DampeningRotationComputation]
    [MustHave(typeof(AngularControlData))]
    [DoesNotHave(typeof(TargetControlData))]
    [DoesNotHave(typeof(DampeningData))]
    internal static void GyroFree(in AngularControlData control, in WorldTransform wt, in RigidBodyData rb, in RigidBodyMassProperties mass,
        in MaxTorqueData maxTorque, in MovementData movement, Writable<GridGyroscopesComponent.ActiveGridGyroscopeData> gyro)
    {
        try
        {
            if (!RealPhysics.On || !Settings.RealGyroFree) return;
            Vector3 t = control.TargetAngularVelocity;
            Vector3 mask = new(t.X == 0f ? 0f : 1f, t.Y == 0f ? 0f : 1f, t.Z == 0f ? 0f : 1f);
            if (mask == Vector3.One) return; // every axis commanded: the vanilla result stands
            Vector3 cur = WorldTransform.TransformDirectionInv(rb.AngularVelocity, wt) * mask;
            Vector3 err = (t - cur) * mask;
            Vector3 result = Vector3.Zero;
            if (!Vector3.IsZero(err) && maxTorque.MaxTorque > 0f)
            {
                var rot = movement.Definition.Value.AngularGyroRotation;
                Vector3 step = err;
                if (t.LengthSquared() < cur.LengthSquared() || Vector3.Dot(t, cur) < 0f)
                    step = Vector3.ClampToSphere(err, rot.MaximumAngularDeceleration * (float)Dt);
                Vector3 impulse = mass.LocalAngularVelocityToAngularImpulseLocal(in step);
                Vector3 returning = new(Returning(cur.X, t.X), Returning(cur.Y, t.Y), Returning(cur.Z, t.Z));
                Vector3 budget = (Vector3.One + (rot.TorqueDampenerPowerMultiplier - 1f) * returning) * new Vector3((float)Dt * maxTorque.MaxTorque);
                float scale = Vector3.Clamp(Vector3.Abs(budget / impulse), Vector3.Zero, Vector3.One).Min();
                result = impulse * scale * GridGyroscopesComponent.ComputeEaseOutMultiplier(err, cur, in rot);
                if (!result.IsValid()) result = Vector3.Zero;
            }
            if (result == gyro.Value.ComputedGyroscopePerFrame) return;
            gyro.WritableValue.ComputedGyroscopePerFrame = result; // zero is skipped by ApplyAngularImpulseAngular (:44-48)
            Interlocked.Increment(ref RealPhysics.GyroFreeGrids);
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("real: GyroFree error: " + e);
        }
    }

    /// <summary>Phase 4a-damp: with dampeners ON, vanilla Grid_MovementDefinition sets TorqueDampenerPowerMultiplier=0.25
    /// (cutting gyro torque to 25% when arresting a spin) and MaximumAngularDeceleration=6.28 rad/s². With AngularDamping=0
    /// and high spin limits (100 rad/s), that makes dampeners feel useless once a ship enters a spin. Restore full 100%
    /// rated gyro torque and up to 25 rad/s² deceleration when dampeners are fighting an unwanted spin.</summary>
    [GridGyroscopesComponent.OnComputeRotation.DampeningRotationComputation]
    [MustHave(typeof(AngularControlData))]
    [MustHave(typeof(DampeningData))]
    [DoesNotHave(typeof(TargetControlData))]
    internal static void GyroSpinRecovery(in AngularControlData control, in WorldTransform wt, in RigidBodyData rb, in RigidBodyMassProperties mass,
        in MaxTorqueData maxTorque, in MovementData movement, Writable<GridGyroscopesComponent.ActiveGridGyroscopeData> gyro)
    {
        try
        {
            if (!RealPhysics.On || maxTorque.MaxTorque <= 0f) return;
            Vector3 cur = WorldTransform.TransformDirectionInv(rb.AngularVelocity, wt);
            Vector3 t = control.TargetAngularVelocity;
            Vector3 err = t - cur;
            // Only boost when actually arresting a spin (returning toward zero or high angular speed > 0.2 rad/s)
            if (err.LengthSquared() < 0.01f) return;
            if (cur.LengthSquared() < 0.04f && t.LengthSquared() >= cur.LengthSquared()) return;

            var rot = movement.Definition.Value.AngularGyroRotation;
            float maxDecel = Math.Max(rot.MaximumAngularDeceleration, 25f);
            Vector3 step = err;
            if (t.LengthSquared() < cur.LengthSquared() || Vector3.Dot(t, cur) < 0f)
                step = Vector3.ClampToSphere(err, maxDecel * Dt);

            Vector3 impulse = mass.LocalAngularVelocityToAngularImpulseLocal(in step);
            // Full 100% rated gyro torque (1.0x instead of vanilla's 0.25x dampener nerf)
            Vector3 budget = new Vector3(Dt * maxTorque.MaxTorque);
            float scale = Vector3.Clamp(Vector3.Abs(budget / impulse), Vector3.Zero, Vector3.One).Min();
            Vector3 result = impulse * scale * GridGyroscopesComponent.ComputeEaseOutMultiplier(err, cur, in rot);
            if (!result.IsValid() || Vector3.IsZero(result)) return;

            if (result.LengthSquared() > gyro.Value.ComputedGyroscopePerFrame.LengthSquared())
                gyro.WritableValue.ComputedGyroscopePerFrame = result;
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("real: GyroSpinRecovery error: " + e);
        }
    }

    // GridGyroscopesComponent.<ComputeTorqueAngular>g__IsReturningToZero|6_1 (:532-537)
    private static float Returning(float cur, float tar)
    {
        float a = float.Sign(cur), b = float.Sign(tar);
        return (a != b && a * b != 0f) || (cur > tar && tar > 0f) || (cur < tar && tar < 0f) ? 1f : 0f;
    }

    /// <summary>Phase 4b: per-thruster torque. Vanilla applies the grid's summed thrust at the centre of mass
    /// (ThrustComponent.ApplyThrust :234-253, ApplyLinearImpulse only). Each direction's thrust is applied at the
    /// thrust-weighted centre of the thrusters firing that way, adding the torque (c - com) × F.
    /// The thrust is not ComputedThrustPerFrame: in gravity, ComputeThrust (:159-173) folds the gravity-holding part into a
    /// reduced CustomGravity and leaves it out of ComputedThrustPerFrame, so a hovering ship got no torque in run 74.
    /// VoluntaryThrustData.VoluntaryThrust is the whole thrust (gravity hold + dampening + input) as a fraction of
    /// MaxThrustData.Regular per direction (UnscaleThrust :381-386). It is scaled back here, clamped to ±1 (vanilla clamps to
    /// the speed-dependent max, which is at least Regular), plus OverriddenThrustData. Runs in OnComputeThrust.End, after
    /// Compute and ApplyThrust of the same tick. Thruster list and directions: GridMovementCollectorComponent.
    /// ThrusterCollectionData (:1648-1700); Direction→axis: Thrust6Directions.GetDirectionValue (Forward=-Z, Backward=+Z,
    /// Left=-X, Right=+X, Up=+Y, Down=-Y). Welded grids are skipped: shared motion already turns each member's linear
    /// impulse into torque about the group centre (MultiEntitySharedMotionSessionComponent.ApplyPendingVelocityChanges).</summary>
    [ThrustComponent.OnComputeThrust.End]
    [WhenSimulated]
    [MustHave(typeof(ThrustComponent.ThrustEnabled))]
    [MustHave(typeof(ThrustComponent.NeedsUpdateThrust))]
    [DoesNotHave(typeof(ConnectedToStaticEntity))]
    internal static void ThrustTorque(in VoluntaryThrustData voluntary, in MaxThrustData max, Optional<OverriddenThrustData> overridden,
        in GridMovementCollectorComponent.ThrusterCollectionData thrusters, in WorldTransform wt, in RigidBodyMassProperties mass,
        Optional<SharedMotionData> shared, EntityData<RelativeTransform> transforms, Writable<RigidBodyData> body)
    {
        try
        {
            if (!RealPhysics.On || !Settings.RealThrustTorque || Settings.RealFlightComputer || shared.HasValue || mass.InvMass <= 0f) return;
            Vector3 frac = Vector3.Clamp(voluntary.VoluntaryThrust, -Vector3.One, Vector3.One);
            Vector3 force = frac * max.Regular.SignSelectAbs(in frac);
            if (overridden.HasValue) force += overridden.Value.DirectionalThrust;
            if (!force.IsValid() || Vector3.IsZero(force)) return;
            Vector3 f = force * (float)Dt; // local impulse this tick
            Span<Vector3> sum = stackalloc Vector3[6];
            Span<float> w = stackalloc float[6];
            // Weight each thruster by its real output: CurrentMaxThrust is raw, the class efficiency (atmospheric thrusters in
            // vacuum, ion in air) is applied per class (GridMovementCollectorComponent.cs:248-249).
            int total = thrusters.Thrusters.Count;
            for (int c = 0; c < thrusters.ThrustClasses.Count; c++)
            {
                var cls = thrusters.ThrustClasses[c];
                float eff = cls.Efficiency;
                if (eff <= 1e-4f) continue;
                int end = Math.Min(cls.StartIndex + cls.Count, total);
                for (int k = cls.StartIndex; k < end; k++)
                {
                    var t = thrusters.Thrusters[k];
                    float thrustN = t.CurrentMaxThrust * eff;
                    if (thrustN <= 0f || !transforms.TryReadFrom(t.Thruster, out RelativeTransform rt)) continue;
                    int i = Slot(t.Direction);
                    sum[i] += rt.Position * thrustN;
                    w[i] += thrustN;
                }
            }
            Vector3 angular = Vector3.Zero;
            Vector3 com = mass.CenterOfMass;
            Accumulate(f.X, 0, 1, new Vector3(1, 0, 0), sum, w, com, ref angular);
            Accumulate(f.Y, 2, 3, new Vector3(0, 1, 0), sum, w, com, ref angular);
            Accumulate(f.Z, 4, 5, new Vector3(0, 0, 1), sum, w, com, ref angular);
            if (Vector3.IsZero(angular)) return;
            body.WritableValue.ApplyAngularImpulseLocal(in mass, in wt, in angular);
            Interlocked.Increment(ref RealPhysics.ThrustTorqueGrids);
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("real: ThrustTorque error: " + e);
        }
    }

    // slots: 0 +X(Right) 1 -X(Left) 2 +Y(Up) 3 -Y(Down) 4 +Z(Backward) 5 -Z(Forward)
    private static int Slot(Base6Directions.Direction d) => d switch
    {
        Base6Directions.Direction.Right => 0,
        Base6Directions.Direction.Left => 1,
        Base6Directions.Direction.Up => 2,
        Base6Directions.Direction.Down => 3,
        Base6Directions.Direction.Backward => 4,
        _ => 5,
    };

    private static void Accumulate(float component, int pos, int neg, Vector3 axis, Span<Vector3> sum, Span<float> w, Vector3 com, ref Vector3 angular)
    {
        if (component == 0f) return;
        int s = component > 0f ? pos : neg;
        if (w[s] <= 0f) return;
        Vector3 at = sum[s] / w[s];
        angular += Vector3.Cross(at - com, axis * component);
    }

    private static void RealPhysicsMax(ref long slot, double value)
    {
        if (!double.IsFinite(value) || value <= 0.0) return;
        long v = (long)Math.Min(value * 1000.0, 1e12);
        long cur;
        while (v > (cur = Interlocked.Read(ref slot)) && Interlocked.CompareExchange(ref slot, v, cur) != cur) { }
    }
}

/// <summary>Phase 8: client extrapolation cap. MotionStateApplicationSystem.RecordMotionState blends ahead by at most
/// MotionInterpolationConfiguration.MaxExtrapolation (default 5 s, MotionInterpolationConfiguration.cs:95); at orbital speed
/// a stalled stream would carry a body kilometres past where it is. Init(builder) copies the value (:164).</summary>
internal static class RealExtrapolation
{
    internal static void Install(Harmony harmony)
    {
        var t = AccessTools.TypeByName("Keen.VRage.Multiplayer.Replications.MotionInterpolationConfiguration");
        var ob = AccessTools.TypeByName("Keen.VRage.Multiplayer.Replications.MotionInterpolationConfigurationObjectBuilder");
        var init = t == null || ob == null ? null : AccessTools.Method(t, "Init", new[] { ob });
        if (init == null) { ProbeLog.Write("real: MotionInterpolationConfiguration.Init not found"); return; }
        harmony.Patch(init, postfix: new HarmonyMethod(typeof(RealExtrapolation), nameof(Postfix)));
        ProbeLog.Write("real: extrapolation cap patch installed");
    }

    private static void Postfix(object __instance)
    {
        try
        {
            var p = AccessTools.Property(__instance.GetType(), "MaxExtrapolation");
            object? old = p.GetValue(__instance);
            p.SetValue(__instance, TimeSpan.FromSeconds(Settings.RealMaxExtrapolation));
            ProbeLog.Write($"real: MaxExtrapolation {old} -> {Settings.RealMaxExtrapolation} s");
        }
        catch (Exception e) { ProbeLog.Write("real: extrapolation cap failed: " + e.Message); }
    }
}
