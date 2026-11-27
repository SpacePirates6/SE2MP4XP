using System;
using System.Collections.Generic;
using System.Text;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Character;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Havok;
using Keen.VRage.Physics.Havok.Character;
using Keen.VRage.Physics.Havok.Core;
using Keen.VRage.Voxels.Components;

namespace SpinProbe;

/// <summary>
/// Plugin-owned DCS jobs, registered by <see cref="ProbeConfigurator"/> through SceneBuilder.AddJobsFromType and compiled
/// by the shipped VRage.DCS.Generator (referenced as an analyzer in SpinProbe.csproj).
/// <para>Placement: <c>PhysicsUpdate.OnCollectMassProperties</c> (VRage.Physics\...\Havok\PhysicsUpdate.cs:156) runs after
/// OnCollectDataForBodies and before OnCollectChangesForDynamicBodies, so a WorldTransform written here is collected by
/// CollectChangesForTransform (HavokSessionComponent.cs:1748, OnChanged(WorldTransform)+DynamicTag) in the SAME step and
/// handed to DriveBodyTo by CollectDataForTransformDrivenKinematicBodies (:1811-1821). A different job group means a
/// different DataEventGroup, so the write does trigger those OnChanged listeners (Scene.cs:5220-5223).</para>
/// </summary>
internal static class ProbeJobs
{
    internal sealed class Spin
    {
        internal WorldTransform Start;
        internal long Frames;
        internal WorldTransform Current;
    }

    internal static readonly Dictionary<DEntity, Spin> Spins = new();
    private static readonly HashSet<DEntity> NotPlanets = new();
    private static readonly HashSet<VoxelPhysicsComponent> DeadComponents = new(ReferenceEqualityComparer.Instance);
    internal static readonly Vector3 AxisLocal = new(0f, 1f, 0f); // planet-local pole
    internal static double Omega => Settings.DegreesPerSecond * Math.PI / 180.0;

    /// <summary>Server ticks, counted once per tick by Profiler.TickPostfix (run 2 counted once per planet; run 3's Tick job ran once per ENTITY).</summary>
    internal static long Ticks => Profiler.Ticks;
    private static long _lastSampleTick = -1;
    private static bool _tuned;
    private static bool _firstRunLogged;
    private static int _errors;

    /// <summary>Turns each planet and writes every kinematic chunk to planetWT(t) * offset.</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    internal static void DrivePlanets(VoxelPhysicsComponent physics, EntityData<WorldTransform> transforms, EntityData<BodyProxy> proxies)
    {
        try
        {
            if (Settings.Vanilla || physics == null || Settings.Mode != "kinematic") return;
            if (!_firstRunLogged)
            {
                _firstRunLogged = true;
                ProbeLog.Write("T1: PASS - plugin job DrivePlanets is running inside PhysicsUpdate.OnCollectMassProperties");
            }
            // Source: RigidBodyComponent exposes its entity id as DEntity (VRage.Physics\...\Components\RigidBodyComponent.cs:108-113).
            // Run 4: physics.DEntity threw NullReferenceException every tick for a component being torn down - and an
            // exception per tick is expensive. Remember such components instead of throwing again.
            if (DeadComponents.Contains(physics)) return;
            DEntity planet;
            try { planet = physics.DEntity; }
            catch (NullReferenceException) { DeadComponents.Add(physics); return; }
            if (NotPlanets.Contains(planet)) return;
            if (!Spins.TryGetValue(planet, out Spin? spin))
            {
                // Run 2 crashed here: physics.Entity was null for a component being added/removed. Only real planets
                // (VRage.Voxels\...\Components\VoxelPlanetComponent.cs) are driven.
                var entity = physics.Entity;
                if (entity == null) return;
                if (entity.TryGet<VoxelPlanetComponent>() == null) { NotPlanets.Add(planet); return; }
                spin = new Spin { Start = transforms.ReadFrom(planet) };
                Spins[planet] = spin;
                ProbeLog.Write($"E1: driving planet {planet} at {Settings.DegreesPerSecond} deg/s about its local Y; start {spin.Start.Position}");
            }

            spin.Frames++;
            // Fixed 1/60 s sim step: Game2.Simulation\...\UpdateTime.cs:36; physics StepFrequency 60 (WorldSessionComponent.cs:498-507).
            double theta = Omega * spin.Frames / 60.0;
            WorldTransform wt = spin.Start;
            wt.Orientation = Quaternion.Normalize(spin.Start.Orientation * Quaternion.CreateFromAxisAngle(AxisLocal, (float)theta));
            spin.Current = wt;
            transforms.WriteTo(planet, wt);

            if (ChunkPatch.Chunks.TryGetValue(planet, out var chunks))
            {
                List<DEntity>? dead = null;
                foreach (var pair in chunks)
                {
                    if (!transforms.TryWriteTo(pair.Key, wt * new RelativeTransform(pair.Value)))
                        (dead ??= new List<DEntity>()).Add(pair.Key);
                }
                if (dead != null)
                    foreach (DEntity d in dead) chunks.Remove(d);
            }
            if (Ticks % 60 == 0 && Ticks != _lastSampleTick)
            {
                _lastSampleTick = Ticks;
                SampleChunks(proxies);
            }
        }
        catch (Exception e)
        {
            if (_errors++ < 5) ProbeLog.Write("DrivePlanets error: " + e);
        }
    }

    /// <summary>
    /// One-shot: raise the DriveBodyTo teleport thresholds. Source: HavokSessionComponent.cs:740-746 builds
    /// `_bodyDrivingSettings` from Settings.MinDrivingTeleportDistance (5.5 m) etc., and :1820 passes it to every DriveBodyTo.
    /// HavokSessionComponent is a [JobContext] (run 3: a job taking it ran once per ENTITY), so the job is bound to planets
    /// with MustHave to keep it to a handful of calls per tick.
    /// </summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [Keen.VRage.DCS.Annotations.MustHave(typeof(Keen.VRage.DCS.CoreData.InstanceBind<VoxelPlanetComponent>))]
    internal static void TuneDriving(HavokSessionComponent havok)
    {
        if (_tuned || !Settings.RaiseTeleportThreshold || Settings.Vanilla || Settings.Mode != "kinematic") return;
        _tuned = true;
        var s = havok._bodyDrivingSettings;
        ProbeLog.Write($"E1: driving settings were teleportDist^2={s.m_minTeleportDistanceSquared} teleportAngle={s.m_minTeleportAngleRadians}");
        s.m_minTeleportDistanceSquared = 1000f * 1000f;
        s.m_minTeleportAngleRadians = 3.0f;
        havok._bodyDrivingSettings = s;
        ProbeLog.Write("E1: raised DriveBodyTo teleport thresholds to 1000 m / 3 rad");
    }

    /// <summary>E1 directly: read three driven chunks' Havok motion and compare with omega x r.</summary>
    private static unsafe void SampleChunks(EntityData<BodyProxy> proxies)
    {
        foreach (var planet in Spins)
        {
            if (!ChunkPatch.Chunks.TryGetValue(planet.Key, out var chunks) || chunks.Count == 0) continue;
            Vector3 omegaWorld = Vector3.Transform(AxisLocal, planet.Value.Current.Orientation) * (float)Omega;
            int n = 0;
            var sb = new StringBuilder();
            foreach (var pair in chunks)
            {
                if (n >= 3) break;
                if (!proxies.TryReadFrom(pair.Key, out BodyProxy proxy) || proxy.World == null) continue;
                WorldTransform cwt = default;
                RigidBodyData crb = default;
                // Source: VRage.Physics\...\Havok\Core\HknpWorld.cs:108-117 GetBodyInfo - the Havok body's transform and its
                // motion's linear/angular velocity.
                HknpWorld.GetBodyInfo(proxy.World, pair.Key, ref cwt, ref crb);
                WorldTransform target = planet.Value.Current * new RelativeTransform(pair.Value);
                // A keyframed motion's linear velocity is the velocity of its centre of mass; compare with omega x r at the
                // body origin (the chunk body origin is the sector centre).
                Vector3 exp = Vector3.Cross(omegaWorld, (Vector3)(cwt.Position - planet.Value.Current.Position));
                double err = (cwt.Position - target.Position).Length();
                sb.Append($" [{pair.Key} lin={crb.LinearVelocity.Length():0.00} exp={exp.Length():0.00} ang={crb.AngularVelocity.Length():0.000000} expAng={Omega:0.000000} poseErr={err:0.000}m]");
                n++;
            }
            if (n > 0)
                ProbeLog.Write($"E1 chunks: t={Ticks / 60.0:0.0}s planet {planet.Key} chunks={chunks.Count}{sb}");
        }
    }

    private static readonly Dictionary<DEntity, Vector3D> Anchors = new();
    private static long _reportTick = -1;
    private static int _reportedThisTick;

    /// <summary>
    /// Every character, once a second: is it standing on a driven chunk, what support velocity Havok reports, its own
    /// velocity against omega x r, and its drift in planet-local coordinates (E3).
    /// Placement: PhysicsUpdate.OnUpdateCharacterPhysicsState (PhysicsUpdate.cs:325) runs after
    /// UpdateSupportDataFromCharacters (HavokSessionComponent.cs:421-463), which fills CharacterSupportData.
    /// </summary>
    [PhysicsUpdate.OnUpdateCharacterPhysicsState]
    internal static void ReportCharacter(DEntity character, in WorldTransform wt, in CharacterSupportData support,
        in RigidBodyData body, CharacterInfo info)
    {
        try
        {
            if (Settings.Vanilla || Ticks % 60 != 0 || Ticks == 0) return;
            if (Settings.Mode == "frame")
            {
                if (!Frame.Active || !Frame.Inside(wt.Position)) return;
                WorldTransform f = wt;
                Frame.ToFrame(ref f, Frame.ThetaForWorld);
                if (!Anchors.TryGetValue(character, out Vector3D a)) Anchors[character] = a = f.Position;
                DEntity sup = default;
                foreach (ref readonly CharacterSupportData.Support c in support.CharacterContacts) if (c.IsSupportContact) { sup = c.Entity; break; }
                Vector3 exp = Frame.CoRotating(wt.Position);
                ProbeLog.Write($"FRAME char {character}: t={Ticks / 60.0:0.0}s support={sup} |charV|={body.LinearVelocity.Length():0.00} |omega x r|={exp.Length():0.00} " +
                               $"v.(omega x r)/|omega x r|^2={Vector3.Dot(body.LinearVelocity, exp) / Math.Max(exp.LengthSquared(), 1e-9f):0.00} " +
                               $"surfaceV={support.SupportingSurface.SurfaceLinearVelocity.Length():0.00} planetFrameDrift={(f.Position - a).Length():0.00} m");
                return;
            }
            if (Spins.Count == 0) return;
            if (_reportTick != Ticks) { _reportTick = Ticks; _reportedThisTick = 0; }

            DEntity planetId = default;
            WorldTransform planet = default;
            double best = double.MaxValue;
            foreach (var pair in Spins)
            {
                double d = (pair.Value.Current.Position - wt.Position).Length();
                if (d < best) { best = d; planet = pair.Value.Current; planetId = pair.Key; }
            }

            bool onChunk = false;
            DEntity supportEntity = default;
            foreach (ref readonly CharacterSupportData.Support c in support.CharacterContacts)
            {
                if (!c.IsSupportContact) continue;
                supportEntity = c.Entity;
                if (ChunkPatch.Chunks.TryGetValue(planetId, out var chunks) && chunks.ContainsKey(c.Entity)) { onChunk = true; break; }
            }
            if (!onChunk && _reportedThisTick >= 2) return; // always report chunk-supported characters, plus two others
            _reportedThisTick++;

            Vector3D r = wt.Position - planet.Position;
            Vector3 omegaWorld = Vector3.Transform(AxisLocal, planet.Orientation) * (float)Omega;
            Vector3 expected = Vector3.Cross(omegaWorld, (Vector3)r);
            Vector3D local = WorldTransform.GetRelativeTransform(wt, planet).Position;
            if (!Anchors.TryGetValue(character, out Vector3D anchor)) Anchors[character] = anchor = local;
            double drift = (local - anchor).Length();

            ProbeLog.Write($"E1 char {character}: t={Ticks / 60.0:0.0}s onDrivenChunk={onChunk} support={supportEntity} " +
                           $"|r|={r.Length():0} |omega x r|={expected.Length():0.00} surfaceV|={support.SupportingSurface.SurfaceLinearVelocity.Length():0.00} " +
                           $"surfaceDynamic={support.SupportingSurface.IsSurfaceDynamic} |charV|={body.LinearVelocity.Length():0.00} " +
                           $"charV.expected={Vector3.Dot(body.LinearVelocity, expected) / Math.Max(expected.LengthSquared(), 1e-9f):0.00} " +
                           $"planetLocalDrift={drift:0.00} m");
        }
        catch (Exception e)
        {
            if (_errors++ < 5) ProbeLog.Write("ReportCharacter error: " + e);
        }
    }

    private static readonly Dictionary<DEntity, Vector3D> FloatAnchors = new();
    private static long _floatTick = -1;
    private static int _floatsThisTick;

    /// <summary>
    /// Carry test without a character: dynamic floating objects (crash-site debris) resting on driven terrain. If kinematic
    /// terrain carries them by contact (like a moving ship carries cargo), their planet-local position stays put; if not,
    /// it drifts at |omega x r| (~21.5 m/s here). Source: Game2.Simulation\...\FloatingObjects\FloatingObjectComponent.cs:44.
    /// </summary>
    [PhysicsUpdate.OnUpdateCharacterPhysicsState]
    [Keen.VRage.DCS.Annotations.MustHave(typeof(Keen.VRage.DCS.CoreData.InstanceBind<Keen.Game2.Simulation.WorldObjects.FloatingObjects.FloatingObjectComponent>))]
    internal static void ReportFloating(DEntity entity, in WorldTransform wt, in RigidBodyData body)
    {
        try
        {
            if (Settings.Vanilla || Ticks % 120 != 0 || Ticks == 0) return;
            if (Settings.Mode == "frame")
            {
                if (!Frame.Active || !Frame.Inside(wt.Position)) return;
                if (_floatTick != Ticks) { _floatTick = Ticks; _floatsThisTick = 0; }
                if (_floatsThisTick++ >= 6) return;
                WorldTransform f = wt;
                Frame.ToFrame(ref f, Frame.ThetaForWorld);
                if (!FloatAnchors.TryGetValue(entity, out Vector3D a)) FloatAnchors[entity] = a = f.Position;
                Vector3 exp = Frame.CoRotating(wt.Position);
                ProbeLog.Write($"FRAME float {entity}: t={Ticks / 60.0:0.0}s theta={Frame.ThetaEcs * 180 / Math.PI:0.000}deg |v|={body.LinearVelocity.Length():0.00} " +
                               $"|omega x r|={exp.Length():0.00} v.(omega x r)/|omega x r|^2={Vector3.Dot(body.LinearVelocity, exp) / Math.Max(exp.LengthSquared(), 1e-9f):0.00} " +
                               $"planetFrameDrift={(f.Position - a).Length():0.00} m");
                return;
            }
            if (Spins.Count == 0) return;
            if (_floatTick != Ticks) { _floatTick = Ticks; _floatsThisTick = 0; }
            if (_floatsThisTick++ >= 6) return;
            WorldTransform planet = default; double best = double.MaxValue;
            foreach (var pair in Spins)
            {
                double d = (pair.Value.Current.Position - wt.Position).Length();
                if (d < best) { best = d; planet = pair.Value.Current; }
            }
            Vector3 omegaWorld = Vector3.Transform(AxisLocal, planet.Orientation) * (float)Omega;
            Vector3 expected = Vector3.Cross(omegaWorld, (Vector3)(wt.Position - planet.Position));
            Vector3D local = WorldTransform.GetRelativeTransform(wt, planet).Position;
            if (!FloatAnchors.TryGetValue(entity, out Vector3D anchor)) FloatAnchors[entity] = anchor = local;
            ProbeLog.Write($"E1 float {entity}: t={Ticks / 60.0:0.0}s |v|={body.LinearVelocity.Length():0.00} |omega x r|={expected.Length():0.00} " +
                           $"v.(omega x r)/|omega x r|^2={Vector3.Dot(body.LinearVelocity, expected) / Math.Max(expected.LengthSquared(), 1e-9f):0.00} " +
                           $"planetLocalDrift={(local - anchor).Length():0.00} m");
        }
        catch (Exception e)
        {
            if (_errors++ < 5) ProbeLog.Write("ReportFloating error: " + e);
        }
    }
}
