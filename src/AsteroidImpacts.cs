using System;
using System.Collections.Generic;
using HarmonyLib;
using Keen.Game2.Simulation.WorldObjects.CubeGrids;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Systems;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.Scenes;
using Keen.VRage.Library.Definitions;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Library.Utils;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Materials;
using Keen.VRage.Voxels.Components;
using Keen.VRage.Voxels.Definitions;
using Keen.VRage.Voxels.Storage.Operations;

namespace SpinProbe;

/// <summary>
/// Fixes vanilla VoxelDestructionComponent on moving/rotating asteroids: Init() caches _worldToStorageCenter once
/// at spawn (VoxelDestructionComponent.cs:48), so any impact cutout after the asteroid moves would carve at the stale
/// spawn pose. Refreshing it from live ComputeCenteredTransform() before RunCapsuleDestruction keeps cuts aligned.
/// </summary>
[HarmonyPatch(typeof(VoxelDestructionComponent), "RunCapsuleDestruction")]
internal static class VoxelDestructionRefreshPatch
{
    [HarmonyPrefix]
    private static void Prefix(VoxelDestructionComponent __instance)
    {
        if (__instance._storage == null) return;
        __instance._worldToStorageCenter = WorldTransform.GetRelativeTransform(
            WorldTransform.Identity, __instance._storage.ComputeCenteredTransform());
    }
}

/// <summary>
/// Density-based voxel impact cratering &amp; catastrophic shattering for movable asteroids (and planets/static voxels
/// struck by movable asteroids). Runs on the server after physics simulation whenever a movable asteroid chunk reports
/// high-impulse contacts via ContactImpulseData.
/// </summary>
internal static class AsteroidImpactJobs
{
    private const float MinImpactSpeed = 10.0f;         // m/s normal speed to excavate rock (gentle bumps bounce elastically)
    private const float MinImpactImpulse = 50000f;      // N*s impulse threshold (filters walking / light scrapes)
    private static readonly long CooldownTicks = TimeSpan.FromMilliseconds(350).Ticks;
    private static readonly long SettleGraceTicks = TimeSpan.FromSeconds(3).Ticks;

    private static Dictionary<MaterialId, (string name, double density, int hardness)>? _matByPhysicsId;
    private static readonly object MatLock = new();
    private static int _errors;

    private static (string name, double density, int hardness) LookupPhysicsMaterial(MaterialId id, double fallbackDensity)
    {
        if (_matByPhysicsId == null)
        {
            lock (MatLock)
            {
                if (_matByPhysicsId == null)
                {
                    var map = new Dictionary<MaterialId, (string, double, int)>();
                    try
                    {
                        foreach (var def in Singleton<DefinitionManager>.Instance.GetDefinitionsOfType<VoxelMaterialDefinition>())
                        {
                            var pm = def.PhysicsMaterial;
                            if (pm == null || map.ContainsKey(pm.MaterialId)) continue;
                            string name = def.Name.ToString().Trim();
                            double d = MovableAsteroidMass.DensityForName(name, Settings.AsteroidDensity);
                            int h = Math.Clamp(pm.Hardness, 1, 10);
                            map[pm.MaterialId] = (name, d, h);
                        }
                    }
                    catch { }
                    _matByPhysicsId = map;
                }
            }
        }
        if (_matByPhysicsId.TryGetValue(id, out var hit)) return hit;
        int fallbackHardness = fallbackDensity < 1500 ? 2 : fallbackDensity > 4500 ? 7 : 5;
        return ("rock", fallbackDensity, fallbackHardness);
    }

    [OnAfterSimulation]
    [OnChanged(typeof(ContactImpulseData))]
    internal static void OnChunkContact(DEntity entity, VoxelPhysicsComponent physics, in ContactImpulseData contacts)
    {
        if (!MovableAsteroids.On || !Settings.AsteroidImpacts || physics == null) return;
        if (!MovableAsteroids.States.TryGetValue(physics, out MovableAsteroids.State? s) || s.IsClient) return;
        if (contacts.ImpulsesFromLastFrame.Count == 0) return;

        long now = DateTime.UtcNow.Ticks;
        // Source: none (plugin-only). Stage 1.1: any reported contact (impulse >= MinimalRequiredImpulse 5000 N*s) with a grid,
        // a character or an already player-touched asteroid marks this asteroid as player-touched (then Watch promotes it).
        if (!s.PlayerTouched)
        {
            try
            {
                Scene sc = physics.Entity.Scene;
                foreach (ref ContactImpulseData.Contact c in contacts.ImpulsesFromLastFrame)
                {
                    string? why = PlayerCause(sc, c.OtherBody);
                    if (why != null) { MovableAsteroids.MarkPlayerTouched(s, why); break; }
                }
            }
            catch { }
        }
        if (now < s.ImpactImmuneUntilTicks) return;   // fresh shard separating from its siblings
        if (now - s.LastImpactTicks < CooldownTicks) return;
        // Skip spontaneous world-load settling overlaps on naturally spawned asteroids (pasted/thrown asteroids arm immediately).
        if (!s.IsPasted && now - s.SpawnTicks < SettleGraceTicks) return;

        try
        {
            // Find the strongest qualifying contact on this body this tick.
            // Verify real relative speed from RigidBodyData too so resting/stuck penetration recovery (e.g. a
            // 0.1 m/s asteroid resting in a cluster producing a 28.9 m/s Havok depenetration spike) never craters.
            Scene scene = physics.Entity.Scene;
            Vector3 selfVel = physics.Data.TryGet<RigidBodyData>(out var selfRb) ? selfRb.LinearVelocity : Vector3.Zero;
            ContactImpulseData.Contact best = default;
            double bestEnergy = 0;
            float bestSpeed = 0f;
            Entity? bestOtherEntity = null;
            foreach (ref ContactImpulseData.Contact c in contacts.ImpulsesFromLastFrame)
            {
                float vn = c.SeparationSpeedAlongNormalAbsolute;
                float j = c.ImpulseMagnitude;
                if (vn < MinImpactSpeed || j < MinImpactImpulse || !float.IsFinite(vn) || !float.IsFinite(j)) continue;

                Entity? other = null;
                Vector3 otherVel = Vector3.Zero;
                double mOther = double.PositiveInfinity;
                try
                {
                    if (!c.OtherBody.IsEmpty)
                    {
                        DEntityContext otherCtx = new DEntityContext(scene, c.OtherBody);
                        other = Entity.TryGetFromDataEntity(otherCtx);
                        if (other != null)
                        {
                            if (other.Data.TryGet<RigidBodyData>(out var orb)) otherVel = orb.LinearVelocity;
                            var ov = other.TryGet<VoxelPhysicsComponent>();
                            if (ov != null)
                            {
                                // Movable asteroid -> use its real voxel mass; static asteroid/planet -> infinite mass.
                                if (MovableAsteroids.States.TryGetValue(ov, out var ost) && ost.Mass > 1f)
                                    mOther = ost.Mass;
                                else
                                    mOther = double.PositiveInfinity;
                            }
                            else if (other.Data.TryGet<RigidBodyMassProperties>(out var emp) && emp.InvMass > 1e-12f)
                                mOther = 1.0 / emp.InvMass;
                        }
                        else if (otherCtx.TryGet<RigidBodyData>(out var crb))
                        {
                            otherVel = crb.LinearVelocity;
                            if (otherCtx.TryGet<RigidBodyMassProperties>(out var cmp) && cmp.InvMass > 1e-12f)
                                mOther = 1.0 / cmp.InvMass;
                        }
                    }
                }
                catch { }

                // Require real body velocity (not a stationary depenetration spike!) to exceed half MinImpactSpeed.
                float relSpeed = (selfVel - otherVel).Length();
                float maxVel = Math.Max(selfVel.Length(), otherVel.Length());
                float realSpeed = Math.Max(relSpeed, maxVel);
                if (realSpeed < MinImpactSpeed * 0.6f) continue;

                double effectiveSpeed = Math.Min(Math.Min(vn, realSpeed * 1.5 + 5.0), 500.0);
                double mSelf = s.Mass > 1f ? s.Mass : 1e5;
                double mEff = double.IsInfinity(mOther) ? mSelf : (mSelf * mOther) / (mSelf + mOther);
                double kePhys = 0.5 * mEff * effectiveSpeed * effectiveSpeed;
                double keImp = 0.5 * j * effectiveSpeed;
                double ek = Math.Min(keImp, kePhys);
                if (ek > bestEnergy) { bestEnergy = ek; best = c; bestSpeed = (float)effectiveSpeed; bestOtherEntity = other; }
            }
            if (bestEnergy <= 0) return;

            s.LastImpactTicks = now;

            // Identify impactor entity & density.
            Entity? otherEntity = bestOtherEntity;

            double impactorDensity = Settings.AsteroidDensity;
            VoxelPhysicsComponent? otherVoxel = null;
            if (otherEntity != null)
            {
                otherVoxel = otherEntity.TryGet<VoxelPhysicsComponent>();
                if (otherVoxel != null && MovableAsteroids.States.TryGetValue(otherVoxel, out var os))
                    impactorDensity = os.AvgDensity > 0 ? os.AvgDensity : Settings.AsteroidDensity;
                else if (otherEntity.TryGet<CubeGridComponent>() != null)
                    impactorDensity = 3500.0; // steel/armor ship grid
            }

            double targetFallbackDensity = s.AvgDensity > 0 ? s.AvgDensity : Settings.AsteroidDensity;
            var (matName, targetDensity, hardness) = LookupPhysicsMaterial(best.CurrentBodyMaterial, targetFallbackDensity);

            // Inward direction into THIS asteroid: from contact point toward asteroid centre, blended with impact normal.
            Vector3D asteroidCenter = physics.Data.GetWorldTransform().Position;
            Vector3D toCenter = asteroidCenter - best.Position;
            Vector3 inwardWorld = toCenter.LengthSquared() > 1e-4
                ? (Vector3)Vector3D.Normalize(toCenter)
                : (best.Normal.LengthSquared() > 1e-4 ? -Vector3.Normalize(best.Normal) : Vector3.UnitY);

            string otherName = otherEntity?.ToString() ?? best.OtherBody.ToString();
            // Source: none (plugin-only). Stage 2.1: three exclusive regimes - bounce (filtered above), crater (e < Q*),
            // catastrophic shatter (e >= Q*, AsteroidShatter) which replaces the crater on this asteroid. Q* is a whole-body
            // strength, so it uses the asteroid's bulk density, not the struck voxel's material.
            bool shattering = false;
            if (Settings.AsteroidShatter && s.Mass > 1000f)
            {
                double bulk = s.AvgDensity > 0 ? s.AvgDensity : targetDensity;
                double e = bestEnergy / s.Mass;
                double qStar = AsteroidShatter.QStar(bulk);
                if (e >= qStar)
                    shattering = AsteroidShatter.TryQueue(physics, s, best.Position, inwardWorld, bestEnergy, e, qStar, bulk, bestSpeed, otherName);
            }
            if (!shattering)
                CarveImpact(physics.Entity, physics._storage, s, best.Position, inwardWorld, bestEnergy,
                    bestSpeed, matName, targetDensity, hardness, impactorDensity, now, otherName);

            // If we slammed into a static voxel (planet or static asteroid), carve a crater into it too!
            if (otherEntity != null && otherVoxel != null && !MovableAsteroids.States.TryGetValue(otherVoxel, out _))
            {
                var (otherMat, otherDensity, otherHardness) = LookupPhysicsMaterial(best.OtherBodyMaterial, Settings.AsteroidDensity);
                Vector3 otherInward = -inwardWorld;
                CarveImpact(otherEntity, otherVoxel._storage, null, best.Position, otherInward, bestEnergy,
                    bestSpeed, otherMat, otherDensity, otherHardness, targetDensity, now, s.Name);
            }
        }
        catch (Exception ex)
        {
            if (_errors++ < 8) ProbeLog.Write("asteroid: impact handler error: " + ex.Message);
        }
    }

    // Reads: Game2.Simulation/Keen/Game2/Simulation/WorldObjects/CubeGrids/CubeGridComponent; Characters/CharacterComponent.cs:30;
    // VRage.DCS/Keen/VRage/DCS/CoreData/InstanceBind.cs (chunk / part bodies bind to their owner entity, as our asteroid chunks do).
    private static string? PlayerCause(Scene scene, DEntity body)
    {
        if (body.IsEmpty) return null;
        var ctx = new DEntityContext(scene, body);
        Entity? other = Entity.TryGetFromDataEntity(ctx);
        if (other == null && ctx.TryGet<Keen.VRage.DCS.CoreData.InstanceBind<Entity>>(out var bind)) other = bind.Instance;
        if (other == null) return null;
        if (other.TryGet<CubeGridComponent>() != null) return "contact with grid " + other;
        if (other.TryGet<Keen.Game2.Simulation.WorldObjects.Characters.CharacterComponent>() != null) return "contact with a character";
        var ov = other.TryGet<VoxelPhysicsComponent>();
        if (ov != null && MovableAsteroids.States.TryGetValue(ov, out var os) && os.PlayerTouched && !os.IsClient)
            return "contact with touched asteroid " + os.Name;
        return null;
    }

    private static void CarveImpact(
        Entity targetEntity,
        VoxelStorageComponentBase storage,
        MovableAsteroids.State? state,
        Vector3D worldHitPos,
        Vector3 inwardWorldDir,
        double energyJ,
        float impactSpeed,
        string matName,
        double targetDensity,
        int hardness,
        double impactorDensity,
        long nowTicks,
        string impactorName)
    {
        // Effective resistance combines density and Mohs hardness: ice craters much larger than iron/gold.
        // Scaled against real reduced-mass KE: ~15 m/s rock hit -> ~4-6 m crater, ~50 m/s -> ~10-14 m, ~100 m/s -> ~18 m + shatter.
        double resistance = Math.Max(400.0, targetDensity * (1.0 + 0.4 * hardness));
        float craterRadius = (float)Math.Clamp(0.14 * Math.Cbrt(energyJ / resistance), 1.2, 18.0);

        // Density-ratio penetration depth: dense iron punches deep into ice; ice splatters shallow on iron.
        float depthRatio = (float)Math.Clamp(Math.Sqrt(impactorDensity / Math.Max(300.0, targetDensity)), 0.5, 2.2);
        float depth = Math.Clamp(craterRadius * depthRatio, 1.0f, 26.0f);

        // Convert world hit & inward direction into the voxel entity's live storage-centred space.
        WorldTransform centered = storage.ComputeCenteredTransform();
        WorldTransform worldToStorage = WorldTransform.GetRelativeTransform(WorldTransform.Identity, centered);
        Vector3 hitLocal = (Vector3)WorldTransform.Transform(in worldHitPos, in worldToStorage);
        Vector3 inwardLocal = Vector3.Transform(inwardWorldDir, worldToStorage.Orientation);
        if (inwardLocal.LengthSquared() < 1e-4f) inwardLocal = Vector3.UnitY;
        else inwardLocal = Vector3.Normalize(inwardLocal);
        CarveLocal(targetEntity, storage, state, hitLocal, inwardLocal, craterRadius, depth, energyJ, impactSpeed, matName,
            targetDensity, hardness, impactorDensity, nowTicks, impactorName);
    }

    /// <summary>Crater radius / depth from impact energy, densities and hardness (shared by live and off-screen impacts).</summary>
    // Source: none (plugin-only) - the sizing CarveImpact always used.
    internal static void CraterSize(double energyJ, double targetDensity, int hardness, double impactorDensity, out float craterRadius, out float depth)
    {
        double resistance = Math.Max(400.0, targetDensity * (1.0 + 0.4 * hardness));
        craterRadius = (float)Math.Clamp(0.14 * Math.Cbrt(energyJ / resistance), 1.2, 18.0);
        float depthRatio = (float)Math.Clamp(Math.Sqrt(impactorDensity / Math.Max(300.0, targetDensity)), 0.5, 2.2);
        depth = Math.Clamp(craterRadius * depthRatio, 1.0f, 26.0f);
    }

    /// <summary>
    /// Carves the crater with the hit point already in the storage CORNER frame (voxel coordinates). The capsules given to
    /// VoxelDestructionComponent.OnSphereDestruction are storage-local (ApplyCapsulesDestruction :157-160, [Server][Clients]);
    /// only the engine's own impulse path (RunCapsuleDestruction :103-107) maps world capsules through _worldToStorageCenter.
    /// So a railed asteroid whose written pose is seconds old (stage 3.4 off-screen impacts) is carved correctly from its
    /// analytic local hit point.
    /// </summary>
    // Reads: VRage.Voxels/Keen/VRage/Voxels/Components/VoxelDestructionComponent.cs :62-160.
    internal static void CarveLocal(
        Entity targetEntity,
        VoxelStorageComponentBase storage,
        MovableAsteroids.State? state,
        Vector3 hitLocal,
        Vector3 inwardLocal,
        float craterRadius,
        float depth,
        double energyJ,
        float impactSpeed,
        string matName,
        double targetDensity,
        int hardness,
        double impactorDensity,
        long nowTicks,
        string impactorName)
    {
        WorldTransform worldToStorage = WorldTransform.GetRelativeTransform(WorldTransform.Identity, storage.ComputeCenteredTransform());
        var builder = new CutOutCapsulesBuilder(0.25f);
        float voxelSize = storage.VoxelSize;

        // 1. Main impact crater / penetration tunnel (start slightly outside hitLocal so inset proxy hits always breach the surface).
        float mouthBack = Math.Min(2.5f, craterRadius * 0.5f);
        var mainCapsule = new Capsule(hitLocal - inwardLocal * mouthBack, hitLocal + inwardLocal * depth, craterRadius);
        builder.AddCapsule(in mainCapsule, voxelSize);

        // 2. Organic perturbed rim lobes around the crater opening so it looks jagged, not a perfect sphere.
        Vector3 tangentA = Vector3.Cross(inwardLocal, Math.Abs(inwardLocal.Y) < 0.9f ? Vector3.UnitY : Vector3.UnitX);
        tangentA = Vector3.Normalize(tangentA);
        Vector3 tangentB = Vector3.Normalize(Vector3.Cross(inwardLocal, tangentA));
        var rng = Random.Shared;
        int lobes = craterRadius >= 4.0f ? 5 : 3;
        for (int i = 0; i < lobes; i++)
        {
            double ang = (2.0 * Math.PI * i / lobes) + (rng.NextDouble() - 0.5) * 0.6;
            float radial = craterRadius * (0.45f + (float)rng.NextDouble() * 0.35f);
            float lobeR = craterRadius * (0.42f + (float)rng.NextDouble() * 0.28f);
            Vector3 offset = (tangentA * (float)Math.Cos(ang) + tangentB * (float)Math.Sin(ang)) * radial
                             + inwardLocal * (depth * (0.15f + (float)rng.NextDouble() * 0.35f));
            Vector3 p = hitLocal + offset;
            var lobe = new Capsule(p, p + inwardLocal * (lobeR * 0.4f), lobeR);
            builder.AddCapsule(in lobe, voxelSize);
        }

        // 3. Catastrophic shattering: if specific energy E/M exceeds material Q*, carve deep fracture fissures through the rock.
        // Legacy (asteroid_shatter=0): deep fissure trenches stand in for a shatter. With shattering on, AsteroidShatter splits
        // the rock instead and this crater is never carved for e >= Q*.
        bool shattered = false;
        if (state != null && state.Mass > 1000f && !Settings.AsteroidShatter)
        {
            double specificEnergy = energyJ / state.Mass;
            double qStar = targetDensity < 1500 ? 200.0 : targetDensity > 4500 ? 900.0 : 450.0;
            if (specificEnergy >= qStar)
            {
                shattered = true;
                float span = (float)(state.Radius * 1.9);
                float fissureR = Math.Clamp(craterRadius * 0.45f, 1.5f, 4.5f);
                int fissures = specificEnergy >= qStar * 2.5 ? 3 : 2;
                for (int f = 0; f < fissures; f++)
                {
                    double ang = (Math.PI * f / fissures) + (rng.NextDouble() - 0.5) * 0.4;
                    Vector3 fDir = Vector3.Normalize(
                        inwardLocal * 0.75f + (tangentA * (float)Math.Cos(ang) + tangentB * (float)Math.Sin(ang)) * 0.66f);
                    var trench = new Capsule(hitLocal - fDir * 2.0f, hitLocal + fDir * span, fissureR);
                    builder.AddCapsule(in trench, voxelSize);
                }
            }
        }

        builder.FinalPass();
        if (!builder.IsValid) return;

        // Source: none (plugin-only). Stage 1.1: a crater from a player-caused impact makes the asteroid permanent (promoted
        // BEFORE the edit, so vanilla's own auto-promotion has nothing to do); a background rock-on-rock crater opens a 5 s
        // window in which MovableAsteroids.AutoPromoteGatePatch keeps vanilla from promoting it (unless the background budget
        // allows), so it stays procedural and the crater is forgotten when the rock unloads.
        if (state != null)
        {
            state.Impacted = true;
            if (!state.Promoted)
            {
                if (state.PlayerTouched)
                {
                    var vp = targetEntity.TryGet<VoxelPhysicsComponent>();
                    if (vp != null) MovableAsteroids.Promote(vp, state, "crater from a player-caused impact");
                }
                else state.BackgroundCarveUntilTicks = nowTicks + TimeSpan.TicksPerSecond * 5;
            }
        }

        // Apply via VoxelDestructionComponent.OnSphereDestruction so the edit replicates to client clipmaps + rebuilds
        // physics chunks without being silently swallowed by scenario safe-zone CanModifyVoxel rules.
        var destruction = targetEntity.TryGet<VoxelDestructionComponent>();
        if (destruction != null)
        {
            destruction._worldToStorageCenter = worldToStorage;
            destruction.OnSphereDestruction(builder);
        }
        else
        {
            foreach (CutOutCapsules cut in builder.BuildClustersLinear(4f, 64f))
            {
                CutOutCapsules localCut = cut;
                try { storage.Storage.CutOutCapsulesFast(ref localCut); }
                finally { localCut.Dispose(); }
            }
        }

        // Recompute real mass/CoM/inertia + convex proxy after losing rock (throttled to avoid spamming worker threads).
        if (state != null)
        {
            state.MassApplied = false;
            state.MassScheduled = false;
            state.MassTries = 0;
            state.MassRetryTicks = nowTicks + TimeSpan.TicksPerSecond;
            ProbeLog.Write($"asteroid: {(shattered ? "SHATTER" : "CRATER")} on {state.Name} vs {impactorName} ({matName}, rho={targetDensity:0} H={hardness}, imp={impactorDensity:0}) " +
                           $"v={impactSpeed:0.0} m/s E={energyJ / 1e6:0.00} MJ -> R={craterRadius:0.0} m depth={depth:0.0} m");
        }
        else
        {
            ProbeLog.Write($"asteroid: impact crater on static voxel {targetEntity} vs {impactorName} ({matName}, rho={targetDensity:0} H={hardness}) " +
                           $"v={impactSpeed:0.0} m/s E={energyJ / 1e6:0.00} MJ -> R={craterRadius:0.0} m depth={depth:0.0} m");
        }
    }
}
