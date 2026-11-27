using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using HarmonyLib;
using Keen.Game2.Simulation.Replication;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Game.GameSystems;
using Keen.VRage.Core.Game.GameSystems.SpatialTrigger;
using Keen.VRage.Core.Game.Systems;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.CoreData;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics;
using Keen.VRage.Physics.Colliders;
using Keen.VRage.Physics.Components;
using Keen.VRage.Physics.Constraints;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Havok;
using Keen.VRage.Voxels.Client.Components;
using Keen.VRage.Voxels.Components;

namespace SpinProbe;

/// <summary>
/// Movable asteroids (prototype, server side, single player first).
/// <para>Design 2 (run 57 crashed in native HkSession.StepCollide about a second after voxel chunk colliders were nested in a
/// dynamic compound - vanilla never nests a voxel shape; it is always the root shape of its own body, and its shape keys,
/// HkVoxelShape::ComposeShapeKey = (leaf &lt;&lt; 3) | quad | triangle, declare no key-bit budget for a parent compound).</para>
/// <para>Now: every chunk stays exactly as vanilla builds it - the voxel shape is the ROOT shape of its own body - except the
/// body is Dynamic and is welded to the asteroid's own body by a shared-motion constraint, the engine's mechanism for several
/// bodies moving as one (SharedMotionPhysicsConstraintComponent / ConstraintHelper.CreateSharedFixed, created through
/// IPhysics.AllocateConstraint as PhysicsConstraintBaseComponent.cs:101-113 does). MultiEntitySharedMotionSessionComponent
/// merges the group into one Havok motion only if every member has DynamicTag (GetMotionTypeForNewGroup) and recomputes the
/// group mass as the sum of the members' RigidBodyMassProperties (ComputeGroupMassProperties). Chunks keep vanilla's mass
/// (VoxelShape.TryGetMassProperties is a dummy, 0.008 kg - VoxelShape.cs:155-164), so the asteroid's own body, which carries
/// our estimated mass, is effectively the whole mass.</para>
/// <para>The asteroid's own body: VoxelPhysicsComponent : RigidBodyComponent allocates it in RigidBodyComponent.OnAddedToScene
/// (:292) from the base _bodyArgs, hard-coded Static / Empty (VoxelPhysicsComponent.cs:456-458, Init :461-489). After Init we
/// make it Dynamic with a tiny sphere collider (an Empty collider may not have valid bounds for a dynamic body).</para>
/// <para>Chunk entities carry InstanceBind&lt;Entity&gt;(asteroid), so without NoAutoSync each would send its own pose as the
/// asteroid's motion state (ServerSessionComponent.SyncMotionState, the same bug fixed for planets in BubbleChunkSync.cs).</para>
/// Not handled yet: client-side body (clients keep vanilla static chunks and snap), exact mass, saved velocity, gravity.
/// </summary>
internal static class MovableAsteroids
{
    internal sealed class State
    {
        internal readonly Dictionary<DEntity, IPhysicsConstraint> Welds = new();
        /// <summary>Chunks built while a railed asteroid core was still Static; welded once DynamicTag arrives.</summary>
        internal readonly Dictionary<DEntity, Vector3> DeferredChunkWelds = new();
        internal GroupCollisionFilterData Filter;
        internal double Radius;
        internal float Mass;
        internal int Size;
        internal bool Promoted;
        internal int Added, Removed;
        internal long LastLogTicks;
        internal string Name = "";
        /// <summary>Client copy: kinematic core (smooth playback), vanilla static chunks moved along every frame.</summary>
        internal bool IsClient;
        /// <summary>Real mass from voxel content (computed off-thread once, applied on the DCS thread).</summary>
        internal bool MassScheduled;
        internal volatile bool MassDone;
        internal RigidBodyMassProperties? MassResult;
        internal IPhysicsCollider? ProxyCollider;
        internal string? MassError;
        internal bool MassApplied;
        internal int MassTries;
        internal long MassRetryTicks;
        /// <summary>Grids welded to this asteroid (server), in weld order; grids[0] is the Ship-link hub.</summary>
        internal readonly List<RigidBodyComponent> Grids = new();
        internal readonly Dictionary<DEntity, Vector3> ClientChunks = new();
        /// <summary>Average voxel density (kg/m3) computed by MovableAsteroidMass, used for density-based impacts.</summary>
        internal double AvgDensity;
        /// <summary>Initial velocity (random drift or clipboard paste).</summary>
        internal Vector3 InitialLinVel;
        internal Vector3 InitialAngVel;
        internal bool IsPasted;
        internal bool DriftApplied;
        internal bool Impacted;
        internal long LastImpactTicks;
        internal long SpawnTicks;
        // Source: none (plugin-only). Stage 1.1 pile-up containment: only a player-caused disturbance makes a procedural
        // asteroid permanent (TurnManual). Set by grid/character contact, clipboard paste, grid weld, a player voxel edit, or
        // contact with another player-touched asteroid (contagion). Saved (manual) asteroids start touched.
        internal bool PlayerTouched;
        internal string? TouchReason;
        /// <summary>Until this time an edit to this asteroid's voxels came from our own background (non-player) crater, so
        /// vanilla's auto-promotion on PermanentMutation is suppressed (AutoPromoteGatePatch).</summary>
        internal long BackgroundCarveUntilTicks;
        /// <summary>Stage 2: fresh shards neither crater nor shatter until these times (no 1 -> 6 -> 36 cascade).</summary>
        internal long ImpactImmuneUntilTicks;
        internal long ShatterImmuneUntilTicks;
        /// <summary>Stage 2: shards start in their parent's collision group (siblings never collide while separating);
        /// after this time Watch gives the shard its own group.</summary>
        internal long ShardGroupUntilTicks;
        internal bool IsShard;
        /// <summary>Procedural entity id as a string (same on the server and client copy); stage 3.2 client lookup, stores.</summary>
        internal string? IdKey;
        /// <summary>Stage 4.2 belt catalog record index, -1 = not a catalog rock.</summary>
        internal int CatalogId = -1;
        internal bool FirstWatchDone;
        /// <summary>Natural asteroid loaded before Orbits.ClockReady: assign its prograde belt orbit once the clock/bodies are ready.</summary>
        internal bool PendingOrbitInit;
        /// <summary>Distance from the centre of mass to the nearest empty cell (m): the largest sphere that fits inside.</summary>
        internal double Inscribed;

        internal void DisposeProxy()
        {
            try { ProxyCollider?.Dispose(); } catch { }
            ProxyCollider = null;
        }
    }

    internal static readonly ConditionalWeakTable<VoxelPhysicsComponent, State> States = new();
    /// <summary>Live movable asteroids split by scene (server vs client), pruned on OnBeforeRemovedFromScene.
    /// Avoids enumerating ConditionalWeakTable (which retains unloaded/destroyed components until GC and caused an
    /// exception storm on Data.GetWorldTransform() + cross-scene reads from the client main thread).</summary>
    internal static readonly List<(VoxelPhysicsComponent comp, State state)> LiveServer = new();
    internal static readonly List<(VoxelPhysicsComponent comp, State state)> LiveClient = new();
    internal static int Count;

    /// <summary>Planets seen so far (centre, radius). Asteroids within PlanetExclusion x radius of one stay vanilla
    /// (just outside every planet's local grid bubble FrameRadiusFactor = 1.6 R so planetary boulders, stamps and scenario
    /// orbital rocks inside the bubble stay untouched, while the entire asteroid belt outside the bubble orbits).</summary>
    internal static readonly List<(Vector3D c, double r)> Planets = new();
    private static double PlanetExclusion => Settings.FrameRadiusFactor + 0.02;

    internal static bool On => Settings.MovableAsteroidMax > 0;

    internal static int Promotions;
    internal static int BackgroundPromoted;
    internal static int Touched;

    // Source: none (plugin-only). Stage 1.1: marks an asteroid as disturbed by a player (Watch then promotes it).
    internal static void MarkPlayerTouched(State s, string why)
    {
        if (s.PlayerTouched || s.IsClient) return;
        s.PlayerTouched = true;
        s.TouchReason = why;
        System.Threading.Interlocked.Increment(ref Touched);
        ProbeLog.Write($"asteroid: {s.Name} player-touched ({why})");
    }

    // Source: none (plugin-only). Budget for promotions no player caused (asteroid_background_promotions).
    internal static bool TryTakeBackgroundPromotion()
    {
        if (System.Threading.Interlocked.Increment(ref BackgroundPromoted) <= Settings.AsteroidBackgroundPromotions) return true;
        System.Threading.Interlocked.Decrement(ref BackgroundPromoted);
        return false;
    }

    // Reads: VRage.Voxels/Keen/VRage/Voxels/Components/ProceduralVoxelEntityComponent.cs  TurnManual (:112) -> PromoteToManual
    // (:118: SetSerializable(true), base.PromoteToManual, unsubscribe the edit signal) - the same call vanilla makes when a grid
    // anchors to a procedural asteroid (ConvertGridToDynamicWhenNotAnchoredComponent.cs:191).
    internal static void Promote(VoxelPhysicsComponent physics, State s, string why)
    {
        if (s.Promoted && s.CatalogId < 0) return;
        // Stage 4.2: a belt-catalog rock is already manual (it must replicate); "promoting" it records it in the delta store
        // so it is never despawned or respawned from the catalog again.
        if (s.CatalogId >= 0)
        {
            if (AsteroidStore.CatalogPersistent(s.CatalogId)) { s.Promoted = true; return; }
            s.Promoted = true;
            AsteroidStore.MarkCatalog(s.CatalogId, destroyed: false);
            ProbeLog.Write($"asteroid: catalog rock #{s.CatalogId} {s.Name} is now permanent ({why})");
            return;
        }
        s.Promoted = true;
        try
        {
            var proc = physics.Entity.TryGet<ProceduralVoxelEntityComponent>();
            if (proc != null && !proc.IsManual) proc.TurnManual();
            int n = System.Threading.Interlocked.Increment(ref Promotions);
            ProbeLog.Write($"asteroid: {s.Name} promoted to a saved asteroid ({why}; promotion #{n}, background {BackgroundPromoted})");
        }
        catch (Exception ex) { ProbeLog.Write("asteroid: promote failed: " + ex.Message); }
    }

    /// <summary>
    /// Vanilla promotes a procedural asteroid to a saved one on ANY permanent voxel edit (ProceduralVoxelEntityComponent
    /// subscribes OnStorageEditedExternally to VoxelStorageComponentBase.OnStorageRangeChangedSignal while not manual, :41-44,
    /// and promotes on StorageChangeSignalSource.PermanentMutation, :100-106). Our rock-on-rock craters go through
    /// VoxelDestructionComponent, which is such an edit, so without this gate every background crater would make the asteroid
    /// permanent and the pile-up containment of stage 1.1 would do nothing. A permanent edit we did not cause (drill, hand tool,
    /// weapon) is a player edit: it marks the asteroid touched and promotes as vanilla does.
    /// </summary>
    // Patches: VRage.Voxels/Keen/VRage/Voxels/Components/ProceduralVoxelEntityComponent.cs  OnStorageEditedExternally (:100)
    [HarmonyPatch(typeof(ProceduralVoxelEntityComponent), "OnStorageEditedExternally")]
    internal static class AutoPromoteGatePatch
    {
        [HarmonyPrefix]
        private static bool Prefix(ProceduralVoxelEntityComponent __instance, Keen.VRage.Voxels.Storage.StorageChangeSignalSource source)
        {
            if (source != Keen.VRage.Voxels.Storage.StorageChangeSignalSource.PermanentMutation) return true;
            try
            {
                var physics = __instance.Entity?.TryGet<VoxelPhysicsComponent>();
                if (physics == null || !States.TryGetValue(physics, out State? s) || s.IsClient) return true;
                if (!s.PlayerTouched && DateTime.UtcNow.Ticks < s.BackgroundCarveUntilTicks)
                {
                    if (!TryTakeBackgroundPromotion()) return false;   // our own background crater: stay procedural
                    s.Promoted = true;
                    ProbeLog.Write($"asteroid: {s.Name} background crater kept (background promotion {BackgroundPromoted}/{Settings.AsteroidBackgroundPromotions})");
                    return true;
                }
                MarkPlayerTouched(s, "voxel edit");
                s.Promoted = true;
            }
            catch { }
            return true;
        }
    }

    /// <summary>
    /// Run 64 froze: a movable asteroid (and each welded chunk body) has DynamicTag, and procedural sector triggers fire for
    /// every DynamicTag entity except those with ProcedurallyGeneratedTag / ManagedByWorldAreaTag
    /// (ProceduralGeneratorSessionComponent.GetMustNotHaveTriggerTypeIds, installed :1528-1535). So every movable asteroid
    /// loaded sectors around itself, which spawned more small asteroids, which became movable... (client movable count
    /// climbed to 174, then the game hung). ProcedurallyGeneratedTag excludes them, as vanilla excludes spawned content.
    /// NonBlockingTag keeps them out of other voxels' "Voxel : Block" trigger (VoxelPhysicsComponent.OnAddedToScene :515),
    /// so a moving asteroid never makes the simulation wait (FinishBefore&lt;Simulation&gt;) on a neighbour's chunk; the
    /// prediction trigger still builds those chunks ahead of time. Vanilla's anchoring probe entity uses NonBlockingTag the
    /// same way (ConvertGridToDynamicWhenNotAnchoredComponent).
    /// </summary>
    internal static void MarkNonTriggering(DEntityContext data)
    {
        data.Set(default(Keen.VRage.Core.Game.GameSystems.ProceduralGeneration.ProcedurallyGeneratedTag));
        data.Set(default(Keen.VRage.Core.Game.GameSystems.ProceduralGeneration.NonBlockingTag));
    }

    internal static RigidBodyMassProperties PrototypeMass(float storageSize, float voxelSize = 1f)
    {
        double r = 0.35 * Math.Max(1.0, storageSize) * voxelSize;
        double volume = 4.0 / 3.0 * Math.PI * r * r * r;
        float mass = (float)(volume * Settings.AsteroidDensity);
        float inertia = (float)(0.4 * mass * r * r);
        return new RigidBodyMassProperties
        {
            InvMass = 1f / mass,
            CenterOfMass = Vector3.Zero,
            InvInertiaTensor = new Vector3(1f / inertia),
            InertiaMajorAxisRotation = Quaternion.Identity,
            Volume = (float)volume,
        };
    }

    /// <summary>
    /// Pre-populates all DCS data read synchronously during IEntitySpawner.SpawnEntity -> AddEntityToScene
    /// (VoxelStorageComponentBase.RegisterSpaceProbe needs BoundingBoxData; HavokWorldHelper.AllocateBody needs
    /// RigidBodyMassProperties for Dynamic bodies; BodyCreationInfo.SetForNewEntity reads RigidBodyData and
    /// GroupCollisionFilterData). Data.Set inside Init() is deferred until Scene.Sync(), so omitting these from
    /// DEntityBuilder throws Data not found after world->AllocateBody has already leaked a native Havok body.
    /// </summary>
    internal static Keen.VRage.DCS.Builders.DEntityBuilder BuildDynamicSpawnData(
        Keen.VRage.Core.Game.Systems.Session session, Vector3 halfExtents, in RigidBodyMassProperties mass, Vector3 linVel, Vector3 angVel,
        GroupCollisionFilterData? filter = null)
    {
        var eb = new Keen.VRage.DCS.Builders.DEntityBuilder();
        eb.Set(new BoundingBoxData { BoundingBox = new BoundingBox(-halfExtents, halfExtents) });
        eb.Set(mass);
        eb.Set(new RigidBodyData { LinearVelocity = linVel, AngularVelocity = angVel });
        GroupCollisionFilterData f = filter ?? new GroupCollisionFilterData
        {
            SystemGroupId = session.Get<IPhysics>().AllocateCollisionSystemGroupId(),
            SubsystemId = 1,
            SubsystemNotToCollideWith = 1,
        };
        eb.Set(f);
        if (Settings.AsteroidImpacts)
            eb.Set(new ContactImpulseData { MinimalRequiredImpulse = 5000f });
        eb.Set(default(Keen.VRage.Core.Game.GameSystems.ProceduralGeneration.ProcedurallyGeneratedTag));
        eb.Set(default(Keen.VRage.Core.Game.GameSystems.ProceduralGeneration.NonBlockingTag));
        return eb;
    }

    private static Vector3 RandomUnit(Random rng)
    {
        double z = rng.NextDouble() * 2.0 - 1.0;
        double phi = rng.NextDouble() * 2.0 * Math.PI;
        double r = Math.Sqrt(Math.Max(0.0, 1.0 - z * z));
        return new Vector3((float)(r * Math.Cos(phi)), (float)z, (float)(r * Math.Sin(phi)));
    }

    [HarmonyPatch(typeof(VoxelPhysicsComponent), "Init", new[] { typeof(VoxelPhysicsComponentObjectBuilder) })]
    internal static class InitPatch
    {
        [HarmonyPostfix]
        private static void Postfix(VoxelPhysicsComponent __instance)
        {
            if (!On) return;
            if (AsteroidClipboard.SpawningPreview)
            {
                try
                {
                    if (!__instance.Data.Has<BoundingBoxData>() && __instance._storage != null)
                    {
                        float vs = __instance._storage.VoxelSize;
                        Vector3 hb = new Vector3(__instance._storageSize.X, __instance._storageSize.Y, __instance._storageSize.Z) * (vs * 0.5f);
                        __instance.Data.Set(new BoundingBoxData { BoundingBox = new BoundingBox(-hb, hb) });
                    }
                }
                catch { }
                return;
            }
            try
            {
                var e = __instance.Entity;
                bool isClient = e.TryGet<VoxelRenderComponent>() != null;
                if (isClient && !Settings.ClientKinematicAsteroids) return;   // isolation switch: client copies stay vanilla
                if (e.TryGet<VoxelPlanetComponent>() is VoxelPlanetComponent planet)
                {
                    var entry = (__instance.Data.GetWorldTransform().Position, (double)planet.Radius);
                    lock (Planets) if (!Planets.Exists(x => (x.c - entry.Position).LengthSquared() < 1.0)) Planets.Add(entry);
                    return;
                }
                if (e.TryGet<ProceduralVoxelEntityComponent>() is not ProceduralVoxelEntityComponent procComp) return; // procedural asteroids only
                // Never touch planetary surface boulders, voxel stamps, or planet-attached rock prefabs.
                string entName = e.ToString() ?? "";
                if (entName.Contains("Boulder", StringComparison.OrdinalIgnoreCase) ||
                    entName.Contains("Stamp", StringComparison.OrdinalIgnoreCase) ||
                    entName.Contains("Planet", StringComparison.OrdinalIgnoreCase))
                    return;
                int size = __instance._storageSize.Max();
                float voxel = __instance._storage.VoxelSize;

                // Always ensure BoundingBoxData exists before VoxelStorageComponentBase.OnAddedToScene -> RegisterSpaceProbe()
                // runs (pasted/cloned asteroids via IEntitySpawner do not get BoundingBoxData from ProceduralAsteroidPrefabGenerator).
                if (!__instance.Data.Has<BoundingBoxData>())
                {
                    Vector3 halfBox = new Vector3(__instance._storageSize.X, __instance._storageSize.Y, __instance._storageSize.Z) * (voxel * 0.5f);
                    __instance.Data.Set(new BoundingBoxData { BoundingBox = new BoundingBox(-halfBox, halfBox) });
                }

                Vector3D at = __instance.Data.GetWorldTransform().Position;
                Vector3 pasteLin = Vector3.Zero, pasteAng = Vector3.Zero;
                bool isPasted = !isClient && AsteroidClipboard.TryConsumeSpawnMotion(at, out pasteLin, out pasteAng);
                // Source: none (plugin-only). Stage 2: a shard spawned by AsteroidShatter carries its exact LOD0 mass
                // properties, velocity, and its parent's collision group.
                AsteroidShatter.ShardSpawn? shard = null;
                if (!isClient && !isPasted && AsteroidShatter.TryConsumeShard(at, out shard) && shard != null)
                {
                    isPasted = true;
                    pasteLin = shard.LinVel;
                    pasteAng = shard.AngVel;
                }
                // Source: none (plugin-only). Stage 4.2: a belt-catalog rock just spawned by OrbitalBelts (velocity on its
                // orbit), or loaded from a save (recognised by its catalog id).
                int catalogId = -1;
                if (!isClient && !isPasted && OrbitalBelts.TryConsumeSpawn(at, out int cid, out Vector3 cLin, out Vector3 cAng))
                {
                    isPasted = true;
                    catalogId = cid;
                    pasteLin = cLin;
                    pasteAng = cAng;
                }
                if (catalogId < 0)
                    try { if (OrbitalBelts.TryCatalogId(procComp.ID, out int loadedId)) catalogId = loadedId; } catch { }
                // Vanilla static rocks inside a belt torus would sit still among orbiting ones: delete them for good (next tick)
                // without wasting dynamic body allocation or voxel mass scans on them.
                if (!isClient && !isPasted && catalogId < 0 && OrbitalBelts.On && Settings.BeltsSuppressVanilla && OrbitalBelts.InBeltZone(at))
                {
                    OrbitalBelts.OfferTemplate(__instance);
                    bool manual = false;
                    try { manual = procComp.IsManual; } catch { }
                    if (!manual) { OrbitalBelts.QueueSuppress(__instance); ProbeLog.Write($"belts: vanilla {e} inside a belt - suppressed"); return; }
                }

                // No size or count cap: every deep-space / belt asteroid is natively non-static.
                // Anything inside a planet's exclusion zone (1.8 R = cleanly outside the 1.6 R planet bubble) stays vanilla
                // static so planetary boulders, stamps, and near-planet scenario/orbital rocks are never touched.
                if (!isPasted)
                {
                    // Offer any server procedural asteroid as a belt template before PlanetExclusion skips it so
                    // OrbitalBelts gets its template immediately on load instead of waiting for a deep-space rock.
                    if (!isClient && OrbitalBelts.On) OrbitalBelts.OfferTemplate(__instance);
                    double excl = PlanetExclusion;
                    lock (Planets)
                    {
                        foreach (var (c, pr) in Planets)
                            if ((at - c).Length() < pr * excl)
                            {
                                ProbeLog.Write($"asteroid: {e} is {(at - c).Length() / 1000:0.0} km from a planet (R {pr / 1000:0} km) - left static");
                                return;
                            }
                        if (Planets.Count == 0) ProbeLog.Write($"asteroid: WARNING {e} made movable before any planet was seen");
                    }
                    // Also check live planet bubbles directly (covers client asteroids loading before Planets populates).
                    if (Bubbles.Ready)
                    {
                        foreach (Bubbles.Bubble b in Bubbles.All)
                        {
                            double bubbleGuard = b.Radius * 1.0125; // 1.6 * 1.0125 = 1.62 R (strictly outside the bubble)
                            if ((at - b.C0).Length() < bubbleGuard || (at - Bubbles.Centre(b, Bubbles.T)).Length() < bubbleGuard)
                                return;
                        }
                    }
                }

                IPhysics physics = ((RigidBodyComponent)__instance)._physics;
                // Prototype mass: a solid sphere filling ~70% of the storage cube's width.
                double r = 0.35 * size * voxel;
                double volume = 4.0 / 3.0 * Math.PI * r * r * r;
                float mass = (float)(volume * Settings.AsteroidDensity);
                float inertia = (float)(0.4 * mass * r * r);

                // Server core gets an initial inset sphere proxy immediately (refined to 0.78*eqR once MovableAsteroidMass finishes)
                // so two movable asteroids collide physically right from frame 1 (Havok has no dynamic VoxelShape-vs-VoxelShape agent).
                float coreRadius = isClient ? 0.25f : Math.Max(1.0f, (float)(r * 0.65));
                Vector3 coreCentre = Vector3.Zero;
                if (shard != null)
                {
                    mass = 1f / shard.Mass.InvMass;
                    coreRadius = shard.ProxyRadius;
                    coreCentre = shard.Mass.CenterOfMass;
                }
                // Reads: VRage.Physics/Keen/VRage/Physics/Colliders/SphereColliderArgs.cs (Vector3 center, float radius) ctor;
                // Havok/Colliders/SphereShape.cs:61-66 builds a native hknpSphereShape with that centre - no compound needed.
                SphereColliderArgs sphereArgs = new SphereColliderArgs(coreCentre, coreRadius);
                RigidBodyComponent rb = __instance;
                // Client copy: KINEMATIC. The client applies replicated motion with playback/interpolation only to entities
                // with DynamicTag (VRage.Game MotionStateApplicationSystem: AttachMotionHistory needs EntityMotionState +
                // DynamicTag; without it ApplyMotionState snaps the pose on arrival - the run 58 stutter). Kinematic gets
                // DynamicTag (HavokWorldHelper.SetBodyMotionType / AllocateBody for non-static) but is never pushed by the
                // client's own physics, which the server owns.
                rb._bodyArgs.MotionType = isClient ? BodyArgs.Motion.Kinematic : BodyArgs.Motion.Dynamic;
                rb._bodyArgs.Collider = physics.AllocateCollider(in sphereArgs);
                __instance.Data.Set(shard != null ? shard.Mass : new RigidBodyMassProperties
                {
                    InvMass = 1f / mass,
                    CenterOfMass = Vector3.Zero,
                    InvInertiaTensor = new Vector3(1f / inertia),
                    InertiaMajorAxisRotation = Quaternion.Identity,
                    Volume = (float)volume,
                });

                // Run 58: welding a new chunk took one physics step, during which overlapping chunk bodies (neighbour chunks
                // share a voxel of margin) were pushed apart; the welded asteroid kept that kick (up to 9 m/s, 1.5 rad/s).
                // One collision system group per asteroid with SubsystemNotToCollideWith == SubsystemId: its core and its
                // chunks never collide with each other; everything else uses normal layer rules
                // (Keen.VRage.Physics.Data.GroupCollisionFilterData, applied at body creation in BodyCreationInfo.cs:98).
                GroupCollisionFilterData filter;
                if (shard != null)
                {
                    filter = shard.Filter;
                }
                else if (__instance.Data.TryGet<GroupCollisionFilterData>(out GroupCollisionFilterData preFilter) && preFilter.SystemGroupId != 0)
                {
                    filter = preFilter;
                }
                else
                {
                    filter = new GroupCollisionFilterData
                    {
                        SystemGroupId = physics.AllocateCollisionSystemGroupId(),
                        SubsystemId = 1,
                        SubsystemNotToCollideWith = 1,
                    };
                }
                __instance.Data.Set(filter);
                if (!isClient && Settings.AsteroidImpacts)
                    __instance.Data.Set(new ContactImpulseData { MinimalRequiredImpulse = 5000f });
                MarkNonTriggering(__instance.Data);

                // Initial velocity: explicit paste/shard/catalog velocity, real prograde belt orbit (when real_physics +
                // asteroid_belt_orbits are on), or gentle natural drift fallback.
                Vector3 initLin = Vector3.Zero, initAng = Vector3.Zero;
                bool pendingOrbitInit = false;
                string? orbitParentName = null;
                if (isPasted)
                {
                    initLin = pasteLin;
                    initAng = pasteAng;
                    __instance.Data.Set(new RigidBodyData { LinearVelocity = initLin, AngularVelocity = initAng });
                }
                else if (!isClient && (Settings.AsteroidBeltOrbits || Settings.AsteroidDriftSpeed > 0 || Settings.AsteroidSpinRate > 0))
                {
                    var proc = e.TryGet<ProceduralVoxelEntityComponent>();
                    if (proc != null && !proc.IsManual)
                    {
                        var rng = Random.Shared;
                        bool orbited = false;
                        if (Settings.RealPhysics && Settings.AsteroidBeltOrbits)
                        {
                            if (Orbits.ClockReady && Orbits.TryBeltVelocity(at, Orbits.Now, out Vector3 orbVel, out Orbits.Body? orbParent))
                            {
                                initLin = orbVel;
                                orbitParentName = orbParent?.Name;
                                orbited = true;
                            }
                            else if (!Orbits.ClockReady)
                            {
                                pendingOrbitInit = true;
                            }
                        }
                        if (!orbited && !pendingOrbitInit && Settings.AsteroidDriftSpeed > 0)
                        {
                            Vector3 dir = RandomUnit(rng);
                            double minSpd = Math.Min(0.4, Settings.AsteroidDriftSpeed);
                            float spd = (float)(minSpd + rng.NextDouble() * Math.Max(0.0, Settings.AsteroidDriftSpeed - minSpd));
                            initLin = dir * spd;
                        }
                        if (Settings.AsteroidSpinRate > 0)
                        {
                            Vector3 axis = RandomUnit(rng);
                            double minSpin = Math.Min(0.015, Settings.AsteroidSpinRate);
                            float spin = (float)(minSpin + rng.NextDouble() * Math.Max(0.0, Settings.AsteroidSpinRate - minSpin));
                            initAng = axis * spin;
                        }
                        if (initLin != Vector3.Zero || initAng != Vector3.Zero)
                            __instance.Data.Set(new RigidBodyData { LinearVelocity = initLin, AngularVelocity = initAng });
                    }
                }

                bool alreadyManual = false;
                try { alreadyManual = procComp.IsManual; } catch { }
                long nowTicks = DateTime.UtcNow.Ticks;
                var s = new State
                {
                    Radius = r, Mass = mass, Size = size, Name = e.ToString() ?? "asteroid",
                    Filter = filter, IsClient = isClient, AvgDensity = Settings.AsteroidDensity,
                    InitialLinVel = initLin, InitialAngVel = initAng,
                    IsPasted = isPasted, Promoted = isPasted || alreadyManual,
                    // A saved asteroid was disturbed by someone before; a pasted one by the player who pasted it.
                    PlayerTouched = !isClient && (alreadyManual || (isPasted && shard == null)),
                    TouchReason = alreadyManual ? "saved" : isPasted ? "pasted" : null,
                    SpawnTicks = nowTicks,
                    PendingOrbitInit = pendingOrbitInit,
                    DriftApplied = orbitParentName != null,
                };
                if (catalogId >= 0 && !isClient)
                {
                    s.CatalogId = catalogId;
                    s.PlayerTouched = AsteroidStore.CatalogPersistent(catalogId);
                    s.TouchReason = s.PlayerTouched ? "permanent catalog rock" : null;
                    s.Promoted = s.PlayerTouched;
                    OrbitalBelts.Register(catalogId, __instance);
                }
                if (shard != null)
                {
                    s.IsShard = true;
                    s.PlayerTouched = shard.PlayerTouched;
                    s.TouchReason = shard.PlayerTouched ? "shard of a touched asteroid" : null;
                    s.MassApplied = true;   // exact LOD0 values from the shatter worker
                    s.AvgDensity = shard.Density;
                    s.Inscribed = shard.ProxyRadius;
                    s.Radius = shard.Radius;
                    s.ImpactImmuneUntilTicks = nowTicks + AsteroidShatter.ImpactImmuneTicks;
                    s.ShatterImmuneUntilTicks = nowTicks + AsteroidShatter.ShatterImmuneTicks;
                    s.ShardGroupUntilTicks = nowTicks + AsteroidShatter.SharedGroupTicks;
                }
                States.AddOrUpdate(__instance, s);
                var liveList = isClient ? LiveClient : LiveServer;
                lock (liveList) liveList.Add((__instance, s));
                int n = System.Threading.Interlocked.Increment(ref Count);
                string driftInfo = (!isClient && (initLin != Vector3.Zero || initAng != Vector3.Zero))
                    ? (isPasted ? $", THROWN {initLin.Length():0.0} m/s spin {initAng.Length():0.00} rad/s"
                                : orbitParentName != null ? $", ORBIT {initLin.Length():0.0} m/s about {orbitParentName} spin {initAng.Length():0.000} rad/s"
                                : $", drift {initLin.Length():0.00} m/s spin {initAng.Length():0.000} rad/s")
                    : "";
                ProbeLog.Write($"asteroid: MOVABLE{(isClient ? " (client, kinematic)" : "")} #{n} {s.Name} storage {size} voxels x {voxel} m, mass {mass / 1000:0} t (density {Settings.AsteroidDensity} kg/m3, r {r:0} m{driftInfo})");
            }
            catch (Exception ex) { ProbeLog.Write("asteroid: init failed, left vanilla: " + ex); }
        }
    }

    /// <summary>
    /// VoxelPhysicsComponent.CreateRigidBody (:842-855), line for line, except: Dynamic body, the asteroid's current velocity,
    /// NoAutoSync, and a shared-motion weld to the asteroid's own body.
    /// </summary>
    [HarmonyPatch(typeof(VoxelPhysicsComponent), "CreateRigidBody")]
    internal static class CreateChunkPatch
    {
        [HarmonyPrefix]
        private static bool Prefix(VoxelPhysicsComponent __instance, Vector3I chunk, IVoxelCollider voxelCollider,
            ref VoxelPhysicsComponent.ChunkReference chunkReference)
        {
            if (!States.TryGetValue(__instance, out State? s) || s.IsClient) return true;
            // Stage 3.3: something dynamic came close enough to a railed (static) asteroid to build a physics chunk.
            bool railed = AsteroidRails.IsRailed(__instance);
            if (railed) AsteroidRails.RequestWake(__instance, "chunk requested next to it");
            try
            {
                IPhysics physics = ((RigidBodyComponent)__instance)._physics;
                BodyArgs body = __instance._bodyArgs;          // VoxelPhysicsComponent's own chunk args (layer, material, ...)
                body.Collider = voxelCollider;
                body.MotionType = BodyArgs.Motion.Dynamic;
                Vector3D local = __instance._trigger.GetLocalSectorCenter(chunk) + 0.5f;
                voxelCollider.TryGetMassProperties(out RigidBodyMassProperties mass);
                WorldTransform asteroidPose;
                RigidBodyData velocity;
                if (AsteroidRails.TryLivePose(__instance, out WorldTransform livePose, out Vector3 liveLin, out Vector3 liveAng))
                {
                    asteroidPose = livePose;
                    velocity = new RigidBodyData { LinearVelocity = liveLin, AngularVelocity = liveAng };
                }
                else
                {
                    asteroidPose = __instance.Data.GetWorldTransform();
                    velocity = __instance.Data.TryGet<RigidBodyData>(out RigidBodyData v) ? v : default;
                }
                DEntityContext ctx = __instance.Entity.Scene.AddEntity(asteroidPose * new RelativeTransform((Vector3)local),
                    new InstanceBind<Entity>(__instance.Entity), velocity, mass);
                ctx.Set(default(NoAutoSync));
                ctx.Set(s.Filter);
                if (Settings.AsteroidImpacts)
                    ctx.Set(new ContactImpulseData { MinimalRequiredImpulse = 5000f });
                MarkNonTriggering(ctx);
                physics.AllocateBody(ctx, in body);

                // Never weld a dynamic chunk onto a still-static railed core (that would force the shared-motion group static);
                // defer the weld until DynamicTag arrives in AsteroidRails.Land / Watch.
                if (!railed && __instance.Data.Has<DynamicTag>())
                {
                    var weldArgs = ConstraintHelper.CreateSharedFixed(__instance.Data, ctx, asteroidPose.Position);
                    IPhysicsConstraint weld = physics.AllocateConstraint(__instance.DEntity, ctx, weldArgs);
                    s.Welds[ctx] = weld;
                }
                else
                {
                    lock (s.DeferredChunkWelds) s.DeferredChunkWelds[ctx] = (Vector3)local;
                }
                chunkReference.SetPhysical(ctx, voxelCollider);
                s.Added++;
                if (s.Added <= 3 || s.Added % 25 == 0)
                    ProbeLog.Write($"asteroid: {s.Name} chunk {chunk} -> dynamic body {(s.Welds.ContainsKey(ctx) ? "welded" : "weld-deferred")} at {local} (added {s.Added}, removed {s.Removed}, welds {s.Welds.Count})");
                return false;
            }
            catch (Exception ex)
            {
                ProbeLog.Write("asteroid: chunk weld failed, vanilla static chunk used: " + ex);
                return true;
            }
        }

        /// <summary>Client copy: vanilla static chunk body, remembered with its local offset so it can follow the asteroid.</summary>
        [HarmonyPostfix]
        private static void Postfix(VoxelPhysicsComponent __instance, Vector3I chunk, ref VoxelPhysicsComponent.ChunkReference chunkReference)
        {
            if (!States.TryGetValue(__instance, out State? s) || !s.IsClient) return;
            DEntity chunkEntity = chunkReference._dataEntity;
            if (chunkEntity.IsEmpty) return;
            s.ClientChunks[chunkEntity] = (Vector3)(__instance._trigger.GetLocalSectorCenter(chunk) + 0.5f);
            s.Added++;
        }
    }

    /// <summary>Remove unloaded/destroyed movable asteroids from LiveServer / LiveClient so TryFindAsteroid never touches dead entities.</summary>
    [HarmonyPatch(typeof(VoxelPhysicsComponent), nameof(VoxelPhysicsComponent.OnBeforeRemovedFromScene))]
    internal static class RemovePatch
    {
        [HarmonyPrefix]
        private static void Prefix(VoxelPhysicsComponent __instance)
        {
            if (!States.TryGetValue(__instance, out State? s)) return;
            var liveList = s.IsClient ? LiveClient : LiveServer;
            lock (liveList)
            {
                for (int i = liveList.Count - 1; i >= 0; i--)
                    if (ReferenceEquals(liveList[i].comp, __instance)) { liveList.RemoveAt(i); break; }
            }
            s.ClientChunks.Clear();
            s.Welds.Clear();
            lock (s.DeferredChunkWelds) s.DeferredChunkWelds.Clear();
            s.Grids.Clear();
            s.DisposeProxy();
            if (!s.IsClient)
            {
                AsteroidRails.Forget(__instance);
                OrbitalBelts.Forget(__instance);
            }
        }
    }

    /// <summary>Release (ChunkReference.Destroy :138-146): remove the weld first, then let vanilla destroy the chunk body.</summary>
    [HarmonyPatch(typeof(VoxelPhysicsComponent.ChunkReference), nameof(VoxelPhysicsComponent.ChunkReference.Destroy))]
    internal static class DestroyChunkPatch
    {
        [HarmonyPrefix]
        private static void Prefix(VoxelPhysicsComponent.ChunkReference __instance, VoxelPhysicsComponent component)
        {
            if (!States.TryGetValue(component, out State? s)) return;
            DEntity chunkEntity = __instance._dataEntity;
            if (s.IsClient) { if (s.ClientChunks.Remove(chunkEntity)) s.Removed++; return; }
            if (chunkEntity.IsEmpty) return;
            lock (s.DeferredChunkWelds) s.DeferredChunkWelds.Remove(chunkEntity);
            if (!s.Welds.Remove(chunkEntity, out IPhysicsConstraint? weld)) return;
            try { ((RigidBodyComponent)component)._physics.DestroyConstraint(weld); }
            catch (Exception ex) { ProbeLog.Write("asteroid: weld destroy failed: " + ex.Message); }
            s.Removed++;
        }
    }

    /// <summary>
    /// Prevents movable asteroids (and their welded chunks, all tagged NonBlockingTag) from triggering voxel physics chunk
    /// creation on OTHER movable asteroids (or themselves). Vanilla's "Voxel : Prediction" trigger only checks DynamicTag,
    /// so two nearby 512 m movable asteroids triggered 1,099 + 1,079 = 2,178 dynamic voxel chunks in run 94 (865 ms/tick in
    /// Havok StepCollide — 1 FPS!). Asteroid-to-asteroid collision uses the core SphereCollider proxy (Havok has no dynamic
    /// VoxelShape-vs-VoxelShape agent anyway), while ships, characters and welded grids have no NonBlockingTag and still
    /// trigger exact voxel chunks on movable asteroids. Planets and static asteroids are NOT in States, so falling movable
    /// asteroids still trigger planet terrain chunks and crater them normally.
    /// </summary>
    // Patches: VRage.Core.Game/.../SpatialTrigger/SpatialTriggerSystemSessionComponent.cs:278 Subscribe(OrientedBoundingBoxD, TriggerArgs, SectorArgs, Entity, string)
    [HarmonyPatch(typeof(SpatialTriggerSystemSessionComponent), nameof(SpatialTriggerSystemSessionComponent.Subscribe),
        new[] { typeof(OrientedBoundingBoxD), typeof(TriggerArgs), typeof(SectorArgs), typeof(Entity), typeof(string) })]
    internal static class VoxelTriggerFilterPatch
    {
        private static int _errs;

        [HarmonyPrefix]
        private static void Prefix(ref TriggerArgs args, Entity? attachedEntity, string? debugName)
        {
            if (!On || debugName != "Voxel : Prediction" || attachedEntity == null) return;
            try
            {
                var vp = attachedEntity.TryGet<VoxelPhysicsComponent>();
                if (vp == null || !States.TryGetValue(vp, out _)) return;
                using var builder = default(Keen.VRage.Core.Game.Utils.TypeConstraintBuilder);
                builder.MustHave<DynamicTag>();
                builder.MustNotHave<Keen.VRage.Core.Game.GameSystems.ProceduralGeneration.NonBlockingTag>();
                args.TypeConstraints = new[] { builder.Build() };
            }
            catch (Exception ex) { if (_errs++ < 5) ProbeLog.Write("asteroid: VoxelTriggerFilterPatch error: " + ex.Message); }
        }
    }

    /// <summary>
    /// Belt-and-suspenders guard on VoxelPhysicsComponent.OnPredictedChunkEntered: if a movable asteroid's prediction
    /// trigger is entered by another movable asteroid or its welded chunks (NonBlockingTag), ignore it so two nearby
    /// 512 m asteroids can never trigger 1,000+ dynamic voxel chunks on each other.
    /// </summary>
    // Patches: VRage.Voxels/Keen/VRage/Voxels/Components/VoxelPhysicsComponent.cs  OnPredictedChunkEntered (:260)
    [HarmonyPatch(typeof(VoxelPhysicsComponent), "OnPredictedChunkEntered")]
    internal static class ChunkPredictFilterPatch
    {
        private static int _logged;

        [HarmonyPrefix]
        private static bool Prefix(VoxelPhysicsComponent __instance, DEntity entity)
        {
            if (!On || !States.TryGetValue(__instance, out State? s)) return true;
            try
            {
                var scene = __instance.Entity?.Scene;
                if (scene == null || entity.IsEmpty || !scene.IsEntityAlive(entity)) return true;
                var ctx = new DEntityContext(scene, entity);
                if (ctx.Has<Keen.VRage.Core.Game.GameSystems.ProceduralGeneration.NonBlockingTag>() ||
                    ctx.Has<Keen.VRage.Core.Game.GameSystems.ProceduralGeneration.ProcedurallyGeneratedTag>())
                    return false;
                if (!s.IsClient && _logged < 8)
                {
                    _logged++;
                    Entity? ent = Entity.TryGetFromDataEntity(ctx);
                    ProbeLog.Write($"asteroid: chunk trigger on {s.Name} entered by {ent?.ToString() ?? entity.ToString()}");
                }
            }
            catch { }
            return true;
        }
    }

    /// <summary>Welds any physics chunks that were built while a railed asteroid core was still Static, once DynamicTag is back.</summary>
    internal static void FlushDeferredChunkWelds(VoxelPhysicsComponent asteroid, State s, in WorldTransform asteroidPose, in RigidBodyData vel)
    {
        KeyValuePair<DEntity, Vector3>[] pending;
        lock (s.DeferredChunkWelds)
        {
            if (s.DeferredChunkWelds.Count == 0) return;
            pending = new KeyValuePair<DEntity, Vector3>[s.DeferredChunkWelds.Count];
            int idx = 0;
            foreach (var kv in s.DeferredChunkWelds) pending[idx++] = kv;
            s.DeferredChunkWelds.Clear();
        }
        if (asteroid.Entity == null || asteroid.DEntity.IsEmpty) return;
        IPhysics physics = ((RigidBodyComponent)asteroid)._physics;
        var scene = asteroid.Entity.Scene;
        foreach (var (chunkEntity, local) in pending)
        {
            try
            {
                if (chunkEntity.IsEmpty || !scene.IsEntityAlive(chunkEntity, Keen.VRage.DCS.Scenes.Scene.EntityAliveStates.Materialized)) continue;
                var ctx = new DEntityContext(scene, chunkEntity);
                WorldTransform target = asteroidPose * new RelativeTransform(local);
                ctx.Teleport(in target);
                ctx.Set(vel);
                var weldArgs = ConstraintHelper.CreateSharedFixed(asteroid.Data, ctx, asteroidPose.Position);
                s.Welds[chunkEntity] = physics.AllocateConstraint(asteroid.DEntity, chunkEntity, weldArgs);
            }
            catch (Exception ex) { ProbeLog.Write($"asteroid: deferred chunk weld on {s.Name} failed: {ex.Message}"); }
        }
    }
}

/// <summary>Server-side watcher: promotes a moved asteroid to a saved ("manual") one and logs its motion.
/// DoesNotHave(NoAutoSync) skips the chunk bodies (they bind to the same VoxelPhysicsComponent through InstanceBind).</summary>
internal static class MovableAsteroidJobs
{
    [Keen.VRage.Physics.Havok.PhysicsUpdate.OnCollectMassProperties]
    [DoesNotHave(typeof(NoAutoSync))]
    internal static void Watch(VoxelPhysicsComponent physics, in WorldTransform wt, in RigidBodyData rb,
        EntityData<GroupCollisionFilterData> filters)
    {
        if (physics == null || !MovableAsteroids.States.TryGetValue(physics, out MovableAsteroids.State? s) || s.IsClient) return;
        MovableAsteroidGrids.ProcessDeferred(physics, s);
        MovableAsteroidMass.Update(physics, s);
        // Source: none (plugin-only). Stage 2: a shard leaves its parent's shared collision group once the siblings have
        // separated. Reads: VRage.Physics/Keen/VRage/Physics/Havok/HavokSessionComponent.cs:1650-1655
        // OnUpdateBodyCollisionFilterInfo ([OnChanged(GroupCollisionFilterData)]) re-applies the filter to the live body.
        if (s.ShardGroupUntilTicks != 0 && DateTime.UtcNow.Ticks > s.ShardGroupUntilTicks)
        {
            s.ShardGroupUntilTicks = 0;
            try
            {
                var own = new GroupCollisionFilterData
                {
                    SystemGroupId = ((RigidBodyComponent)physics)._physics.AllocateCollisionSystemGroupId(),
                    SubsystemId = 1,
                    SubsystemNotToCollideWith = 1,
                };
                s.Filter = own;
                filters.TryWriteTo(physics.DEntity, own);
                foreach (DEntity chunk in s.Welds.Keys) filters.TryWriteTo(chunk, own);
                ProbeLog.Write($"asteroid: shard {s.Name} separated - own collision group {own.SystemGroupId}");
            }
            catch (Exception ex) { ProbeLog.Write("asteroid: shard ungroup failed: " + ex.Message); }
        }
        if (!s.FirstWatchDone)
        {
            s.FirstWatchDone = true;
            // Source: none (plugin-only). Stage 3.1: vanilla saves no asteroid velocity (VoxelPhysicsComponent.Serialize :185-189
            // passes RigidBodyComponent.Serialize a throw-away builder); a saved moving asteroid gets its velocity back from the
            // sidecar store (AsteroidRails.SavePatch), if it was saved at this position.
            string? key = AsteroidRails.KeyOf(physics, s);
            if (s.CatalogId < 0)
                try { if (OrbitalBelts.TryCatalogId(physics.Entity.TryGet<ProceduralVoxelEntityComponent>()?.ID, out int cid)) { s.CatalogId = cid; s.PlayerTouched = AsteroidStore.CatalogPersistent(cid); OrbitalBelts.Register(cid, physics); } } catch { }
            if (key != null && !s.IsPasted && AsteroidStore.TryVelocity(key, wt.Position, out Vector3 sv, out Vector3 sw) && sv.LengthSquared() > 1e-6f)
            {
                s.InitialLinVel = sv; s.InitialAngVel = sw; s.DriftApplied = true;
                physics.Data.Set(new RigidBodyData { LinearVelocity = sv, AngularVelocity = sw });
                ProbeLog.Write($"asteroid: {s.Name} saved velocity restored {sv.Length():0.00} m/s");
            }
            if (!s.IsShard && s.CatalogId < 0) OrbitalBelts.OfferTemplate(physics);
        }
        if (s.PendingOrbitInit && Orbits.ClockReady && Orbits.Bodies.Count > 0)
        {
            s.PendingOrbitInit = false;
            if (!s.Impacted && !s.IsPasted && !s.IsShard && !s.PlayerTouched &&
                Orbits.TryBeltVelocity(wt.Position, Orbits.Now, out Vector3 orbVel, out Orbits.Body? orbParent))
            {
                s.InitialLinVel = orbVel;
                s.DriftApplied = true;
                physics.Data.Set(new RigidBodyData { LinearVelocity = orbVel, AngularVelocity = s.InitialAngVel });
                ProbeLog.Write($"asteroid: {s.Name} belt orbit initialized {orbVel.Length():0.0} m/s about {orbParent?.Name ?? "planet"}");
            }
        }
        // Stage 3: on rails the body is static and moved analytically by AsteroidRails; nothing below applies.
        if (AsteroidRails.Observe(physics, s, wt, rb)) return;
        if (s.DeferredChunkWelds.Count > 0 && physics.Data.Has<DynamicTag>())
            MovableAsteroids.FlushDeferredChunkWelds(physics, s, in wt, in rb);
        // Apply initial drift/throw velocity once chunks are welded so MultiEntitySharedMotionSessionComponent wakes
        // the shared-motion group (ACTIVE) and transfers velocity cleanly — without overwriting a post-impact bounce.
        if (!s.DriftApplied && (s.Welds.Count > 0 || s.Impacted))
        {
            s.DriftApplied = true;
            if (!s.Impacted && (s.InitialLinVel != Vector3.Zero || s.InitialAngVel != Vector3.Zero))
            {
                if (rb.LinearVelocity.LengthSquared() < 0.01f || (rb.LinearVelocity - s.InitialLinVel).LengthSquared() < 4.0f)
                    physics.Data.Set(new RigidBodyData { LinearVelocity = s.InitialLinVel, AngularVelocity = s.InitialAngVel });
            }
        }
        float speed = rb.LinearVelocity.Length();
        float deltaV = (rb.LinearVelocity - s.InitialLinVel).Length();
        // Source: none (plugin-only). Stage 1.1: was "any dV > 0.25 m/s or impact promotes", which made every rock-on-rock
        // nudge permanent. Now a player cause promotes; a background disturbance only within asteroid_background_promotions.
        if (!s.Promoted)
        {
            if (s.PlayerTouched)
                MovableAsteroids.Promote(physics, s, $"player-touched: {s.TouchReason}, v {speed:0.00} m/s");
            else if ((s.Impacted || deltaV > 0.25f) && Settings.AsteroidBackgroundPromotions > 0 && MovableAsteroids.TryTakeBackgroundPromotion())
                MovableAsteroids.Promote(physics, s, $"background disturbance, dV {deltaV:0.00} m/s");
        }
        long now = DateTime.UtcNow.Ticks;
        if ((s.Promoted || deltaV > 0.25f) && speed > 0.01f && now - s.LastLogTicks > TimeSpan.TicksPerSecond * 2)
        {
            s.LastLogTicks = now;
            ProbeLog.Write($"asteroid: {s.Name} at {wt.Position.X:0},{wt.Position.Y:0},{wt.Position.Z:0} v {speed:0.00} m/s " +
                           $"w {rb.AngularVelocity.Length():0.000} rad/s welds {s.Welds.Count} (added {s.Added}, removed {s.Removed})");
        }
    }
}

/// <summary>
/// Client: keeps the client copy's vanilla static chunk bodies at asteroidPose * offset every frame (VoxelPhysicsComponent
/// places them once, :851, and never moves them; a stale chunk is a phantom rock the local character can hit). Writing
/// WorldTransform of a static body teleports it (HavokSessionComponent CollectDataForStaticBodies). Chunk entities bind to
/// the same VoxelPhysicsComponent through InstanceBind, so only the asteroid's own entity (entity == physics.DEntity) acts.
/// </summary>
internal static class MovableAsteroidClientJobs
{
    private static int _errors;

    [Keen.VRage.Physics.Havok.PhysicsUpdate.OnCollectMassProperties]
    internal static void Follow(DEntity entity, VoxelPhysicsComponent physics, in WorldTransform wt, EntityData<WorldTransform> transforms)
    {
        try
        {
            if (physics == null || !MovableAsteroids.States.TryGetValue(physics, out MovableAsteroids.State? s) || !s.IsClient) return;
            if (entity != physics.DEntity || s.ClientChunks.Count == 0) return;
            List<DEntity>? dead = null;
            foreach (var pair in s.ClientChunks)
            {
                WorldTransform target = wt * new RelativeTransform(pair.Value);
                if (!transforms.TryReadFrom(pair.Key, out WorldTransform cur)) { (dead ??= new()).Add(pair.Key); continue; }
                if ((cur.Position - target.Position).LengthSquared() < 1e-6 && cur.Orientation == target.Orientation) continue;
                transforms.TryWriteTo(pair.Key, target);
            }
            if (dead != null) foreach (DEntity d in dead) s.ClientChunks.Remove(d);
        }
        catch (Exception ex) { if (_errors++ < 5) ProbeLog.Write("asteroid: client follow failed: " + ex.Message); }
    }
}

/// <summary>
/// Grids built on a movable asteroid. Vanilla makes a grid placed against voxels STATIC: the client's placement validation
/// sets NewGridArgs.Static from "occluded by voxel" (Game2.Client BlockPlacementValidation.CheckCollision ->
/// SupportedInVoxel), and the server applies it in BlockPlacerNetworkStories.PlaceStandalone (installed :4096,
/// GridPhysicsObjectBuilder.MotionType = newGrid.Static ? Static : Dynamic). A static grid inside a moving asteroid is a
/// huge overlap that Havok resolves by flinging the asteroid (user report, run 58). Instead the grid is made dynamic and,
/// once its body exists, welded to the asteroid with the same shared-motion weld magnets / landing gear use
/// (MagneticBlockComponent.OnUpdateLockedEntity -> SharedMotionPhysicsConstraintComponent -> ConstraintHelper.CreateSharedFixed)
/// and put in the asteroid's collision system group (as vanilla wheels do, WheelCornerModuleComponent
/// SetupWheelGroupCollisionFilter). Only the grid's main DEntity is welded: large-collider section bodies share its motion
/// and copy its filter (LargeColliderBodyPartProvider UseSameMotion | UseSameLayer).
/// </summary>
internal static class MovableAsteroidGrids
{
    private sealed class Pending
    {
        internal VoxelPhysicsComponent Asteroid = null!;
        internal Vector3D Position;
        internal long Ticks;
    }

    private static readonly List<Pending> PendingGrids = new();
    internal static int Welded;

    /// <summary>Existing static grids found on a movable asteroid (e.g. scenario stations on asteroid 199, run 59: the asteroid
    /// was kicked to 24.7 m/s and stopped dead against one). They are switched to dynamic with the public
    /// GridPhysicsComponent.SetGridMotion(false) (installed :746); the motion change is queued (HavokSessionComponent
    /// _motionTypeChanges), and a weld to a still-static grid would make the whole shared-motion group static
    /// (MultiEntitySharedMotionSessionComponent.GetMotionTypeForNewGroup), so the weld waits for DynamicTag.</summary>
    private sealed class Deferred
    {
        internal RigidBodyComponent Grid = null!;
        internal VoxelPhysicsComponent Asteroid = null!;
        internal int Frames;
    }
    private static readonly List<Deferred> DeferredWelds = new();

    private static void Weld(RigidBodyComponent grid, VoxelPhysicsComponent asteroid, MovableAsteroids.State s, string why)
    {
        Vector3D p = grid.Data.GetWorldTransform().Position;
        grid.Data.Set(s.Filter);
        var args = ConstraintHelper.CreateSharedFixed(grid.Data, asteroid.Data, p);
        grid._physics.AllocateConstraint(grid.DEntity, asteroid.DEntity, args);
        int n = System.Threading.Interlocked.Increment(ref Welded);
        ProbeLog.Write($"asteroid: grid {grid.Entity} welded to asteroid {s.Name} ({why}, grid weld #{n})");
        MovableAsteroids.MarkPlayerTouched(s, "grid welded");
        try
        {
            s.Grids.RemoveAll(g => g.Entity == null);
            if (s.Grids.Count > 0) LinkShip(s.Grids[0], grid, s);
            s.Grids.Add(grid);
            MovableAsteroidControl.Register(grid, s);
        }
        catch (Exception ex) { ProbeLog.Write("asteroid: ship link failed: " + ex.Message); }
    }

    /// <summary>
    /// Shared terminal / control panel: a "Ship" grid-graph link, exactly as a rotor or connector makes one
    /// (MechanicalBlockComponent.CreateConnection: GridGraphHandler.Connect(other, ConnectionGroups), then AnnounceConnections:
    /// grid.GridsConnectionChanged(this, other, groups, true) on both grids). The Ship type is
    /// CubeGridComponent.GetTerminalConnectionType() (GridConfiguration ShipConnectionType); it drives the control panel
    /// (StreamedControlPanelSessionComponent.IsInShip), toolbars and GetAllGridsInShip. Physical is NOT added, so copy /
    /// blueprint of one grid does not pull in every grid on the asteroid; conveyors are block-to-block and not linked.
    /// Graph links are runtime-only (rebuilt on load by the weld path). Not yet removed when a grid is destroyed.
    /// </summary>
    private static void LinkShip(RigidBodyComponent hub, RigidBodyComponent grid, MovableAsteroids.State s)
    {
        var a = hub.Entity.Get<Keen.Game2.Simulation.WorldObjects.CubeGrids.CubeGridComponent>();
        var b = grid.Entity.Get<Keen.Game2.Simulation.WorldObjects.CubeGrids.CubeGridComponent>();
        var ship = System.Collections.Immutable.ImmutableArray.Create(a.GetTerminalConnectionType());
        a.GridGraphHandler.Connect(b.GridGraphHandler, ship.AsSpan());
        a.GridsConnectionChanged(a, b, ship, true);
        b.GridsConnectionChanged(b, a, ship, true);
        ProbeLog.Write($"asteroid: grid {grid.Entity} ship-linked to {hub.Entity} on {s.Name} (shared terminal; {s.Grids.Count + 1} grids)");
    }

    /// <summary>Called from the server watch job once per asteroid per tick.</summary>
    internal static void ProcessDeferred(VoxelPhysicsComponent asteroid, MovableAsteroids.State s)
    {
        lock (DeferredWelds)
        {
            for (int i = DeferredWelds.Count - 1; i >= 0; i--)
            {
                Deferred d = DeferredWelds[i];
                if (d.Asteroid != asteroid) continue;
                d.Frames++;
                bool dynamicNow;
                try { dynamicNow = d.Grid.Data.Has<DynamicTag>(); } catch { DeferredWelds.RemoveAt(i); continue; }
                if (!dynamicNow)
                {
                    if (d.Frames > 600) { DeferredWelds.RemoveAt(i); ProbeLog.Write($"asteroid: grid {d.Grid.Entity} never became dynamic - not welded"); }
                    continue;
                }
                DeferredWelds.RemoveAt(i);
                try { Weld(d.Grid, asteroid, s, $"existing station converted after {d.Frames} ticks"); }
                catch (Exception ex) { ProbeLog.Write("asteroid: deferred grid weld failed: " + ex.Message); }
            }
        }
    }

    private static bool TryFindAsteroid(in Vector3D p, out VoxelPhysicsComponent? asteroid) => TryFindAsteroid(p, null, false, out asteroid);

    /// <summary>Nearest live movable asteroid whose storage cube reaches p, restricted to the caller's DCS Scene when provided.</summary>
    internal static bool TryFindAsteroid(in Vector3D p, Keen.VRage.DCS.Scenes.Scene? scene, bool includeClient, out VoxelPhysicsComponent? asteroid)
    {
        asteroid = null;
        double best = double.MaxValue;
        if (SearchList(MovableAsteroids.LiveServer, p, scene, ref best, ref asteroid)) return true;
        if (includeClient) SearchList(MovableAsteroids.LiveClient, p, scene, ref best, ref asteroid);
        return asteroid != null;
    }

    private static bool SearchList(List<(VoxelPhysicsComponent comp, MovableAsteroids.State state)> liveList, in Vector3D p,
        Keen.VRage.DCS.Scenes.Scene? scene, ref double best, ref VoxelPhysicsComponent? asteroid)
    {
        (VoxelPhysicsComponent comp, MovableAsteroids.State state)[] snap;
        lock (liveList)
        {
            if (liveList.Count == 0) return false;
            snap = liveList.ToArray();
        }
        bool matchedScene = false;
        foreach (var (comp, st) in snap)
        {
            if (comp.Entity == null || comp.DEntity.IsEmpty) continue;
            if (scene != null && !ReferenceEquals(comp.Entity.Scene, scene)) continue;
            matchedScene = true;
            try
            {
                Vector3D c = comp.Data.GetWorldTransform().Position;
                double reach = st.Size * comp._storage.VoxelSize * 0.87 + 5.0;   // half-diagonal of the storage cube + margin
                double dSq = (p - c).LengthSquared();
                if (dSq < reach * reach && dSq < best) { best = dSq; asteroid = comp; }
            }
            catch { }
        }
        return matchedScene && asteroid != null;
    }

    [HarmonyPatch(typeof(Keen.Game2.Simulation.GameSystems.BlockPlacement.BlockPlacerNetworkStories),
        nameof(Keen.Game2.Simulation.GameSystems.BlockPlacement.BlockPlacerNetworkStories.PlaceStandalone))]
    internal static class PlacePatch
    {
        [HarmonyPrefix]
        private static void Prefix(ref Keen.Game2.Simulation.GameSystems.BlockPlacement.BlockPlacerNetworkStories.NewGridArgs newGrid)
        {
            if (!MovableAsteroids.On) return;
            if (!TryFindAsteroid(newGrid.Transform.Position, null, false, out VoxelPhysicsComponent? asteroid) || asteroid == null) return;
            if (!MovableAsteroids.States.TryGetValue(asteroid, out MovableAsteroids.State? st)) return;
            // Keep newGrid.Static untouched so server-side allowVoxels passes for line/plane builds;
            // also record dynamic placements that land inside the asteroid's voxel radius (e.g. when client chunk lag
            // or adjacent-block placement caused SupportedInVoxel=false on a moving asteroid).
            double dist = (newGrid.Transform.Position - asteroid.Data.GetWorldTransform().Position).Length();
            if (!newGrid.Static && dist > st.Radius + 3.0) return;
            lock (PendingGrids)
            {
                PendingGrids.RemoveAll(x => DateTime.UtcNow.Ticks - x.Ticks > TimeSpan.TicksPerSecond * 30);
                PendingGrids.Add(new Pending { Asteroid = asteroid, Position = newGrid.Transform.Position, Ticks = DateTime.UtcNow.Ticks });
            }
            ProbeLog.Write($"asteroid: PlaceStandalone on movable asteroid {st.Name} at {newGrid.Transform.Position} (static={newGrid.Static}, dist={dist:0.0}m/{st.Radius:0.0}m) - weld pending");
        }
    }

    [HarmonyPatch(typeof(RigidBodyComponent), nameof(RigidBodyComponent.OnAddedToScene))]
    internal static class GridAddedPatch
    {
        /// <summary>Before the body is allocated (RigidBodyComponent.OnAddedToScene :292): a static or pending-weld grid
        /// on a movable (server) asteroid joins its collision group first, so it can never shove the asteroid.</summary>
        [HarmonyPrefix]
        private static void Prefix(RigidBodyComponent __instance)
        {
            if (!MovableAsteroids.On || __instance is not Keen.Game2.Simulation.WorldObjects.CubeGrids.Physicss.GridPhysicsComponent) return;
            try
            {
                Vector3D p = __instance.Data.GetWorldTransform().Position;
                bool isPending = false;
                lock (PendingGrids)
                {
                    foreach (Pending x in PendingGrids)
                        if ((x.Position - p).LengthSquared() < 10.0 * 10.0) { isPending = true; break; }
                }
                if (!isPending && __instance._bodyArgs.MotionType != BodyArgs.Motion.Static) return;
                if (TryFindAsteroid(p, __instance.Entity.Scene, false, out VoxelPhysicsComponent? host) && MovableAsteroids.States.TryGetValue(host!, out MovableAsteroids.State? hs))
                    __instance.Data.Set(hs.Filter);
            }
            catch { }
        }

        [HarmonyPostfix]
        private static void Postfix(RigidBodyComponent __instance)
        {
            if (!MovableAsteroids.On || __instance is not Keen.Game2.Simulation.WorldObjects.CubeGrids.Physicss.GridPhysicsComponent) return;
            Pending? match = null;
            Vector3D p;
            try { p = __instance.Data.GetWorldTransform().Position; } catch { return; }
            lock (PendingGrids)
            {
                foreach (Pending x in PendingGrids)
                    if ((x.Position - p).LengthSquared() < 10.0 * 10.0 && ReferenceEquals(x.Asteroid.Entity?.Scene, __instance.Entity.Scene)) { match = x; break; }
                if (match != null) PendingGrids.Remove(match);
            }
            if (match == null)
            {
                // Not a fresh placement: an existing static grid sitting on a movable (server) asteroid.
                if (__instance.CurrentMotionType != BodyArgs.Motion.Static) return;
                if (!TryFindAsteroid(p, __instance.Entity.Scene, false, out VoxelPhysicsComponent? host)) return;
                try
                {
                    // Same collision group first, so the grid and the asteroid do not shove each other while unwelded.
                    if (MovableAsteroids.States.TryGetValue(host!, out MovableAsteroids.State? hs)) __instance.Data.Set(hs.Filter);
                    ((Keen.Game2.Simulation.WorldObjects.CubeGrids.Physicss.GridPhysicsComponent)__instance).SetGridMotion(false);
                    lock (DeferredWelds) DeferredWelds.Add(new Deferred { Grid = __instance, Asteroid = host! });
                    ProbeLog.Write($"asteroid: existing static grid {__instance.Entity} on movable asteroid {host} - switching to dynamic, weld deferred");
                }
                catch (Exception ex) { ProbeLog.Write("asteroid: static grid conversion failed: " + ex.Message); }
                return;
            }
            if (!MovableAsteroids.States.TryGetValue(match.Asteroid, out MovableAsteroids.State? s)) return;
            try
            {
                __instance.Data.Set(s.Filter);
                if (__instance.CurrentMotionType == BodyArgs.Motion.Static)
                {
                    ((Keen.Game2.Simulation.WorldObjects.CubeGrids.Physicss.GridPhysicsComponent)__instance).SetGridMotion(false);
                    lock (DeferredWelds) DeferredWelds.Add(new Deferred { Grid = __instance, Asteroid = match.Asteroid });
                    ProbeLog.Write($"asteroid: placed static grid {__instance.Entity} on {s.Name} - switching to dynamic, weld deferred");
                }
                else
                {
                    Weld(__instance, match.Asteroid, s, "placed dynamic on it");
                }
            }
            catch (Exception ex) { ProbeLog.Write("asteroid: grid weld failed: " + ex); }
        }
    }
}

/// <summary>
/// Real mass, centre of mass and inertia from the asteroid's voxel content, replacing the sphere estimate.
/// Same read the game's deposit scanner does on worker threads (VoxelStorageComponentBase.ProcessCell, installed :1113-1160:
/// Storage.ReadRange(ref StorageData, Content, lod 2, min, max) then Content(i), 0..255). LOD 2 = 4-voxel cells, so a 64-voxel
/// asteroid is 16^3 = 4096 cells. The entity origin is the storage centre (chunk bounds are translated by storageSize >> 1,
/// VoxelPhysicsComponent.GetChunkBoundsInStorageSpace), so local = (cellCentre - size / 2) * voxelSize. Writing
/// RigidBodyMassProperties reaches Havok through HavokSessionComponent.CollectMassProperties / the shared-motion group's
/// mass recalculation (MultiEntitySharedMotionSessionComponent.CollectMassPropertiesForSharedMotion).
/// </summary>
internal static class MovableAsteroidMass
{
    internal static void Update(VoxelPhysicsComponent physics, MovableAsteroids.State s)
    {
        if (s.MassApplied) return;
        if (!s.MassScheduled)
        {
            if (DateTime.UtcNow.Ticks < s.MassRetryTicks) return;
            var storage = physics._storage;
            Vector3I size = physics._storageSize;
            float voxel = storage.VoxelSize;
            double density = Settings.AsteroidDensity;
            string name = s.Name;
            s.MassScheduled = true;
            s.MassDone = false;
            // Run 66 crash: Task.Run put the voxel read on a plain .NET pool thread; streamed octree reads need the engine's
            // per-thread allocator (BumpAllocator.AllocBuffer NullReferenceException), and the unobserved task exception
            // killed the game. Same call the deposit scanner uses (VoxelStorageComponentBase :1101 Parallel.Schedule),
            // and nothing can escape the job.
            Keen.VRage.Library.Threading.Parallel.Schedule(() =>
            {
                try { s.MassResult = Compute(storage, size, voxel, density, name, out double inscribed); s.Inscribed = inscribed; }
                catch (Exception ex) { s.MassResult = null; s.MassError = ex.GetType().Name + ": " + ex.Message; }
                finally { s.MassDone = true; }
            }, physics, Keen.VRage.Library.Threading.TaskDeadline.OneSecond);
            return;
        }
        if (!s.MassDone) return;
        RigidBodyMassProperties? mp = s.MassResult;
        // Run 65: asteroid 202's first read threw a NullReferenceException (storage not ready yet); retry a few times.
        if (mp == null && ++s.MassTries < 4)
        {
            s.MassScheduled = false;
            s.MassRetryTicks = DateTime.UtcNow.Ticks + TimeSpan.TicksPerSecond * 5;
            return;
        }
        s.MassApplied = true;
        if (mp == null) { ProbeLog.Write($"asteroid: {s.Name} mass from voxels failed - keeping the sphere estimate" + (s.MassError != null ? ": " + s.MassError : "")); return; }

        // Single inset SphereCollider sized from actual voxel volume (~0.02 ms narrowphase vs 51 ms/tick for 23-box
        // StaticCompoundCollider overlapping a voxel mesh). Keeps asteroid-to-asteroid collisions solid & fast while
        // ships/players hit the exact VoxelShape chunks outside it.
        float proxyR = 0f;
        try
        {
            // Source: none (plugin-only). Stage 1.2: the sphere is centred on the real centre of mass (it was at the storage
            // centre, which a cratered or lopsided rock may not even contain) and never larger than the inscribed radius,
            // so it cannot poke out of a thin slab. Reads: VRage.Physics/Keen/VRage/Physics/Colliders/SphereColliderArgs.cs
            // (Vector3 center, float radius); Havok/Colliders/SphereShape.cs:61-66 native sphere with centre.
            double eqR = Math.Cbrt(3.0 * Math.Max(1.0, mp.Value.Volume) / (4.0 * Math.PI));
            // If CoM sits inside a cave/tunnel of a hollow giant asteroid (e.g. inscribed 0.8 m on eqR 205 m), fall back to
            // 0.50*eqR so the rock keeps a solid collision core instead of collapsing to a 1.5 m point.
            bool hollowAtCom = s.Inscribed < Math.Max(2.0, eqR * 0.15);
            double fit = !hollowAtCom ? Math.Min(eqR * 0.78, s.Inscribed * 0.95) : eqR * 0.50;
            proxyR = Math.Max(1.5f, (float)fit);
            IPhysics phys = ((RigidBodyComponent)physics)._physics;
            if (physics.Data.Has<Keen.VRage.Physics.Havok.BodyProxy>())
            {
                SphereColliderArgs sArgs = new SphereColliderArgs(mp.Value.CenterOfMass, proxyR);
                IPhysicsCollider sphere = phys.AllocateCollider(in sArgs);
                phys.SetBodyCollider(physics.DEntity, sphere);
                s.DisposeProxy();
                s.ProxyCollider = sphere;
            }
        }
        catch (Exception ex) { ProbeLog.Write($"asteroid: {s.Name} sphere proxy update failed: " + ex.Message); }

        // Always apply mass properties AFTER SetBodyCollider so exact voxel mass/CoM/inertia is preserved.
        physics.Data.Set(mp.Value);
        s.Mass = 1f / mp.Value.InvMass;
        if (mp.Value.Volume > 1f) s.AvgDensity = s.Mass / mp.Value.Volume;
        ProbeLog.Write($"asteroid: {s.Name} mass from voxels {s.Mass / 1000:0} t, volume {mp.Value.Volume:0} m3, avg density {s.AvgDensity:0} kg/m3, centre of mass {mp.Value.CenterOfMass} m, proxy sphere r={proxyR:0.0} m at CoM (inscribed {s.Inscribed:0.0} m)");
    }

    /// <summary>
    /// Density per voxel material (kg/m3). The game has no density anywhere (VoxelPhysicsMaterialDefinition: hardness,
    /// friction, restitution only), so this is ours, keyed by words in the material definition's name. Values are the
    /// typical ore mineral in real rock: iron ~ magnetite, nickel ~ pentlandite, cobalt ~ cobaltite, magnesium ~ magnesite,
    /// silicon ~ quartz, silver ~ argentite, copper ~ chalcopyrite, lead ~ galena, titanium ~ ilmenite, chromium ~ chromite,
    /// uranium ~ uraninite; gold / platinum as native metal in rock. Anything unmatched uses asteroid_density (stone).
    /// </summary>
    private static readonly (string key, double density)[] Densities =
    {
        ("ice", 920), ("snow", 600),
        ("iron", 5200), ("nickel", 4800), ("cobalt", 6300), ("magnesium", 3000), ("silicon", 2650),
        ("silver", 7200), ("gold", 15000), ("platinum", 15000), ("uranium", 8700),
        ("copper", 4200), ("lead", 7500), ("titanium", 4700), ("chromium", 4800),
        ("stone", 2700), ("rock", 2700), ("asteroid", 2700),
    };
    private static readonly Dictionary<byte, (string name, double density)> MaterialCache = new();

    internal static double DensityForName(string name, double fallback)
    {
        string lower = name.ToLowerInvariant();
        foreach (var (key, density) in Densities)
            if (lower.Contains(key)) return density;
        return fallback;
    }

    internal static (string name, double density) MaterialOf(Keen.VRage.Voxels.Components.VoxelStorageComponentBase storage, byte index, double fallback)
    {
        lock (MaterialCache)
        {
            if (MaterialCache.TryGetValue(index, out var hit)) return hit;
            string name = "material " + index;
            double d = fallback;
            try
            {
                if (storage.MaterialProcessor.TryGetMaterialByIndex(index, out Keen.VRage.Voxels.Definitions.VoxelMaterialDefinition? def) && def != null)
                {
                    // Run 63: matching the full def.ToString() hit "iron" inside its path "...\Environment\...", so every
                    // material became iron. Match the material name only (e.g. "Asteroid 2", "Cobalt").
                    name = def.Name.ToString().Trim();
                    d = DensityForName(name, fallback);
                }
            }
            catch { }
            MaterialCache[index] = (name, d);
            ProbeLog.Write($"asteroid: material {index} '{name}' -> {d:0} kg/m3");
            return (name, d);
        }
    }

    private static RigidBodyMassProperties? Compute(
        Keen.VRage.Voxels.Components.VoxelStorageComponentBase storage, Vector3I size, float voxel, double density, string asteroidName,
        out double inscribed)
    {
        inscribed = 0;
        int maxDim = size.Max();
        int lod = 2;
        while ((maxDim >> lod) > 32 && lod < 6) lod++;
        Vector3I cells = new Vector3I(Math.Max(1, size.X >> lod), Math.Max(1, size.Y >> lod), Math.Max(1, size.Z >> lod));
        double cell = voxel * (1 << lod);
        double cellVolume = cell * cell * cell;
        var data = new Keen.VRage.Voxels.Storage.StorageData(cells, Keen.VRage.Voxels.Storage.StorageDataTypeFlags.ContentAndMaterial, true, "MovableAsteroidMass");
        try
        {
            Keen.VRage.Voxels.Storage.VoxelRequestFlags flags = 0;
            storage.Storage.ReadRange(ref data, Keen.VRage.Voxels.Storage.StorageDataTypeFlags.Content, lod, Vector3I.Zero, cells - 1, ref flags);
            storage.Storage.ReadRange(ref data, Keen.VRage.Voxels.Storage.StorageDataTypeFlags.Material, lod, Vector3I.Zero, cells - 1, ref flags);
            var byMaterial = new Dictionary<string, (double vol, double mass)>();
            double m = 0, vol = 0;
            Vector3D sum = Vector3D.Zero;
            var samples = new List<(Vector3D p, double m)>(cells.X * cells.Y * cells.Z / 2);
            Vector3D half = new Vector3D(size.X, size.Y, size.Z) * 0.5 * voxel;
            Vector3I q = default;
            for (q.Z = 0; q.Z < cells.Z; q.Z++)
                for (q.Y = 0; q.Y < cells.Y; q.Y++)
                    for (q.X = 0; q.X < cells.X; q.X++)
                    {
                        int li = data.ComputeLinear(in q);
                        byte c = data.Content(li);
                        if (c == 0) continue;
                        double fill = c / 255.0;
                        var (matName, matDensity) = MaterialOf(storage, data.Material(li), density);
                        double mi = fill * cellVolume * matDensity;
                        byMaterial.TryGetValue(matName, out var acc);
                        byMaterial[matName] = (acc.vol + fill * cellVolume, acc.mass + mi);
                        Vector3D pos = new Vector3D(q.X + 0.5, q.Y + 0.5, q.Z + 0.5) * cell - half;
                        m += mi; vol += fill * cellVolume; sum += pos * mi;
                        samples.Add((pos, mi));
                    }
            if (m <= 0) return null;
            var parts = new List<string>();
            foreach (var kv in byMaterial)
                parts.Add($"{kv.Key} {100 * kv.Value.vol / vol:0}% vol / {kv.Value.mass / 1000:0} t");
            ProbeLog.Write($"asteroid: {asteroidName} composition: " + string.Join(", ", parts));
            Vector3D com = sum / m;
            // Source: none (plugin-only). Inscribed radius: distance from the centre of mass to the nearest mostly-empty
            // cell (content below the 127 iso level, minus half a cell) or the storage edge.
            double ins = Math.Min(Math.Min(half.X - Math.Abs(com.X), half.Y - Math.Abs(com.Y)), half.Z - Math.Abs(com.Z));
            for (q.Z = 0; q.Z < cells.Z; q.Z++)
                for (q.Y = 0; q.Y < cells.Y; q.Y++)
                    for (q.X = 0; q.X < cells.X; q.X++)
                    {
                        if (data.Content(data.ComputeLinear(in q)) >= 127) continue;
                        Vector3D pos = new Vector3D(q.X + 0.5, q.Y + 0.5, q.Z + 0.5) * cell - half;
                        double dd = (pos - com).Length() - 0.5 * cell;
                        if (dd < ins) ins = dd;
                    }
            inscribed = Math.Max(0, ins);
            double ixx = 0, iyy = 0, izz = 0;
            double cellI = cell * cell / 6.0;   // a uniform cube's own inertia per unit mass
            foreach (var (p, mi) in samples)
            {
                Vector3D d = p - com;
                ixx += mi * (d.Y * d.Y + d.Z * d.Z + cellI);
                iyy += mi * (d.X * d.X + d.Z * d.Z + cellI);
                izz += mi * (d.X * d.X + d.Y * d.Y + cellI);
            }
            return new RigidBodyMassProperties
            {
                InvMass = (float)(1.0 / m),
                CenterOfMass = (Vector3)com,
                InvInertiaTensor = new Vector3((float)(1.0 / ixx), (float)(1.0 / iyy), (float)(1.0 / izz)),
                InertiaMajorAxisRotation = Quaternion.Identity,
                Volume = (float)vol,
            };
        }
        finally { ((IDisposable)data).Dispose(); }
    }
}

/// <summary>
/// Cockpit control across the grids welded to one asteroid. Vanilla control is per grid: the cockpit writes ControlData
/// (Movement, Rotation, in the grid's own axes) and AngularControlData to its own grid only
/// (Game2.Client CockpitInputHandlerComponent.UpdateControlData -> EntityMovementExtensions.UpdateControlData), and
/// ThrustComponent.ComputeThrust / GridGyroscopesComponent read their own grid's data. This server job forwards the piloted
/// grid's input to every other grid on the same asteroid, rotated from the piloted grid's axes into each target grid's axes.
/// Values we wrote are remembered so a target's OnChanged is not taken as a new pilot (no echo).
/// Not handled: dampeners (DampeningData) and dampening strength (each grid uses its own mass, ComputeThrust massData.Mass).
/// </summary>
internal static class MovableAsteroidControl
{
    private static readonly Dictionary<DEntity, MovableAsteroids.State> GroupOf = new();
    private static readonly Dictionary<DEntity, (Vector3 m, Vector3 r)> Written = new();
    private static int _errors;

    internal static void Register(RigidBodyComponent grid, MovableAsteroids.State s)
    {
        lock (GroupOf) GroupOf[grid.DEntity] = s;
    }

    [Keen.VRage.Physics.Havok.PhysicsUpdate.OnCollectMassProperties]
    [OnChanged(typeof(Keen.Game2.Simulation.WorldObjects.Movement.ControlData))]
    internal static void Forward(DEntity entity, in Keen.Game2.Simulation.WorldObjects.Movement.ControlData control, in WorldTransform wt,
        EntityData<Keen.Game2.Simulation.WorldObjects.Movement.ControlData> controls,
        EntityData<Keen.Game2.Simulation.WorldObjects.Movement.AngularControlData> angular,
        ReadOnlyEntityData<WorldTransform> transforms)
    {
        try
        {
            MovableAsteroids.State? s;
            lock (GroupOf) if (!GroupOf.TryGetValue(entity, out s)) return;
            lock (Written)
                if (Written.TryGetValue(entity, out var w) && w.m == control.Movement && w.r == control.Rotation) return;   // our own write
            Quaternion toWorld = wt.Orientation;
            foreach (RigidBodyComponent g in s.Grids)
            {
                DEntity t;
                try { t = g.DEntity; } catch { continue; }
                if (t == entity || !transforms.TryReadFrom(t, out WorldTransform tw)) continue;
                Quaternion toTarget = Quaternion.Conjugate(tw.Orientation) * toWorld;
                Vector3 m = Vector3.Transform(control.Movement, toTarget);
                Vector3 r = Vector3.Transform(control.Rotation, toTarget);
                lock (Written) Written[t] = (m, r);
                controls.WriteTo(t, new Keen.Game2.Simulation.WorldObjects.Movement.ControlData { Movement = m, Rotation = r });
                angular.WriteTo(t, new Keen.Game2.Simulation.WorldObjects.Movement.AngularControlData { TargetAngularVelocity = r });
            }
        }
        catch (Exception ex) { if (_errors++ < 5) ProbeLog.Write("asteroid: control forward failed: " + ex.Message); }
    }
}

/// <summary>
/// Building on a movable asteroid like on a static one (placement research, run 62 user report):
/// <list type="bullet">
/// <item>Vanilla lets a grid touch voxel only if it is STATIC: BlockPlacementTester.CanMergeWithVoxels(PlacementTarget) =
/// IsGridStatic(root) (installed :431-437), used by client validation (BlockPlacementValidation.CheckPlacementIsPossible :177
/// -> FailedOccupiedCollision) and the server (IsPlacementPossible allowVoxels, Tester :217). Grids welded to a movable
/// asteroid are dynamic, so every block touching rock failed. Postfix: true for a grid sitting on a movable asteroid.</item>
/// <item>New grids are no longer flipped to dynamic at placement (that also made the server reject extra line/plane blocks
/// touching rock, allowVoxels = MotionType == Static); they spawn static and are converted + welded by GridAddedPatch.</item>
/// <item>Preview alignment is dropped when the aligned grid moves faster than MaxAlignedSpeed 2.5 m/s or turns faster than
/// MaxAlignedAngularSpeed 1 rad/s (Game2.Client BlockPlacementAlignment.UpdateAlignedGrid :204-217, BlockPlacer.def :32-33);
/// a welded grid's velocity is the asteroid's plus omega x r, so snapping worked at some spots and not others.
/// GridPlacementTarget.RigidBody now reports velocity relative to the asteroid for grids on one.</item>
/// </list>
/// </summary>
internal static class MovableAsteroidPlacement
{
    private static bool OnMovableAsteroid(Entity? grid, out VoxelPhysicsComponent? asteroid)
    {
        asteroid = null;
        if (grid == null) return false;
        try
        {
            return MovableAsteroidGrids.TryFindAsteroid(grid.Data.GetWorldTransform().Position, grid.Scene, true, out asteroid);
        }
        catch { return false; }
    }

    [HarmonyPatch(typeof(Keen.Game2.Simulation.GameSystems.BlockPlacement.BlockPlacementTester),
        nameof(Keen.Game2.Simulation.GameSystems.BlockPlacement.BlockPlacementTester.CanMergeWithVoxels))]
    internal static class VoxelMergePatch
    {
        [HarmonyPostfix]
        private static void Postfix(Keen.Game2.Simulation.GameSystems.BlockPlacement.PlacementTarget? extendedGrid, ref bool? __result)
        {
            if (!MovableAsteroids.On || __result != false || extendedGrid == null) return;
            if (OnMovableAsteroid(extendedGrid.RootEntity, out _)) __result = true;
        }
    }

    [HarmonyPatch(typeof(Keen.Game2.Simulation.GameSystems.BlockPlacement.GridPlacementTarget), "get_RigidBody")]
    internal static class AlignSpeedPatch
    {
        [HarmonyPostfix]
        private static void Postfix(Keen.Game2.Simulation.GameSystems.BlockPlacement.GridPlacementTarget __instance, ref RigidBodyData? __result)
        {
            if (!MovableAsteroids.On || __result == null) return;
            if (!OnMovableAsteroid(__instance.RootEntity, out VoxelPhysicsComponent? a) || a == null) return;
            try
            {
                RigidBodyData rel = __result.Value;
                if (a.Data.TryGet<RigidBodyData>(out RigidBodyData ar))
                {
                    Vector3D r = __instance.RootEntity!.Data.GetWorldTransform().Position - a.Data.GetWorldTransform().Position;
                    Vector3 carried = ar.LinearVelocity + (Vector3)Vector3D.Cross((Vector3D)ar.AngularVelocity, r);
                    rel.LinearVelocity -= carried;
                    rel.AngularVelocity -= ar.AngularVelocity;
                }
                else { rel.LinearVelocity = Vector3.Zero; rel.AngularVelocity = Vector3.Zero; }
                __result = rel;
            }
            catch { }
        }
    }
}
