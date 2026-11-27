using System;
using System.Collections.Generic;
using HarmonyLib;
using Keen.Game2.Simulation.Replication;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Components;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Data;
using Keen.VRage.Voxels.Client.Components;
using Keen.VRage.Voxels.Components;
using Keen.VRage.Physics.Colliders;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Game.GameSystems.SpatialTrigger;
using Keen.VRage.DCS.CoreData;

namespace SpinProbe;

/// <summary>
/// Creates the SERVER's voxel chunk bodies as Kinematic + TransformDriven + AlwaysActive + NoAutoSync, instead of Static.
/// <para>Patched: VRage.Voxels\Keen\VRage\Voxels\Components\VoxelPhysicsComponent.cs:367-381 <c>CreateRigidBody</c>, verbatim:</para>
/// <code>
/// BodyArgs bodyArgs = this._bodyArgs;                       // private, MotionType = Static (:89)
/// bodyArgs.Collider = voxelCollider;
/// Vector3D localSectorCenter = this._trigger.GetLocalSectorCenter(chunk);
/// Vector3D vector3D = localSectorCenter + 0.5f;
/// RigidBodyMassProperties mp; bool flag = voxelCollider.TryGetMassProperties(out mp);
/// DEntityContext ctx = base.Entity.Scene.AddEntity&lt;WorldTransform, InstanceBind&lt;Entity&gt;, RigidBodyData, RigidBodyMassProperties&gt;(
///     base.Data.GetWorldTransform() * new RelativeTransform((Vector3)vector3D), new InstanceBind&lt;Entity&gt;(base.Entity), default, mp);
/// this._physics.AllocateBody(ctx, in bodyArgs);
/// chunkReference.SetPhysical(ctx, voxelCollider);
/// </code>
/// <para>Why a full replacement: <c>TransformDriven</c> must be on the entity BEFORE AllocateBody, because
/// BodyCreationInfo.SetForNewEntity reads it (VRage.Physics\...\Havok\BodyCreationInfo.cs:79-87) and
/// HavokWorldHelper.AllocateBody decides the tags from it (HavokWorldHelper.cs:84-97). Kinematic chunks follow the vanilla
/// client-grid pattern (Game2.Simulation\...\GridPhysicsComponent.cs:490-501): a WorldTransform write is then routed to
/// HknpWorld.DriveBodyTo (HavokSessionComponent.cs:1811-1821). NoAutoSync keeps the chunk out of
/// ServerSessionComponent.SyncMotionState (Game2.Game\...\ServerSessionComponent.cs:42-82), which would otherwise send
/// the chunk's pose as the PLANET's (chunks carry InstanceBind&lt;Entity&gt;(planet)). AlwaysActive because
/// TransformDriven bodies are allocated without ActiveTag (HavokWorldHelper.cs:93).</para>
/// <para>IPhysicsMotionProvider.Motion (VoxelPhysicsComponent.cs:71-77) is left Static on purpose: VoxelHand, FastTravel and
/// TryApplyImpulse read it to mean "the planet is static".</para>
/// <para>Client planets are left vanilla: on a listen host only the client planet has a VoxelRenderComponent
/// (client composite, GD\...\Verdure_VS4_ClientComposition.def).</para>
/// </summary>
[HarmonyPatch(typeof(VoxelPhysicsComponent), "CreateRigidBody")]
internal static class ChunkPatch
{
    /// <summary>planet DEntity -> (chunk DEntity -> chunk offset in the planet frame). Read by ProbeJobs on the DCS thread.</summary>
    internal static readonly Dictionary<DEntity, Dictionary<DEntity, Vector3>> Chunks = new();
    private static readonly Dictionary<DEntity, int> Groups = new();
    private static int _created;
    private static int _noMass;

    [HarmonyPrefix]
    private static bool Prefix(VoxelPhysicsComponent __instance, Vector3I chunk, IVoxelCollider voxelCollider,
        ref VoxelPhysicsComponent.ChunkReference chunkReference)
    {
        try
        {
            if (__instance.Entity.TryGet<VoxelRenderComponent>() != null)
                return true; // client planet: vanilla static chunk
            // Only real planets (VRage.Voxels\...\Components\VoxelPlanetComponent.cs). Boulders, voxel stamps and asteroids
            // also carry a VoxelPhysicsComponent; run 1 spun all of them about their own origins.
            if (__instance.Entity.TryGet<VoxelPlanetComponent>() == null)
                return true;

            bool frame = Settings.Mode == "frame";
            BodyArgs bodyArgs = __instance._bodyArgs;
            bodyArgs.Collider = voxelCollider;
            // Frame mode: terrain stays exactly vanilla (Static, destruction layer kept) - Havok never sees it move.
            // Only NoAutoSync is added, because FrameJobs.Refresh rewrites the chunk's WorldTransform every tick and
            // chunks carry InstanceBind<Entity>(planet): without it SyncMotionState would send the chunk pose as the planet's.
            if (!frame) bodyArgs.MotionType = BodyArgs.Motion.Kinematic;
            if (!frame) bodyArgs.ActivationControl = Settings.AutoActivation ? ActivationControl.Automatic : ActivationControl.AlwaysActive;
            // Run 1 crashed in StepSolve: hknpWorld::getBodyTransform+0x69 called from the break handler apply chain
            // (VRage.Physics.Native.dll+0xb8dab). Source: VRage.Physics\...\Havok\BodyCreationInfo.cs:106-117 - a body joins
            // the break handler (USER_FLAG_1 + breaking layer) only when args.DestructionLayer != null. Run 2 tests whether
            // keeping kinematic terrain out of the break handler removes the crash.
            if (!frame && !Settings.KeepDestructionLayer)
                bodyArgs.DestructionLayer = null;
            Vector3D local = __instance._trigger.GetLocalSectorCenter(chunk) + 0.5f;
            if (!voxelCollider.TryGetMassProperties(out RigidBodyMassProperties mass))
                _noMass++;
            DEntityContext ctx = __instance.Entity.Scene.AddEntity<WorldTransform, InstanceBind<Entity>, RigidBodyData, RigidBodyMassProperties>(
                __instance.Data.GetWorldTransform() * new RelativeTransform((Vector3)local),
                new InstanceBind<Entity>(__instance.Entity), default(RigidBodyData), mass);
            if (!frame) ctx.Set(default(TransformDriven));
            ctx.Set(default(NoAutoSync));
            // Run 4: StepWorlds went from 0.27 ms (vanilla) to ~600 ms/tick with kinematic chunks. Static chunks are never
            // tested against each other; kinematic neighbours (overlapping voxel shapes) apparently are. Keep one planet's
            // chunks from colliding with each other exactly the way Keen keeps a wheel from colliding with its own grid:
            // Game2.Simulation\...\Wheels\WheelCornerModuleComponent.cs:365-385 (SystemGroupId from
            // IPhysics.AllocateCollisionSystemGroupId, SubsystemId == SubsystemNotToCollideWith). Read at body creation:
            // VRage.Physics\...\Havok\BodyCreationInfo.cs:101 BuildGroupCollisionFilterInfo(EntityLayerData, GroupCollisionFilterData).
            if (!frame && !Settings.NoChunkGroup)
            {
                if (!Groups.TryGetValue(__instance.DEntity, out int group))
                    Groups[__instance.DEntity] = group = __instance._physics.AllocateCollisionSystemGroupId();
                ctx.Set(new GroupCollisionFilterData { SystemGroupId = group, SubsystemId = 1, SubsystemNotToCollideWith = 1 });
            }
            __instance._physics.AllocateBody(ctx, in bodyArgs);
            chunkReference.SetPhysical(ctx, voxelCollider);

            DEntity planet = __instance.DEntity;
            if (!Chunks.TryGetValue(planet, out var map))
                Chunks[planet] = map = new Dictionary<DEntity, Vector3>();
            map[ctx] = (Vector3)local;
            if (++_created <= 3 || _created % 50 == 0)
                ProbeLog.Write($"E1: kinematic chunk #{_created} {chunk} on planet {planet} (no mass props so far: {_noMass})");
            return false;
        }
        catch (Exception e)
        {
            ProbeLog.Write("E1: chunk replacement failed, falling back to vanilla: " + e);
            return true;
        }
    }
}
