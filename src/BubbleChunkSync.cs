using System;
using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.CoreData;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Colliders;
using Keen.VRage.Physics.Data;
using Keen.Game2.Simulation.Replication;
using Keen.VRage.Voxels.Client.Components;
using Keen.VRage.Voxels.Components;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.Core.Game.GameSystems.SpatialTrigger;

namespace SpinProbe;

/// <summary>
/// Bubbles mode, SERVER planet chunks: the vanilla body, plus NoAutoSync - nothing else changes.
/// <para>Why: VoxelPhysicsComponent.CreateRigidBody (VRage.Voxels\...\VoxelPhysicsComponent.cs:686-701) creates each chunk
/// DEntity with InstanceBind&lt;Entity&gt;(planet). ServerSessionComponent.SyncMotionState (Game2.Game\...\ServerSessionComponent.cs:42-82,
/// [OnChanged(WorldTransform)], [DoesNotHave(NoAutoSync)]) takes its managed Entity from that InstanceBind (stub :273, "->Instance"),
/// so it sends the CHUNK's pose as the PLANET's EntityMotionState. The client applies it to the static planet
/// (VRage.Game\...\MotionStateApplicationSystem.cs:33 ApplyMotionState). Run 35: client planet at 6659,-17469,-272506 vs saved
/// 65233,-10789,-253519 (61.8 km, ~9 km from the player). The other [DoesNotHave(NoAutoSync)] jobs (:86-145) read data chunks
/// do not have, so NoAutoSync only removes this one message. Chunks are runtime-only (never saved).</para>
/// </summary>
[HarmonyPatch(typeof(VoxelPhysicsComponent), "CreateRigidBody")]
internal static class BubbleChunkSyncPatch
{
    private static int _created;

    [HarmonyPrefix]
    private static bool Prefix(VoxelPhysicsComponent __instance, Vector3I chunk, IVoxelCollider voxelCollider,
        ref VoxelPhysicsComponent.ChunkReference chunkReference)
    {
        try
        {
            if (__instance.Entity.TryGet<VoxelRenderComponent>() != null) return true;     // client planet: untouched
            if (__instance.Entity.TryGet<VoxelPlanetComponent>() == null) return true;     // boulders / stamps: untouched
            // Vanilla body, line for line (VoxelPhysicsComponent.cs:690-699), with NoAutoSync set before AllocateBody.
            BodyArgs bodyArgs = __instance._bodyArgs;
            bodyArgs.Collider = voxelCollider;
            Vector3D local = __instance._trigger.GetLocalSectorCenter(chunk) + 0.5f;
            voxelCollider.TryGetMassProperties(out RigidBodyMassProperties mass);
            DEntityContext ctx = __instance.Entity.Scene.AddEntity<WorldTransform, InstanceBind<Entity>, RigidBodyData, RigidBodyMassProperties>(
                __instance.Data.GetWorldTransform() * new RelativeTransform((Vector3)local),
                new InstanceBind<Entity>(__instance.Entity), default(RigidBodyData), mass);
            ctx.Set(default(NoAutoSync));
            __instance._physics.AllocateBody(ctx, in bodyArgs);
            chunkReference.SetPhysical(ctx, voxelCollider);
            if (++_created <= 3 || _created % 50 == 0)
                ProbeLog.Write($"bubblechunk: vanilla chunk + NoAutoSync #{_created} {chunk}");
            return false;
        }
        catch (Exception e)
        {
            ProbeLog.Write("bubblechunk: failed, falling back to vanilla: " + e);
            return true;
        }
    }
}
