using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.GameSystems.OWT;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Scenes;
using Keen.VRage.Game.Client.Render;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Render.Contracts;

namespace SpinProbe;

/// <summary>
/// Global rendering for planet bubbles: the client draws everything at its TRUE global pose. Global space (sun, stars)
/// stays fixed; a bubble's contents are drawn at F_q(t) · local. The client ECS keeps the bubble-local poses replicated
/// from the server, so game logic, physics, interaction raycasts and tools stay in local coordinates; only what is handed
/// to the renderer is converted.
/// </summary>
internal static class GlobalRender
{
    /// <summary>The camera as the game computed it (bubble-local), before it was converted for the renderer.</summary>
    internal static WorldTransform LocalCamera;
    internal static bool HaveCamera;
    /// <summary>The bubble the camera's raw pose belongs to (null = global space).</summary>
    internal static Bubbles.Bubble? CameraBubble;

    /// <summary>
    /// Render camera. Source: VRage.Render\...\Contracts\RenderSettings.cs:727-736 - SetCameraParameters stores
    /// CameraTransform and queues RenderSettings_SetCameraParameters through SetCameraParameters_Impl. It is the only
    /// place the camera reaches the renderer; the game-side camera entity is untouched.
    /// </summary>
    [HarmonyPatch(typeof(RenderSettings), nameof(RenderSettings.SetCameraParameters))]
    internal static class CameraPatch
    {
        [HarmonyPrefix]
        private static bool Prefix(RenderSettings __instance, ref WorldTransform cameraTransform, float fov, float nearPlane,
            float farPlane, float veryFarPlane, Vector2 projectionOffset, bool smooth, bool orthographic)
        {
            LocalCamera = cameraTransform;
            HaveCamera = true;
            Bubbles.Bubble? b = Bubbles.Ready ? SpectatorHandoff.ResolveCameraBubble(cameraTransform.Position) : null;
            CameraBubble = b;
            if (b == null || b.IsIdentity) return true;
            // Map open: the map camera moves with the map by translation only (MapRender.cs), so the map keeps the skybox orientation.
            WorldTransform g = MapRender.Open ? MapRender.Translate(cameraTransform) : Bubbles.ToGlobal(b, cameraTransform, Bubbles.T);
            __instance.CameraTransform = g;
            __instance._fov = fov;
            __instance.SetCameraParameters_Impl(in g, fov, nearPlane, farPlane, veryFarPlane, projectionOffset, smooth, orthographic);
            return false;
        }
    }
}

/// <summary>
/// Runs after the engine's own render-transform jobs (VRage.Game.Client\...\Render\RootEntityRenderComponent.cs:232
/// RenderUpdateJob: UpdateTransform :90-98 pushes WorldTransform to RootEntity on change, UpdateObservedTransform :100-107
/// for hierarchy children) and overrides the pose of every root in a moving bubble with its global pose, every frame -
/// a bubble's contents move in global space even when nothing changes locally.
/// </summary>
[After(typeof(RootEntityRenderComponent.RenderUpdateJob))]
internal class BubbleRenderJob : JobGroup
{
}

/// <summary>
/// Runs after RenderSubmissionBegin and strictly BEFORE VoxelRenderUpdate.OnUpdateClipmap, so every moving bubble's
/// clipmaps (_localToWorld and RootEntityProvider) are updated to Bubbles.T before VoxelRenderUpdateSessionComponent
/// runs UpdateClipmaps. Eliminates the SunUpdate vs OnUpdateClipmap race on _localToWorld.
/// </summary>
[After(typeof(Keen.VRage.Core.Systems.RenderSubmissionBegin))]
[Before(typeof(Keen.VRage.Voxels.Client.VoxelRenderUpdate.OnUpdateClipmap))]
internal class BubbleClipmapMoveJob : JobGroup
{
}

internal static class GlobalRenderJobs
{
    /// <summary>
    /// Samples bubble time once per client frame. WorldUpdateBeginJobGroup (VRage.Core\...\Systems\WorldUpdateBeginJobGroup.cs)
    /// is the prefix of every session job (Game2.Game\...\WorldSessionComponent.cs:567), so everything later in the frame -
    /// camera, render roots, terrain - converts with the same t. Bound to planets so it runs a handful of times, not per entity.
    /// </summary>
    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(Keen.VRage.DCS.CoreData.InstanceBind<Keen.VRage.Voxels.Components.VoxelPlanetComponent>))]
    internal static void FrameClock(in WorldTransform wt) => Bubbles.SampleFrameTime();

    /// <summary>
    /// Moves all bubble voxel clipmaps (planets + adopted boulders/stamps) to their global pose for Bubbles.T strictly
    /// before VoxelRenderUpdate.OnUpdateClipmap runs. Guarded inside Bubbles.TickFrame to run once per frame.
    /// </summary>
    [BubbleClipmapMoveJob]
    [MustHave(typeof(Keen.VRage.DCS.CoreData.InstanceBind<Keen.VRage.Voxels.Components.VoxelPlanetComponent>))]
    internal static void MoveClipmaps(in WorldTransform wt) => Bubbles.TickFrame();

    /// <summary>Source for the call: VRage.Render\...\Contracts\RootEntity.cs:76 UpdateTransform(in WorldTransform).</summary>
    [BubbleRenderJob]
    [DoesNotHave(typeof(ObservedWorldTransform))]
    internal static void PushRoot(Keen.VRage.DCS.Accessors.DEntity entity, in WorldTransform wt, RootEntity render)
    {
        if (!Bubbles.Ready) return;
        if (MapRender.IsMember(entity)) { render.UpdateTransform(MapRender.Translate(wt)); return; }
        Bubbles.Bubble? b = Settings.RealPhysics && Settings.RealBubbleHandoff
            ? BubbleHandoff.ResolveClientBubble(entity, wt.Position)
            : Bubbles.BubbleAt(wt.Position);
        if (b == null || b.IsIdentity) return;
        render.UpdateTransform(Bubbles.ToGlobal(b, wt, Bubbles.T));
        RenderDiag.RootsPushed++;
    }

    /// <summary>Hierarchy children rendered from their observed world transform
    /// (VRage.Core.Game\...\GameSystems\OWT\ObservedWorldTransform.cs:22 Transform).</summary>
    [BubbleRenderJob]
    internal static void PushObservedRoot(Keen.VRage.DCS.Accessors.DEntity entity, in ObservedWorldTransform owt, RootEntity render)
    {
        if (!Bubbles.Ready) return;
        if (MapRender.IsMember(entity)) { render.UpdateTransform(MapRender.Translate(owt.Transform)); return; }
        Bubbles.Bubble? b = Settings.RealPhysics && Settings.RealBubbleHandoff
            ? BubbleHandoff.ResolveClientBubble(entity, owt.Transform.Position)
            : Bubbles.BubbleAt(owt.Transform.Position);
        if (b == null || b.IsIdentity) return;
        render.UpdateTransform(Bubbles.ToGlobal(b, owt.Transform, Bubbles.T));
        RenderDiag.RootsPushed++;
    }
}

/// <summary>
/// Flora, decals and particles place themselves relative to a terrain render root chosen by position.
/// Source: VRage.Voxels.Client\...\Components\VoxelRenderComponent.cs:255-261 ChooseRootEntityFromPosition(position, out
/// rootWorldTransform) -> DistributedRootEntityProvider.GetRootEntity, which returns the root's CURRENT transform
/// (_baseEntityTransform * rootOffset, Clipmap\DistributedRootEntityProvider.cs:52-63). Callers then compute
/// GetRelativeTransform(localPose, rootWorldTransform) - e.g. LogicalFloraClientSectorObjectBuilder.SpawnInstance and
/// PlanetFloraClientComponent.MaterializeSurfaceSector. Under global rendering the root is at its global pose while the
/// caller's pose is bubble-local, so the relative transform bakes in the rotation. Return the root's LOCAL transform
/// instead (planet centred transform * rootOffset), which is what the relative transform must be taken against.
/// </summary>
[HarmonyPatch(typeof(Keen.VRage.Voxels.Client.Components.VoxelRenderComponent), nameof(Keen.VRage.Voxels.Client.Components.VoxelRenderComponent.ChooseRootEntityFromPosition))]
internal static class RootChoicePatch
{
    [HarmonyPostfix]
    private static void Postfix(Keen.VRage.Voxels.Client.Components.VoxelRenderComponent __instance, Vector3D position, ref WorldTransform rootWorldTransform)
    {
        if (!Bubbles.Ready) return;
        var storage = __instance._voxelStorageComponent;
        WorldTransform centredLocal = Bubbles.InitialCentred.TryGetValue(__instance, out WorldTransform initC) ? initC : storage.ComputeCenteredTransform();
        // Same maths as VoxelStorageComponentBase.WorldToStoragePosition (VoxelStorageComponentBase.cs:432-437), but against the
        // SAME centred transform used below - the engine's version reads the live planet transform, which may differ.
        Vector3D sp = WorldTransform.TransformInv(position, centredLocal) / (double)storage.VoxelSize;
        Keen.VRage.Library.Mathematics.Vector3I storagePos = Vector3D.FloorToInt(in sp);
        Keen.VRage.Library.Mathematics.Vector3I rootOffset = storagePos / 1000 * 1000;   // same integer maths as GetRootEntity
        rootWorldTransform = centredLocal * new WorldTransform(rootOffset);
    }
}

/// <summary>
/// DistributedRootEntityProvider.UpdateTransform (Clipmap\DistributedRootEntityProvider.cs:66-75) walks _rootEntities with
/// no lock, while GetRootEntity (:52-63) adds roots under lock from clipmap worker threads. Vanilla almost never moves a
/// planet; global rendering moves it every frame. Same body, under the same lock.
/// </summary>
[HarmonyPatch]
internal static class RootUpdateLockPatch
{
    private static System.Reflection.MethodBase TargetMethod() =>
        AccessTools.Method(AccessTools.TypeByName("Keen.VRage.Voxels.Client.Clipmap.DistributedRootEntityProvider"), "UpdateTransform");

    [HarmonyPrefix]
    private static bool Prefix(ref Keen.VRage.Voxels.Client.Clipmap.DistributedRootEntityProvider __instance, WorldTransform baseTransform)
    {
        __instance._baseEntityTransform = baseTransform;
        var roots = __instance._rootEntities;
        // For moving bubbles, compute each root's global pose directly in double precision from its local position
        // instead of multiplying baseTransform (at the -131 km storage corner) by a 32-bit float quaternion over a
        // ~185 km lever arm, which caused ~5-10 cm of per-frame terrain jitter relative to the double-precision camera/grids.
        if (Bubbles.Ready && Bubbles.RootDictOwner.TryGetValue(roots, out var owner) && !owner.b.IsIdentity)
        {
            WorldTransform localCentred = Bubbles.InitialCentred.TryGetValue(owner.r, out WorldTransform initC)
                ? initC
                : owner.r._voxelStorageComponent.ComputeCenteredTransform();
            double t = Bubbles.T;
            lock (roots)
            {
                foreach (var pair in roots)
                {
                    WorldTransform localRoot = localCentred * new WorldTransform(pair.Key * 1000);
                    WorldTransform wt = Bubbles.ToGlobal(owner.b, localRoot, t);
                    pair.Value.UpdateTransform(in wt);
                }
            }
            return false;
        }
        lock (roots)
        {
            foreach (var pair in roots)
            {
                WorldTransform wt = baseTransform * new WorldTransform(pair.Key * 1000);
                pair.Value.UpdateTransform(in wt);
            }
        }
        return false;
    }
}

/// <summary>
/// VoxelCell.Prepare (VRage.Voxels.Client\...\Clipmap\VoxelCell.cs:184-199) caches _worldTransform from
/// RootEntityProvider.GetRootEntity (the GLOBAL pose) and only ever uses it in UpdateRenderSettings /
/// RecordUpdateRenderSettings (:294, :340) to compute root-local SpherizeCenter = Inv(_worldTransform) *
/// renderSetup.SpherizePosition. Because SpherizePosition stays at the planet's local C0 while an orbiting moon's
/// global root is up to ~392 km away, SpherizeCenter was off by 392 km and the terrain shader spherized the moon's
/// surface away (leaving only the atmosphere/cloud shell). Setting _worldTransform to the root's LOCAL transform makes
/// Inv(localRoot) * C0 == Inv(ToGlobal(localRoot, t)) * ToGlobal(C0, t) exact for all t, spin, and orbit.
/// </summary>
[HarmonyPatch(typeof(Keen.VRage.Voxels.Client.Clipmap.VoxelCell), nameof(Keen.VRage.Voxels.Client.Clipmap.VoxelCell.Prepare))]
internal static class VoxelCellSpherizePatch
{
    [HarmonyPostfix]
    private static void Postfix(Keen.VRage.Voxels.Client.Clipmap.VoxelCell __instance, Keen.VRage.Voxels.Client.Clipmap.VoxelClipmap voxel)
    {
        if (!Bubbles.Ready) return;
        if (!Bubbles.ClipmapBubble.TryGetValue(voxel, out Bubbles.Bubble? b) || b.IsIdentity) return;
        if (!Bubbles.ClipmapRender.TryGetValue(voxel, out var r)) return;
        WorldTransform centredLocal = Bubbles.InitialCentred.TryGetValue(r, out WorldTransform initC)
            ? initC
            : r._voxelStorageComponent.ComputeCenteredTransform();
        __instance._worldTransform = centredLocal * new WorldTransform(__instance._rootOffset);
    }
}
