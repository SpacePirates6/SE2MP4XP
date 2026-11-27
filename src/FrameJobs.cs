using System;
using System.Collections.Generic;
using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Havok;
using Keen.VRage.Physics.Havok.Core;
using Keen.VRage.Voxels.Components;

namespace SpinProbe;

/// <summary>Jobs and patches for the rotating physics frame (see <see cref="Frame"/>).</summary>
internal static class FrameJobs
{
    private static readonly Dictionary<DEntity, (float radius, WorldTransform wt)> Candidates = new();
    private static bool _chosen;
    private static long _refreshed;

    private static Vector3D _characterPos;
    private static bool _haveCharacter;

    /// <summary>Where a character is (any). Characters carry CharacterInfo (VRage.Physics\...\Havok\Character\CharacterInfo.cs).</summary>
    [PhysicsUpdate.OnCollectMassProperties]
    [MustHave(typeof(Keen.VRage.Physics.Havok.Character.CharacterInfo))]
    internal static void SeeCharacter(in WorldTransform wt)
    {
        if (_haveCharacter) return;
        _characterPos = wt.Position;
        _haveCharacter = true;
    }

    /// <summary>
    /// Pick the planet to spin: the VoxelPlanetComponent (VRage.Voxels\...\Components\VoxelPlanetComponent.cs:52 Radius) whose
    /// surface is nearest to a character. Run 11 picked by largest radius and got a different R = 60 km planet 4,200 km away.
    /// Rotation starts at theta = 0, so choosing a few ticks late changes nothing.
    /// </summary>
    [PhysicsUpdate.OnCollectMassProperties]
    internal static void ChoosePlanet(DEntity entity, VoxelPlanetComponent planet, in WorldTransform wt)
    {
        if (_chosen || Settings.Mode != "frame") return;
        Candidates[entity] = (planet.Radius, wt);
        if (Profiler.Ticks < 3 || !_haveCharacter) return;
        _chosen = true;
        DEntity best = default; float bestRadius = 0; WorldTransform bestWt = default; double bestGap = double.MaxValue;
        foreach (var c in Candidates)
        {
            double gap = (c.Value.wt.Position - _characterPos).Length() - c.Value.radius;
            if (gap < bestGap) { bestGap = gap; best = c.Key; bestRadius = c.Value.radius; bestWt = c.Value.wt; }
        }
        ProbeLog.Write($"frame: character at {_characterPos}, {bestGap:0} m above the nearest planet surface");
        Frame.P = bestWt.Position;
        Frame.Axis = Vector3D.Normalize((Vector3D)Vector3.Transform(new Vector3(0, 1, 0), bestWt.Orientation));
        Frame.Omega = Settings.DegreesPerSecond * Math.PI / 180.0;
        Frame.Radius = bestRadius * Settings.FrameRadiusFactor;
        Frame.ThetaEcs = 0;
        Frame.Active = true;
        ProbeLog.Write($"frame: ACTIVE on planet {best} R={bestRadius:0} frameRadius={Frame.Radius:0} centre={Frame.P} axis={Frame.Axis} " +
                       $"omega={Frame.Omega:0.000000} rad/s ({Settings.DegreesPerSecond} deg/s); candidates={Candidates.Count}");
    }

    /// <summary>
    /// Advance every body Havok did not read back this tick (sleeping dynamics, statics - terrain chunks, the planet's own
    /// body, static grids) by one tick of rotation, and give it the co-rotating velocity omega x r.
    /// <para>Placement: PhysicsUpdate.OnUpdateCharacterPhysicsState (VRage.Physics\...\Havok\PhysicsUpdate.cs:325) runs after the
    /// read-back groups. DataEventGroup(InternalBodyUpdateEventGroup) is the group the engine's own read-back uses
    /// (HavokSessionComponent.cs:1965 UpdateDataFromDynamicBodies): writes made in it do NOT trigger
    /// CollectDataForStaticBodies (:1730, teleport), CollectChangesForTransform (:1754) or CollectChangesForVelocities, which
    /// are in the same group - so nothing is re-sent to Havok and nothing wakes up. Other listeners (render, replication,
    /// cluster bookkeeping, attached triggers) do see the change.</para>
    /// <para>ActiveTag mirrors Havok activation (HavokSessionComponent.cs:1904-1950); bodies with it were read back by
    /// UpdateDataFromDynamicBodies and converted by FrameStubs.</para>
    /// </summary>
    [PhysicsUpdate.OnUpdateCharacterPhysicsState]
    [DataEventGroup(typeof(InternalBodyUpdateEventGroup))]
    [DoesNotHave(typeof(ActiveTag))]
    internal static void Refresh(ref WorldTransform wt, ref RigidBodyData rb)
    {
        if (!Frame.Active || !Frame.SteppedThisTick || !Frame.Inside(wt.Position)) return;
        double d = Frame.Delta;
        wt.Position = Frame.Rot(wt.Position - Frame.P, d) + Frame.P;
        wt.Orientation = Quaternion.Normalize(Frame.RotQ(d) * wt.Orientation);
        rb.LinearVelocity = Frame.CoRotating(wt.Position);
        rb.AngularVelocity = (Vector3)Frame.OmegaVec;
        _refreshed++;
    }

    internal static long TakeRefreshed() { long n = _refreshed; _refreshed = 0; return n; }
}

/// <summary>
/// Body creation pose. Source: VRage.Physics\...\Havok\BodyCreationInfo.cs:36-109 SetForNewEntity reads the entity's ECS
/// WorldTransform / RigidBodyData directly (not through a job stub) and sets
/// <c>m_position = SpaceConverter.ToHavokSpaceD(wt.Position, worldData)</c>, <c>m_orientation = wt.Orientation</c>, and the
/// velocities unless TransformDriven. Every body allocation goes through it (HavokWorldHelper.AllocateBody, including statics
/// added to a cluster later: HavokLargeWorldManager.AddStaticEntityToCluster). The postfix rewrites those four fields in the
/// frame.
/// </summary>
[HarmonyPatch(typeof(BodyCreationInfo), nameof(BodyCreationInfo.SetForNewEntity))]
internal static unsafe class FrameCreationPatch
{
    internal static long Converted;

    [HarmonyPostfix]
    private static void Postfix(ref BodyCreationInfo __instance, Keen.VRage.DCS.Accessors.DEntityContext entity, HknpWorld* world)
    {
        if (!Frame.Active) return;
        try
        {
            if (!entity.TryGet(out WorldTransform wt) || !Frame.Inside(wt.Position)) return;
            Vector3D worldPos = wt.Position;
            WorldTransform f = wt;
            Frame.ToFrame(ref f, Frame.ThetaEcs);
            ref HknpBodyCinfo.__Internal m = ref __instance.CInfo.Members;
            m.m_position = f.Position - world->WorldData->ClusterOffset3D;
            m.m_orientation = f.Orientation;
            if (!entity.Has<TransformDriven>() && entity.TryGet(out RigidBodyData rb))
            {
                Frame.VelToFrame(ref rb, worldPos, Frame.ThetaEcs);
                m.m_linearVelocity = new Vector4(rb.LinearVelocity, 0f);
                m.m_angularVelocity = new Vector4(rb.AngularVelocity, 0f);
            }
            Converted++;
        }
        catch (Exception e) { ProbeLog.Write("frame creation patch error: " + e.Message); }
    }
}
