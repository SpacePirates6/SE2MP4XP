using Keen.Game2.Client.WorldObjects.CubeBlocks.Effects;
using Keen.Game2.Simulation.WorldObjects.CubeBlocks;
using Keen.Game2.Simulation.WorldObjects.CubeBlocks.Movement;
using Keen.Game2.Simulation.WorldObjects.Movement;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Components;
using Keen.VRage.Library.Mathematics;

namespace SpinProbe;

/// <summary>
/// Client side of visible RCS (FlightComputer.PublishRcs). Vanilla sets every thruster's flame from its grid's per-axis
/// thrust (ThrusterEffectsComponent.UpdateThrusterAnimationData, ThrustFlameAnimator.Update: flame =
/// Saturate(GetControlMultiplier(parent VoluntaryThrustData, direction)) × efficiency); the next group,
/// ThrustFlameAnimator.Animate, eases the drawn flame toward CurrentFlameScale. In between, a thruster that carries its own
/// replicated VoluntaryThrustData (RCS firing beyond the grid's value) gets the larger of the two.
/// </summary>
[After(typeof(ThrusterEffectsComponent.ThrustFlameAnimator.Update))]
[Before(typeof(ThrusterEffectsComponent.ThrustFlameAnimator.Animate))]
internal class RcsFlameGroup : JobGroup
{
}

internal static class RcsFlameJobs
{
    [RcsFlameGroup]
    [MustHave(typeof(ThrusterEffectsComponent.AnimationData))]
    [DoesNotHave(typeof(ThrusterOverrideData))]
    [DoesNotHave(typeof(DisabledTag))]
    internal static void RaiseRcsFlame(in VoluntaryThrustData own, in ThrustData thrust, in ThrusterEffectsComponent.EfficiencyData efficiency,
        Writable<ThrusterEffectsComponent.AnimationData> animation)
    {
        float u = MathHelper.Saturate(Throttle(own.VoluntaryThrust, thrust.Direction)) * efficiency.Efficiency;
        if (u <= 0f) return;
        var scales = animation.Value.CurrentFlameScale;
        bool raise = false;
        for (int i = 0; i < scales.Count; i++) if (scales[i] < u) { raise = true; break; }
        if (!raise) return;
        var w = animation.WritableValue.CurrentFlameScale;
        for (int i = 0; i < w.Count; i++) if (w[i] < u) w[i] = u;
    }

    // ThrusterEffectsComponent.<UpdateThrusterAnimationData>g__GetControlMultiplier|32_0 (:708): the component along the
    // direction the thruster pushes (Forward = -Z, Backward = +Z, Left = -X, Right = +X, Down = -Y, Up = +Y).
    private static float Throttle(in Vector3 v, Base6Directions.Direction d) => d switch
    {
        Base6Directions.Direction.Forward => -v.Z,
        Base6Directions.Direction.Backward => v.Z,
        Base6Directions.Direction.Left => -v.X,
        Base6Directions.Direction.Right => v.X,
        Base6Directions.Direction.Down => -v.Y,
        Base6Directions.Direction.Up => v.Y,
        _ => 0f,
    };
}
