using System;
using Keen.VRage.Core;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Data;

namespace SpinProbe;

/// <summary>
/// The rotating physics frame (docs/ENGINE-RE-ROTATION.md §10.3, ENGINE-ARCHITECTURE.md Part E).
/// <para>Havok simulates everything within <see cref="Radius"/> of the planet centre <see cref="P"/> in PLANET-FIXED
/// coordinates, so the terrain never moves inside Havok: it stays static, bodies on it sleep, collision caches persist.
/// The ECS (WorldTransform / RigidBodyData) keeps true world coordinates, so render, replication and game logic see the
/// planet and everything on it turning. The rotation lives only in the conversion at the physics boundary:</para>
/// <code>
/// frame  = R(θ)⁻¹ (world − P) + P            q_f = R⁻¹ q_w
/// v_f    = R⁻¹ (v_w − ω × (x_w − P))         w_f = R⁻¹ (w_w − ω)
/// world  = R(θ) (frame − P) + P              q_w = R q_f
/// v_w    = R v_f + ω × (x_w − P)             w_w = R w_f + ω
/// </code>
/// <para>Native Havok then subtracts its translation-only cluster offset as usual (SpaceConverter, HkWorldData.ClusterOffset).</para>
/// <para>Time: <see cref="ThetaEcs"/> is the angle the ECS world poses are expressed at. Inside a server tick, everything
/// collected into Havok is converted with ThetaEcs; everything read back after the step is converted with ThetaEcs + Δ,
/// and <see cref="FrameJobs.Refresh"/> advances the bodies Havok did not read back (sleeping / static) by Δ. At the end
/// of the tick ThetaEcs += Δ.</para>
/// </summary>
internal static class Frame
{
    internal static bool Active;
    internal static Vector3D P;
    internal static Vector3D Axis = new(0, 1, 0);   // world-space unit spin axis (fixed in the inertial frame)
    internal static double Omega;                   // rad/s
    internal static double Radius;                  // frame region radius around P
    internal static double ThetaEcs;
    internal static bool SteppedThisTick;
    internal static double Delta => Omega / 60.0;   // fixed 1/60 s server step (Game2.Simulation\...\UpdateTime.cs:36)

    internal static double ThetaForWorld => ThetaEcs + (SteppedThisTick ? Delta : 0);

    internal static void EndTick()
    {
        if (Active) ThetaEcs += Delta;
        SteppedThisTick = false;
    }

    internal static bool Inside(in Vector3D x) => Active && (x - P).LengthSquared() < Radius * Radius;

    // --- rotation about Axis by angle (double precision, Rodrigues) ---
    internal static Vector3D Rot(in Vector3D v, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        Vector3D k = Axis;
        Vector3D cross = Vector3D.Cross(k, v);
        double dot = Vector3D.Dot(k, v);
        return v * c + cross * s + k * (dot * (1 - c));
    }

    internal static Vector3 RotF(in Vector3 v, double angle) => (Vector3)Rot((Vector3D)v, angle);

    internal static Quaternion RotQ(double angle) =>
        Quaternion.CreateFromAxisAngle((Vector3)Axis, (float)angle);

    internal static Vector3D OmegaVec => Axis * Omega;

    // --- poses ---
    internal static void ToFrame(ref WorldTransform wt, double theta)
    {
        wt.Position = Rot(wt.Position - P, -theta) + P;
        wt.Orientation = Quaternion.Normalize(RotQ(-theta) * wt.Orientation);
    }

    internal static void ToWorld(ref WorldTransform wt, double theta)
    {
        wt.Position = Rot(wt.Position - P, theta) + P;
        wt.Orientation = Quaternion.Normalize(RotQ(theta) * wt.Orientation);
    }

    // --- velocities (worldPos = the body's WORLD position, needed for omega x r) ---
    internal static void VelToFrame(ref RigidBodyData rb, in Vector3D worldPos, double theta)
    {
        Vector3D r = worldPos - P;
        Vector3D v = (Vector3D)rb.LinearVelocity - Vector3D.Cross(OmegaVec, r);
        Vector3D w = (Vector3D)rb.AngularVelocity - OmegaVec;
        rb.LinearVelocity = (Vector3)Rot(v, -theta);
        rb.AngularVelocity = (Vector3)Rot(w, -theta);
    }

    internal static void VelToWorld(ref RigidBodyData rb, in Vector3D worldPos, double theta)
    {
        Vector3D r = worldPos - P;
        Vector3D v = Rot((Vector3D)rb.LinearVelocity, theta) + Vector3D.Cross(OmegaVec, r);
        Vector3D w = Rot((Vector3D)rb.AngularVelocity, theta) + OmegaVec;
        rb.LinearVelocity = (Vector3)v;
        rb.AngularVelocity = (Vector3)w;
    }

    /// <summary>Surface velocity of a point co-rotating with the planet.</summary>
    internal static Vector3 CoRotating(in Vector3D worldPos) => (Vector3)Vector3D.Cross(OmegaVec, worldPos - P);
}
