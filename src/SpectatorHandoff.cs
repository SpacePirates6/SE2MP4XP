using System;
using HarmonyLib;
using Keen.Game2.Client.WorldObjects.Spectators;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Library.Mathematics;

namespace SpinProbe;

/// <summary>
/// Seamless frame hand-off for the client-only SpectatorComponent.
/// When the spectator crosses a moving planet bubble's boundary (with hysteresis), its WorldTransform and linear
/// velocity are converted in-place between bubble-local and global coordinates so the rendered view does not snap
/// in position or orientation — inside the bubble it co-rotates with the planet, outside it sits in global space
/// watching the planet spin underneath.
/// </summary>
internal static class SpectatorHandoff
{
    private const double Band = 100.0;
    private static SpectatorComponent? _active;
    private static Bubbles.Bubble? _member;
    private static bool _initialized;
    private static Vector3D _lastPos;
    private static long _handoffs;

    /// <summary>
    /// Resolves which bubble the camera's raw ECS pose is expressed in. When the possessed spectator is driving
    /// the camera, returns its hysteresis-tracked bubble so CameraPatch and SpectatorComponent always agree across
    /// the ±Band zone; otherwise falls back to Bubbles.BubbleAt(pos).
    /// </summary>
    internal static Bubbles.Bubble? ResolveCameraBubble(in Vector3D cameraPos)
    {
        SpectatorComponent? s = _active;
        if (_initialized && s != null && s._controller != null && !s.IsLockActive)
        {
            try
            {
                Vector3D sp = s.Data.GetWorldTransform().Position;
                if ((sp - cameraPos).LengthSquared() < 25.0)
                    return _member;
            }
            catch { }
        }
        if (Settings.RealPhysics && Settings.RealBubbleHandoff && BubbleHandoff.TryResolveClientCameraBubble(in cameraPos, out Bubbles.Bubble? hb))
            return hb;
        return Bubbles.BubbleAt(cameraPos);
    }

    private static Vector3D RotVec(in Vector3D v, in Vector3D axis, double a)
    {
        double c = Math.Cos(a), s = Math.Sin(a);
        return v * c + Vector3D.Cross(axis, v) * s + axis * (Vector3D.Dot(axis, v) * (1 - c));
    }

    [HarmonyPatch(typeof(SpectatorComponent), nameof(SpectatorComponent.Update))]
    internal static class UpdatePatch
    {
        [HarmonyPostfix]
        private static void Postfix(SpectatorComponent __instance)
        {
            if (!Bubbles.Ready || __instance._controller == null || __instance.IsLockActive) return;
            try
            {
                WorldTransform wt = __instance.Data.GetWorldTransform();
                double t = Bubbles.T;

                if (!ReferenceEquals(_active, __instance) || !_initialized)
                {
                    _active = __instance;
                    _member = Bubbles.BubbleAt(wt.Position);
                    _lastPos = wt.Position;
                    _initialized = true;
                    return;
                }

                // Teleport guard (e.g. spectator reset / jump to another planet): re-sync membership without converting.
                if ((wt.Position - _lastPos).LengthSquared() > 5000.0 * 5000.0)
                {
                    _member = Bubbles.BubbleAt(wt.Position);
                    _lastPos = wt.Position;
                    return;
                }

                if (_member != null)
                {
                    if (_member.IsIdentity)
                    {
                        if ((wt.Position - _member.C0).Length() > _member.Radius + Band) _member = null;
                    }
                    else if ((wt.Position - _member.C0).Length() > _member.Radius + Band)
                    {
                        // Leaving moving bubble -> convert spectator pose & velocity into global space seamlessly.
                        Bubbles.Bubble b = _member;
                        WorldTransform g = Bubbles.ToGlobal(b, wt, t);
                        if (b.SpinRate != 0)
                            __instance._linearVelocity = RotVec(__instance._linearVelocity, b.SpinAxis, b.SpinRate * t);
                        __instance.Data.SetWorldTransform(in g);
                        _member = null;
                        _lastPos = g.Position;
                        long n = ++_handoffs;
                        ProbeLog.Write($"handoff spectator #{n}: {b.Name} -> global at r={(wt.Position - b.C0).Length():0}/{b.Radius:0} t={t:0.0}s");
                        return;
                    }
                }
                else
                {
                    // In global space -> check if we entered any moving bubble's live sphere.
                    foreach (Bubbles.Bubble b in Bubbles.All)
                    {
                        Vector3D cNow = Bubbles.Centre(b, t);
                        if ((wt.Position - cNow).Length() < b.Radius - Band)
                        {
                            if (b.IsIdentity) { _member = b; break; }
                            WorldTransform l = Bubbles.ToLocal(b, wt, t);
                            if (b.SpinRate != 0)
                                __instance._linearVelocity = RotVec(__instance._linearVelocity, b.SpinAxis, -b.SpinRate * t);
                            __instance.Data.SetWorldTransform(in l);
                            _member = b;
                            _lastPos = l.Position;
                            long n = ++_handoffs;
                            ProbeLog.Write($"handoff spectator #{n}: global -> {b.Name} at r={(wt.Position - cNow).Length():0}/{b.Radius:0} t={t:0.0}s");
                            return;
                        }
                    }
                }

                _lastPos = wt.Position;
            }
            catch (Exception e)
            {
                ProbeLog.Write("handoff spectator error: " + e.Message);
            }
        }
    }
}
