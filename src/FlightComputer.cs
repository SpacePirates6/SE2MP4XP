using System;
using System.Collections.Concurrent;
using System.Threading;
using Keen.Game2.Simulation.WorldObjects.CubeBlocks;
using Keen.Game2.Simulation.WorldObjects.CubeBlocks.Movement;
using Keen.Game2.Simulation.WorldObjects.CubeGrids;
using Keen.Game2.Simulation.WorldObjects.CubeGrids.Movement;
using Keen.Game2.Simulation.WorldObjects.Movement;
using Keen.Game2.Simulation.WorldObjects.Shared.Movement;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Data;
using Keen.VRage.Physics.Havok.MotionSharing;

namespace SpinProbe;

/// <summary>
/// 6-DOF Flight Computer (bounded thruster control allocator + RCS turning + whole-ship subgrid CoM).
///
/// Why this exists:
/// Raw per-thruster torque without control allocation forces every ship to be built symmetrically around its exact CoM
/// and violently spins ships that carry rotor/hinge subgrids (landing gear, ramps, bays). Real spacecraft and VTOL
/// aircraft use a flight computer that trims per-thruster throttles u_i in [0, 1] so sleek/asymmetric thruster layouts
/// fly straight, while also using offset thrusters as Reaction Control System (RCS) attitude thrusters for turning!
///
/// Realism preserved:
/// - Every thruster is bounded to u_i in [0, 1] and multiplied by its live ThrustClass efficiency (e.g. atmospheric
///   thrusters produce 0 thrust in space).
/// - Trimming an offset thruster reduces net linear thrust slightly (real thrust-vectoring cost), applied as the exact
///   linear delta from vanilla's CoM thrust.
/// - If a ship has a truly unbalanceable single offset thruster with no counter-thrusters, the solver preserves pilot
///   thrust floor and produces genuine residual offset torque.
/// - Pilot angular steering (AngularControlData) fires opposing offset thrusters as RCS, cooperating smoothly with
///   gyroscopes (or turning ships with zero gyros!).
/// </summary>
internal static class FlightComputerJobs
{
    private const float Dt = 1f / 60f;
    private const int MaxActiveThrusters = 128;
    private const int SolverIterations = 12;

    internal static long SolvedGrids, RcsAssistedGrids, TrimmedGrids;
    private static int _errors;

    private struct ThrusterEntry
    {
        public Vector3 Force;      // Newtons (local) at u = 1
        public Vector3 TorqueNorm; // (r x F) / L_char (force-equivalent Newtons)
        public Vector3 Torque;     // r x F (N*m)
        public float MaxThrust;
        public float Diag;         // coordinate-descent normal diagonal
        public float U;            // solved throttle [0, 1]
        public Vector3 Dir;
        public DEntity Thruster;   // set when not binned
    }

    // ---------- RCS made visible and paid, without thruster overrides ----------
    // The game has one thrust value per axis for the whole grid (VoluntaryThrustData), so a thruster the solver fires
    // in a direction vanilla is not firing would show no flame and draw no power. Such a thruster gets its own
    // VoluntaryThrustData (its direction × throttle) on the thruster entity:
    //  - ServerSessionComponent.SyncVoluntaryThrust replicates VoluntaryThrustData of ANY entity on change (only
    //    NoAutoSync entities are skipped: dead characters, spectators), so clients receive it with no new network code;
    //  - every vanilla reader looks at the grid's (or character's) value: ThrustEffectsComponent/JetpackFuel need
    //    ThrustComponent or JetpackModeData, the flame jobs read the PARENT's value, InformThrusters needs
    //    ThrusterCollectionData - none of them sees a thruster's own value;
    //  - the client raises that thruster's flame (RcsFlameJobs), the server gives it IsThrusting, which is what makes a
    //    thruster consume power/fuel (ThrusterComponent.OnStartFiring :112-122, binary like vanilla).
    // Nothing touches ThrusterOverrideData, so the terminal shows no override.
    internal static readonly ConcurrentDictionary<DEntity, (DEntity thruster, Vector3 dir)[]> RcsByGrid = new();
    internal static readonly ConcurrentDictionary<DEntity, long> RcsLastMs = new();
    internal static readonly ConcurrentDictionary<DEntity, byte> RcsActive = new();
    private const float RcsShowMargin = 0.05f; // throttle above vanilla's before a thruster gets its own flame
    private const long StaleRcsMs = 300;       // release grids that crashed/split/docked/powered off and left SolveThrusterMix
    private static long _lastSweepMs;
    internal static long RcsThrusters;

    private static float VanillaThrottle(in Vector3 frac, in Vector3 dir) => Math.Clamp(Vector3.Dot(frac, dir), 0f, 1f);

    private static void PublishRcs(DEntity grid, Span<ThrusterEntry> entries, int count, in Vector3 frac,
        EntityData<VoluntaryThrustData> vols, EntityData<IsThrusting> firing, EntityData<ThrusterOverrideData> overrides)
    {
        RcsByGrid.TryGetValue(grid, out var prev);
        int n = 0;
        Span<int> shown = stackalloc int[count];
        for (int i = 0; i < count; i++)
        {
            ref ThrusterEntry e = ref entries[i];
            if (e.Thruster.IsEmpty) continue;
            float vanilla = VanillaThrottle(in frac, in e.Dir);
            float show = e.U - vanilla > RcsShowMargin ? MathF.Round(e.U * 20f) / 20f : 0f;
            try
            {
                SetOwn(e.Thruster, e.Dir * show, vols);
                if (show > 0f)
                {
                    shown[n++] = i;
                    RcsActive[e.Thruster] = 0;
                    if (!firing.IsPresentOn(e.Thruster)) firing.WriteTo(e.Thruster, default(IsThrusting));
                }
            }
            catch { }
        }
        // thrusters that stopped RCS firing: hand IsThrusting back to vanilla's rule (direction active or overridden)
        if (prev != null)
            foreach (var (t, dir) in prev)
            {
                bool still = false;
                for (int k = 0; k < n; k++) if (entries[shown[k]].Thruster == t) { still = true; break; }
                if (still) continue;
                Release(t, dir, in frac, vols, firing, overrides);
            }
        if (n == 0)
        {
            RcsByGrid.TryRemove(grid, out _);
            RcsLastMs.TryRemove(grid, out _);
            return;
        }
        var now = new (DEntity, Vector3)[n];
        for (int k = 0; k < n; k++) now[k] = (entries[shown[k]].Thruster, entries[shown[k]].Dir);
        RcsByGrid[grid] = now;
        RcsLastMs[grid] = Environment.TickCount64;
        Interlocked.Add(ref RcsThrusters, n);
    }

    private static void ClearRcs(DEntity grid, in Vector3 frac, EntityData<VoluntaryThrustData> vols, EntityData<IsThrusting> firing,
        EntityData<ThrusterOverrideData> overrides)
    {
        RcsLastMs.TryRemove(grid, out _);
        if (!RcsByGrid.TryRemove(grid, out var prev)) return;
        foreach (var (t, dir) in prev) Release(t, dir, in frac, vols, firing, overrides);
    }

    private static void Release(DEntity t, in Vector3 dir, in Vector3 frac, EntityData<VoluntaryThrustData> vols, EntityData<IsThrusting> firing,
        EntityData<ThrusterOverrideData> overrides)
    {
        // Always remove from RcsActive first so dead/destroyed thrusters never leak rcsLit or keep KeepRcsFiringPatch active.
        RcsActive.TryRemove(t, out _);
        try
        {
            SetOwn(t, Vector3.Zero, vols);
            if (VanillaThrottle(in frac, in dir) <= 0.001f && !overrides.IsPresentOn(t)) firing.TryRemoveFrom(t);
        }
        catch { }
    }

    /// <summary>Sweeps grids that stopped matching SolveThrusterMix (crashed, split, docked, powered off, converted static,
    /// or lost all thrusters) while RCS thrusters were lit, releasing any surviving thrusters and clearing RcsActive.</summary>
    [Keen.VRage.Core.Systems.OnAfterSimulation]
    [MustHave(typeof(Keen.VRage.DCS.CoreData.InstanceBind<Keen.Game2.Simulation.WorldObjects.Characters.CharacterComponent>))]
    internal static void SweepStaleRcs(
        Keen.Game2.Simulation.WorldObjects.Characters.CharacterComponent ch,
        EntityData<VoluntaryThrustData> vols,
        EntityData<IsThrusting> firing,
        EntityData<ThrusterOverrideData> overrides)
    {
        if (ch == null || RcsByGrid.IsEmpty) return;
        long now = Environment.TickCount64;
        long last = Volatile.Read(ref _lastSweepMs);
        if (now - last < 200) return;
        if (Interlocked.CompareExchange(ref _lastSweepMs, now, last) != last) return;

        Vector3 zero = Vector3.Zero;
        foreach (var kv in RcsByGrid)
        {
            if (RcsLastMs.TryGetValue(kv.Key, out long seen) && now - seen <= StaleRcsMs) continue;
            RcsLastMs.TryRemove(kv.Key, out _);
            if (RcsByGrid.TryRemove(kv.Key, out var entries))
            {
                foreach (var (t, dir) in entries) Release(t, dir, in zero, vols, firing, overrides);
            }
        }
    }

    private static void SetOwn(DEntity thruster, in Vector3 value, EntityData<VoluntaryThrustData> vols)
    {
        bool has = vols.TryReadFrom(thruster, out VoluntaryThrustData cur);
        if (has ? cur.VoluntaryThrust == value : Vector3.IsZero(value)) return; // only write changes (each write replicates)
        vols.WriteTo(thruster, new VoluntaryThrustData { VoluntaryThrust = value });
    }

    /// <summary>InformThrustersWhetherTheyAreThrusting (GridMovementCollectorComponent.cs:552-578) rewrites IsThrusting for
    /// every thruster of a grid whenever the grid's thrust changes; this keeps it on the thrusters RCS is firing.</summary>
    [HarmonyLib.HarmonyPatch(typeof(GridMovementCollectorComponent), "InformThrustersWhetherTheyAreThrusting")]
    internal static class KeepRcsFiringPatch
    {
        [HarmonyLib.HarmonyPostfix]
        private static void Postfix(in GridMovementCollectorComponent.ThrusterCollectionData gridThrusters,
            EntityData<DisabledTag> disabledTagData, EntityData<IsThrusting> isThrustingData)
        {
            if (RcsActive.IsEmpty) return;
            try
            {
                foreach (var item in gridThrusters.Thrusters)
                    if (RcsActive.ContainsKey(item.Thruster) && !disabledTagData.IsPresentOn(item.Thruster))
                        isThrustingData.WriteTo(item.Thruster, default(IsThrusting));
            }
            catch { }
        }
    }

    [ThrustComponent.OnComputeThrust.End]
    [WhenSimulated]
    [MustHave(typeof(ThrustComponent.ThrustEnabled))]
    [MustHave(typeof(ThrustComponent.NeedsUpdateThrust))]
    [DoesNotHave(typeof(ConnectedToStaticEntity))]
    internal static void SolveThrusterMix(
        DEntity entity,
        CubeGridComponent grid,
        EntityData<VoluntaryThrustData> voluntaries,
        EntityData<IsThrusting> firing,
        EntityData<ThrusterOverrideData> thrusterOverrides,
        in MaxThrustData max,
        Optional<OverriddenThrustData> overridden,
        in GridMovementCollectorComponent.ThrusterCollectionData thrusters,
        in WorldTransform wt,
        in RigidBodyMassProperties mass,
        Optional<SharedMotionData> shared,
        Optional<AngularControlData> angularControl,
        Optional<DampeningData> dampening,
        Optional<GridGyroscopesComponent.ActiveGridGyroscopeData> activeGyro,
        Optional<MovementData> movement,
        EntityData<RelativeTransform> transforms,
        Writable<RigidBodyData> body)
    {
        bool published = false;
        Vector3 frac = Vector3.Zero;
        try
        {
            if (!RealPhysics.On || !Settings.RealThrustTorque || !Settings.RealFlightComputer) return;
            if (shared.HasValue || mass.InvMass <= 0f) return;
            if (!voluntaries.TryReadFrom(entity, out VoluntaryThrustData voluntary)) return;

            // 1. Reconstruct vanilla linear thrust command (includes hover gravity cancellation + dampeners + WASD + overrides)
            frac = Vector3.Clamp(voluntary.VoluntaryThrust, -Vector3.One, Vector3.One);
            Vector3 fVol = frac * max.Regular.SignSelectAbs(in frac);
            Vector3 fOvr = overridden.HasValue ? overridden.Value.DirectionalThrust : Vector3.Zero;
            Vector3 fCmd = fVol + fOvr;
            if (!fCmd.IsValid()) fCmd = Vector3.Zero;

            // 2. Compute desired RCS angular torque (if enabled and pilot is steering / dampening)
            Vector3 tauRcsCmd = Vector3.Zero;
            bool wantRcs = false;
            if (Settings.RealRcs && angularControl.HasValue && movement.HasValue)
            {
                Vector3 targetW = angularControl.Value.TargetAngularVelocity;
                Vector3 curW = WorldTransform.TransformDirectionInv(body.Value.AngularVelocity, wt);
                bool dampOn = dampening.HasValue;

                Vector3 errW;
                if (dampOn)
                {
                    errW = targetW - curW;
                }
                else
                {
                    Vector3 mask = new(targetW.X == 0f ? 0f : 1f, targetW.Y == 0f ? 0f : 1f, targetW.Z == 0f ? 0f : 1f);
                    errW = (targetW - curW) * mask;
                }

                if (errW.LengthSquared() > 1e-5f)
                {
                    var rot = movement.Value.Definition.Value.AngularGyroRotation;
                    Vector3 stepW = errW;
                    if (targetW.LengthSquared() < curW.LengthSquared() || Vector3.Dot(targetW, curW) < 0f)
                        stepW = Vector3.ClampToSphere(errW, rot.MaximumAngularDeceleration * Dt);

                    // Ease out near zero angular error so RCS never chatters
                    float errMag = errW.Length();
                    float ease = errMag < 0.25f ? Math.Clamp(errMag / 0.25f, 0.15f, 1f) : 1f;
                    Vector3 impNeeded = mass.LocalAngularVelocityToAngularImpulseLocal(in stepW) * ease;

                    // Subtract whatever gyroscopes already applied on the same axis so RCS assists instead of fighting
                    if (activeGyro.HasValue)
                    {
                        Vector3 gImp = activeGyro.Value.ComputedGyroscopePerFrame;
                        impNeeded = new Vector3(
                            SubtractSameSign(impNeeded.X, gImp.X),
                            SubtractSameSign(impNeeded.Y, gImp.Y),
                            SubtractSameSign(impNeeded.Z, gImp.Z));
                    }

                    // Scale torque budget slightly when connected subgrids add inertia
                    tauRcsCmd = impNeeded / Dt;
                    if (tauRcsCmd.IsValid() && tauRcsCmd.LengthSquared() > 1f)
                        wantRcs = true;
                    else
                        tauRcsCmd = Vector3.Zero;
                }
            }

            bool wantLinear = fCmd.LengthSquared() > 1e-2f;
            if (!wantLinear && !wantRcs) return;

            // 3. Whole-ship CoM (includes connected rotor/hinge/piston subgrids!)
            Vector3 com = ShipAssembly.GetWholeShipLocalCoM(grid, in wt, in mass, out ShipAssembly.GroupInfo? group);
            if (group != null && group.HasStaticMember) return;
            float massScale = (group != null && group.PrimaryMass > 1f)
                ? Math.Clamp(group.TotalMass / Math.Max(1f / mass.InvMass, 1f), 1f, 4f)
                : 1f;
            if (wantRcs && massScale > 1.05f)
                tauRcsCmd *= MathF.Sqrt(massScale);

            // 4. Gather active thrusters with live ThrustClass efficiency
            Span<ThrusterEntry> entries = stackalloc ThrusterEntry[MaxActiveThrusters];
            int count = 0;
            float leverSum = 0f;
            float thrustSum = 0f;

            // Track total available thrust in each of 6 directions so we can handle >128-thruster mega-ships via binning
            int totalThrusters = thrusters.Thrusters.Count;
            bool needBinning = totalThrusters > MaxActiveThrusters;

            int classCount = thrusters.ThrustClasses.Count;
            for (int c = 0; c < classCount; c++)
            {
                var cls = thrusters.ThrustClasses[c];
                float eff = cls.Efficiency;
                if (eff <= 1e-4f) continue;
                int start = cls.StartIndex;
                int end = Math.Min(start + cls.Count, totalThrusters);
                for (int i = start; i < end; i++)
                {
                    var item = thrusters.Thrusters[i];
                    float tMax = item.CurrentMaxThrust * eff;
                    if (tMax <= 1e-2f) continue;
                    if (!transforms.TryReadFrom(item.Thruster, out RelativeTransform rt)) continue;

                    Vector3 dir = DirectionVec(item.Direction);
                    Vector3 r = rt.Position - com;
                    Vector3 force = dir * tMax;
                    Vector3 torque = Vector3.Cross(r, force);

                    if (!needBinning)
                    {
                        if (count >= MaxActiveThrusters) break;
                        entries[count++] = new ThrusterEntry
                        {
                            Force = force,
                            Torque = torque,
                            MaxThrust = tMax,
                            Dir = dir,
                            Thruster = item.Thruster,
                        };
                    }
                    else
                    {
                        // Merge into an existing bin sharing the same direction & nearby quadrant
                        int bin = FindOrCreateBin(entries, ref count, dir, r);
                        if (bin >= 0)
                        {
                            entries[bin].Force += force;
                            entries[bin].Torque += torque;
                            entries[bin].MaxThrust += tMax;
                            entries[bin].Dir = dir;
                        }
                    }

                    leverSum += r.Length() * tMax;
                    thrustSum += tMax;
                }
            }

            if (count == 0 || thrustSum <= 1e-2f) return;

            float lChar = Math.Max(2.0f, leverSum / thrustSum);
            float invL = 1f / lChar;
            Vector3 tauCmdNorm = tauRcsCmd * invL;

            // Weights: balance linear thrust accuracy vs torque zeroing/RCS tracking.
            // If we have RCS torque command, give torque strong tracking; for pure linear flight,
            // weight torque trim strongly enough to eliminate spin when opposing/paired thrusters exist.
            const float wF = 1.0f;
            float wTau = wantRcs ? 3.5f : 4.0f;
            float avgT = thrustSum / count;
            float eps = 1e-4f * avgT * avgT;

            // Initialize u_i smartly from vanilla directional fraction so symmetric ships converge in 1-2 sweeps
            Vector3 fCur = Vector3.Zero;
            Vector3 tauCurNorm = Vector3.Zero;

            for (int i = 0; i < count; i++)
            {
                ref ThrusterEntry e = ref entries[i];
                e.TorqueNorm = e.Torque * invL;
                e.Diag = wF * e.Force.LengthSquared() + wTau * e.TorqueNorm.LengthSquared() + eps;

                float u0 = 0f;
                if (wantLinear)
                {
                    float align = Vector3.Dot(e.Dir, frac);
                    if (align > 0f) u0 = Math.Clamp(align, 0f, 1f);
                }
                e.U = u0;
                if (u0 > 0f)
                {
                    fCur += e.Force * u0;
                    tauCurNorm += e.TorqueNorm * u0;
                }
            }

            // Fast check: if already balanced and no RCS needed, skip solver sweeps completely!
            Vector3 resF = fCur - fCmd;
            Vector3 resT = tauCurNorm - tauCmdNorm;
            float tolF2 = Math.Max(1f, fCmd.LengthSquared() * 1e-6f);
            float tolT2 = Math.Max(1f, (fCmd.LengthSquared() + tauCmdNorm.LengthSquared()) * 1e-6f);

            if (resF.LengthSquared() > tolF2 || resT.LengthSquared() > tolT2)
            {
                for (int iter = 0; iter < SolverIterations; iter++)
                {
                    float maxDu = 0f;
                    for (int i = 0; i < count; i++)
                    {
                        ref ThrusterEntry e = ref entries[i];
                        float grad = wF * Vector3.Dot(e.Force, resF)
                                   + wTau * Vector3.Dot(e.TorqueNorm, resT)
                                   + eps * e.U;
                        float uNew = Math.Clamp(e.U - grad / e.Diag, 0f, 1f);
                        float du = uNew - e.U;
                        if (MathF.Abs(du) > 1e-6f)
                        {
                            e.U = uNew;
                            resF += e.Force * du;
                            resT += e.TorqueNorm * du;
                            float adu = MathF.Abs(du);
                            if (adu > maxDu) maxDu = adu;
                        }
                    }
                    if (maxDu < 1e-4f) break;
                }
            }

            // Trim only. The game keeps ONE thrust value per axis (VoluntaryThrustData, which drives IsThrusting, the flames,
            // replication and the thrusters' consumption: GridMovementCollectorComponent.InformThrustersWhetherTheyAreThrusting,
            // ServerSessionComponent.SyncVoluntaryThrust). Thrust the solver adds beyond what vanilla commands in a direction,
            // or in a direction vanilla does not fire at all, would be invisible and unpaid. Without RCS each direction is
            // therefore capped at vanilla's command; trimming down stays paid at the vanilla rate (conservative).
            if (!wantRcs)
            {
                Span<float> total = stackalloc float[6];
                for (int i = 0; i < count; i++) total[DirSlot(entries[i].Dir)] += entries[i].U * entries[i].MaxThrust;
                Span<float> scale = stackalloc float[6];
                for (int s = 0; s < 6; s++)
                {
                    float cap = MathF.Max(Vector3.Dot(fCmd, SlotVec(s)), 0f);
                    scale[s] = total[s] > cap + 1f ? cap / total[s] : 1f;
                }
                for (int i = 0; i < count; i++) entries[i].U *= scale[DirSlot(entries[i].Dir)];
            }

            // RCS flames and power for thrusters firing beyond what the grid's per-axis thrust shows
            if (wantRcs && !needBinning)
            {
                PublishRcs(entity, entries, count, in frac, voluntaries, firing, thrusterOverrides);
                published = true;
            }

            // Reconstruct exact achieved net force & net torque from solved u_i
            Vector3 fSol = Vector3.Zero;
            Vector3 tauSol = Vector3.Zero;
            for (int i = 0; i < count; i++)
            {
                float u = entries[i].U;
                if (u <= 1e-5f) continue;
                fSol += entries[i].Force * u;
                tauSol += entries[i].Torque * u;
            }

            // Unbalanceable single-thruster floor:
            // If the pilot commanded thrust and the solver had to cut >35% of commanded thrust ONLY because no
            // opposing/paired thruster exists to cancel torque, preserve at least 65% of commanded linear thrust
            // so a 1-thruster off-center rocket still flies and realistically torques instead of shutting itself off.
            if (wantLinear)
            {
                float cmdMag = fCmd.Length();
                float solProj = Vector3.Dot(fSol, fCmd / cmdMag);
                if (solProj < 0.65f * cmdMag)
                {
                    // Recompute un-trimmed (vanilla) torque to blend smoothly to the 65% floor
                    Vector3 fRaw = Vector3.Zero, tauRaw = Vector3.Zero;
                    for (int i = 0; i < count; i++)
                    {
                        float align = Vector3.Dot(entries[i].Dir, frac);
                        if (align <= 0f) continue;
                        float u = Math.Clamp(align, 0f, 1f);
                        fRaw += entries[i].Force * u;
                        tauRaw += entries[i].Torque * u;
                    }
                    float rawProj = Vector3.Dot(fRaw, fCmd / cmdMag);
                    if (rawProj > solProj + 1e-2f)
                    {
                        float alpha = Math.Clamp((0.65f * cmdMag - solProj) / (rawProj - solProj), 0f, 1f);
                        fSol = Vector3.Lerp(fSol, fRaw, alpha);
                        tauSol = Vector3.Lerp(tauSol, tauRaw, alpha);
                    }
                }
            }

            // 5. Apply solved linear delta (from vanilla CoM thrust) + solved net torque
            Vector3 dFLocal = (fSol - fCmd) * Dt;
            Vector3 dLLocal = tauSol * Dt;

            bool applyLin = wantLinear && dFLocal.IsValid() && dFLocal.LengthSquared() > 1e-4f;
            bool applyAng = dLLocal.IsValid() && dLLocal.LengthSquared() > 1e-4f;

            if (!applyLin && !applyAng) return;

            ref RigidBodyData wr = ref body.WritableValue;

            if (applyLin)
            {
                // Keep linear delta bounded so trimming never creates a spike against hover gravity cancellation
                Vector3 dFWorld = WorldTransform.TransformDirection(dFLocal, wt);
                Vector3 dv = dFWorld * mass.InvMass;
                float dvLen = dv.Length();
                float maxDv = Math.Max(0.5f, fCmd.Length() * mass.InvMass * Dt);
                if (dvLen > maxDv) dv *= maxDv / dvLen;
                wr.LinearVelocity += dv;
                Interlocked.Increment(ref TrimmedGrids);
            }

            if (applyAng)
            {
                // Clamp per-tick angular velocity delta to realistic bounds (9-21 rad/s^2 max)
                Vector3 dwLocal = wr.ComputeAngularImpulseLocal(in mass, in dLLocal);
                float dwLen = dwLocal.Length();
                float maxDw = group != null ? 0.15f : 0.35f;
                if (dwLen > maxDw && dwLen > 1e-5f)
                    dwLocal *= maxDw / dwLen;
                if (dwLocal.IsValid() && !Vector3.IsZero(dwLocal))
                {
                    wr.ApplyAngularVelocityLocal(in wt, in dwLocal);
                    Interlocked.Increment(ref RealPhysics.ThrustTorqueGrids);
                }
            }

            Interlocked.Increment(ref SolvedGrids);
            if (wantRcs) Interlocked.Increment(ref RcsAssistedGrids);
        }
        catch (Exception e)
        {
            if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("flightcomp: SolveThrusterMix error: " + e);
        }
        finally
        {
            // no RCS this tick (or an early exit): put any RCS-lit thrusters of this grid back to vanilla
            if (!published && RcsByGrid.ContainsKey(entity))
            {
                try { ClearRcs(entity, in frac, voluntaries, firing, thrusterOverrides); }
                catch (Exception e) { if (Interlocked.Increment(ref _errors) <= 5) ProbeLog.Write("flightcomp: ClearRcs error: " + e); }
            }
        }
    }

    private static float SubtractSameSign(float needed, float alreadyApplied)
    {
        if (needed == 0f || alreadyApplied == 0f) return needed;
        if (MathF.Sign(needed) != MathF.Sign(alreadyApplied)) return needed;
        float rem = MathF.Abs(needed) - MathF.Abs(alreadyApplied);
        return rem > 0f ? MathF.Sign(needed) * rem : 0f;
    }

    private static int FindOrCreateBin(Span<ThrusterEntry> entries, ref int count, in Vector3 dir, in Vector3 r)
    {
        // 6 directions * 8 octants around CoM = at most 48 bins (always fits in MaxActiveThrusters = 128!)
        int oct = (r.X >= 0f ? 1 : 0) | (r.Y >= 0f ? 2 : 0) | (r.Z >= 0f ? 4 : 0);
        for (int i = 0; i < count; i++)
        {
            if (entries[i].Dir == dir && (int)entries[i].Diag == oct)
                return i;
        }
        if (count >= entries.Length) return -1;
        int idx = count++;
        entries[idx] = new ThrusterEntry { Dir = dir, Diag = oct };
        return idx;
    }

    private static int DirSlot(in Vector3 dir) =>
        dir.X > 0.5f ? 0 : dir.X < -0.5f ? 1 : dir.Y > 0.5f ? 2 : dir.Y < -0.5f ? 3 : dir.Z > 0.5f ? 4 : 5;

    private static Vector3 SlotVec(int s) => s switch
    {
        0 => new Vector3(1, 0, 0),
        1 => new Vector3(-1, 0, 0),
        2 => new Vector3(0, 1, 0),
        3 => new Vector3(0, -1, 0),
        4 => new Vector3(0, 0, 1),
        _ => new Vector3(0, 0, -1),
    };

    private static Vector3 DirectionVec(Base6Directions.Direction d) => d switch
    {
        Base6Directions.Direction.Right => new Vector3(1, 0, 0),
        Base6Directions.Direction.Left => new Vector3(-1, 0, 0),
        Base6Directions.Direction.Up => new Vector3(0, 1, 0),
        Base6Directions.Direction.Down => new Vector3(0, -1, 0),
        Base6Directions.Direction.Backward => new Vector3(0, 0, 1),
        _ => new Vector3(0, 0, -1),
    };
}
