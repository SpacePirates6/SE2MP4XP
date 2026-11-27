# Realistic physics plan for SE2 (SpinProbe plugin)

Status: implemented 2026-11-26 in `src/RealPhysics.cs` and `src/RealRails.cs`, first in-game run 74. Written 2026-11-26.

## Implementation switches (spinprobe.ini)

| Key | Default | What |
|---|---|---|
| `real_physics` | 0 | Master switch for everything below |
| `real_speed_limit` | 10000 | Linear cap for bodies and characters (m/s) |
| `real_angular_limit` | 100 | Angular cap (rad/s) |
| `real_angular_damping` | 0 | Spin damping (vanilla 0.025) |
| `real_gravity` | 1 | Inverse-square planet gravity, uniform-sphere inside, other bubbles excluded, radial centrifugal folded in |
| `real_gravity_range` | 12 | Planet gravity reach in surface radii |
| `real_rotating_frame` | 1 | Coriolis + tangential centrifugal in the spinning bubble (bodies and airborne characters) |
| `real_gyroscopic` | 1 | Implicit gyroscopic torque (tumbling); welded groups skipped |
| `real_gyro_free` | 1 | With dampeners off, gyros only act on commanded axes |
| `real_thrust_torque` | 1 | Per-thruster torque from thrust-weighted thruster centres; welded grids skipped |
| `real_drag` / `real_drag_cd` | 1 / 1.0 | Quadratic air drag from `AirData.Density` and bounding-box projected area |
| `real_rails` / `real_rails_distance` | 1 / 3000 | Orbits on rails for unmanned coasting grids beyond this distance from every character |
| `real_max_extrapolation` | 0.5 | Client motion extrapolation cap (s), vanilla 5 |

Run 74 facts: planet gravity generators are g=9.81 (R 60 km), 3.27 (R 20 km), 4.09 (R 25 km); vanilla `AccelerationDistance` = 1.05 R, `AffectDistance` = 1.35 R, linear falloff. `GravityMultiplier` is 2 (left as is; rails and the centrifugal term account for it).
Planet spin must be slow for real centrifugal force: at 3 deg/s the surface centrifugal acceleration is ~164 m/s^2. Run 74 uses 0.07 deg/s (0.09 m/s^2 at the equator).
Source references are to the decompiled game code (ILSpy) of build 4.0.0.3169. Short path roots:
- `GS/` = Game2.Simulation, `GG/` = Game2.Game, `VP/` = VRage.Physics, `VCG/` = VRage.Core.Game, `VG/` = VRage.Game
- Shipped data: `Content/` = `SpaceEngineers2/GameData/Vanilla/Content`

Parked as future goals (not in this plan): n-body gravity, real planet scale/density, structural stress and deformation, time warp.

---

## 0. What the engine does today (verified)

| Area | Today | Where |
|---|---|---|
| Linear speed cap | 300 m/s bodies, 310 m/s characters | `Content/System/Configurations/PhysicsSessionConfiguration.def`, fed into Havok at `GG/.../WorldSessionComponent.cs:493-499` |
| Angular speed cap | 6.28 rad/s (wheels 600) | same |
| Angular damping | 0.025 /s exponential, all bodies | same, `VP/.../HavokSessionComponent.cs:814` |
| Gravity multiplier | 2 (physics bodies) | same file, `WorldSessionComponent.cs:469` |
| Planet gravity falloff | Linear to a cutoff. Formula supports `pow(AccelDist/r, FallOffPower)` but planet def locks `FallOffPower` to -1 (linear) | `GS/.../Gravity/GravityGeneratorComponent.cs:983-1010`, `Content/Procedural/VS1_5/System/Data/Planets/DefaultGravityGenerator.def` |
| Gravity cost today | Recomputed every tick for every simulated body in range; only written when bit-different | `GravityGeneratorComponent.cs:730-742`, `GeneratorBaseComponent.cs:138-145` |
| Gravity clamp | Gravity magnitude clamped to `LinearSpeedLimit * 60` | `GravityGeneratorComponent.cs:135-147` |
| Dampeners | Tag `DampeningData` on the grid; already player-toggleable | `GS/.../Movement/DampeningData.cs:31`, `ThrustComponent.cs:390-443` |
| Low-speed snap | Thrust snaps velocity axes below `MinSpeed` to 0 even with dampeners off | `ThrustComponent.cs:499-513` |
| Gyro auto-brake | Always brakes to target 0; ignores dampener toggle | `GridGyroscopesComponent.cs:111-118,160-224` |
| Thrust | Applied at centre of mass (no torque) for normal ships | `ThrustComponent.cs:496-498` |
| Thrust on welded asteroids | Gives torque: shared-motion code converts each member's impulse with a lever arm about the group CoM. This is why an offset thruster spins an asteroid | `VP/.../MotionSharing/MultiEntitySharedMotionSessionComponent.cs:640-665` |
| Air drag | None as such. Wind applies `0.5*rho*A*windSpeed*|relVel|`, only when wind speed > 0 | `GS/.../Wind/WindGeneratorComponent.cs:146-183` |
| Air density | Per-entity `AirData.Density`, recomputed every tick, linear falloff | `GS/.../Atmosphere/AtmosphereGeneratorComponent.cs:63-74,271-277` |
| Collision detail by speed | Grids use mid-detail shapes above 20 m/s, low above 80 m/s | `Content/System/Configurations/Override of GridShapeLoddingConfiguration.def` |
| Tumbling (gyroscopic precession) | Native Havok DLL has no gyroscopic option (string scan). Assume absent until tested | `Game2/VRage.Physics.Native.dll` |

Baseline cost (run 72 profile): server tick 5.9 ms avg of 16.7 ms. Largest: `UpdateMovedStaticBodyClusters` 2.05 ms (58 static bodies whose transform changed), `StepWorlds` 0.41 ms, `SyncMotionState` 0.29 ms (53 entities), `StepCollide` 0.29 ms.

---

## Phase 0 - Measurement harness (do first)

- Add profiler hooks (our `Profiler.cs` stub patching) for every new job below plus vanilla `Gravitate`, `UpdateAtmosphere`, `ApplyWindForce`, `SyncMotionState`, `UpdateMovingBodyClusters`, `MigrateBodies`.
- Stress scene: spawn N small debris grids (10 / 100 / 1000) in orbit-like motion near a planet, log ms/tick and bytes/s sent.
- Pass/fail per phase: server tick stays under ~10 ms with 100 moving bodies.

## Phase 1 - Remove the artificial limits

- Patch the `SceneSettings` built in `WorldSessionComponent` (or postfix `PhysicsSessionConfiguration` init) from plugin settings:
  - `LinearSpeedLimit` -> configurable (start 10 000 m/s)
  - `AngularSpeedLimit` -> configurable (start 100 rad/s, the engine default)
  - `AngularDamping` -> 0
  - `GravityMultiplier` -> 1
  - Character limit stays separate.
- Also update `IVelocityLimitProvider` (`WorldSessionComponent.cs:296`) since the gravity clamp reads it.
- Optional realism switch: skip the `MinSpeed` snap in `ThrustComponent.ApplyThrust` (`:499-513`).
- Risks: collision tunnelling and joint instability at high speed. Grids already drop to coarse collision above 80 m/s.
- Cost: none directly.

## Phase 2 - Real gravity

- Inverse-square: allow `FallOffPower = 2` (patch `GravityGeneratorProcessorComponent.UpdateOB` or the `Min/MaxFallOffPower` limits). Then `g(r) = g0 * (AccelerationDistance / r)^2`; set `AccelerationDistance` = surface radius so surface gravity is unchanged.
- Range: raise `AffectDistance` toward the planet's sphere of influence (`MaxAffectDistance` is already unlimited). Measure: bigger range means more bodies run `Gravitate` every tick, and larger trigger volumes.
- Inside the planet (r < R) keep the current clamp so nothing explodes underground.
- Cost: about zero per body (one `pow`); grows with the number of bodies in range.

## Phase 3 - Rotating-frame forces (Coriolis and centrifugal)

Our bubbles run the game in each planet's rotating local frame, so fictitious forces must be added by hand. Planet angular velocity Omega comes from our `Bubbles` / `DrivePlanets` state.
- **Centrifugal** `-Omega x (Omega x r)`: position-only. Add it inside the gravity sum (postfix the gravity accumulation for entities inside a bubble). No extra job, no extra writes.
- **Coriolis** `-2 Omega x v`: new job, modelled on `ApplyWindForce`:
  - `[WhenSimulated] [MustHave(DynamicTag)] [MustHave(ActiveTag)]`, own `JobGroup` `[During(RangedAffectGenerators.Update)] [After(WindUpdate.ApplyForceGroup)]`.
  - Use `Writable<RigidBodyData>` and only write when the change is meaningful. A `ref` parameter marks every visited body changed every tick, which also triggers a network send.
  - Welded bodies (shared motion): write each member's own `RigidBodyData`; the shared-motion code sums member deltas into the group motion (`MultiEntitySharedMotionSessionComponent.cs:613-675`).
- **Gameplay warning:** at our test spin rates Coriolis is enormous (3 deg/s spin: 10 m/s^2 at 100 m/s). Realistic play needs much slower planet spin.
- Cost: small, only for awake bodies inside a spinning bubble.

## Phase 4 - Control realism

- **Gyros obey the dampener toggle:** use the empty vanilla slot `OnComputeRotation.DampeningRotationComputation` (`GridGyroscopesComponent.cs:61`). For grids without `DampeningData` and no rotation input, remove `ActiveGridGyroscopeData` for that tick (or zero the braking axes).
- **Thruster torque:** replace `ApplyThrust`'s centre-of-mass impulse with per-thruster `ApplyImpulseAt` (`VP/.../RigidBodyDataFunctions.cs:102`), so off-centre thrusters spin ships the way they already spin welded asteroids. Needs per-thruster position and thrust (vanilla sums them per grid into `ActiveThrustData`).
- **Dampeners:** already optional in vanilla (`DampeningData`). No change needed beyond making gyros follow them.

## Phase 5 - Tumbling (gyroscopic torque)

- First test whether Havok already precesses: spin a long thin grid with no gyros, check whether its axis wanders.
- If not: job applying `dL = -(omega x I omega) * dt` with the body's local diagonal inertia from `RigidBodyMassProperties`, via `ApplyAngularImpulseLocal` (`RigidBodyDataFunctions.cs:63`). Only for awake bodies with `|omega|` above a threshold.
- Welded groups need the group inertia, not each member's; skip them at first.
- Cost: small.

## Phase 6 - Atmosphere

- **Drag:** new job based on `ApplyWindForce`: `F = 0.5 * rho * Cd * A_projected * |v_rel|^2`, with `v_rel` = body velocity minus air velocity (air co-rotates with the planet, so in the bubble frame it is the wind vector). Read `AirData.Density`. Cap impulses like vanilla does (`5 / InvMass`).
- **Check first:** Verdure's atmosphere definition may leave `Density` at 0 (object-builder default 0). Read `AirData.Density` in game before relying on it.
- **Lift (later):** cache per-grid face areas per axis, rebuild on grid change.
- Cost: small with cached areas. The water drag code loops every block and corner every tick; don't copy that.

## Phase 7 - Orbits on rails (KSP-style)

Goal: coasting objects far from players cost nothing per tick and nothing on the network.

What the engine gives us:
- Large-world manager only keeps Havok worlds ("clusters") around dynamic/kinematic bodies. A Static body with no cluster nearby has no Havok body at all, only a static tree entry (`VP/.../WorldManager/HavokLargeWorldManager.cs:478-491`). A cluster is freed when its last dynamic body leaves (`:726-748`).
- `SyncMotionState` sends whenever `WorldTransform` or `RigidBodyData` changes, Unreliable normally, Reliable when resting or teleported (`GG/.../ServerSessionComponent.cs:35-73`).
- Moving a static body triggers `UpdateMovedStaticBodyClusters` each time (expensive; that's our current 2 ms).
- Kinematic rails would keep a Havok world alive per object and cost per-tick driving. Avoid at scale.

Design:
1. **Going on rails** (server): when a body is coasting (no thrust, no contacts, out of atmosphere, no player within range), compute orbit elements around the dominant body (patched conics, inertial frame, our planet orbit state), store them, switch the body to Static (queued; or `IPhysics.DestroyBody`).
2. **While on rails:** don't write `WorldTransform` every tick. Update rarely, or only when a player or cluster approaches.
3. **Clients:** replicate the orbit elements as a custom component (`[OnDataReplication.ServerToClient] [OnChanged(OrbitData)] replication.Override`, pattern from `GravityGeneratorComponent.cs:796-828`). Clients compute the pose themselves. Use `NoAutoSync` or rare transform writes so `SyncMotionState` doesn't fight it (`NoAutoSync` suppresses all auto-sync jobs, so re-replicate anything else needed).
4. **Coming off rails:** switch to Dynamic, then in the same tick `Teleport()` (`VCG/.../EntityTransformFunctions.cs:44-59`) to the analytic pose and write `RigidBodyData` with the analytic velocity. Teleport forces a reliable send and makes clients reset interpolation (`VG/.../MotionStateApplicationSystem.cs:88-91`). The motion-type change lands one physics update later; static grids get their velocity zeroed (`GridPhysicsComponent.cs:746-756`), so write velocity after the switch is applied.
5. **Save/load:** keep orbit elements in a session component with a serialized dictionary (pattern: `GS/.../EntityNameSessionComponent.cs:112-135`), so no entity definitions change.
6. **Rotating frame:** orbits are inertial; convert to the planet's rotating bubble frame on hand-off (our `Bubbles` code already does this conversion for planets).

### Scene subset (SE2's version of SE1 sync distance)

Assembly `VRage.Multiplayer.dll`, namespace `Keen.VRage.Multiplayer.Replications.SceneSubset` (decompile with `ilspycmd -t <Type>`).
- **Replication only.** It decides what each client receives and how often. It never freezes, sleeps or unloads anything on the server; physics code has no reference to it.
- **Per-entity sphere, not a per-player radius.** Each root entity publishes a sphere of radius `clamp(boundingRadius / 0.005, 250 m, 20 km) * 1.15`; a client sees the entity when its controlled entity is inside that sphere. Small debris: ~290 m. A 50 m ship: ~11.5 km. Anything 100 m+: ~23 km. Exit hysteresis 150 m.
- **Enforcement is off in our runs** (`EnforceSceneSubset: false`, checked in the run 72 game log). With it off every client receives every entity, but send rate is still scaled 1x / 2x / 4x by distance shell.
- With enforcement on: entering a client's subset sends the full entity; leaving destroys it on that client; unobserved roots get `UnobservedTag` and their data/signals are dropped.
- Hooks: events `EntityEnteredSubset` / `EntityExitedSubset`, queries `IsInSubset` / `TryGetShell`. `AlwaysReplicatedTag` opts out (planets use it). `ExclusiveTag` never replicates.

How rails use it:
1. "No player near" signal: subscribe to the enter/exit events and keep our own observer count per rail object (`UnobservedTag` only exists with enforcement on). Orbital speeds may need a bigger rails radius than the subset sphere, so keep our own distance check as well.
2. If enforcement is ever turned on, rail objects outside a client's subset vanish from that client. Either give rail objects `AlwaysReplicatedTag` and send only orbit elements (rare changes, so cheap), or replicate orbit elements from a session component that isn't subset-managed.
3. A rail object entering a client's subset gets a full snapshot, which is the natural moment to put it back into physics.
4. Turning enforcement on is itself a big bandwidth win for multiplayer with many objects, independent of rails.

## Phase 8 - Multiplayer at high speed

- Client extrapolation error = speed x latency (770 m/s x 100 ms = 77 m). Check `MotionInterpolationSettings` / `MaxExtrapolation` (`VG/.../MotionStateApplicationSystem.cs:83-130,225-266`) and tune for fast objects.
- Bandwidth: every awake moving body is sent every tick it moves. Rails (Phase 7) is the main fix.

---

## Order and effort

| Phase | Effort | Depends on |
|---|---|---|
| 0 Measurement | 1-2 days | - |
| 1 Limits | hours | 0 |
| 2 Gravity | 1-2 days | 1 |
| 3 Coriolis/centrifugal | 2-4 days | 2, bubbles |
| 4 Gyros/thruster torque | 2-3 days | - |
| 5 Tumbling | 1-2 days | test first |
| 6 Drag (lift later) | 2-3 days (+1 week lift) | - |
| 7 Rails | 1-2 weeks | 1, 2 |
| 8 MP tuning | 2-3 days | 1 |

Total for phases 0-6 and 8: about 3-4 weeks. Rails adds 1-2 weeks.

## Open questions to settle in-game
1. Does Havok precess spinning bodies? (Phase 5 test)
2. What `AirData.Density` does Verdure actually have?
3. Cost of `Gravitate` with planet gravity reaching the sphere of influence.
4. Can planet spin be slowed enough that Coriolis is playable while still visible?

## Run 74 review fixes (run 75)

A second review of run 74 against the decompile found four real bugs; all fixed:

| Bug | Cause (decompile) | Fix |
|---|---|---|
| Gravity still stopped at 81 km (1.35 R) although the log said 756 km | `GravityGeneratorComponent.Init` calls `GravityGeneratorProcessorComponent.UpdateOB` (:35-42) which overwrites `AffectDistance = min(R × 1.35, MaxAffectDistance)` before `base.Init` builds the trigger (:408); our prefix on Init ran before that | Postfix on `UpdateOB`; a postfix on `Init` now logs the value the trigger is really built from |
| Thruster torque was 0 for a hovering ship | `ThrustComponent.ComputeThrust` (:159-173) folds the gravity-holding thrust into a reduced `CustomGravity`, not into `ComputedThrustPerFrame`; job also ran in the wrong slot (previous tick's data) | Uses `VoluntaryThrustData` × `MaxThrustData.Regular` (+ override), runs in `OnComputeThrust.End` |
| Free gyros: a free spin on one axis throttled steering on the others | `ComputeTorqueAngular` (:85-130) scales all axes by the tightest budget and eases out on the total error before our slot | Torque recomputed with the same steps for the commanded axes only |
| Rails switched motion type from a parallel physics job; static pose written 2×/s | `SetGridMotion` raises the replicated MotionTypeProperty and `Data.Set` (:466-477) | Queued and run in `WorldUpdateBeginJobGroup`; pose written every `real_rails_write_s` (default 5 s), release checks use the analytic pose |

Not changed: dividing the centrifugal correction by `GravityMultiplier` assumes characters are scaled by it too (native `SetGravityMultiplier`, can't be checked in managed code; the error is at most 0.045 m/s² at 0.07°/s). Seated pilots are counted by their character position (the pilot is a hierarchy child of the seat, SeatComponent.cs).

## Follow-up fixes after run 88 (not yet run in game)

| Item | Fix |
|---|---|
| Client flicker at bubble crossings | Client no longer switches frames on its own hysteresis; it switches only when the replicated pose jumps to the converted pose (the server's handoff), with a 2 km fallback past the band |
| Floating objects / debris drawn in the wrong frame | Server hands off every awake dynamic body (`BubbleHandoff.CheckBodyHandoff`); welded groups excluded |
| Moon gravity outside its bubble | Engine gravity of an orbiting planet is 0 outside its bubble; `MovingPlanetGravity` adds the pull toward the moon's current centre for global-space bodies and airborne characters |
| Rails near the moon | Membership decides coordinates; orbits dominated by a moving moon in global space are not railed. Reasons a coasting grid is not railed are logged (`rails: … not railed: …`) |
| run.sh blamed other instances' crashes | Game log is matched by our plugin folder in its command line; one automatic retry when the game dies during startup |
| Client frame table grew forever | Entries not seen for 30 s are pruned |
| Flight-computer thrust without flames/power | Without RCS, each direction is capped at vanilla's command (trim only). RCS is on (user choice); its torque still shows no flames and uses no power, because the game keeps one thrust value per axis |
| Thruster torque ignored class efficiency | Weighted by CurrentMaxThrust × class efficiency |
| Centrifugal vs GravityMultiplier | Whole centrifugal term applied in the force jobs; gravity magnitude no longer corrected (characters get CustomGravity unscaled, rigid bodies scaled natively) |
| Tumbling counted twice? | `real: spin …` logs energy and |L| of the fastest spinner every 10 s; both must stay constant in vacuum |
| Does Havok scale rigid-body gravity? | `real: gravity check …` logs measured/engine gravity for a body in vacuum; an unpowered free fall should read the multiplier (2) — rails' μ depends on it |
| Sealed rooms added drag | Outside-air flood fill: only faces next to outside air count |

## Visible, paid RCS without thruster overrides

The grid has one thrust value per axis (VoluntaryThrustData), so thrusters RCS fires in a direction vanilla is not firing
showed no flame and drew no power. Now (FlightComputer.PublishRcs, RcsFlames.cs):
- a thruster firing more than 5 % above what the grid's value shows gets its own VoluntaryThrustData (direction × throttle,
  rounded to 5 %); vanilla's SyncVoluntaryThrust replicates it like any other entity's, no new network code;
- vanilla readers only look at the grid/character value, so nothing else sees it;
- the server gives that thruster IsThrusting (power/fuel, binary like vanilla) and keeps it through
  InformThrustersWhetherTheyAreThrusting (postfix); when RCS stops, IsThrusting goes back to vanilla's rule;
- the client raises that thruster's flame between ThrustFlameAnimator.Update and .Animate (RcsFlameJobs);
- ThrusterOverrideData is never touched, so no override shows in the terminal.
Log: `rcsThrusters` (thruster-ticks lit by RCS per 10 s) and `rcsLit` (lit now). Untested in game.
