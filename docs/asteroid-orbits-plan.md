# "The Expanse" Orbital Mechanics & Voronoi Shattering — Full Implementation Plan

> **Scope:** Turn Space Engineers 2 into a full orbital-mechanics sandbox where planets, moons, ships, and asteroids all move on real trajectories affected by player actions, asteroids collide even with no player nearby, and high-energy impacts physically shatter asteroids into jagged, ore-exposed Voronoi shards.
>
> **Target codebase:** `D:\SpinProbe\src\` → `D:\SpinProbe\bin\Release\net9.0\ChildlessParents.dll`
> **Decompile reference:** `D:\SE2MPNEW\Se2Decomp` (`Stable_VS4.1`, `4.0.0.3169`)

---

## 0. Current Baseline & Hard Engine Invariants

### Already Built & Verified Live (`run90–run93`)
- **Rotating/orbiting planet bubbles + global rendering:** [Bubbles.cs](file:///D:/SpinProbe/src/Bubbles.cs), [GlobalRender.cs](file:///D:/SpinProbe/src/GlobalRender.cs), [BubbleHandoff.cs](file:///D:/SpinProbe/src/BubbleHandoff.cs), [SpectatorHandoff.cs](file:///D:/SpinProbe/src/SpectatorHandoff.cs)
- **Inverse-square gravity + orbiting-moon gravity + gyro spin recovery + flight computer/RCS + cached occluded aerodynamics:** [RealPhysics.cs](file:///D:/SpinProbe/src/RealPhysics.cs), [FlightComputer.cs](file:///D:/SpinProbe/src/FlightComputer.cs), [RcsFlames.cs](file:///D:/SpinProbe/src/RcsFlames.cs), [Aerodynamics.cs](file:///D:/SpinProbe/src/Aerodynamics.cs), [ShipAssembly.cs](file:///D:/SpinProbe/src/ShipAssembly.cs)
- **Grid orbital rails (universal-variable Kepler):** [RealRails.cs](file:///D:/SpinProbe/src/RealRails.cs)
- **Unlimited-size movable asteroids (single inset sphere proxy + welded `HkVoxelShape` chunks + voxel mass/CoM/inertia):** [MovableAsteroids.cs](file:///D:/SpinProbe/src/MovableAsteroids.cs) (`60.0 TPS`, `3.6 ms/tick` avg, `StepCollide = 0.15 ms/tick` in `run93`)
- **Density-based impact cratering (real velocity check + reduced-mass KE cap):** [AsteroidImpacts.cs](file:///D:/SpinProbe/src/AsteroidImpacts.cs)
- **Blueprint-style asteroid Ctrl+X / Ctrl+C / Ctrl+V with `UI3D` bounding box, client 3D preview, and vanilla velocity inheritance:** [AsteroidClipboard.cs](file:///D:/SpinProbe/src/AsteroidClipboard.cs)

### Non-Negotiable Engine Invariants (Learned Live in Runs 57–93)
1. **Never nest `HkVoxelShape` inside a compound collider.** Native `HkSession.StepCollide` crashes (run 57). Voxel chunks must remain root shapes of their own welded bodies.
2. **Never use multi-box `StaticCompoundCollider` (20+ boxes) as an asteroid proxy.** Overlapping `HkVoxelShape` triangle meshes spiked `HkSession.StepCollide` from `0.15 ms/tick` to `51.1 ms/tick` (run 92). Keep a **single `SphereCollider`** (or a 1-child compound solely to offset that single sphere to `CenterOfMass`).
3. **Always populate `BoundingBoxData` during `Init` before `OnAddedToScene`.** `VoxelStorageComponentBase.RegisterSpaceProbe()` throws `Data not found` otherwise, aborting `AddToSimulation` before `AllocateBody` runs (runs 91–93).
4. **Never trust raw Havok `ContactImpulseData` speeds/impulses alone.** Depenetration recovery on resting/overlapping rocks reports fake `28.9 m/s` and multi-GJ spikes (runs 91–92). Always gate on real relative velocity from `RigidBodyData` and cap energy by reduced-mass kinetic energy $\frac{1}{2} m_{\text{eff}} v_n^2$ using the **parent** asteroid's mass (not the chunk's `0.008 kg` dummy `VoxelShape` mass).
5. **Never write `WorldTransform` on Static server bodies every tick.** `HavokLargeWorldManager.UpdateMovedStaticBodyClusters` already takes `0.65 ms/tick` for 38 bodies. Keep server pose writes rare (every `5–10 s`) and interpolate smoothly at render framerate on the client in `GlobalRenderJobs`.
6. **Voxel thread & replication rules:**
   - All `IStorage.ReadRange` / `WriteRange` work must run on `Keen.VRage.Library.Threading.Parallel.Schedule` (never plain `.NET` `Task.Run` — `BumpAllocator` NRE, run 66).
   - `IStorage.WriteRange` is **LOD0-only** and does **not** push live diffs to already-replicated clients.
   - Edits to **existing** asteroids must use `VoxelDestructionComponent.OnSphereDestruction` (server → clients signal).
   - **Brand-new** shard entities replicate via `VoxelStorageComponentBase.Serialize` (`SharedStorage` in-process + `SaveStorage` → `Offloaded` over network).
7. **Always tag dynamic asteroids/chunks with `ProcedurallyGeneratedTag` + `NonBlockingTag`** so they never recursively trigger sector loads or block the simulation thread.
8. **Separate server and client tables.** In single-player host, server and client run in one process with overlapping `DEntity` IDs.

---

## Stage 1 — Pile-Up Containment, CoM Proxy Fix & Baseline Profiling

**Goal:** Eliminate unbounded save bloat, fix asymmetric-asteroid proxy alignment, and replace performance estimates with measured baselines before building shattering and rails.

### 1.1 Player-Only Promotion (`MovableAsteroids.cs`)
- Currently `Watch()` calls `proc.TurnManual()` whenever `s.Impacted || deltaV > 0.25f`. Once all asteroids are movable, background rock-on-rock nudges would permanently promote hundreds of rocks and bloat saves/RAM.
- **Change:** Add a `bool PlayerTouched` flag on `MovableAsteroids.State`, set only when:
  - Spawned via clipboard paste (`isPasted`),
  - Collided with a `CubeGridComponent` or `CharacterComponent` (or another `PlayerTouched` asteroid!),
  - A grid is welded to it, or
  - Modified by a player drill/weapon (`OnStorageChanged` when a player/grid is within 100 m).
- Only call `TurnManual()` when `PlayerTouched` is true (with an optional safety cap setting `max_promoted_asteroids`, default `0 = unlimited` or configurable).

### 1.2 Offset Proxy Sphere to Real `CenterOfMass` (`MovableAsteroids.cs`)
- Currently `MovableAsteroidMass.Update` computes `mp.CenterOfMass` accurately, but allocates `SphereColliderArgs(proxyR)` centered at entity origin `(0, 0, 0)`.
- For heavily cratered rocks or off-center shards, `(0, 0, 0)` can sit outside the remaining rock.
- **Change:** When `mp.CenterOfMass.LengthSquared() > 1.0f`, wrap the single sphere inside a **1-instance `StaticCompoundCollider`** with `Translation = mp.CenterOfMass` (still 1 sphere in narrowphase → same `~0.02 ms` cost!). Dispose both child sphere and compound cleanly on update/removal.

### 1.3 Diagnose Existing `UpdateMovedStaticBodyClusters` (`0.65 ms/tick`, 38 bodies)
- Add a one-shot diagnostic log (behind `audit=1` or a temporary debug flag) inspecting which entities enter `HavokLargeWorldManager.UpdateMovedStaticBodyClusters` in `run93`. Knowing whether those 38 bodies are planet subparts, doors, or our client/server static bodies ensures Stage 3 doesn't compound an existing leak.

### 1.4 50–100 Server Asteroid Benchmark
- Fly into an asteroid field with unlimited-size movable asteroids enabled (no `size` cap) and log per-tick cost of:
  - `HavokSessionComponent.StepWorlds` / `StepCollide` / `StepSolve`
  - `SyncMotionState`
  - `AttachedEntityTransformChanged`
  - `Mem_Voxels` and `Mem_HavokMemory`
- **Deliverable:** Real measured `µs/asteroid` and `MB/asteroid` numbers to set exact wake/sleep thresholds in Stage 3.

---

## Stage 2 — Real Jagged Voronoi Voxel Shattering (Loaded Movable Asteroids)

**Goal:** Replace fissure-trench "shatter" with **real jagged Voronoi voxel shards** cut directly from the parent asteroid's voxels + ore materials, mutually exclusive with cratering, working cleanly in both single-player and multiplayer.

### 2.1 Mutually Exclusive Impact Regimes (`AsteroidImpacts.cs`)
Cleanly separate impacts into three regimes based on real relative closing speed $v_n$ and specific energy $e = E_{\text{eff}} / M$:
1. **Elastic Bounce** ($v_n < 10\text{ m/s}$ or $J < 50\text{ kN·s}$): Rigid-body bounce + tumble spin, zero voxel damage.
2. **Cratering** ($e < Q^*$): Intact rock survives; carves organic crater via `VoxelDestructionComponent.OnSphereDestruction` (existing path).
3. **Catastrophic Shatter** ($e \ge Q^*$): **Do NOT carve a crater on the parent.** Queue a Voronoi shatter job instead!
   - Material-dependent $Q^*$ thresholds (from `AvgDensity` / hit material):
     - Ice / Snow ($\rho < 1500$): `Q* = 180 J/kg` (shatters dramatically into jagged crystalline shards at modest impact speeds)
     - Stone / Silicon ($1500 \le \rho \le 4500$): `Q* = 450 J/kg`
     - Metallic / Iron / Cobalt / Gold ($\rho > 4500$): `Q* = 950 J/kg` (harder to shatter; dents/craters until hypervelocity)

### 2.2 Anti-Chain-Reaction Guardrails
- **Minimum shatter radius:** Do not shatter any asteroid with equivalent radius `< 10 m` (or `< 12–16 m` diameter, or `Mass < 8,000 t`). Tiny shards just crater/bounce (or vanish cleanly if vaporized at extreme $e > 10 Q^*$).
- **Shatter immunity cooldown:** Newly spawned shards get `ShatterImmuneUntilTicks = now + 4 s` so initial shard-on-shard brushing during separation never triggers a `1 → 6 → 36` cascade.
- **Global shatter budget:** Max `1` shatter job processed per `250 ms`, and cap total live shards per shatter to **`4–6`** (scales with $e / Q^*$, hard-capped at `6`).

### 2.3 Off-Thread 3D Noise-Perturbed Voronoi Partitioner (`AsteroidShatter.cs`)
Runs on `Keen.VRage.Library.Threading.Parallel.Schedule` (worker thread, never `Task.Run`):

1. **Choose strategy by parent size:**
   - **Standard asteroids (`size ≤ 128`, i.e., `64³` or `128³`):** Full `LOD 0` shatter.
     - Read parent `StorageData(ContentAndMaterial)` at `LOD 0`.
   - **Giant asteroids (`size ≥ 256`):** **Hemisphere Spalling / Calving** (avoids reading 134M voxels at `LOD 0` and is physically realistic!):
     - Read only a `96³` or `128³` sub-box centered around the impact point at `LOD 0`.
     - Partition that impact zone into `3–5` jagged spall shards, AND carve that exact extracted volume out of the surviving giant parent via `OnSphereDestruction` (leaving a huge jagged bite out of the giant rock while `3–5` sharp slabs fly off!).
2. **Generate `K` Voronoi fracture seeds** (in local voxel coords, `K = 4..6`):
   - Place `2` seeds close to `hitLocal` (smaller sharp fragments near the impact site) and `K - 2` seeds deeper/wider across the remaining solid rock (larger angular slabs).
3. **Noise-perturbed Voronoi assignment (the secret to jagged rocky shards, NOT smooth polygons):**
   - For each solid voxel $\vec{p} = (x, y, z)$ (`Content > 0`):
     - Compute perturbed squared distance to each seed $\vec{s}_k$:
       $$d_k(\vec{p}) = \|\vec{p} - \vec{s}_k\| + A \cdot \text{Noise3D}(\vec{p} \cdot f + \text{seedOffset}_k)$$
       with amplitude $A \approx 2.5\text{–}4.5\text{ m}$ and low-frequency 3D gradient/value noise $f \approx 0.12\text{ m}^{-1}$ (plus optional second octave for craggy facets).
     - Find closest seed $k_1$ and second-closest seed $k_2$.
     - **Fracture gap & impact pulverization:** If $(d_{k_2} - d_{k_1}) < 1.5\text{ voxels}$ (along the crack plane) OR within a small pulverized core around `hitLocal`, set `Content = 0` (air). This cleanly separates shards so they don't spawn overlapping in Havok.
     - Otherwise assign `(Content, Material)` to shard $k_1$, tracking each shard's tight `min/max` voxel bounds and mass/CoM.
4. **Recenter each shard into a compact power-of-2 octree (`32³` or `64³`):**
   - Compute each shard's tight bounding size + 4-voxel padding → pick smallest power-of-2 storage size (`32` or `64`, or `128` for huge slabs).
   - Shift shard voxels so the shard's **CoM / center sits right at the new storage center `(0, 0, 0)`**, and record the world offset $\Delta \vec{p}_k$ from the parent's pose!
   - Drop any tiny degenerate shard (`< 150` solid voxels).

### 2.4 Spawning Shards & Multiplayer Replication Verification
- On the server DCS thread (`WorldUpdateBeginJobGroup`, staggered `1–2` shards per tick if needed to keep frame times `< 10 ms`):
  1. **Primary path (cleanest — recentered tight octrees via `WriteRange` before replication):**
     - Clone a manual voxel asteroid `EntityObjectBuilder` with the target storage size (`32`/`64`), set `WorldTransform` to `parentPose * offset_k`, clear `SharedStorage`, set fresh `SpaceEntityId` + `MarkedAsManual = true` + `BoundingBoxData`.
     - Spawn via `IEntitySpawner.SpawnEntity(shardOB)` and immediately write the shard's `StorageData` via `storage.Storage.WriteRange(ref shardData, ContentAndMaterial, Vector3I.Zero, shardSize - 1)`.
     - Verify in a live test (and inspect `VoxelStorageComponentBase.Serialize`) that:
       - Single-player client renders the exact jagged shard immediately (`SharedStorage`), AND
       - Save/reload + remote client receives the exact modified octree via `SaveStorage` → `Offloaded`.
     - **Important check:** Ensure `ProceduralVoxelStorageComponent` doesn't overwrite `WriteRange` data on reload (either mark storage modified/dirty via `SaveStorage`, or spawn using `VoxelOctreeStorageComponent` / `SimpleVoxelStorageComponent` definition if `ProceduralVoxelStorageComponent` re-seeds empty chunks).
  2. **Fallback path (if remote MP streaming skips raw `WriteRange`):**
     - Opus 5.5's copy-and-trim route: clone parent `K` times, trim each copy with dense cut-out capsules along the Voronoi half-spaces via `VoxelDestructionComponent.OnSphereDestruction`, and rely on Stage 1.2's CoM-offset sphere proxy.
- **Conserved Shatter Kinematics:**
  - Compute each shard's ejection velocity away from `hitWorld` + away from parent CoM:
    $$\vec{v}_{\text{rel},k} = u_{\text{eject}} \cdot \text{Normalize}(\vec{p}_k - \vec{p}_{\text{hit}}) + \text{jitter}$$
    where $u_{\text{eject}} \approx \text{Clamp}(\sqrt{0.15 \cdot (e - Q^*)},\; 3,\; 45)\text{ m/s}$.
  - Subtract the mass-weighted mean $\frac{\sum m_k \vec{v}_{\text{rel},k}}{\sum m_k}$ so **total linear momentum is strictly conserved**:
    $$\vec{v}_k = \vec{v}_{\text{parent}} + \vec{v}_{\text{rel},k} - \bar{\vec{v}}_{\text{rel}}$$
  - Give each shard realistic tumble angular velocity $\vec{\omega}_k = \vec{\omega}_{\text{parent}} + \vec{\omega}_{\text{tumble},k}$ (`0.1–1.2 rad/s`).
  - Delete the parent asteroid entity once all shards are spawned.

---

## Stage 3 — Asteroid Orbital Rails, Moving-Parent SOI & Off-Screen Collisions

**Goal:** Every coasting asteroid moves on a true analytic Kepler orbit (around planets, the orbiting moon, or in heliocentric space), glides smoothly at 120 FPS on the client with near-zero server cost, and physically collides even when no player is anywhere nearby.

### 3.1 Generalize `RealRails.cs` to Asteroids + Moving-Parent Patched Conics
1. **Unified `Rail` support for `VoxelPhysicsComponent` alongside `GridPhysicsComponent`:**
   - Currently `RealRails.Consider` only matches `GridPhysicsComponent` and toggles motion via `SetGridMotion`.
   - For asteroids, add `SetAsteroidRailState(VoxelPhysicsComponent, bool onRails)` queued in `WorldUpdateBeginJobGroup` (`FlushMotion`):
     - **Going on rails:** Remove/destroy welded chunk bodies (if any), switch the asteroid's core body to `BodyArgs.Motion.Static` (so `HavokLargeWorldManager` frees its Havok body when far from dynamic bodies!), store analytic Kepler state `(ParentId, Mu, T0, R0, V0, Q0, W0, Radius, Mass)`, and record last analytic velocity for save serialization.
     - **Waking off rails:** Switch core body back to `BodyArgs.Motion.Dynamic`, teleport `WorldTransform` to exact live analytic `(pos, ori)`, restore exact live analytic `RigidBodyData(linVel, angVel)`, and restore `GroupCollisionFilterData` + `ContactImpulseData`.
   - **Save patch:** Patch `VoxelPhysicsComponent` / `RigidBodyComponent.Serialize` (just like `RealRails.SavePatch` does for grids) so a railed asteroid saves its live analytic pose & velocity and resumes dynamic cleanly on reload.
2. **Moving-Parent Patched-Conic SOI (`RealRails.cs` + `RealPhysics.cs`):**
   - Replace the static `moving == true -> reject rails` restriction in `RealRails.Dominant`!
   - Define the nested SOI hierarchy using `Bubbles.All`:
     - **Moon SOI:** Hill sphere $R_{\text{SOI}} \approx a \left(\frac{\mu_{\text{moon}}}{3 \mu_{\text{planet}}}\right)^{1/3}$ (or gravity dominance radius) around `Bubbles.Centre(moonBubble, t)` with live parent velocity $\vec{v}_{\text{moon}}(t) = \vec{\omega}_{\text{orbit}} \times (\vec{C}_{\text{moon}}(t) - \vec{C}_{\text{planet}}(t))$.
     - **Planet SOI:** Inside planet's gravity range / Hill sphere around `Bubbles.Centre(planetBubble, t)`.
     - **Star / Global SOI:** Outside all planet SOIs (Stage 4 adds $\mu_{\text{star}}$; until then $\mu = 0$ straight-line inertial coast).
   - Propagate every railed object's state $(\vec{r}_{\text{rel}}, \vec{v}_{\text{rel}})$ analytically relative to its parent's live center!
   - **SOI boundary transition:** Whenever $|\vec{r}_{\text{rel}}| > R_{\text{SOI}}$ (leaving child SOI) or a body enters a child's $R_{\text{SOI}}$:
     - Convert state cleanly: $\vec{r}_{\text{new}} = \vec{r}_{\text{old}} + \vec{C}_{\text{old}}(t) - \vec{C}_{\text{new}}(t)$ and $\vec{v}_{\text{new}} = \vec{v}_{\text{old}} + \vec{v}_{\text{oldParent}}(t) - \vec{v}_{\text{newParent}}(t)$, then re-anchor `(T0, R0, V0)` in the new parent SOI!
   - **Bubble boundary alignment:** Coordinate with [BubbleHandoff.cs](file:///D:/SpinProbe/src/BubbleHandoff.cs) so waking a railed asteroid near a planet's `1.6 R` bubble boundary puts it in the exact right frame (local rotating inside bubble, global inertial outside).

### 3.2 Smooth 120 FPS Client Rendering of Railed Asteroids (Zero Server Cluster Spam)
- **Server side:** Write the Static asteroid's `WorldTransform` only every `5–10 s` (or when waking up) so `UpdateMovedStaticBodyClusters` stays near zero.
- **Client side:** Replicate/sync the lightweight orbital state `(ParentId, Mu, T0, R0, V0, Q0, W0)` (or share directly in single-player + replicate via a tiny custom sync/property for MP).
- In `GlobalRenderJobs` (`GlobalRender.cs`): for any client asteroid currently on rails, compute its exact analytic `(pos(t), ori(t))` at render clock `Bubbles.T` every frame and push directly to its render root/clipmap — identical to how planets and moons already move smoothly at 120 FPS with zero stutter and zero Havok cluster rebuilds!

### 3.3 Four Wake-Up Triggers (All Evaluated Against Live Analytic Pose!)
Run a lightweight server job every `0.25–0.5 s` over all railed asteroids (using their **live analytic positions**, never the stale static server pose):
- **Trigger A — Asteroid-to-Asteroid Close Approach:**
  - Insert all railed asteroids into a coarse 3D spatial hash (cell size `~2–4 km`) and test nearby pairs + linear closest-approach over the next `dt_lookahead = 4 s`:
    $$d_{\min} < r_1 + r_2 + 200\text{ m}$$
  - If either asteroid is loaded / has a player within `30 km` / has a welded grid → **wake BOTH into Havok** so they physically collide, crater, or Voronoi-shatter live, then re-rail after `10 s` of clean coasting.
- **Trigger B — Player or Active Ship Proximity:**
  - Wake when distance to any player or active dynamic grid is `< Math.Max(2500.0, vRel * 5.0)` (speed-scaled lead time so high-speed `750 m/s` orbital rocks are always awake in Havok at least `5 seconds` before intercept!).
- **Trigger C — Atmosphere / Planet Surface Entry:**
  - Wake when altitude drops below `planetR * 1.15` (above atmosphere top) so [Aerodynamics.cs](file:///D:/SpinProbe/src/Aerodynamics.cs) drag, atmospheric deceleration, and planetary surface cratering/shattering execute accurately.
- **Trigger D — Attached Powered Grid / Active Thrusters:**
  - Asteroids with welded grids stay dynamic whenever any welded grid has active thrust/gyro control; once thrust stops and players leave, the entire asteroid+station assembly enters rails on its **new player-engineered orbit**.

### 3.4 Off-Screen Collision & Background Voxel Editing (No Player Nearby)
When **Trigger A** detects a guaranteed impact between two unloaded/distant railed asteroids (`> 30 km` from all players):
1. **Analytic Impact Solver (~1 µs):**
   - Advance both orbits to exact time of impact $t_{\text{imp}}$ where $|\vec{r}_1(t) - \vec{r}_2(t)| = r_1 + r_2$.
   - Compute normal $\hat{n}$, closing speed $v_n$, reduced mass $m_{\text{eff}}$, impulse $J = -(1 + e) m_{\text{eff}} v_n$ (low restitution $e \approx 0.15\text{–}0.30$ for rock/ice), and specific energy $e_i = E / m_i$.
   - Update both orbits immediately with post-impact velocities $(\vec{v}_1', \vec{v}_2')$.
2. **Throttled Background Voxel Update Queue (`Max 1–2 concurrent workers`):**
   - **If cratering regime ($e_i < Q^*$):** Queue a `Parallel.Schedule` worker to load/access the rock's `IStorage`, run `CutOutCapsulesFast` directly on its octree, save/offload via `OffloadedVoxelStorage`, update mass/CoM, and release RAM if unloaded.
   - **If shattering regime ($e_i \ge Q^*$):** Queue the Stage 2 Voronoi shatter worker to split the rock's octree into `K` jagged shards, offload each shard's octree, remove the parent orbital record, and insert the `K` new shard orbital records onto their real conserved Kepler orbits immediately!
   - Result: Even if nobody is watching, the crater or shattered shard cloud **already exists** on real orbits and in saved voxel storage the moment impact happens.

---

## Stage 4 — "The Expanse" Solar System (Heliocentric Gravity, Orbiting Planets & Orbital Belts)

**Goal:** Put planets on real orbits around a central star and replace static procedural rocks with real orbiting asteroid belts.

### 4.1 Synthetic Central Star Gravity & Orbiting Planets
- Add configurable `star_pos` (e.g. system barycenter / sun direction) and `star_mu` ($\mu_{\text{star}}$) in `Settings` / `RealPhysics.cs` + `Bubbles.cs`:
  - Put planets on real heliocentric Kepler/circular orbits around the star bubble using `Bubbles.cs`'s existing recursive `OrbitAround` chain (`Star → Planets → Moons`).
  - Outside all planet SOIs (global space), apply central star gravity $\vec{a}_{\text{star}} = -\frac{\mu_{\text{star}}}{r^3}\vec{r}$ in `RealPhysics` and `RealRails` so deep-space ships and belts orbit the star instead of drifting linearly forever.
  - Render/align the colonization map (`MapRender.cs`) with heliocentric planet orbit rings + belt rings.

### 4.2 Deterministic Orbital Asteroid Belt Catalog (Bypassing Static `EntitySector` Cubes)
- **Why not just patch SE2's `EntitySector` grid:** SE2's `ProceduralGeneratorSessionComponent` assigns each procedural rock to a fixed axis-aligned cube sector. An orbiting rock leaves its spawn sector within minutes, causing duplicate respawns on return and missing rocks in neighboring sectors.
- **Clean Architecture:**
  1. **Belt Catalog (`OrbitalBeltSystem`):** Deterministically generate `500–2,000` orbital asteroid records from the world seed:
     - **Planetary rings / belts** around Verdure and outer planets ($r \in [1.8 R, 10 R]$, tangent velocity $v = \sqrt{\mu_{\text{planet}} / r}$, small eccentricity $e \in [0, 0.08]$, small inclination dispersion).
     - **Heliocentric asteroid belt(s)** orbiting the star ($v = \sqrt{\mu_{\text{star}} / r}$).
     - Each record is ~128 bytes (`Id`, `Seed`, `Size`, `ParentId`, Kepler elements, `SpinAxis/Rate`, `Mass`, `Radius`, `ModifiedOffloadKey?`).
  2. **Proximity Spawner / Despawner:**
     - Using the same spatial hash as Stage 3.3, spawn an actual asteroid entity (`IEntitySpawner.SpawnEntity` with `MarkedAsManual = false` / custom tag so SE2's sector generator ignores it) when a player comes within visual range (`~25–30 km`) — starting on rails (client-rendered smoothly in `GlobalRenderJobs`) and waking to full Havok physics at Trigger B (`max(2.5 km, v_rel * 5 s)`).
     - When no player is within `35 km` and the rock is back on rails, despawn the entity back down to its 128-byte orbital record (RAM/VRAM stays bounded forever!).
  3. **Persistent Delta Store (Only Modified Rocks Saved):**
     - Untouched catalog rocks require **zero** save data (reconstructed deterministically from `Seed` + world time `t`).
     - Any rock modified by a player (pushed into a new orbit, cratered, mined, shattered into shards, or having a base welded to it) writes its updated orbital record + `OffloadedVoxelStorage` reference to a compact sidecar file next to the save.
  4. **Patch `ProceduralGeneratorSessionComponent`** inside planetary/heliocentric belt zones (optional toggle `orbital_belts=1`) so vanilla static floating rocks don't duplicate or sit stationary inside moving orbital belts.

---

## Verification & Acceptance Gates (Per Stage)

| Stage | Live Verification Criteria |
|---|---|
| **Stage 1** | 10-minute belt run shows `0` unprompted background `TurnManual()` promotions; 50–100 server asteroid benchmark logged with exact `ms/tick` and `MB` figures; 38 `UpdateMovedStaticBodyClusters` bodies identified; cratered rock proxy sphere aligns with `CenterOfMass`. |
| **Stage 2** | Throwing a `64 m` ice/rock asteroid at `40–80 m/s` into another triggers `SHATTER` (no duplicate crater on parent), spawns `4–6` jagged, ore-exposed Voronoi shards flying apart with conserved momentum, no Havok overlap explosion, no recursive cascade, and shards survive save/reload cleanly. |
| **Stage 3** | Thrown asteroid leaves `5 km` player radius → enters `RealRails` (Static on server, `0` Havok body, smooth 120 FPS motion in `GlobalRenderJobs`, `UpdateMovedStaticBodyClusters` unchanged). Aiming a rock at another rock `50 km` away and staying out of range triggers **Trigger A**, resolves collision off-screen, carves/shatters in background, and places resulting body/shards on new real Kepler orbits. Moon-orbiting rock stays bound to the moving moon and hands off cleanly at SOI/bubble boundaries. |
| **Stage 4** | Planets visibly orbit the central star on `MapRender` and in 3D space; belt asteroids orbit smoothly at $v = \sqrt{\mu/r}$; matching orbital velocity in a ship lets you land, mine, attach thrusters, and de-orbit a belt asteroid onto Verdure while keeping server ticks `< 8 ms` (`60 TPS`). |

---

## Implementation status (Claude, 2026-11-27) — Stage 1 + Stage 2 code, not yet tested in game

Built clean (`ChildlessParents.dll`); no run yet. New settings: `asteroid_background_promotions` (0), `asteroid_shatter` (1), `static_census` (1).

### Decompile findings that changed the plan
- **Vanilla auto-promotes on any permanent voxel edit.** `ProceduralVoxelEntityComponent` subscribes `OnStorageEditedExternally` while not manual and calls `PromoteToManual` on `PermanentMutation` (:41-44, :100-106). Our rock-on-rock craters are such edits, so player-only promotion needs a gate there too: `MovableAsteroids.AutoPromoteGatePatch`. An edit we did not cause (drill, tool, weapon) counts as a player touch.
- **The sphere collider has a native centre** (`SphereColliderArgs(Vector3 center, float radius)`, `SphereShape.cs:61-66`). The CoM fix needs no compound collider. The proxy is also capped at 0.95 × the inscribed radius, so it cannot poke out of a thin slab.
- **`ComputeCenteredTransform` is the storage CORNER frame** (world × `StorageOffset`, `StorageOffset = -Size/2`, `VoxelStorageComponentBase.cs:85-91, :414-417`). Crater capsules and hit points are in voxel coordinates; mass properties and shard origins are in the entity (centre) frame.
- **`ProceduralVoxelStorageComponent.LoadStorage` (:61-73) regenerates from the seed** when the loaded storage's provider is not a fully loaded `CompositeShapeProvider`. Shard storages are therefore `new OctreeStorage(parentProvider, S)`, with the whole S³ box overwritten by `WriteRange`.
- **`Offload` reuses the old blob whenever the save state is valid** (`VoxelStorageComponentBase.cs:943-963`). Shard object builders therefore carry an `Offloaded` with no `Identifier` and the shard storage preloaded (`SetPreloaded`, consumed by `LoadStorage` :356-362). Replication and saves then serialize the shard's own voxels. All writes happen before the entity exists, which avoids the "WriteRange does not reach replicated clients" problem.
- **`UpdateMovedStaticBodyClusters` is `[OnChanged(WorldTransform)]` over static bodies** (`HavokLargeWorldManager.cs:1295`). The 38 calls per tick in run 93 are 38 static bodies moved every tick. `StaticCensus` names them every 10 s ("census:" log line).

### What was built
| Plan item | Where |
|---|---|
| 1.1 player-only promotion: contact with a grid, character or touched asteroid; paste; weld; player voxel edit; background budget | `MovableAsteroids` (`MarkPlayerTouched`, `Promote`, `AutoPromoteGatePatch`), `AsteroidImpacts.PlayerCause`, `MovableAsteroidJobs.Watch` |
| 1.2 proxy sphere at CoM, inscribed-radius cap | `MovableAsteroidMass.Update/Compute`, `InitPatch` |
| 1.3 static-mover census | `StaticCensus.cs` |
| 1.4 bench line every 10 s ("asteroid bench:") | `Profiler.AsteroidBench` |
| 2.1 bounce / crater / shatter exclusive, Q* 180/450/950 on bulk density | `AsteroidImpacts.OnChunkContact` |
| 2.2 guards: eqR ≥ 10 m, ≥150 voxels per shard, 1 queue per 250 ms, ≤2 in flight; shards immune to impacts 3 s and to shatter 6 s, in the parent's collision group for 4 s | `AsteroidShatter`, `Watch` (ungroup) |
| 2.3 domain-warped Voronoi at LOD0 on a Keen worker: K = 4–6 (spall 3–5), 1.5-voxel crack gap, pulverized core, sub-voxel crack fade, tight power-of-2 shard storages (≥32), exact mass/CoM/inertia | `AsteroidShatter.Work` |
| 2.3 giants (storage > 128): spall sphere ≤48 m read at LOD0; the parent is carved by the same sphere over the replicated destruction signal and takes the recoil | `AsteroidShatter.Work/Finish` |
| 2.4 spawn at WorldUpdateBegin from the parent's object builder (SaveStorage skipped for the template), momentum conserved (full split) or recoil (spall), ω×r carried, tumble, parent deleted via `TryDeleteManually` | `AsteroidShatter.Finish`, `AsteroidShatterJobs` |

### Must verify in game
1. The census line names the 38 static movers.
2. 10 min in a belt: no "promoted … background" lines with `asteroid_background_promotions=0`.
3. Throw a ≥64 m asteroid at another at 40–80 m/s and look for "SHATTER queued" → "SHATTER … shards". Check that shards render on the client, have no Havok explosion, separate and then log "own collision group", and survive save/reload. The shard object builder path (preloaded storage, no Identifier) is the main unknown; the copy-and-trim route is the fallback.
4. The bench line at 50–100 server asteroids.

---

## Stages 3 and 4: implemented (Claude, 2026-11-27); builds clean, not yet run in game

New files: `Orbits.cs` (bodies, SOI, patched conics, Kepler elements, star, sun), `AsteroidRails.cs` (stage 3 plus the save sidecar), `OrbitalBelts.cs` (stage 4.2). Edited: `RealRails.cs`, `RealPhysics.cs`, `Bubbles.cs`, `MapRender.cs`, `MovableAsteroids.cs`, `AsteroidImpacts.cs`, `AsteroidShatter.cs`, `Probe.cs`.

### Settings
| key | default | meaning |
|---|---|---|
| `asteroid_rails` | 1 | Stage 3 on. Needs `real_physics=1` and `movable_asteroid_max>0`. |
| `asteroid_wake_distance` | 2500 | Trigger B minimum, in m. The actual distance is max(this, 5 s × closing speed + 500). |
| `asteroid_rail_write_near_s` | 0.25 | Pose-write interval for railed rocks within 30 km of a player; elsewhere `real_rails_write_s`. |
| `star_distance` / `star_period` | 0 / 14400 | Stage 4.1 star. 0 = off. |
| `orbital_belts` | 0 | Stage 4.2 belts. |
| `belt_inner` / `belt_outer` | 2.5 / 4.5 | Home belt radii, in planet radii. Kept clear of the home bubble (×1.3) and of the moon's orbit (± 2 moon-bubble radii). |
| `belt_home_count` / `belt_star_count` | 300 / 500 | Number of catalog rocks. The heliocentric belt (0.55–0.75 × star distance) exists only with a star. |
| `belt_spawn_range` / `belt_despawn_range` | 25000 / 35000 | Proximity spawner range, in m. |
| `belts_suppress_vanilla` | 1 | Deletes vanilla procedural rocks inside a belt torus (`TryDeleteManually`, so they never come back). |

### Design decisions that differ from the plan
- **The star (4.1) is placed in the home planet's frame, not the reverse.** The procedural world is laid out at fixed coordinates around the home planet. If the home planet orbited a star through global space, it would leave every asteroid sector and station behind. So global space is the home planet's non-rotating, freely falling frame:
  - The star apparently orbits home. It is a virtual `Bubble` with `OrbitAround = home`, not in `Bubbles.All`.
  - Free bodies feel the star's tidal acceleration g★(p) − g★(home), which is exact heliocentric motion written in that frame.
  - A conic about the star is Kepler about its apparent centre plus its apparent velocity, so the moving-parent conic code is exact for it.
  - The engine's sun direction is rotated by the orbital angle (`SunRenderSessionComponent.UpdateSun` postfix).
  - The map draws the star's ring plus the belt rings.
- **Patched conics (3.1)** are Kepler relative to the dominant body's live centre (`Bubbles.Centre`) plus its velocity (`BubbleHandoff.OrbitalVelocity`).
  - SOI is Laplace a(μ/μ_parent)^(2/5), capped by the planet's gravity reach. The star yields to any planet's SOI.
  - Re-anchoring swaps in a new conic object, so other threads never read a half-updated state.
  - Grid rails (`RealRails`) use the same conics in global space. The old "orbiting a moving moon" rejection is gone; rails now release near any bubble instead.
- **Moving a railed asteroid:** all motion-type changes, pose writes and teleports happen at WorldUpdateBegin (`AsteroidRailJobs.Schedule`), never in parallel jobs.
  - `RigidBodyComponent.SetMotionType(Static/Dynamic)` (:198-205) switches the body.
  - `EntityTransformFunctions.SetWorldTransform` writes the pose; `Teleport` is used on landing.
- **Client rendering (3.2):** the in-process client draws railed rocks at the exact analytic pose every tick (`AsteroidRailsClientJobs`), matched by procedural id.
  - Remote clients get 4 Hz server samples within 30 km, which their kinematic copy interpolates. That costs about 17 µs per write (run 93: 0.65 ms / 38 static moves).
  - There is no custom replication of conic parameters.
- **Trigger D:** assemblies (an asteroid with welded grids) are never railed. Railing a shared-motion group static is untested and risky.
- **Off-screen impacts (3.4)** are solved with rigid spheres at the eq-radius, restitution 0.2, and both conics re-anchored at contact. Damage then follows the same regimes as a watched hit:
  - Crater via `AsteroidImpacts.CarveLocal`. It is storage-local and replicated, so the stale written pose does not matter.
  - Shatter via `AsteroidShatter.TryQueueLocal`. `Finish` takes the analytic pose and velocity of a railed parent.
- **Saves:** vanilla never saves an asteroid's velocity (`VoxelPhysicsComponent.Serialize` :185-189 uses a throw-away builder). The sidecar `asteroid-store/<save>.json` next to the DLL records the velocity of each serialized movable asteroid. It is restored on load if the asteroid is within 50 m of where it was saved. The same file holds the catalog delta store (permanent / destroyed catalog ids).
- **Belt catalog (4.2):**
  - Rocks spawn as manual entities with a deterministic `SpaceEntityId` (sector X = 2,000,000,000). Only serializable voxel entities replicate (`IsEntitySerializableVoxel`), so clients could not generate them.
  - Voxels come from the seed (no stored storage → `CreateStorageInternal`), using a template object builder serialized from the first movable asteroid seen.
  - Untouched rocks beyond the despawn range are closed. Touched rocks become permanent; shattered rocks are marked destroyed.

### Known limits and risks (verify in game)
- The orbit clock is `Bubbles.T` (real time). If the server runs below 60 TPS, railed objects move in real time while Havok bodies move in sim time.
- A chunk built next to a railed asteroid (`CreateRigidBody`) requests a wake, but that chunk is welded to a still-static core for a tick.
- Catalog rocks saved while loaded are saved with their storage, and are recognised on load by their id.
- Vanilla suppression in belts relies on the generator's deleted-id list reaching clients.
- Patched conics ignore the other bodies' pulls inside an SOI, while dynamic bodies feel every planet in range plus the star's tide. Small mismatches show at hand-over.
