using System;
using System.Collections.Generic;
using System.Threading;
using HarmonyLib;
using Keen.Game2.Simulation.WorldObjects.Characters;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Game.Systems;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.CoreData;
using Keen.VRage.DCS.ObjectBuilders;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Components;
using Keen.VRage.Physics.Data;
using Keen.VRage.Voxels.Components;
using Keen.VRage.Voxels.Storage;
using Keen.VRage.Voxels.Storage.OcTree;
using Keen.VRage.Voxels.Storage.Operations;

namespace SpinProbe;

/// <summary>
/// Stage 2 of docs/asteroid-orbits-plan.md: catastrophic impacts (specific energy E/M at or above the material's Q*) split a
/// movable asteroid into jagged Voronoi shards cut from its real LOD0 voxels and ore materials, instead of carving a crater.
/// <para>Pipeline: AsteroidImpactJobs.OnChunkContact -> <see cref="TryQueue"/> (DCS thread) -> worker
/// (Keen Parallel.Schedule, like the deposit scanner and MovableAsteroidMass) reads the voxels, partitions them and builds one
/// OctreeStorage per shard -> <see cref="AsteroidShatterJobs.Flush"/> at WorldUpdateBegin spawns the shard entities from a clone
/// of the parent's object builder whose storage is the preloaded shard storage, then deletes the parent (full shatter) or
/// carves the spalled volume out of it (giant asteroids).</para>
/// <para>Replication (checked in the decompile): a NEW entity reaches clients through entity serialization, which calls
/// VoxelStorageComponentBase.Serialize (:217-222: SharedStorage in-process + SaveStorage -> Offload, :943-963). Offload only
/// re-serializes when the save state is NOT valid, so the shard object builder carries an Offloaded with no Identifier and the
/// shard storage preloaded (LoadStorage :356-362 takes TryExtractPreloaded first). Raw WriteRange on an already-replicated
/// asteroid would NOT reach clients, so all writes happen before the shard entity exists. The shard storage keeps the parent's
/// procedural data provider because ProceduralVoxelStorageComponent.LoadStorage (:61-73) discards a loaded storage whose
/// provider is not a fully loaded CompositeShapeProvider and regenerates the rock from its seed.</para>
/// </summary>
internal static class AsteroidShatter
{
    // Source: none (plugin-only) - tuning.
    internal static readonly long ImpactImmuneTicks = TimeSpan.FromSeconds(3).Ticks;
    internal static readonly long ShatterImmuneTicks = TimeSpan.FromSeconds(6).Ticks;
    internal static readonly long SharedGroupTicks = TimeSpan.FromSeconds(4).Ticks;
    private static readonly long QueueSpacingTicks = TimeSpan.FromMilliseconds(250).Ticks;
    private const double MinShatterRadius = 10.0;   // m, equivalent-sphere radius of the parent
    private const int MinShardVoxels = 150;
    private const int FullShatterMaxSize = 128;     // storage size; bigger asteroids spall instead of splitting whole
    private const int MinShardStorage = 32;
    private const float GapVoxels = 1.5f;

    internal static int Queued, Shatters, Spalls, ShardsSpawned, Failed;
    private static int _inFlight;
    private static long _lastQueueTicks;

    /// <summary>Catastrophic disruption threshold Q* (J/kg) by bulk density: ice, stone, metal.</summary>
    // Source: none (plugin-only) - order of magnitude of the strength regime for metre-to-100-m bodies.
    internal static double QStar(double density) => density < 1500 ? 180.0 : density > 4500 ? 950.0 : 450.0;

    private static Vector3I Max3(Vector3I a, Vector3I b) => new(Math.Max(a.X, b.X), Math.Max(a.Y, b.Y), Math.Max(a.Z, b.Z));
    private static Vector3I Min3(Vector3I a, Vector3I b) => new(Math.Min(a.X, b.X), Math.Min(a.Y, b.Y), Math.Min(a.Z, b.Z));

    // ---------------------------------------------------------------------------------------------------------------
    // Hand-off to MovableAsteroids.InitPatch: the spawned shard's exact mass properties, velocity and collision group.
    // ---------------------------------------------------------------------------------------------------------------

    internal sealed class ShardSpawn
    {
        internal Vector3D Pos;
        internal Vector3 LinVel, AngVel;
        internal RigidBodyMassProperties Mass;
        internal float ProxyRadius;
        internal double Radius;
        internal double Density;
        internal GroupCollisionFilterData Filter;
        internal bool PlayerTouched;
        internal long Ticks;
    }

    private static readonly List<ShardSpawn> PendingShards = new();

    // Source: none (plugin-only). Same position-keyed hand-off AsteroidClipboard.TryConsumeSpawnMotion uses.
    internal static bool TryConsumeShard(in Vector3D at, out ShardSpawn? shard)
    {
        shard = null;
        lock (PendingShards)
        {
            if (PendingShards.Count == 0) return false;
            long now = DateTime.UtcNow.Ticks;
            PendingShards.RemoveAll(x => now - x.Ticks > TimeSpan.TicksPerSecond * 10);
            double best = 0.5 * 0.5;
            int bi = -1;
            for (int i = 0; i < PendingShards.Count; i++)
            {
                double d = (PendingShards[i].Pos - at).LengthSquared();
                if (d < best) { best = d; bi = i; }
            }
            if (bi < 0) return false;
            shard = PendingShards[bi];
            PendingShards.RemoveAt(bi);
            return true;
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Queue (DCS thread, from AsteroidImpactJobs.OnChunkContact)
    // ---------------------------------------------------------------------------------------------------------------

    private sealed class ShardResult
    {
        internal IStorage Storage = null!;
        internal Vector3 OriginLocal;          // shard storage centre in the parent's storage-centred frame (m)
        internal Vector3 ComParentLocal;       // shard centre of mass in the parent's frame (m)
        internal RigidBodyMassProperties Mass; // CenterOfMass relative to the shard's own origin
        internal float ProxyRadius;
        internal double Radius, Density, MassKg;
        internal int Voxels, StorageSize;
        internal string Composition = "";
    }

    private sealed class Job
    {
        internal VoxelPhysicsComponent Parent = null!;
        internal MovableAsteroids.State State = null!;
        internal IStorage Raw = null!;
        internal VoxelStorageComponentBase StorageComp = null!;
        internal Vector3I Size;
        internal float VoxelSize;
        // HitLocal: storage CORNER frame (VoxelStorageComponentBase.ComputeCenteredTransform = world * StorageOffset, StorageOffset
        // = -Storage.Size * 0.5, VoxelStorageComponentBase.cs:85-91, :414-417), i.e. voxel coordinates - what CutOutCapsules expects.
        internal Vector3 HitLocal, InwardLocal;
        internal double Energy, SpecificEnergy, QStar, Density, FallbackDensity;
        internal float ImpactSpeed;
        internal bool Spall;
        internal float SpallRadius;            // m
        internal string Impactor = "";
        internal readonly List<ShardResult> Shards = new();
        internal double PulverizedKg, RegionKg;
        internal string? Error;
        internal double WorkerMs;
        internal volatile bool Done;
    }

    private static readonly Queue<Job> Completed = new();

    /// <summary>Returns true when the impact was taken as a shatter (the caller must then NOT carve a crater into this
    /// asteroid). False: not eligible (too small, immune, grids welded, budget) - the caller craters as before.</summary>
    internal static bool TryQueue(VoxelPhysicsComponent physics, MovableAsteroids.State s, in Vector3D worldHit, Vector3 inwardWorld,
        double energy, double specificEnergy, double qStar, double density, float impactSpeed, string impactor)
    {
        var storage = physics._storage;
        if (storage?.Storage == null) return false;
        // Reads: AsteroidImpacts.CarveImpact - storage corner frame (VoxelStorageComponentBase.ComputeCenteredTransform), from
        // the analytic pose when the asteroid is on rails (its written pose is up to a few seconds old).
        WorldTransform centered = AsteroidRails.LiveCentered(physics, storage);
        WorldTransform worldToStorage = WorldTransform.GetRelativeTransform(WorldTransform.Identity, centered);
        Vector3 hitLocal = (Vector3)WorldTransform.Transform(in worldHit, in worldToStorage);
        Vector3 inwardLocal = Vector3.Transform(inwardWorld, worldToStorage.Orientation);
        inwardLocal = inwardLocal.LengthSquared() < 1e-4f ? Vector3.UnitY : Vector3.Normalize(inwardLocal);
        return TryQueueLocal(physics, s, hitLocal, inwardLocal, energy, specificEnergy, qStar, density, impactSpeed, impactor);
    }

    /// <summary>Same with the hit point already in the storage corner frame (stage 3.4 off-screen impacts).</summary>
    internal static bool TryQueueLocal(VoxelPhysicsComponent physics, MovableAsteroids.State s, Vector3 hitLocal, Vector3 inwardLocal,
        double energy, double specificEnergy, double qStar, double density, float impactSpeed, string impactor)
    {
        long now = DateTime.UtcNow.Ticks;
        if (s.IsClient || s.Grids.Count > 0 || now < s.ShatterImmuneUntilTicks) return false;
        if (now - Interlocked.Read(ref _lastQueueTicks) < QueueSpacingTicks || Volatile.Read(ref _inFlight) >= 2) return false;
        var storage = physics._storage;
        if (storage?.Storage == null) return false;
        double eqR = Math.Cbrt(3.0 * Math.Max(1.0, s.Mass / Math.Max(300.0, s.AvgDensity)) / (4.0 * Math.PI));
        if (eqR < MinShatterRadius) return false;
        Vector3I size = physics._storageSize;
        float vs = storage.VoxelSize;

        bool spall = size.Max() > FullShatterMaxSize;
        float spallR = 0f;
        if (spall)
        {
            // Spall volume grows with the excess specific energy; at most 48 m (a 97-voxel box read at LOD0).
            double ratio = specificEnergy / qStar;
            spallR = (float)Math.Clamp(eqR * 0.18 * Math.Cbrt(ratio), 8.0, 48.0) / vs * vs;
        }

        var job = new Job
        {
            Parent = physics, State = s, Raw = storage.Storage, StorageComp = storage, Size = size, VoxelSize = vs,
            HitLocal = hitLocal, InwardLocal = inwardLocal, Energy = energy, SpecificEnergy = specificEnergy, QStar = qStar,
            Density = density, FallbackDensity = s.AvgDensity > 0 ? s.AvgDensity : Settings.AsteroidDensity,
            ImpactSpeed = impactSpeed, Spall = spall, SpallRadius = spallR, Impactor = impactor,
        };
        Interlocked.Exchange(ref _lastQueueTicks, now);
        Interlocked.Increment(ref _inFlight);
        Interlocked.Increment(ref Queued);
        s.ShatterImmuneUntilTicks = now + ShatterImmuneTicks;   // one shatter per asteroid at a time
        s.LastImpactTicks = now;
        s.Impacted = true;
        ProbeLog.Write($"asteroid: SHATTER queued on {s.Name} vs {impactor}: e={specificEnergy:0} J/kg >= Q*={qStar:0} " +
                       $"(rho={density:0}, v={impactSpeed:0.0} m/s, E={energy / 1e6:0.00} MJ, {(spall ? $"spall r={spallR:0} m" : "full split")})");

        // Reads: VRage.Library/Keen/VRage/Library/Threading/Parallel.cs:123 Schedule(Action, object lifetime, TaskDeadline?).
        // Run 66: voxel reads need the engine's worker threads (BumpAllocator), never Task.Run.
        Keen.VRage.Library.Threading.Parallel.Schedule(() =>
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            try { Work(job); }
            catch (Exception ex) { job.Error = ex.GetType().Name + ": " + ex.Message; }
            finally
            {
                job.WorkerMs = sw.Elapsed.TotalMilliseconds;
                job.Done = true;
                lock (Completed) Completed.Enqueue(job);
            }
        }, physics, Keen.VRage.Library.Threading.TaskDeadline.OneSecond);
        return true;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Worker: noise-warped Voronoi partition of the LOD0 voxels
    // ---------------------------------------------------------------------------------------------------------------

    // Source: none (plugin-only). Value noise in [-1, 1] (hash lattice, smoothstep trilinear).
    private static float Hash(int x, int y, int z, int seed)
    {
        unchecked
        {
            uint h = (uint)(x * 374761393 + y * 668265263 + z * 1274126177 + seed * 1442695041);
            h = (h ^ (h >> 13)) * 1274126177u;
            h ^= h >> 16;
            return (h & 0xFFFFFF) / (float)0x7FFFFF - 1f;
        }
    }

    private static float Noise(float x, float y, float z, int seed)
    {
        int xi = (int)MathF.Floor(x), yi = (int)MathF.Floor(y), zi = (int)MathF.Floor(z);
        float fx = x - xi, fy = y - yi, fz = z - zi;
        fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy); fz = fz * fz * (3 - 2 * fz);
        float c000 = Hash(xi, yi, zi, seed), c100 = Hash(xi + 1, yi, zi, seed);
        float c010 = Hash(xi, yi + 1, zi, seed), c110 = Hash(xi + 1, yi + 1, zi, seed);
        float c001 = Hash(xi, yi, zi + 1, seed), c101 = Hash(xi + 1, yi, zi + 1, seed);
        float c011 = Hash(xi, yi + 1, zi + 1, seed), c111 = Hash(xi + 1, yi + 1, zi + 1, seed);
        float x00 = c000 + (c100 - c000) * fx, x10 = c010 + (c110 - c010) * fx;
        float x01 = c001 + (c101 - c001) * fx, x11 = c011 + (c111 - c011) * fx;
        float y0 = x00 + (x10 - x00) * fy, y1 = x01 + (x11 - x01) * fy;
        return y0 + (y1 - y0) * fz;
    }

    private static void Work(Job job)
    {
        Vector3I N = job.Size;
        float vs = job.VoxelSize;
        Vector3 halfN = new Vector3(N.X, N.Y, N.Z) * 0.5f;
        // Voxel index v <-> storage-centred local (v + 0.5 - N/2) * vs (same convention as MovableAsteroidMass.Compute).
        Vector3 hitV = job.HitLocal / vs;   // corner frame -> voxel index space
        float spallRV = job.SpallRadius / vs;

        Vector3I min, max;
        if (job.Spall)
        {
            Vector3I h = new Vector3I((int)MathF.Round(hitV.X), (int)MathF.Round(hitV.Y), (int)MathF.Round(hitV.Z));
            int r = (int)MathF.Ceiling(spallRV) + 2;
            min = Max3(Vector3I.Zero, h - new Vector3I(r));
            max = Min3(N - 1, h + new Vector3I(r));
        }
        else { min = Vector3I.Zero; max = N - 1; }
        Vector3I D = max - min + 1;
        if (D.X <= 0 || D.Y <= 0 || D.Z <= 0) throw new InvalidOperationException("empty region");
        int count = D.X * D.Y * D.Z;

        // Reads: VRage.Voxels/Keen/VRage/Voxels/Storage/IStorage.cs:163 ReadRange (LOD0 is the only writable LOD) and
        // StorageData.cs:137 (size, flags) ctor, :378/:411 Content/Material(linear). Same read as MovableAsteroidMass.Compute.
        var data = new StorageData(D, StorageDataTypeFlags.ContentAndMaterial, true, "AsteroidShatter");
        byte[] label = new byte[count];          // 255 = not part of the broken region, 254 = pulverized / crack, else shard index
        byte[] newContent = new byte[count];
        try
        {
            VoxelRequestFlags flags = 0;
            job.Raw.ReadRange(ref data, StorageDataTypeFlags.Content, 0, min, max, ref flags);
            job.Raw.ReadRange(ref data, StorageDataTypeFlags.Material, 0, min, max, ref flags);

            // Density per material index (MovableAsteroidMass.MaterialOf caches names / densities under a lock - one call per
            // material, not per voxel).
            double[] dens = new double[256];
            for (int i = 0; i < 256; i++) dens[i] = double.NaN;
            double voxelVol = (double)vs * vs * vs;

            // Solid voxels of the region (spall: inside the spall sphere, 1 voxel inside the carve radius).
            var solid = new List<int>(count / 2);
            Vector3I q = default;
            for (q.Z = 0; q.Z < D.Z; q.Z++)
                for (q.Y = 0; q.Y < D.Y; q.Y++)
                    for (q.X = 0; q.X < D.X; q.X++)
                    {
                        int li = data.ComputeLinear(in q);
                        label[li] = 255;
                        if (data.Content(li) == 0) continue;
                        if (job.Spall)
                        {
                            Vector3 p = new Vector3(q.X + min.X, q.Y + min.Y, q.Z + min.Z);
                            if ((p - hitV).Length() > spallRV - 1f) continue;
                        }
                        solid.Add(li);
                    }
            if (solid.Count < MinShardVoxels * 2) throw new InvalidOperationException($"only {solid.Count} solid voxels in the region");

            // Region size for seed placement / noise scale.
            double regionR = job.Spall ? spallRV : Math.Cbrt(3.0 * solid.Count / (4.0 * Math.PI));
            double ratio = job.SpecificEnergy / job.QStar;
            int K = job.Spall ? Math.Clamp(3 + (int)Math.Floor(Math.Log2(Math.Max(1.0, ratio))), 3, 5)
                              : Math.Clamp(4 + (int)Math.Floor(Math.Log2(Math.Max(1.0, ratio))), 4, 6);
            var rng = new Random(unchecked((int)DateTime.UtcNow.Ticks) ^ job.State.Name.GetHashCode());

            Vector3 PosOf(int li) { data.ComputePosition(li, out Vector3I pp); return new Vector3(pp.X, pp.Y, pp.Z); }

            // Seeds: two near the impact (smaller sharp fragments), the rest spread by farthest-point sampling over the rock
            // (larger angular slabs).
            var seeds = new List<Vector3>(K);
            Vector3 hitRegion = hitV - new Vector3(min.X, min.Y, min.Z);
            int samples = Math.Min(512, solid.Count);
            var cand = new Vector3[samples];
            for (int i = 0; i < samples; i++) cand[i] = PosOf(solid[rng.Next(solid.Count)]);
            for (int n = 0; n < 2; n++)
            {
                Vector3 target = hitRegion + job.InwardLocal * (float)(regionR * (0.15 + 0.15 * n))
                               + new Vector3((float)(rng.NextDouble() - 0.5), (float)(rng.NextDouble() - 0.5), (float)(rng.NextDouble() - 0.5)) * (float)(regionR * 0.25);
                Vector3 bestC = cand[0];
                float bestD = float.MaxValue;
                foreach (Vector3 c in cand) { float d = (c - target).LengthSquared(); if (d < bestD) { bestD = d; bestC = c; } }
                seeds.Add(bestC);
            }
            while (seeds.Count < K)
            {
                Vector3 bestC = cand[0];
                float bestD = -1f;
                foreach (Vector3 c in cand)
                {
                    float dmin = float.MaxValue;
                    foreach (Vector3 sd in seeds) dmin = Math.Min(dmin, (c - sd).LengthSquared());
                    if (dmin > bestD) { bestD = dmin; bestC = c; }
                }
                seeds.Add(bestC);
            }

            // Domain-warped Voronoi: d_k = |p + A*warp(p) - s_k|. Warping the point once (3 noise lookups, 2 octaves) gives
            // the same craggy, non-planar fracture faces as perturbing each seed distance (Gemini's d_k + A*noise_k) at a
            // third of the cost for K = 6.
            float amp = (float)Math.Clamp(0.12 * regionR * vs, 2.5, 4.5) / vs;   // voxels
            float freq = 0.12f * vs;                                             // per voxel
            float pulvR = (float)Math.Max(1.5, 0.10 * regionR);
            int noiseSeed = rng.Next();

            var cnt = new int[K];
            var m = new double[K];
            var sx = new double[K]; var sy = new double[K]; var sz = new double[K];
            var sxx = new double[K]; var syy = new double[K]; var szz = new double[K];
            var bmin = new Vector3I[K]; var bmax = new Vector3I[K];
            for (int k = 0; k < K; k++) { bmin[k] = new Vector3I(int.MaxValue); bmax[k] = new Vector3I(int.MinValue); }
            var matMass = new Dictionary<byte, double>[K];
            for (int k = 0; k < K; k++) matMass[k] = new Dictionary<byte, double>();
            double pulverized = 0, regionKg = 0;

            foreach (int li in solid)
            {
                data.ComputePosition(li, out Vector3I pi);
                Vector3 p = new Vector3(pi.X, pi.Y, pi.Z);
                byte c = data.Content(li);
                byte mat = data.Material(li);
                double rho = dens[mat];
                if (double.IsNaN(rho)) rho = dens[mat] = MovableAsteroidMass.MaterialOf(job.StorageComp, mat, job.FallbackDensity).density;
                double mi = c / 255.0 * voxelVol * rho;
                regionKg += mi;

                if ((p - hitRegion).Length() < pulvR) { label[li] = 254; pulverized += mi; continue; }

                float wx = p.X * freq, wy = p.Y * freq, wz = p.Z * freq;
                Vector3 warp = new Vector3(
                    Noise(wx, wy, wz, noiseSeed) + 0.5f * Noise(wx * 2.3f, wy * 2.3f, wz * 2.3f, noiseSeed + 11),
                    Noise(wx + 31.7f, wy, wz, noiseSeed + 1) + 0.5f * Noise(wx * 2.3f + 5f, wy * 2.3f, wz * 2.3f, noiseSeed + 12),
                    Noise(wx, wy + 17.3f, wz, noiseSeed + 2) + 0.5f * Noise(wx * 2.3f, wy * 2.3f + 9f, wz * 2.3f, noiseSeed + 13));
                Vector3 pw = p + warp * (amp / 1.5f);

                int k1 = 0, k2 = 0;
                float d1 = float.MaxValue, d2 = float.MaxValue;
                for (int k = 0; k < K; k++)
                {
                    float d = (pw - seeds[k]).Length();
                    if (d < d1) { d2 = d1; k2 = k1; d1 = d; k1 = k; }
                    else if (d < d2) { d2 = d; k2 = k; }
                }
                float gap = d2 - d1;
                if (gap < GapVoxels) { label[li] = 254; pulverized += mi; continue; }
                // Sub-voxel crack face: fade content across one voxel beyond the gap so the iso surface is not stair-stepped.
                float keep = Math.Clamp(gap - GapVoxels, 0f, 1f);
                byte nc = (byte)Math.Max(1, (int)(c * (0.35f + 0.65f * keep)));
                label[li] = (byte)k1;
                newContent[li] = nc;
                double mk = nc / 255.0 * voxelVol * rho;
                cnt[k1]++;
                m[k1] += mk;
                double gx = (pi.X + min.X + 0.5 - halfN.X) * vs, gy = (pi.Y + min.Y + 0.5 - halfN.Y) * vs, gz = (pi.Z + min.Z + 0.5 - halfN.Z) * vs;
                sx[k1] += mk * gx; sy[k1] += mk * gy; sz[k1] += mk * gz;
                sxx[k1] += mk * gx * gx; syy[k1] += mk * gy * gy; szz[k1] += mk * gz * gz;
                bmin[k1] = Min3(bmin[k1], pi);
                bmax[k1] = Max3(bmax[k1], pi);
                matMass[k1][mat] = (matMass[k1].TryGetValue(mat, out double mm) ? mm : 0) + mk;
            }
            job.RegionKg = regionKg;

            // Build one storage per surviving shard.
            IStorageDataProvider? provider = job.Raw.DataProvider;
            for (int k = 0; k < K; k++)
            {
                if (cnt[k] < MinShardVoxels || m[k] <= 0)
                {
                    pulverized += m[k];
                    continue;
                }
                Vector3I ext = bmax[k] - bmin[k] + 1;
                int S = Math.Max(MinShardStorage, MathHelper.RoundUpToPowerOfTwo(ext.Max() + 4));
                // Shard storage voxel w <-> parent voxel w + o, o integer, so the shard's voxels land exactly where they were.
                Vector3I centreRegion = new Vector3I((bmin[k].X + bmax[k].X) / 2, (bmin[k].Y + bmax[k].Y) / 2, (bmin[k].Z + bmax[k].Z) / 2);
                Vector3I o = centreRegion + min - new Vector3I(S / 2);

                var sd = new StorageData(new Vector3I(S), StorageDataTypeFlags.ContentAndMaterial, true, "AsteroidShard");
                try
                {
                    byte dominant = 0;
                    double dm = -1;
                    foreach (var kv in matMass[k]) if (kv.Value > dm) { dm = kv.Value; dominant = kv.Key; }
                    sd.ClearContent(0);
                    sd.ClearMaterials(dominant);
                    Vector3 com = new Vector3((float)(sx[k] / m[k]), (float)(sy[k] / m[k]), (float)(sz[k] / m[k]));   // parent-local m
                    Vector3 comV = com / vs + halfN - new Vector3(0.5f) - new Vector3(min.X, min.Y, min.Z);           // region voxels
                    // Inscribed radius: nearest non-shard voxel inside the shard's bounds (or the bounds edge) from the CoM.
                    float ins = Math.Min(Math.Min(Math.Min(comV.X - bmin[k].X, bmax[k].X - comV.X), Math.Min(comV.Y - bmin[k].Y, bmax[k].Y - comV.Y)),
                                         Math.Min(comV.Z - bmin[k].Z, bmax[k].Z - comV.Z)) + 0.5f;
                    Vector3I w = default;
                    for (q.Z = bmin[k].Z; q.Z <= bmax[k].Z; q.Z++)
                        for (q.Y = bmin[k].Y; q.Y <= bmax[k].Y; q.Y++)
                            for (q.X = bmin[k].X; q.X <= bmax[k].X; q.X++)
                            {
                                int li = data.ComputeLinear(in q);
                                if (label[li] != k)
                                {
                                    float dd = (new Vector3(q.X, q.Y, q.Z) - comV).Length() - 0.5f;
                                    if (dd < ins) ins = dd;
                                    continue;
                                }
                                w = q + min - o;
                                int wi = sd.ComputeLinear(in w);
                                sd.Content(wi, newContent[li]);
                                sd.Material(wi, data.Material(li));
                            }

                    // Reads: VRage.Voxels/Keen/VRage/Voxels/Storage/OcTree/OctreeStorage.cs:68 OctreeStorage(provider, size) -
                    // a pristine copy of the parent's procedural shape - then IStorage.cs:174 WriteRange (LOD0 only) overwrites
                    // the whole box with the shard, so nothing of the provider's own shape remains visible.
                    IStorage shardStorage = new OctreeStorage(provider, new Vector3I(S));
                    shardStorage.WriteRange(ref sd, StorageDataTypeFlags.ContentAndMaterial, Vector3I.Zero, new Vector3I(S - 1), false);

                    Vector3 originLocal = (new Vector3(o.X, o.Y, o.Z) + new Vector3(S * 0.5f) - halfN) * vs;
                    double M = m[k];
                    double cxx = sxx[k] / M - com.X * com.X, cyy = syy[k] / M - com.Y * com.Y, czz = szz[k] / M - com.Z * com.Z;
                    double cell = vs * vs / 6.0;
                    double ixx = M * (cyy + czz + cell), iyy = M * (cxx + czz + cell), izz = M * (cxx + cyy + cell);
                    double vol = 0;
                    foreach (var kv in matMass[k]) vol += kv.Value / (double.IsNaN(dens[kv.Key]) ? job.FallbackDensity : dens[kv.Key]);
                    double eqR = Math.Cbrt(3.0 * Math.Max(1.0, vol) / (4.0 * Math.PI));
                    var parts = new List<string>();
                    foreach (var kv in matMass[k])
                        if (kv.Value > 0.02 * M) parts.Add($"{MovableAsteroidMass.MaterialOf(job.StorageComp, kv.Key, job.FallbackDensity).name} {100 * kv.Value / M:0}%");
                    job.Shards.Add(new ShardResult
                    {
                        Storage = shardStorage,
                        OriginLocal = originLocal,
                        ComParentLocal = com,
                        Mass = new RigidBodyMassProperties
                        {
                            InvMass = (float)(1.0 / M),
                            CenterOfMass = com - originLocal,
                            InvInertiaTensor = new Vector3((float)(1.0 / ixx), (float)(1.0 / iyy), (float)(1.0 / izz)),
                            InertiaMajorAxisRotation = Quaternion.Identity,
                            Volume = (float)vol,
                        },
                        ProxyRadius = (float)Math.Max(1.0, Math.Min(0.78 * eqR, 0.95 * Math.Max(0f, ins) * vs)),
                        Radius = eqR,
                        Density = M / Math.Max(1.0, vol),
                        MassKg = M,
                        Voxels = cnt[k],
                        StorageSize = S,
                        Composition = string.Join(", ", parts),
                    });
                }
                finally { ((IDisposable)sd).Dispose(); }
            }
            job.PulverizedKg = pulverized;
            if (job.Shards.Count == 0 || (!job.Spall && job.Shards.Count < 2))
                throw new InvalidOperationException($"{job.Shards.Count} shard(s) survived the minimum size");
        }
        catch
        {
            foreach (ShardResult r in job.Shards) try { r.Storage.Dispose(); } catch { }
            job.Shards.Clear();
            throw;
        }
        finally { ((IDisposable)data).Dispose(); }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Spawn (DCS thread, WorldUpdateBegin)
    // ---------------------------------------------------------------------------------------------------------------

    /// <summary>While set, VoxelStorageComponentBase.SaveStorage returns an empty save state: serializing the parent only to
    /// get an object builder template must not offload (serialize) its whole voxel storage, which the shards replace anyway.</summary>
    [ThreadStatic] private static bool _skipSave;

    // Patches: VRage.Voxels/Keen/VRage/Voxels/Components/VoxelStorageComponentBase.cs  SaveStorage (:341-349; called from
    // Serialize :217-222). Planets override it (VoxelPlanetComponent.cs:170) and are never shattered.
    [HarmonyPatch(typeof(VoxelStorageComponentBase), "SaveStorage")]
    internal static class SkipSaveStoragePatch
    {
        [HarmonyPrefix]
        private static bool Prefix(ref OffloadedVoxelStorage __result)
        {
            if (!_skipSave) return true;
            __result = default;
            return false;
        }
    }

    /// <summary>The entity's object builder without offloading its voxel storage (shard and belt-catalog templates).</summary>
    // Reads: AsteroidClipboard.CaptureJob - Scene.EntitySerializer.Serialize(entity).
    internal static EntityObjectBuilder SerializeTemplate(Entity e)
    {
        _skipSave = true;
        try { return e.Scene.EntitySerializer.Serialize(e); }
        finally { _skipSave = false; }
    }

    internal static void FlushCompleted(Session session)
    {
        while (true)
        {
            Job? job;
            lock (Completed) { if (Completed.Count == 0) return; job = Completed.Dequeue(); }
            Interlocked.Decrement(ref _inFlight);
            try { Finish(session, job); }
            catch (Exception ex)
            {
                Interlocked.Increment(ref Failed);
                ProbeLog.Write("asteroid: shatter spawn failed: " + ex);
                foreach (ShardResult r in job.Shards) try { r.Storage.Dispose(); } catch { }
            }
        }
    }

    private static void Finish(Session session, Job job)
    {
        MovableAsteroids.State s = job.State;
        VoxelPhysicsComponent parent = job.Parent;
        Entity? pe = parent.Entity;
        if (job.Error != null || pe == null || parent.DEntity.IsEmpty)
        {
            Interlocked.Increment(ref Failed);
            foreach (ShardResult r in job.Shards) try { r.Storage.Dispose(); } catch { }
            ProbeLog.Write($"asteroid: shatter of {s.Name} abandoned ({job.Error ?? "asteroid gone"}) after {job.WorkerMs:0} ms - it keeps its shape");
            s.ShatterImmuneUntilTicks = DateTime.UtcNow.Ticks + ShatterImmuneTicks;
            return;
        }

        // Live parent state now (the worker took a few ticks; shard poses are parent-local).
        WorldTransform pose = parent.Data.GetWorldTransform();
        RigidBodyData prb = parent.Data.TryGet<RigidBodyData>(out RigidBodyData v) ? v : default;
        // A railed parent (stage 3.4, shattered off-screen) is static: its written pose is stale and it has no velocity.
        if (AsteroidRails.TryLivePose(parent, out WorldTransform livePose, out Vector3 liveLin, out Vector3 liveAng))
        {
            pose = livePose;
            prb = new RigidBodyData { LinearVelocity = liveLin, AngularVelocity = liveAng };
        }
        Vector3 parentCom = parent.Data.TryGet<RigidBodyMassProperties>(out RigidBodyMassProperties pmp) ? pmp.CenterOfMass : Vector3.Zero;
        double parentMass = s.Mass > 1f ? s.Mass : 1e6;

        // Ejection: speed from the energy above Q* (10 % of the excess as fragment kinetic energy), away from the impact and
        // from the parent's centre, smaller shards faster.
        // Source: none (plugin-only).
        double excess = Math.Max(0.0, job.SpecificEnergy - job.QStar);
        float uEject = (float)Math.Clamp(Math.Sqrt(2.0 * 0.10 * excess), 2.0, 45.0);
        var rng = new Random(unchecked((int)DateTime.UtcNow.Ticks));
        // Hit point in the entity (storage-centre) frame, the frame of RigidBodyMassProperties and the shard origins.
        Vector3 hitCentre = job.HitLocal - new Vector3(job.Raw.Size.X, job.Raw.Size.Y, job.Raw.Size.Z) * (0.5f * job.VoxelSize);
        double shardMass = 0;
        foreach (ShardResult r in job.Shards) shardMass += r.MassKg;
        double meanMass = shardMass / job.Shards.Count;
        var vRel = new Vector3[job.Shards.Count];
        Vector3 momentum = Vector3.Zero;
        for (int i = 0; i < job.Shards.Count; i++)
        {
            ShardResult r = job.Shards[i];
            Vector3 fromHit = r.ComParentLocal - hitCentre;
            Vector3 fromCentre = r.ComParentLocal - parentCom;
            Vector3 dir = (fromHit.LengthSquared() > 1e-3f ? Vector3.Normalize(fromHit) : -job.InwardLocal) * 0.7f
                        + (fromCentre.LengthSquared() > 1e-3f ? Vector3.Normalize(fromCentre) : -job.InwardLocal) * 0.3f
                        + new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f) * 0.3f;
            if (job.Spall) dir += -job.InwardLocal * 0.8f;   // spall leaves the struck face
            dir = dir.LengthSquared() > 1e-4f ? Vector3.Normalize(dir) : -job.InwardLocal;
            float sizeScale = (float)Math.Clamp(Math.Pow(meanMass / r.MassKg, 0.15), 0.6, 1.8);
            vRel[i] = dir * uEject * sizeScale;
            momentum += vRel[i] * (float)r.MassKg;
        }
        // Full split: subtract the mass-weighted mean so total linear momentum is exactly the parent's. Spall: the parent takes
        // the opposite momentum.
        Vector3 parentKick = Vector3.Zero;
        if (!job.Spall)
        {
            Vector3 mean = momentum / (float)shardMass;
            for (int i = 0; i < vRel.Length; i++) vRel[i] -= mean;
        }
        else
        {
            double remaining = Math.Max(parentMass * 0.1, parentMass - shardMass - job.PulverizedKg);
            parentKick = -momentum / (float)remaining;
        }

        // Template object builder: the parent itself, without offloading its storage.
        EntityObjectBuilder template = SerializeTemplate(pe);

        IEntitySpawner spawner = session.Get<IEntitySpawner>();
        Vector3D parentComWorld = WorldTransform.Transform((Vector3D)parentCom, in pose);
        int spawned = 0;
        var log = new System.Text.StringBuilder();
        for (int i = 0; i < job.Shards.Count; i++)
        {
            ShardResult r = job.Shards[i];
            try
            {
                // Reads: AsteroidClipboard.ExecutePaste - DeepClone, WorldTransform, SanitizeVoxelOB(new procedural id, manual).
                EntityObjectBuilder clone = template.DeepClone();
                WorldTransform shardPose = pose * new RelativeTransform(r.OriginLocal);
                clone.OB<WorldTransformComponent, WorldTransformComponentObjectBuilder>().Transform = shardPose;
                AsteroidClipboard.SanitizeVoxelOB(clone, newProceduralId: true);
                int vIdx = clone.TryGetIndexOf<VoxelStorageComponentBase>();
                if (vIdx < 0 || clone.ObjectBuilders[vIdx] is not VoxelStorageComponentBaseObjectBuilder vob)
                    throw new InvalidOperationException("no voxel storage object builder");
                // Reads: VoxelStorageComponentBaseObjectBuilder.cs:188 Offloaded / :195 SharedStorage; OffloadedVoxelStorage
                // SetPreloaded / TryExtractPreloaded. No Identifier, so Offload (VoxelStorageComponentBase.cs:943) serializes
                // the shard's own voxels when it is saved or replicated.
                var off = default(OffloadedVoxelStorage);
                off.SetPreloaded(r.Storage);
                vob.Offloaded = off;
                vob.SharedStorage = default;
                if (vob is ProceduralVoxelStorageComponentObjectBuilder pvob) pvob.Size = r.StorageSize;

                Vector3D comWorld = WorldTransform.Transform((Vector3D)r.Mass.CenterOfMass, in shardPose);
                Vector3 carried = prb.LinearVelocity + (Vector3)Vector3D.Cross((Vector3D)prb.AngularVelocity, comWorld - parentComWorld);
                Vector3 lin = carried + Vector3.Transform(vRel[i], pose.Orientation);
                Vector3 axis = Vector3.Normalize(new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f) + new Vector3(1e-3f));
                float tumble = (float)Math.Clamp(vRel[i].Length() / (2.0 * Math.Max(2.0, r.Radius)), 0.1, 1.2);
                Vector3 ang = prb.AngularVelocity + axis * tumble;

                lock (PendingShards)
                    PendingShards.Add(new ShardSpawn
                    {
                        Pos = shardPose.Position, LinVel = lin, AngVel = ang, Mass = r.Mass, ProxyRadius = r.ProxyRadius,
                        Radius = r.Radius, Density = r.Density, Filter = s.Filter, PlayerTouched = s.PlayerTouched,
                        Ticks = DateTime.UtcNow.Ticks,
                    });
                // Materialize BoundingBoxData + RigidBodyMassProperties + filter/tags in DEntityBuilder so synchronous
                // SpawnEntity -> RegisterSpaceProbe and HavokWorldHelper.AllocateBody never throw Data not found.
                float half = r.StorageSize * 0.5f * job.VoxelSize;
                var eb = MovableAsteroids.BuildDynamicSpawnData(session, new Vector3(half), in r.Mass, lin, ang, s.Filter);
                Entity e = spawner.SpawnEntity(clone, eb);
                try { e.Data.Set(new RigidBodyData { LinearVelocity = lin, AngularVelocity = ang }); } catch { }
                spawned++;
                Interlocked.Increment(ref ShardsSpawned);
                log.Append($"\n    shard {e}: {r.MassKg / 1000:0} t ({r.Voxels} voxels, storage {r.StorageSize}^3, r {r.Radius:0.0} m, proxy {r.ProxyRadius:0.0} m) " +
                           $"v_rel {vRel[i].Length():0.0} m/s, spin {tumble:0.00} rad/s [{r.Composition}]");
            }
            catch (Exception ex)
            {
                try { r.Storage.Dispose(); } catch { }
                ProbeLog.Write($"asteroid: shard {i} of {s.Name} failed to spawn: {ex.Message}");
            }
        }

        if (!job.Spall)
        {
            // Reads: VoxelStorageComponentBase.cs:830 TryDeleteManually (tells the procedural generator the rock is gone, so it
            // is not regenerated at its seed position); AsteroidClipboard's cut uses the same fallback.
            if (spawned > 0)
            {
                if (s.CatalogId >= 0) AsteroidStore.MarkCatalog(s.CatalogId, destroyed: true);   // stage 4.2: never respawn it
                VoxelStorageComponentBase? storage = pe.TryGet<VoxelStorageComponentBase>();
                if (storage == null || !storage.TryDeleteManually()) session.MarkEntityForClose(pe, true);
            }
            Interlocked.Increment(ref Shatters);
        }
        else
        {
            // Spall: carve the spalled sphere out of the giant parent through the replicated destruction signal (clients see
            // it), push the parent back, and recompute its mass.
            // Reads: AsteroidImpacts.CarveImpact - CutOutCapsulesBuilder + VoxelDestructionComponent.OnSphereDestruction
            // (VRage.Voxels/.../Components/VoxelDestructionComponent.cs:158 [Server][Clients]).
            if (spawned > 0)
            {
                var builder = new CutOutCapsulesBuilder(0.25f);
                var cap = new Capsule(job.HitLocal - job.InwardLocal * 0.01f, job.HitLocal + job.InwardLocal * 0.01f, job.SpallRadius);
                builder.AddCapsule(in cap, job.VoxelSize);
                builder.FinalPass();
                if (builder.IsValid)
                {
                    if (s.PlayerTouched) MovableAsteroids.Promote(parent, s, "spalled by a touched impactor");
                    else s.BackgroundCarveUntilTicks = DateTime.UtcNow.Ticks + TimeSpan.TicksPerSecond * 5;
                    var destruction = pe.TryGet<VoxelDestructionComponent>();
                    if (destruction != null)
                    {
                        WorldTransform centered = job.StorageComp.ComputeCenteredTransform();
                        destruction._worldToStorageCenter = WorldTransform.GetRelativeTransform(WorldTransform.Identity, centered);
                        destruction.OnSphereDestruction(builder);
                    }
                }
                if (parentKick != Vector3.Zero && AsteroidRails.IsRailed(parent))
                    AsteroidRails.Kick(parent, Vector3.Transform(parentKick, pose.Orientation));
                else if (parentKick != Vector3.Zero)
                    parent.Data.Set(new RigidBodyData
                    {
                        LinearVelocity = prb.LinearVelocity + Vector3.Transform(parentKick, pose.Orientation),
                        AngularVelocity = prb.AngularVelocity,
                    });
                s.MassApplied = false;
                s.MassScheduled = false;
                s.MassTries = 0;
                s.MassRetryTicks = DateTime.UtcNow.Ticks + TimeSpan.TicksPerSecond;
            }
            Interlocked.Increment(ref Spalls);
        }

        ProbeLog.Write($"asteroid: {(job.Spall ? "SPALL" : "SHATTER")} {s.Name}: {spawned}/{job.Shards.Count} shards, " +
                       $"{shardMass / 1000:0} t in shards + {job.PulverizedKg / 1000:0} t pulverized of {job.RegionKg / 1000:0} t broken " +
                       $"(parent {parentMass / 1000:0} t), eject {uEject:0.0} m/s, worker {job.WorkerMs:0} ms" + log);
    }
}

/// <summary>Server jobs for <see cref="AsteroidShatter"/>: spawn finished shatters at the start of the tick, outside the voxel
/// archetype iteration (as AsteroidClipboard.PasteJobPlanet and RealRails.FlushMotion do).</summary>
internal static class AsteroidShatterJobs
{
    private static readonly object Gate = new();

    // Reads: RealRails.FlushMotion / AsteroidClipboard.PasteJobPlanet - one WorldUpdateBegin job per planet; the first one in a
    // tick drains the queue.
    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(InstanceBind<VoxelPlanetComponent>))]
    internal static void Flush(VoxelPlanetComponent planet)
    {
        if (planet?.Entity == null || !Monitor.TryEnter(Gate)) return;
        try { AsteroidShatter.FlushCompleted(planet.Entity.GetSession()); }
        finally { Monitor.Exit(Gate); }
    }

    /// <summary>Planet-less worlds.</summary>
    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(InstanceBind<CharacterComponent>))]
    internal static void FlushCharacter(CharacterComponent character)
    {
        if (character?.Entity == null || !Monitor.TryEnter(Gate)) return;
        try { AsteroidShatter.FlushCompleted(character.Entity.GetSession()); }
        finally { Monitor.Exit(Gate); }
    }
}
