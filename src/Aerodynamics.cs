using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;
using Keen.Game2.Simulation.WorldObjects.CubeBlocks;
using Keen.Game2.Simulation.WorldObjects.CubeGrids;
using Keen.Game2.Simulation.WorldObjects.CubeGrids.BlockOctrees;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Data;

namespace SpinProbe;

/// <summary>
/// Fast, cached shape-aware aerodynamics + closed-bay subgrid occlusion.
///
/// Performance & accuracy architecture:
/// - Cached against BlockOctreeComponent.Version (rebuilt ONLY on block add/remove/destruction).
/// - Coarse 3D occupancy grid capped at 32^3 (4 KB bitset, fits in L1 cache) and walked in a SINGLE sparse pass
///   using BitOperations.TrailingZeroCount (skipping 64 empty voxels per instruction) + 2D min/max heightmaps for
///   windward vs recessed weighting. Even large ships rebuild in fractions of a millisecond.
/// - Word/CamelCase token-based block classifier (explicitly excludes industrial/functional blocks like Refinery so
///   "fin" never false-matches "refinery").
/// - Zero airfoil lift for blunt cube/brick hulls; real thin-airfoil lift with stall (~24-35 deg) kicks in only for
///   genuinely flat wing/lifting-body aspect ratios or sloped aerodynamic fuselages.
/// - Real Stopwatch timing telemetry (aeroEvalUs, aeroBuildUs) logged in RealPhysics.MaybeLog().
/// </summary>
internal static class Aerodynamics
{
    private const float CellSize = 0.25f; // SE2 unified grid cell size (CubeGridCoords.cs:16)
    private const int MaxDim = 32;        // 32^3 max -> 4 KB bitset (L1-resident, ~8x faster rebuild)
    private const int MinBlocksForVoxelProfile = 4; // tiny 1-3 block crash debris uses instant AABB drag (0 build cost)
    private const long RebuildCooldownTicks = 15;   // at most 4 rebuilds/sec during active crash/grinding

    internal static long EvalBodies, SubgridsOccluded, ProfilesBuilt;
    internal static long LiftMaxMilli;
    internal static long EvalStopwatchTicks, BuildStopwatchTicks;
    private static int _slowLogs;

    private enum BlockAeroKind : byte
    {
        Blunt = 0,     // full armor cube, functional/industrial blocks (Cd ~ 0.92)
        Sloped = 1,    // 45-deg slope, ramp, corner, half, rounded (Cd ~ 0.44)
        Sleek = 2,     // 2x1 long slope, nose, tip, cone, wing, fin (Cd ~ 0.24)
        ThinPlate = 3, // panel, plate, catwalk, cover
        Permeable = 4, // open grate / truss / permeable block
    }

    internal sealed class GridAeroProfile
    {
        internal int Version;
        internal long BuiltTick;
        internal int BlockCount;

        internal Vector3I OriginCell;
        internal int Step;
        internal int SizeX, SizeY, SizeZ;
        internal ulong[] Bits = Array.Empty<ulong>();
        internal BoundingBox LocalBounds;

        // 6 cardinal faces: 0=+X, 1=-X, 2=+Y, 3=-Y, 4=+Z, 5=-Z
        internal readonly float[] FaceArea = new float[6];
        internal readonly float[] FaceCdArea = new float[6];
        internal readonly Vector3[] FaceCoP = new Vector3[6];

        internal Vector3 LiftArea;
        internal Vector3 RotDamping;
        internal float CharRadius;
        internal float Sleekness;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal bool IsVoxelSolid(int vx, int vy, int vz)
        {
            if ((uint)vx >= (uint)SizeX || (uint)vy >= (uint)SizeY || (uint)vz >= (uint)SizeZ) return false;
            int bit = (vz * SizeY + vy) * SizeX + vx;
            return (Bits[bit >> 6] & (1UL << (bit & 63))) != 0UL;
        }

        internal bool LocalToVoxel(in Vector3 localPos, out int vx, out int vy, out int vz)
        {
            float inv = 1f / (CellSize * Step);
            vx = (int)MathF.Floor((localPos.X - (OriginCell.X - 0.5f) * CellSize) * inv);
            vy = (int)MathF.Floor((localPos.Y - (OriginCell.Y - 0.5f) * CellSize) * inv);
            vz = (int)MathF.Floor((localPos.Z - (OriginCell.Z - 0.5f) * CellSize) * inv);
            return (uint)vx < (uint)SizeX && (uint)vy < (uint)SizeY && (uint)vz < (uint)SizeZ;
        }
    }

    private static readonly ConditionalWeakTable<CubeGridComponent, GridAeroProfile> Profiles = new();
    private static readonly ConcurrentDictionary<Guid, BlockAeroKind> KindCache = new();

    internal static GridAeroProfile? GetOrBuildProfile(CubeGridComponent? grid)
    {
        if (grid == null) return null;
        BlockOctreeComponent? octree = grid._cubeBlockStorage;
        if (octree == null || octree.CubeBlockCount < MinBlocksForVoxelProfile) return null;

        int ver = octree.Version;
        long now = Profiler.Ticks;
        if (Profiles.TryGetValue(grid, out GridAeroProfile? existing))
        {
            if (existing.Version == ver || (now - existing.BuiltTick) < RebuildCooldownTicks)
                return existing;
        }

        long t0 = Stopwatch.GetTimestamp();
        GridAeroProfile built = BuildProfile(octree, ver, now);
        Interlocked.Add(ref BuildStopwatchTicks, Stopwatch.GetTimestamp() - t0);
        Profiles.AddOrUpdate(grid, built);
        Interlocked.Increment(ref ProfilesBuilt);
        return built;
    }

    private static GridAeroProfile BuildProfile(BlockOctreeComponent octree, int version, long builtTick)
    {
        Span<CubeBlockComponent?> blocks = octree.GetAllCubeBlocks();
        BoundingBoxI totalBox = BoundingBoxI.CreateInvalid();
        int validCount = 0;

        for (int i = 0; i < blocks.Length; i++)
        {
            CubeBlockComponent? b = blocks[i];
            if (b == null || b.IsDestroyed) continue;
            BoundingBoxI bb = b.AABB;
            totalBox.Include(in bb);
            validCount++;
        }

        var p = new GridAeroProfile { Version = version, BuiltTick = builtTick, BlockCount = validCount };
        if (validCount == 0 || !totalBox.IsValid())
        {
            for (int f = 0; f < 6; f++) { p.FaceArea[f] = 1f; p.FaceCdArea[f] = 1f; }
            p.CharRadius = 1f;
            return p;
        }

        Vector3I spanCells = totalBox.Max - totalBox.Min + Vector3I.One;
        int maxCells = Math.Max(spanCells.X, Math.Max(spanCells.Y, spanCells.Z));
        int step = 2;
        while ((maxCells + step - 1) / step > MaxDim) step *= 2;

        p.OriginCell = totalBox.Min;
        p.Step = step;
        int sx = p.SizeX = Math.Clamp((spanCells.X + step - 1) / step, 1, MaxDim);
        int sy = p.SizeY = Math.Clamp((spanCells.Y + step - 1) / step, 1, MaxDim);
        int sz = p.SizeZ = Math.Clamp((spanCells.Z + step - 1) / step, 1, MaxDim);
        p.LocalBounds = CubeGridCoords.GridToLocal(totalBox);

        int totalVoxels = sx * sy * sz;
        p.Bits = new ulong[(totalVoxels + 63) >> 6];
        byte[] cdGrid = new byte[totalVoxels];

        // 2D min/max heightmaps (max 32x32 = 1024 bytes each) for O(1) windward-vs-recessed check in ONE pass!
        Span<byte> minX = stackalloc byte[sy * sz]; minX.Fill(255);
        Span<byte> maxX = stackalloc byte[sy * sz]; maxX.Fill(0);
        Span<byte> minY = stackalloc byte[sx * sz]; minY.Fill(255);
        Span<byte> maxY = stackalloc byte[sx * sz]; maxY.Fill(0);
        Span<byte> minZ = stackalloc byte[sx * sy]; minZ.Fill(255);
        Span<byte> maxZ = stackalloc byte[sx * sy]; maxZ.Fill(0);

        int sleekVoxels = 0, solidVoxels = 0;

        for (int i = 0; i < blocks.Length; i++)
        {
            CubeBlockComponent? b = blocks[i];
            if (b == null || b.IsDestroyed) continue;

            float perm = b.EffectivePermeability;
            if (perm >= 0.85f) continue;

            BlockAeroKind kind = ClassifyBlock(b.Definition, perm);
            float baseCd = kind switch
            {
                BlockAeroKind.Sleek => 0.24f,
                BlockAeroKind.Sloped => 0.44f,
                BlockAeroKind.ThinPlate => 0.38f,
                BlockAeroKind.Permeable => 0.30f,
                _ => 0.92f,
            };
            byte cdByte = (byte)Math.Clamp((int)MathF.Round(baseCd * (1f - 0.5f * perm) * 100f), 10, 120);

            BoundingBoxI bb = b.AABB;
            int x0 = Math.Clamp((bb.Min.X - totalBox.Min.X) / step, 0, sx - 1);
            int y0 = Math.Clamp((bb.Min.Y - totalBox.Min.Y) / step, 0, sy - 1);
            int z0 = Math.Clamp((bb.Min.Z - totalBox.Min.Z) / step, 0, sz - 1);
            int x1 = Math.Clamp((bb.Max.X - totalBox.Min.X) / step, 0, sx - 1);
            int y1 = Math.Clamp((bb.Max.Y - totalBox.Min.Y) / step, 0, sy - 1);
            int z1 = Math.Clamp((bb.Max.Z - totalBox.Min.Z) / step, 0, sz - 1);

            for (int vz = z0; vz <= z1; vz++)
            {
                int zOff = vz * sy;
                int xzBase = vz * sx;
                for (int vy = y0; vy <= y1; vy++)
                {
                    int rowOff = (zOff + vy) * sx;
                    int yzIdx = zOff + vy;
                    int xyBase = vy * sx;
                    if (x0 < minX[yzIdx]) minX[yzIdx] = (byte)x0;
                    if (x1 > maxX[yzIdx]) maxX[yzIdx] = (byte)x1;

                    for (int vx = x0; vx <= x1; vx++)
                    {
                        int idx = rowOff + vx;
                        if (cdGrid[idx] == 0)
                        {
                            p.Bits[idx >> 6] |= 1UL << (idx & 63);
                            cdGrid[idx] = cdByte;
                            solidVoxels++;
                            if (kind == BlockAeroKind.Sleek || kind == BlockAeroKind.Sloped) sleekVoxels++;

                            int xzIdx = xzBase + vx;
                            if (vy < minY[xzIdx]) minY[xzIdx] = (byte)vy;
                            if (vy > maxY[xzIdx]) maxY[xzIdx] = (byte)vy;

                            int xyIdx = xyBase + vx;
                            if (vz < minZ[xyIdx]) minZ[xyIdx] = (byte)vz;
                            if (vz > maxZ[xyIdx]) maxZ[xyIdx] = (byte)vz;
                        }
                        else if (cdByte < cdGrid[idx])
                        {
                            cdGrid[idx] = cdByte;
                        }
                    }
                }
            }
        }

        if (solidVoxels == 0)
        {
            for (int f = 0; f < 6; f++) { p.FaceArea[f] = 1f; p.FaceCdArea[f] = 0.9f; }
            p.CharRadius = 1f;
            return p;
        }

        float voxelSize = CellSize * step;
        float faceArea = voxelSize * voxelSize;
        Vector3 originM = new Vector3(totalBox.Min.X, totalBox.Min.Y, totalBox.Min.Z) * CellSize + new Vector3(0.5f * voxelSize - 0.5f * CellSize);
        Vector3 boxCenter = p.LocalBounds.Center;

        Span<Vector3> copSum = stackalloc Vector3[6];
        Vector3 rotDamp = Vector3.Zero;
        int planeStride = sx * sy;

        // Outside air: empty voxels reachable from the box boundary (flood fill, <= 32^3 voxels). A face only meets the
        // airflow if the empty voxel next to it is outside air; walls of sealed rooms and closed bays face trapped air and
        // add no drag. (Before, they counted at the 25 % "recessed" weight.)
        byte[] outside = new byte[totalVoxels];
        int[] queue = new int[totalVoxels];
        int qHead = 0, qTail = 0;
        for (int vz = 0; vz < sz; vz++)
            for (int vy = 0; vy < sy; vy++)
                for (int vx = 0; vx < sx; vx++)
                {
                    if (vx != 0 && vx != sx - 1 && vy != 0 && vy != sy - 1 && vz != 0 && vz != sz - 1) continue;
                    int i0 = (vz * sy + vy) * sx + vx;
                    if (cdGrid[i0] == 0 && outside[i0] == 0) { outside[i0] = 1; queue[qTail++] = i0; }
                }
        while (qHead < qTail)
        {
            int i0 = queue[qHead++];
            int qz = i0 / planeStride, qr = i0 - qz * planeStride, qy = qr / sx, qx = qr - qy * sx;
            if (qx > 0) Visit(i0 - 1); if (qx < sx - 1) Visit(i0 + 1);
            if (qy > 0) Visit(i0 - sx); if (qy < sy - 1) Visit(i0 + sx);
            if (qz > 0) Visit(i0 - planeStride); if (qz < sz - 1) Visit(i0 + planeStride);
        }
        void Visit(int n) { if (cdGrid[n] == 0 && outside[n] == 0) { outside[n] = 1; queue[qTail++] = n; } }

        // SINGLE sparse pass over occupied bits only (64 empty voxels skipped per CPU instruction!)
        ulong[] bits = p.Bits;
        for (int wIdx = 0; wIdx < bits.Length; wIdx++)
        {
            ulong word = bits[wIdx];
            while (word != 0UL)
            {
                int tz = System.Numerics.BitOperations.TrailingZeroCount(word);
                word &= word - 1UL;
                int idx = (wIdx << 6) + tz;
                if (idx >= totalVoxels) break;

                int vz = idx / planeStride;
                int rem = idx - vz * planeStride;
                int vy = rem / sx;
                int vx = rem - vy * sx;

                float cd = cdGrid[idx] * 0.01f;
                Vector3 pos = new(originM.X + vx * voxelSize, originM.Y + vy * voxelSize, originM.Z + vz * voxelSize);
                Vector3 rel = pos - boxCenter;

                // +X face
                if (vx == sx - 1 || outside[idx + 1] != 0)
                {
                    float w = vx == maxX[vz * sy + vy] ? 1.0f : 0.25f;
                    float a = faceArea * w;
                    float cda = a * cd;
                    p.FaceArea[0] += a; p.FaceCdArea[0] += cda; copSum[0] += pos * cda;
                    rotDamp.Y += cda * rel.Z * rel.Z; rotDamp.Z += cda * rel.Y * rel.Y;
                }
                // -X face
                if (vx == 0 || outside[idx - 1] != 0)
                {
                    float w = vx == minX[vz * sy + vy] ? 1.0f : 0.25f;
                    float a = faceArea * w;
                    float cda = a * cd;
                    p.FaceArea[1] += a; p.FaceCdArea[1] += cda; copSum[1] += pos * cda;
                    rotDamp.Y += cda * rel.Z * rel.Z; rotDamp.Z += cda * rel.Y * rel.Y;
                }
                // +Y face
                if (vy == sy - 1 || outside[idx + sx] != 0)
                {
                    float w = vy == maxY[vz * sx + vx] ? 1.0f : 0.25f;
                    float a = faceArea * w;
                    float cda = a * cd;
                    p.FaceArea[2] += a; p.FaceCdArea[2] += cda; copSum[2] += pos * cda;
                    rotDamp.X += cda * rel.Z * rel.Z; rotDamp.Z += cda * rel.X * rel.X;
                }
                // -Y face
                if (vy == 0 || outside[idx - sx] != 0)
                {
                    float w = vy == minY[vz * sx + vx] ? 1.0f : 0.25f;
                    float a = faceArea * w;
                    float cda = a * cd;
                    p.FaceArea[3] += a; p.FaceCdArea[3] += cda; copSum[3] += pos * cda;
                    rotDamp.X += cda * rel.Z * rel.Z; rotDamp.Z += cda * rel.X * rel.X;
                }
                // +Z face
                if (vz == sz - 1 || outside[idx + planeStride] != 0)
                {
                    float w = vz == maxZ[vy * sx + vx] ? 1.0f : 0.25f;
                    float a = faceArea * w;
                    float cda = a * cd;
                    p.FaceArea[4] += a; p.FaceCdArea[4] += cda; copSum[4] += pos * cda;
                    rotDamp.X += cda * rel.Y * rel.Y; rotDamp.Y += cda * rel.X * rel.X;
                }
                // -Z face
                if (vz == 0 || outside[idx - planeStride] != 0)
                {
                    float w = vz == minZ[vy * sx + vx] ? 1.0f : 0.25f;
                    float a = faceArea * w;
                    float cda = a * cd;
                    p.FaceArea[5] += a; p.FaceCdArea[5] += cda; copSum[5] += pos * cda;
                    rotDamp.X += cda * rel.Y * rel.Y; rotDamp.Y += cda * rel.X * rel.X;
                }
            }
        }

        for (int f = 0; f < 6; f++)
            p.FaceCoP[f] = p.FaceCdArea[f] > 1e-4f ? copSum[f] / p.FaceCdArea[f] : boxCenter;

        // Lifting-body / wing planform area per normal axis:
        // Blunt cubes (aspect <= 1.5 and sleekness < 0.2) get ZERO airfoil lift!
        // Thin winged shapes (aspect > 1.5) or sloped aerodynamic fuselages earn real lift smoothly.
        float areaX = 0.5f * (p.FaceArea[0] + p.FaceArea[1]);
        float areaY = 0.5f * (p.FaceArea[2] + p.FaceArea[3]);
        float areaZ = 0.5f * (p.FaceArea[4] + p.FaceArea[5]);
        Vector3 sizeM = p.LocalBounds.Size;
        float sleekBonus = Math.Clamp((float)sleekVoxels / Math.Max(solidVoxels, 1), 0f, 1f);
        p.Sleekness = sleekBonus;

        float flatX = LiftEfficiency(sizeM.X, MathF.Sqrt(Math.Max(sizeM.Y * sizeM.Z, 0.25f)), sleekBonus);
        float flatY = LiftEfficiency(sizeM.Y, MathF.Sqrt(Math.Max(sizeM.X * sizeM.Z, 0.25f)), sleekBonus);
        float flatZ = LiftEfficiency(sizeM.Z, MathF.Sqrt(Math.Max(sizeM.X * sizeM.Y, 0.25f)), sleekBonus);
        p.LiftArea = new Vector3(areaX * flatX, areaY * flatY, areaZ * flatZ);

        p.RotDamping = rotDamp * 0.5f;
        p.CharRadius = Math.Max(0.5f, sizeM.Length() * 0.5f);
        return p;
    }

    private static float LiftEfficiency(float thickness, float chordSpan, float sleekBonus)
    {
        float aspect = chordSpan / Math.Max(thickness, 0.25f);
        // Blunt cube/brick (aspect <= 1.5, sleekBonus <= 0.2): 0.0 airfoil lift!
        // Wings & flat sleek fuselages (aspect 1.6 .. 5+): smoothly ramps up to 0.90.
        float aspectLift = Math.Clamp((aspect - 1.5f) * 0.30f, 0f, 0.80f);
        float sleekLift = aspect > 1.25f ? Math.Clamp((sleekBonus - 0.20f) * 0.45f, 0f, 0.25f) : 0f;
        return Math.Clamp(aspectLift + sleekLift, 0f, 0.90f);
    }

    private static BlockAeroKind ClassifyBlock(CubeBlockDefinition? def, float perm)
    {
        if (def == null) return BlockAeroKind.Blunt;
        if (perm >= 0.35f) return BlockAeroKind.Permeable;
        return KindCache.GetOrAdd(def.Guid, _ => ClassifyByTokens(def.DebugName, def.Permeability));
    }

    /// <summary>
    /// Tokenizes CamelCase / underscore / digit-delimited block definition names into exact words so e.g.
    /// "Refinery" never false-matches "fin" and functional blocks stay Blunt.
    /// </summary>
    private static BlockAeroKind ClassifyByTokens(string? rawName, bool permeability)
    {
        if (string.IsNullOrEmpty(rawName))
            return permeability ? BlockAeroKind.Permeable : BlockAeroKind.Blunt;

        bool isFunctional = false;
        bool hasSleek = false;
        bool hasSlope = false;
        bool hasLong = false;
        bool hasThin = false;

        int i = 0;
        int len = rawName.Length;
        while (i < len)
        {
            while (i < len && !char.IsLetter(rawName[i])) i++;
            if (i >= len) break;
            int start = i;
            i++;
            while (i < len && char.IsLetter(rawName[i]) && !char.IsUpper(rawName[i])) i++;
            string tok = rawName.Substring(start, i - start).ToLowerInvariant();

            switch (tok)
            {
                case "refinery":
                case "assembler":
                case "reactor":
                case "generator":
                case "battery":
                case "cargo":
                case "container":
                case "tank":
                case "thruster":
                case "gyro":
                case "gyroscope":
                case "cockpit":
                case "turret":
                case "connector":
                case "drill":
                case "welder":
                case "grinder":
                case "beacon":
                case "antenna":
                case "cryo":
                case "medical":
                case "survival":
                case "crate":
                case "warhead":
                    isFunctional = true;
                    break;

                case "long":
                    hasLong = true;
                    break;

                case "tip":
                case "nose":
                case "cone":
                case "wedge":
                case "wing":
                case "fin":
                case "aero":
                case "aerodynamic":
                    hasSleek = true;
                    break;

                case "slope":
                case "sloped":
                case "ramp":
                case "corner":
                case "round":
                case "rounded":
                case "bevel":
                case "half":
                case "angled":
                case "transition":
                    hasSlope = true;
                    break;

                case "panel":
                case "plate":
                case "catwalk":
                case "cover":
                    hasThin = true;
                    break;
            }
        }

        if (isFunctional) return BlockAeroKind.Blunt;
        if (hasSleek || (hasLong && hasSlope)) return BlockAeroKind.Sleek;
        if (hasSlope) return BlockAeroKind.Sloped;
        if (hasThin) return BlockAeroKind.ThinPlate;
        return permeability ? BlockAeroKind.Permeable : BlockAeroKind.Blunt;
    }

    internal static float ComputeSubgridExposure(
        DEntity selfId,
        in WorldTransform selfWt,
        in BoundingBox selfLocalBounds,
        in Vector3 airflowDirWorld,
        ShipAssembly.GroupInfo group)
    {
        Vector3 c = selfLocalBounds.Center;
        Vector3 he = selfLocalBounds.HalfExtents * 0.75f;
        Span<Vector3D> ptsWorld = stackalloc Vector3D[7];
        ptsWorld[0] = WorldTransform.Transform((Vector3D)c, in selfWt);
        ptsWorld[1] = WorldTransform.Transform((Vector3D)(c + new Vector3(he.X, 0, 0)), in selfWt);
        ptsWorld[2] = WorldTransform.Transform((Vector3D)(c - new Vector3(he.X, 0, 0)), in selfWt);
        ptsWorld[3] = WorldTransform.Transform((Vector3D)(c + new Vector3(0, he.Y, 0)), in selfWt);
        ptsWorld[4] = WorldTransform.Transform((Vector3D)(c - new Vector3(0, he.Y, 0)), in selfWt);
        ptsWorld[5] = WorldTransform.Transform((Vector3D)(c + new Vector3(0, 0, he.Z)), in selfWt);
        ptsWorld[6] = WorldTransform.Transform((Vector3D)(c - new Vector3(0, 0, he.Z)), in selfWt);

        Span<float> pointExp = stackalloc float[7];
        for (int k = 0; k < 7; k++) pointExp[k] = 1f;

        ShipAssembly.Member[] members = group.Members;
        for (int m = 0; m < members.Length; m++)
        {
            ref var other = ref members[m];
            if (other.DEntity == selfId) continue;
            GridAeroProfile? hull = GetOrBuildProfile(other.Grid);
            if (hull == null || hull.BlockCount < 4) continue;

            Vector3 upwindLocal = WorldTransform.TransformDirectionInv(airflowDirWorld, other.Transform);
            float uLen = upwindLocal.Length();
            if (uLen < 1e-4f) continue;
            upwindLocal /= uLen;

            BoundingBox inflated = hull.LocalBounds;
            inflated.Inflate(hull.CharRadius + 4f);

            for (int k = 0; k < 7; k++)
            {
                if (pointExp[k] <= 0f) continue;
                Vector3 pLocal = (Vector3)WorldTransform.TransformInv(ptsWorld[k], other.Transform);
                if (!inflated.Contains(pLocal)) continue;

                if (hull.LocalBounds.Contains(pLocal))
                {
                    int blockedDirs = CountBlockedCardinals(hull, in pLocal);
                    if (blockedDirs >= 5)
                    {
                        pointExp[k] = 0f;
                        continue;
                    }
                    if (blockedDirs == 4)
                    {
                        pointExp[k] = Math.Min(pointExp[k], 0.35f);
                    }
                }

                if (RayHitsHull(hull, in pLocal, in upwindLocal))
                {
                    pointExp[k] = Math.Min(pointExp[k], 0.08f);
                }
            }
        }

        float sum = 0f;
        for (int k = 0; k < 7; k++) sum += pointExp[k];
        float exp = sum / 7f;
        if (exp < 0.05f) exp = 0f;
        if (exp < 0.9f) Interlocked.Increment(ref SubgridsOccluded);
        return exp;
    }

    private static int CountBlockedCardinals(GridAeroProfile hull, in Vector3 pLocal)
    {
        if (!hull.LocalToVoxel(in pLocal, out int vx, out int vy, out int vz)) return 0;
        int blocked = 0;
        for (int x = vx + 1; x < hull.SizeX; x++) if (hull.IsVoxelSolid(x, vy, vz)) { blocked++; break; }
        for (int x = vx - 1; x >= 0; x--) if (hull.IsVoxelSolid(x, vy, vz)) { blocked++; break; }
        for (int y = vy + 1; y < hull.SizeY; y++) if (hull.IsVoxelSolid(vx, y, vz)) { blocked++; break; }
        for (int y = vy - 1; y >= 0; y--) if (hull.IsVoxelSolid(vx, y, vz)) { blocked++; break; }
        for (int z = vz + 1; z < hull.SizeZ; z++) if (hull.IsVoxelSolid(vx, vy, z)) { blocked++; break; }
        for (int z = vz - 1; z >= 0; z--) if (hull.IsVoxelSolid(vx, vy, z)) { blocked++; break; }
        return blocked;
    }

    private static bool RayHitsHull(GridAeroProfile hull, in Vector3 startLocal, in Vector3 dirUnit)
    {
        float voxelSize = CellSize * hull.Step;
        float stepDist = voxelSize * 0.85f;
        Vector3 minB = hull.LocalBounds.Min;
        Vector3 maxB = hull.LocalBounds.Max;

        float tMin = 0f, tMax = hull.CharRadius * 2.2f;
        if (!ClipSlab(startLocal.X, dirUnit.X, minB.X, maxB.X, ref tMin, ref tMax)) return false;
        if (!ClipSlab(startLocal.Y, dirUnit.Y, minB.Y, maxB.Y, ref tMin, ref tMax)) return false;
        if (!ClipSlab(startLocal.Z, dirUnit.Z, minB.Z, maxB.Z, ref tMin, ref tMax)) return false;
        if (tMax < tMin) return false;

        float t = Math.Max(tMin, voxelSize * 0.5f);
        for (int s = 0; s < 40 && t <= tMax; s++, t += stepDist)
        {
            Vector3 q = startLocal + dirUnit * t;
            if (hull.LocalToVoxel(in q, out int vx, out int vy, out int vz) && hull.IsVoxelSolid(vx, vy, vz))
                return true;
        }
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ClipSlab(float o, float d, float min, float max, ref float tMin, ref float tMax)
    {
        if (MathF.Abs(d) < 1e-6f) return o >= min && o <= max;
        float invD = 1f / d;
        float t1 = (min - o) * invD;
        float t2 = (max - o) * invD;
        if (t1 > t2) (t1, t2) = (t2, t1);
        if (t1 > tMin) tMin = t1;
        if (t2 < tMax) tMax = t2;
        return tMin <= tMax;
    }

    internal static void Evaluate(
        DEntity entity,
        CubeGridComponent? grid,
        ShipAssembly.GroupInfo? group,
        in WorldTransform wt,
        in RigidBodyMassProperties mass,
        in RigidBodyData rb,
        Optional<BoundingBoxData> bounds,
        float airDensity,
        float dt,
        out Vector3 dvWorld,
        out Vector3 dwLocal,
        out float dragAccelOut)
    {
        dvWorld = Vector3.Zero;
        dwLocal = Vector3.Zero;
        dragAccelOut = 0f;

        // Guard against degenerate/dying crash fragments (InvMass > 1 => mass < 1 kg, or non-finite values)
        if (airDensity <= 1e-5f || !float.IsFinite(airDensity) || !float.IsFinite(mass.InvMass) || mass.InvMass <= 0f || mass.InvMass > 1f) return;

        Vector3 vWorld = rb.LinearVelocity;
        Vector3 wWorld = rb.AngularVelocity;
        if (!vWorld.IsValid() || !wWorld.IsValid()) return;
        float speedSq = vWorld.LengthSquared();
        float wSq = wWorld.LengthSquared();
        if (!float.IsFinite(speedSq) || !float.IsFinite(wSq) || (speedSq < 1e-3f && wSq < 1e-4f)) return;

        long t0 = Stopwatch.GetTimestamp();
        long tProf = 0, tExp = 0;
        try
        {
            Interlocked.Increment(ref EvalBodies);

            long tp0 = Stopwatch.GetTimestamp();
            GridAeroProfile? profile = GetOrBuildProfile(grid);
            tProf = Stopwatch.GetTimestamp() - tp0;
            bool isSubgrid = ShipAssembly.IsConstrainedSubgrid(entity, group);

            float exposure = 1f;
            if (group != null && !group.IsSolo && group.Members.Length > 1 && speedSq > 1e-3f)
            {
                long te0 = Stopwatch.GetTimestamp();
                BoundingBox localBox = profile != null ? profile.LocalBounds
                    : (bounds.HasValue ? bounds.Value.BoundingBox : new BoundingBox(-Vector3.Half, Vector3.Half));
                Vector3 airflowDir = vWorld / MathF.Sqrt(speedSq);
                exposure = ComputeSubgridExposure(entity, in wt, in localBox, in airflowDir, group);
                tExp = Stopwatch.GetTimestamp() - te0;
                if (exposure <= 0f) return;
            }

            Vector3 wLocal = WorldTransform.TransformDirectionInv(wWorld, wt);
            float speed = MathF.Sqrt(speedSq);
            float safeSpeed = MathF.Min(speed, 10000f);

            if (profile == null)
            {
                if (safeSpeed > 0.05f)
                {
                    Vector3 dir = vWorld / speed;
                    float area = bounds.HasValue
                        ? Math.Clamp(Keen.Game2.Simulation.GameSystems.RangedAffectGenerators.Wind.WindGeneratorComponent
                            .ComputeProjectedAreaLocal(bounds.Value.BoundingBox.Size, WorldTransform.TransformDirectionInv(dir, wt)), 0.05f, 10000f)
                        : 1f;
                    float accel = 0.5f * airDensity * (float)Settings.RealDragCd * area * safeSpeed * safeSpeed * mass.InvMass * exposure;
                    if (float.IsFinite(accel))
                    {
                        dragAccelOut = MathF.Min(accel, 50000f);
                        dvWorld = -dir * Math.Min(accel * dt, 0.4f * safeSpeed);
                    }
                }
                return;
            }

            Vector3 fLocalTotal = Vector3.Zero;
            Vector3 tauLocalTotal = Vector3.Zero;
            Vector3 comLocal = isSubgrid ? mass.CenterOfMass : ShipAssembly.GetLocalCoMFromGroup(group, in wt, in mass);

            if (safeSpeed > 0.05f)
            {
                Vector3 vLocal = WorldTransform.TransformDirectionInv(vWorld, wt);
                Vector3 u = vLocal / speed;
                float q = 0.5f * airDensity * safeSpeed * safeSpeed * exposure;

                float mach = safeSpeed / 340f;
                float machMult = 1f;
                if (mach > 0.75f)
                {
                    float peak = 0.45f * (1f - 0.55f * profile.Sleekness);
                    float dm = (mach - 1.05f) / 0.25f;
                    float waveBump = peak * MathF.Exp(-dm * dm);
                    float superBase = mach > 1.05f ? 0.15f * (1f - 0.4f * profile.Sleekness) * Math.Min((mach - 1.05f) / 0.5f, 1f) : 0f;
                    machMult = 1f + waveBump + superBase;
                }

                int fx = u.X >= 0f ? 0 : 1;
                int fy = u.Y >= 0f ? 2 : 3;
                int fz = u.Z >= 0f ? 4 : 5;
                float wx = MathF.Abs(u.X), wy = MathF.Abs(u.Y), wz = MathF.Abs(u.Z);

                float cdaX = wx * profile.FaceCdArea[fx];
                float cdaY = wy * profile.FaceCdArea[fy];
                float cdaZ = wz * profile.FaceCdArea[fz];
                float cdaSum = cdaX + cdaY + cdaZ;
                float cdaEff = cdaSum * (float)Settings.RealDragCd;

                Vector3 cop = cdaSum > 1e-4f
                    ? (profile.FaceCoP[fx] * cdaX + profile.FaceCoP[fy] * cdaY + profile.FaceCoP[fz] * cdaZ) / cdaSum
                    : comLocal;

                Vector3 fDragLocal = -u * (q * machMult * cdaEff);
                fLocalTotal += fDragLocal;

                if (Settings.RealAeroLift)
                {
                    Vector3 fLiftLocal = ComputeAxisLift(in u, new Vector3(1, 0, 0), profile.LiftArea.X, q)
                                       + ComputeAxisLift(in u, new Vector3(0, 1, 0), profile.LiftArea.Y, q)
                                       + ComputeAxisLift(in u, new Vector3(0, 0, 1), profile.LiftArea.Z, q);
                    fLocalTotal += fLiftLocal;
                    float liftAccel = fLiftLocal.Length() * mass.InvMass;
                    if (float.IsFinite(liftAccel)) Max(ref LiftMaxMilli, MathF.Min(liftAccel, 50000f));
                }

                if (Settings.RealAeroTorque)
                {
                    Vector3 lever = cop - comLocal;
                    float torqueScale = isSubgrid ? 0.25f : 0.75f;
                    tauLocalTotal += Vector3.Cross(lever, fLocalTotal) * torqueScale;
                }

                float dAcc = fDragLocal.Length() * mass.InvMass;
                if (float.IsFinite(dAcc)) dragAccelOut = MathF.Min(dAcc, 50000f);
            }

            if (wSq > 1e-4f)
            {
                float wMag = MathF.Min(MathF.Sqrt(wSq), 100f);
                float refSpeed = safeSpeed + wMag * profile.CharRadius;
                float dampPref = 0.5f * airDensity * refSpeed * exposure;
                tauLocalTotal -= new Vector3(
                    profile.RotDamping.X * wLocal.X,
                    profile.RotDamping.Y * wLocal.Y,
                    profile.RotDamping.Z * wLocal.Z) * dampPref;
            }

            if (!Vector3.IsZero(fLocalTotal))
            {
                Vector3 fWorld = WorldTransform.TransformDirection(fLocalTotal, wt);
                Vector3 dv = fWorld * (mass.InvMass * dt);
                float maxDv = isSubgrid ? Math.Min(0.25f * safeSpeed, 12f * dt) : 0.45f * safeSpeed;
                float dvLen = dv.Length();
                if (dvLen > maxDv && dvLen > 1e-5f && float.IsFinite(dvLen))
                    dv *= maxDv / dvLen;
                if (dv.IsValid()) dvWorld = dv;
            }

            if (!Vector3.IsZero(tauLocalTotal))
            {
                Vector3 angImpLocal = tauLocalTotal * dt;
                RigidBodyData rbCopy = rb;
                Vector3 dw = rbCopy.ComputeAngularImpulseLocal(in mass, in angImpLocal);
                float maxDw = isSubgrid ? 0.25f : Math.Max(0.5f, wLocal.Length() * 0.45f + 0.25f);
                float dwLen = dw.Length();
                if (dwLen > maxDw && dwLen > 1e-5f && float.IsFinite(dwLen))
                    dw *= maxDw / dwLen;
                if (dw.IsValid()) dwLocal = dw;
            }
        }
        finally
        {
            long elapsed = Stopwatch.GetTimestamp() - t0;
            Interlocked.Add(ref EvalStopwatchTicks, elapsed);
            double ms = elapsed * 1000.0 / Stopwatch.Frequency;
            if (ms > 5.0 && Interlocked.Increment(ref _slowLogs) <= 15)
            {
                double profMs = tProf * 1000.0 / Stopwatch.Frequency;
                double expMs = tExp * 1000.0 / Stopwatch.Frequency;
                ProbeLog.Write($"aero SLOW: entity={entity} total={ms:0.1}ms prof={profMs:0.1}ms exp={expMs:0.1}ms blocks={grid?._cubeBlockStorage?.CubeBlockCount ?? 0} v={vWorld.Length():0.0} invM={mass.InvMass:0.###E+0}");
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3 ComputeAxisLift(in Vector3 travelDir, in Vector3 normalAxis, float liftArea, float q)
    {
        if (liftArea <= 1e-3f) return Vector3.Zero;
        float sinA = -Vector3.Dot(travelDir, normalAxis);
        float absSin = MathF.Abs(sinA);
        if (absSin < 1e-3f || absSin > 0.995f) return Vector3.Zero;

        float cosA = MathF.Sqrt(Math.Max(0f, 1f - sinA * sinA));
        float sin2A = 2f * sinA * cosA;
        float stallBlend = Math.Clamp((absSin - 0.40f) / 0.22f, 0f, 1f);
        float clScale = 1.65f * (1f - stallBlend) + 0.65f * stallBlend;
        float cl = clScale * sin2A;

        Vector3 liftDir = normalAxis - travelDir * Vector3.Dot(normalAxis, travelDir);
        float ldLen = liftDir.Length();
        if (ldLen < 1e-4f) return Vector3.Zero;
        liftDir /= ldLen;

        return liftDir * (q * cl * liftArea);
    }

    private static void Max(ref long slot, double value)
    {
        if (!double.IsFinite(value) || value <= 0.0) return;
        long v = (long)Math.Min(value * 1000.0, 1e12);
        long cur;
        while (v > (cur = Interlocked.Read(ref slot)) && Interlocked.CompareExchange(ref slot, v, cur) != cur) { }
    }
}
