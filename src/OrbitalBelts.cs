using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.Core.Game.GameSystems.ProceduralGeneration;
using Keen.VRage.Core.Game.Systems;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.ObjectBuilders;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Voxels.Components;

namespace SpinProbe;

/// <summary>
/// Stage 4.2 of docs/asteroid-orbits-plan.md: deterministic orbital asteroid belts.
/// <para>Why not the vanilla generator: ProceduralGeneratorSessionComponent ties every procedural rock to a fixed sector
/// cube; an orbiting rock leaves its sector, so the sector would regenerate it at its seed position (duplicates) and
/// neighbours would miss it. Instead each belt is a catalog of orbital records generated from the save name: Keplerian
/// elements about a parent body (home planet, or the star of stage 4.1), seed, size and spin - ~100 bytes each, no entity.</para>
/// <para>Proximity spawner: a record within belt_spawn_range of a player becomes a real asteroid entity at its analytic
/// pose and velocity (a clone of a template procedural asteroid's object builder with the record's seed / size, a
/// deterministic SpaceEntityId and no stored voxels, so ProceduralVoxelStorageComponent.CreateStorageInternal generates it
/// from the seed, ProceduralVoxelStorageComponent.cs:61-85). It is then an ordinary movable asteroid: AsteroidRails puts it
/// on rails until something comes near. An untouched catalog rock beyond belt_despawn_range is closed again - nothing about
/// it needs saving, it is recomputed from its seed and the orbit clock. A rock a player touched (or that shattered) is
/// recorded in the delta store (AsteroidStore) and never respawned from the catalog: the touched one is a saved entity,
/// the shattered one's shards are.</para>
/// <para>Spawned rocks are manual (MarkedAsManual) because only serializable voxel entities replicate to clients
/// (VoxelStorageComponentBase.IsEntitySerializableVoxel :862-866); clients cannot generate them on their own.</para>
/// </summary>
internal static class OrbitalBelts
{
    internal static bool On => Settings.OrbitalBelts && AsteroidRails.On;
    private const int IdSectorX = 2_000_000_000;

    internal sealed class Belt
    {
        internal string Name = "";
        internal Orbits.Body Parent = null!;
        internal Vector3D Up;
        internal double Inner, Outer, HalfThickness;
        internal readonly List<Record> Records = new();
    }

    internal sealed class Record
    {
        internal int Id;
        internal Belt Belt = null!;
        internal double A, E, Inc, Node, Argp, M0;
        internal int Seed;
        internal float Size;
        internal Vector3 SpinAxis;
        internal float SpinRate;
        internal Quaternion Orientation;
    }

    internal static readonly List<Belt> Belts = new();
    private static readonly Dictionary<int, Record> ById = new();
    private static bool _built;
    private static readonly Dictionary<int, VoxelPhysicsComponent> Live = new();
    private static readonly object Gate = new();
    private static EntityObjectBuilder? _template;
    private static VoxelPhysicsComponent? _templateFrom;
    private static int Spawned, Despawned, Suppressed;
    private static readonly List<VoxelPhysicsComponent> SuppressQueue = new();

    internal static string Stats()
    {
        if (!On) return "off";
        int live; lock (Gate) live = Live.Count;
        return $"{Belts.Count} belt(s) {ById.Count} records, {live} live, spawned {Spawned}, despawned {Despawned}, vanilla suppressed {Suppressed}" +
               (_template == null ? ", NO TEMPLATE yet" : "");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Catalog
    // ---------------------------------------------------------------------------------------------------------------

    // Source: none (plugin-only). Deterministic from the save name so every session (and every client) agrees.
    private static void Build()
    {
        if (_built) return;
        if (Orbits.StarOn && Orbits.Star == null) return;
        List<Orbits.Body> bodies = Orbits.Bodies;
        Bubbles.Bubble? homeBubble = Orbits.Home;
        Orbits.Body? home = homeBubble == null ? null : bodies.Find(b => b.Frame == homeBubble);
        if (home == null || home.Mu <= 0) return;
        _built = true;
        int seed = StableHash(Settings.SaveName);
        int nextId = 0;

        // Home-planet belt(s): keep clear of the home bubble and of any orbiting moon's swept bubble band.
        // Verdure (R=60 km, bubble=96 km) has a moon at 196 km (bubble=32 km -> swept 164-228 km). If a moon cuts
        // through [inner, outer], split cleanly into an inner ring (outside home bubble, inside moon) and an outer belt
        // (beyond moon up to max(outer, 6.5 R)), so the moon never collapses the home belt to zero width.
        double bubbleEdge = (home.Frame?.Radius ?? home.Radius * 1.6);
        double inner = Math.Max(home.Radius * Settings.BeltInner, bubbleEdge * 1.15);
        double outer = Math.Min(Math.Max(home.Radius * Settings.BeltOuter, home.Radius * 6.0), home.Soi * 0.85);
        double moonLow = double.MaxValue, moonHigh = double.MinValue;
        foreach (Orbits.Body m in bodies)
        {
            if (m.Parent != home || m.Frame == null) continue;
            double dm = (m.C0 - home.C0).Length(), clear = m.Frame.Radius * 1.35;
            moonLow = Math.Min(moonLow, dm - clear);
            moonHigh = Math.Max(moonHigh, dm + clear);
        }
        if (Settings.BeltHomeCount > 0)
        {
            bool moonSplits = moonLow < outer && moonHigh > inner;
            if (moonSplits)
            {
                double innerStart = Math.Max(bubbleEdge + 12000, home.Radius * 1.85);
                double innerEnd = moonLow;
                double outerStart = Math.Max(inner, moonHigh);
                double outerEnd = outer;
                bool hasInner = innerEnd > innerStart + home.Radius * 0.3;
                bool hasOuter = outerEnd > outerStart + home.Radius * 0.3;
                int innerCount = hasInner ? (hasOuter ? Math.Max(40, Settings.BeltHomeCount / 3) : Settings.BeltHomeCount) : 0;
                int outerCount = hasOuter ? Settings.BeltHomeCount - innerCount : 0;
                if (innerCount > 0)
                    AddBelt($"inner belt about {home.Name}", home, homeBubble!.SpinAxis, innerStart, innerEnd, innerCount, seed, ref nextId);
                if (outerCount > 0)
                    AddBelt($"outer belt about {home.Name}", home, homeBubble!.SpinAxis, outerStart, outerEnd, outerCount, seed ^ 0x27d4eb2d, ref nextId);
            }
            else if (outer > inner + home.Radius * 0.3)
            {
                AddBelt($"belt about {home.Name}", home, homeBubble!.SpinAxis, inner, outer, Settings.BeltHomeCount, seed, ref nextId);
            }
        }

        Orbits.Body? star = bodies.Find(b => b.Radius == 0);
        if (star != null && Settings.BeltStarCount > 0)
        {
            double d = (home.C0 - star.C0).Length();
            AddBelt("heliocentric belt", star, Orbits.Star!.OrbitAxis, d * 0.55, d * 0.75, Settings.BeltStarCount, seed ^ 0x5bd1e995, ref nextId);
        }
        foreach (Belt b in Belts)
            ProbeLog.Write($"belts: {b.Name}: {b.Records.Count} asteroids, r {b.Inner / 1000:0}-{b.Outer / 1000:0} km about {b.Parent.Name}, " +
                           $"circular speed {Math.Sqrt(b.Parent.Mu / b.Inner):0}-{Math.Sqrt(b.Parent.Mu / b.Outer):0} m/s");
    }

    private static void AddBelt(string name, Orbits.Body parent, Vector3D up, double inner, double outer, int count, int seed, ref int nextId)
    {
        var belt = new Belt { Name = name, Parent = parent, Up = Vector3D.Normalize(up), Inner = inner, Outer = outer };
        var rng = new Random(seed);
        double maxInc = 0;
        for (int i = 0; i < count; i++)
        {
            double u = rng.NextDouble();
            var r = new Record
            {
                Id = nextId++,
                Belt = belt,
                A = inner + (outer - inner) * u,
                E = rng.NextDouble() * 0.04,
                Inc = (rng.NextDouble() + rng.NextDouble() - 1.0) * 1.5 * Math.PI / 180,
                Node = rng.NextDouble() * 2 * Math.PI,
                Argp = rng.NextDouble() * 2 * Math.PI,
                M0 = rng.NextDouble() * 2 * Math.PI,
                Seed = rng.Next(),
                Size = rng.NextDouble() < 0.75 ? 64f : 128f,
                SpinAxis = Vector3.Normalize(new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f) + new Vector3(1e-3f)),
                SpinRate = (float)(0.005 + rng.NextDouble() * 0.045),
            };
            r.Orientation = Quaternion.CreateFromYawPitchRoll((float)(rng.NextDouble() * 6.283), (float)(rng.NextDouble() * 6.283), (float)(rng.NextDouble() * 6.283));
            maxInc = Math.Max(maxInc, Math.Abs(r.Inc));
            belt.Records.Add(r);
            ById[r.Id] = r;
        }
        belt.HalfThickness = outer * Math.Sin(maxInc) + 2000;
        Belts.Add(belt);
    }

    private static int StableHash(string s)
    {
        unchecked { int h = 23; foreach (char c in s) h = h * 31 + c; return h; }
    }

    /// <summary>Global position / velocity of a record at time t (Kepler elements about its parent's live centre).</summary>
    internal static void State(Record r, double t, out Vector3D pos, out Vector3D vel)
    {
        Orbits.Body p = r.Belt.Parent;
        double n = Math.Sqrt(p.Mu / (r.A * r.A * r.A));
        double M = (r.M0 + n * t) % (2 * Math.PI);
        Orbits.FromElements(p.Mu, r.A, r.E, r.Inc, r.Node, r.Argp, M, r.Belt.Up, out Vector3D rr, out Vector3D vv);
        pos = Orbits.CentreAt(p, t) + rr;
        vel = Orbits.VelocityAt(p, t) + vv;
    }

    internal static SpaceEntityId IdOf(int id) => new(new Vector3I(IdSectorX, 0, 7), id);

    internal static bool TryCatalogId(SpaceEntityId? id, out int index)
    {
        index = -1;
        if (id == null || id.Value.SectorID.X != IdSectorX || id.Value.SectorID.Z != 7) return false;
        index = id.Value.EntityID;
        return true;
    }

    /// <summary>True when p (global) lies inside a belt torus - vanilla static rocks are suppressed there.</summary>
    internal static bool InBeltZone(in Vector3D p)
    {
        if (!_built) return false;
        double t = Orbits.Now;
        foreach (Belt b in Belts)
        {
            Vector3D d = p - Orbits.CentreAt(b.Parent, t);
            double h = Vector3D.Dot(d, b.Up);
            double rho = (d - b.Up * h).Length();
            if (Math.Abs(h) < b.HalfThickness && rho > b.Inner * 0.95 && rho < b.Outer * 1.05) return true;
        }
        return false;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Hooks from MovableAsteroids
    // ---------------------------------------------------------------------------------------------------------------

    private sealed class PendingSpawn
    {
        internal Record Rec = null!;
        internal Vector3D Pos;
        internal Vector3 Lin, Ang;
        internal long Ticks;
    }
    private static readonly List<PendingSpawn> Pending = new();

    /// <summary>MovableAsteroids.InitPatch: is this a catalog rock we just spawned (velocity) or one loaded from a save?</summary>
    internal static bool TryConsumeSpawn(in Vector3D at, out int catalogId, out Vector3 lin, out Vector3 ang)
    {
        catalogId = -1; lin = default; ang = default;
        lock (Pending)
        {
            if (Pending.Count == 0) return false;
            long now = DateTime.UtcNow.Ticks;
            Pending.RemoveAll(x => now - x.Ticks > TimeSpan.TicksPerSecond * 10);
            for (int i = 0; i < Pending.Count; i++)
            {
                if ((Pending[i].Pos - at).LengthSquared() > 1.0) continue;
                catalogId = Pending[i].Rec.Id; lin = Pending[i].Lin; ang = Pending[i].Ang;
                Pending.RemoveAt(i);
                return true;
            }
        }
        return false;
    }

    internal static void Register(int catalogId, VoxelPhysicsComponent physics)
    {
        lock (Gate) Live[catalogId] = physics;
    }

    internal static void Forget(VoxelPhysicsComponent physics)
    {
        lock (Gate)
        {
            foreach (var kv in Live) if (ReferenceEquals(kv.Value, physics)) { Live.Remove(kv.Key); break; }
            if (ReferenceEquals(_templateFrom, physics)) _templateFrom = null;
        }
    }

    internal static void QueueSuppress(VoxelPhysicsComponent physics)
    {
        lock (SuppressQueue) SuppressQueue.Add(physics);
    }

    internal static void OfferTemplate(VoxelPhysicsComponent physics)
    {
        if (_template != null || _templateFrom != null) return;
        string name = physics.Entity?.ToString() ?? "";
        if (!name.Contains("ProceduralAsteroid", StringComparison.OrdinalIgnoreCase)) return;
        _templateFrom = physics;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Spawner / despawner (AsteroidRails scheduler, WorldUpdateBegin)
    // ---------------------------------------------------------------------------------------------------------------

    private const int SpawnCheckTicks = 15;
    private const int MaxSpawnsPerCheck = 3;

    internal static void Tick(Session session, List<(Vector3D p, Vector3D v, bool player)> movers, double t, long tick)
    {
        if (!On) return;
        Build();
        // Capture template BEFORE DrainSuppress deletes any offered vanilla rock.
        if (_template == null && _templateFrom != null) CaptureTemplate();
        DrainSuppress();
        if (!_built || tick % SpawnCheckTicks != 0) return;
        var players = movers.Where(m => m.player).Select(m => m.p).ToList();
        if (players.Count == 0) return;

        // Despawn untouched catalog rocks nobody is near.
        List<(int id, VoxelPhysicsComponent p)> live;
        lock (Gate) live = Live.Select(kv => (kv.Key, kv.Value)).ToList();
        foreach (var (id, phys) in live)
        {
            try
            {
                if (phys.Entity == null) { lock (Gate) Live.Remove(id); continue; }
                if (!MovableAsteroids.States.TryGetValue(phys, out MovableAsteroids.State? s) || s.PlayerTouched) continue;
                Vector3D p = AsteroidRails.TryLivePose(phys, out var wt, out _, out _) ? wt.Position : phys.Data.GetWorldTransform().Position;
                double nearest = players.Min(q => (q - p).Length());
                if (nearest < Settings.BeltDespawnRange) continue;
                lock (Gate) Live.Remove(id);
                session.MarkEntityForClose(phys.Entity, true);
                Interlocked.Increment(ref Despawned);
            }
            catch { }
        }

        // Spawn records that came within range.
        if (_template == null) return;
        var due = new List<(double d, Record r, Vector3D p, Vector3D v)>();
        foreach (Belt b in Belts)
            foreach (Record r in b.Records)
            {
                lock (Gate) if (Live.ContainsKey(r.Id)) continue;
                if (AsteroidStore.CatalogSkipped(r.Id)) continue;
                State(r, t, out Vector3D p, out Vector3D v);
                double nearest = double.MaxValue;
                foreach (Vector3D q in players) nearest = Math.Min(nearest, (q - p).Length());
                if (nearest < Settings.BeltSpawnRange) due.Add((nearest, r, p, v));
            }
        foreach (var (_, r, p, v) in due.OrderBy(x => x.d).Take(MaxSpawnsPerCheck))
            Spawn(session, r, p, v);
    }

    private static void CaptureTemplate()
    {
        VoxelPhysicsComponent? from = _templateFrom;
        if (from?.Entity == null) { _templateFrom = null; return; }
        try
        {
            _template = AsteroidShatter.SerializeTemplate(from.Entity);
            ProbeLog.Write($"belts: template captured from {from.Entity}");
        }
        catch (Exception e) { _templateFrom = null; ProbeLog.Write("belts: template capture failed: " + e.Message); }
    }

    // Reads: AsteroidClipboard.ExecutePaste / SanitizeVoxelOB (clone, transform, manual + id); ProceduralVoxelStorageComponent
    // ObjectBuilder Seed / Size / DataID (:150-165) and Init (:34-41); VoxelStorageComponentBaseObjectBuilder Offloaded /
    // SharedStorage (:188, :195) cleared so the rock is generated from its seed.
    private static void Spawn(Session session, Record r, Vector3D pos, Vector3D vel)
    {
        try
        {
            EntityObjectBuilder clone = _template!.DeepClone();
            clone.OB<WorldTransformComponent, WorldTransformComponentObjectBuilder>().Transform = new WorldTransform(pos, r.Orientation);
            AsteroidClipboard.SanitizeVoxelOB(clone, newProceduralId: false);
            int pIdx = clone.TryGetIndexOf<ProceduralVoxelEntityComponent>();
            if (pIdx < 0) pIdx = clone.TryGetIndexOf<ProceduralEntityComponentBase>();
            if (pIdx >= 0 && clone.ObjectBuilders[pIdx] is ProceduralEntityComponentBaseObjectBuilder pob)
            {
                pob.ID = IdOf(r.Id);
                pob.MarkedAsManual = true;
            }
            int vIdx = clone.TryGetIndexOf<VoxelStorageComponentBase>();
            if (vIdx < 0 || clone.ObjectBuilders[vIdx] is not ProceduralVoxelStorageComponentObjectBuilder vob)
                throw new InvalidOperationException("template has no procedural voxel storage");
            vob.Seed = r.Seed;
            vob.Size = r.Size;
            vob.Offloaded = default;
            vob.SharedStorage = default;
            vob.PreparedProvider = null!;

            Vector3 ang = r.SpinAxis * r.SpinRate;
            lock (Pending) Pending.Add(new PendingSpawn { Rec = r, Pos = pos, Lin = (Vector3)vel, Ang = ang, Ticks = DateTime.UtcNow.Ticks });
            // Pass BoundingBoxData + RigidBodyMassProperties + motion/filter/tags in DEntityBuilder so they are
            // materialized before VoxelStorageComponentBase.OnAddedToScene and HavokWorldHelper.AllocateBody run.
            var mass = MovableAsteroids.PrototypeMass(r.Size, 1f);
            var eb = MovableAsteroids.BuildDynamicSpawnData(session, new Vector3(r.Size * 0.5f), in mass, (Vector3)vel, ang);
            Entity e = session.Get<IEntitySpawner>().SpawnEntity(clone, eb);
            try { e.Data.Set(new Keen.VRage.Physics.Data.RigidBodyData { LinearVelocity = (Vector3)vel, AngularVelocity = ang }); } catch { }
            Interlocked.Increment(ref Spawned);
            if (Spawned <= 10 || Spawned % 50 == 0)
                ProbeLog.Write($"belts: spawned #{r.Id} ({r.Belt.Name}, seed {r.Seed}, size {r.Size}) at {pos.X:0},{pos.Y:0},{pos.Z:0} v={vel.Length():0} m/s");
        }
        catch (Exception ex) { ProbeLog.Write($"belts: spawn #{r.Id} failed: {ex}"); }
    }

    // Reads: VoxelStorageComponentBase.cs:830 TryDeleteManually -> ProceduralVoxelEntityComponent.DeletedExternally (:126-130)
    // tells the sector generator the rock is gone for good.
    private static void DrainSuppress()
    {
        List<VoxelPhysicsComponent> list;
        lock (SuppressQueue) { if (SuppressQueue.Count == 0) return; list = SuppressQueue.ToList(); SuppressQueue.Clear(); }
        foreach (var p in list)
        {
            try
            {
                if (p.Entity == null) continue;
                var storage = p.Entity.TryGet<VoxelStorageComponentBase>();
                if (storage != null && storage.TryDeleteManually()) Interlocked.Increment(ref Suppressed);
            }
            catch { }
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Map (MapRender draws each belt as two rings)
    // ---------------------------------------------------------------------------------------------------------------

    internal static IEnumerable<(Vector3D centre, Vector3D up, double radius)> MapRings(double t)
    {
        // Stage 4.1: the star's (apparent) orbit about the home planet.
        Bubbles.Bubble? star = Orbits.Star;
        if (star?.OrbitAround != null) yield return (Bubbles.Centre(star.OrbitAround, t), star.OrbitAxis, (star.C0 - star.OrbitAround.C0).Length());
        if (!On) yield break;
        foreach (Belt b in Belts)
        {
            Vector3D c = Orbits.CentreAt(b.Parent, t);
            yield return (c, b.Up, b.Inner);
            yield return (c, b.Up, b.Outer);
        }
    }
}
