using System;
using System.Collections.Generic;
using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Render.SessionComponents;

namespace SpinProbe;

/// <summary>
/// Shared orbital mechanics for docs/asteroid-orbits-plan.md stages 3 and 4: the gravitating bodies (optional synthetic star,
/// planets, orbiting moons), their live centres and velocities (Bubbles.Centre / BubbleHandoff.OrbitalVelocity), spheres of
/// influence, and patched-conic coasting states (universal-variable Kepler relative to a possibly moving parent, re-anchored
/// when the state crosses an SOI boundary).
/// <para>Clock: <see cref="Now"/> is Bubbles.T (the same clock that moves every bubble) in bubbles mode, otherwise server ticks.
/// Rails are only created once Bubbles.Ready in bubbles mode, so a conic never mixes the two clocks.</para>
/// </summary>
internal static class Orbits
{
    // ---------------------------------------------------------------------------------------------------------------
    // Clock
    // ---------------------------------------------------------------------------------------------------------------

    internal static bool ClockReady => Settings.Mode != "bubbles" || Bubbles.Ready;
    internal static double Now => Settings.Mode == "bubbles" ? Bubbles.T : Profiler.Ticks / 60.0;

    // ---------------------------------------------------------------------------------------------------------------
    // Star (stage 4.1)
    // ---------------------------------------------------------------------------------------------------------------

    internal static bool StarOn => Settings.StarDistance > 0 && Settings.StarPeriod > 0 && Settings.Mode == "bubbles";
    internal static volatile Bubbles.Bubble? Star;
    internal static volatile Bubbles.Bubble? Home;
    internal static double StarMu;
    /// <summary>The engine's own sun direction (unit, from SunSettings azimuth/elevation), captured every client frame.</summary>
    internal static volatile bool HaveEngineSun;
    internal static Vector3 EngineSun = Vector3.UnitX;

    /// <summary>
    /// Stage 4.1 in the home planet's frame. The procedural world (asteroid sectors, stations, the moon) is laid out around the
    /// home planet at fixed server coordinates, so the home planet cannot move through global space without leaving all of it
    /// behind. Global space is therefore the home planet's NON-ROTATING, freely falling frame: the star appears to orbit the
    /// home planet (a virtual Bubble with OrbitAround = home, Radius 0, never in Bubbles.All, so no renders / handoff / bubble
    /// lookups ever see it), and a free body feels the star's TIDAL acceleration g★(p) - g★(home) (<see cref="StarGravity"/>),
    /// which is exactly heliocentric motion written in that accelerating frame. A conic about the star is Kepler relative to
    /// the star's apparent centre plus its apparent velocity (Orbits.Conic with a moving parent), also exact.
    /// The star sits along the engine's sun direction at t = 0, so the light already points at it; SunPatch turns the light
    /// as the star goes round. Circular: rate 2π/period, μ = 4π²d³/period² (Kepler III).
    /// </summary>
    // Source: none (plugin-only). Called from Bubbles.Configure (client, first frame with a camera).
    internal static void ConfigureStar(Bubbles.Bubble home)
    {
        if (StarOn && Star == null)
        {
            double d = Settings.StarDistance, period = Settings.StarPeriod;
            Vector3D toSun = HaveEngineSun ? (Vector3D)EngineSun : new Vector3D(1, 0, 0);
            toSun = Vector3D.Normalize(toSun);
            Vector3D axis = home.SpinAxis - toSun * Vector3D.Dot(home.SpinAxis, toSun);
            if (axis.LengthSquared() < 1e-6) axis = Vector3D.Cross(toSun, new Vector3D(0, 0, 1));
            axis = Vector3D.Normalize(axis);
            StarMu = 4 * Math.PI * Math.PI * d * d * d / (period * period);
            Star = new Bubbles.Bubble
            {
                Name = "star", C0 = home.C0 + toSun * d, Radius = 0,
                OrbitAround = home, OrbitAxis = axis, OrbitRate = 2 * Math.PI / period,
            };
            ProbeLog.Write($"orbits: STAR at {Star.C0.X:0},{Star.C0.Y:0},{Star.C0.Z:0} ({d / 1000:0} km along the engine sun{(HaveEngineSun ? "" : " (default +X)")}), " +
                           $"mu={StarMu:0.###E+0} m3/s2; year {period / 3600:0.00} h, orbital speed {2 * Math.PI * d / period:0} m/s");
        }
        lock (BodiesGate)
        {
            Home = home;
            _bodiesBuiltTick = -1000;
        }
    }

    /// <summary>Tidal star acceleration for a body in global space (the home planet's freely falling frame); zero without a
    /// star. Inside a bubble the frame itself falls with its planet, so it is not applied there.</summary>
    // Source: none (plugin-only) - a = μ (S - p)/|S - p|³ - μ (S - H)/|S - H|³. Applied directly to velocity (not through
    // Havok gravity), so StarMu is NOT scaled by RealRails.GravityMultiplier — keeping exact cancellation with the star's
    // kinematic home-frame acceleration ω²d = StarMu / d².
    internal static Vector3D StarGravity(in Vector3D p)
    {
        Bubbles.Bubble? s = Star;
        Bubbles.Bubble? h = Home;
        if (s == null || h == null || StarMu <= 0 || !Bubbles.Ready) return Vector3D.Zero;
        Vector3D sc = Bubbles.Centre(s, Bubbles.T);
        return Pull(sc - p) - Pull(sc - h.C0);
    }

    private static Vector3D Pull(in Vector3D d)
    {
        double r2 = d.LengthSquared();
        if (r2 < 1e6) return Vector3D.Zero;
        double r = Math.Sqrt(r2);
        return d * (StarMu / (r2 * r));
    }

    /// <summary>
    /// The light turns with the year: the direction from the home planet to the star at time t is the t = 0 direction rotated
    /// by the orbital angle about the orbit axis (star - C_home(t) = Rot(star - C_home(0), axis, θ)).
    /// </summary>
    // Patches: VRage.Render/Keen/VRage/Render/SessionComponents/SunRenderSessionComponent.cs  UpdateSun (:13-28: copies
    // ISun.GetSunDirection azimuth/elevation (degrees) into RenderSettings.SunlightSettings). Reads: VRage.Library Vector3.cs:1273
    // GetAzimuthAndElevation / :1299 CreateFromAzimuthAndElevation (radians); SunHelper.cs:18-42 for the degree convention.
    [HarmonyPatch(typeof(SunRenderSessionComponent), "UpdateSun")]
    internal static class SunPatch
    {
        [HarmonyPostfix]
        private static void Postfix(SunRenderSessionComponent __instance)
        {
            try
            {
                var settings = __instance._render.GetSettings();
                var sun = settings.SunlightSettings;
                Vector3 l = Vector3.CreateFromAzimuthAndElevation(MathHelper.ToRadians(sun.Azimuth), MathHelper.ToRadians(sun.Elevation));
                EngineSun = l;
                HaveEngineSun = true;
                Bubbles.Bubble? home = Home;
                if (Star == null || home == null || Star.OrbitAround != home || !Bubbles.Ready) return;
                double theta = Star.OrbitRate * Bubbles.T;
                Vector3D r = Rot((Vector3D)l, Star.OrbitAxis, theta);
                var (az, el) = Vector3.GetAzimuthAndElevation((Vector3)r);
                sun.Azimuth = MathHelper.ToDegrees(az);
                sun.Elevation = MathHelper.ToDegrees(el);
                settings.SunlightSettings = sun;
            }
            catch (Exception e) { if (_sunErrors++ < 3) ProbeLog.Write("orbits: sun update failed: " + e.Message); }
        }
    }
    private static int _sunErrors;

    // ---------------------------------------------------------------------------------------------------------------
    // Bodies and spheres of influence
    // ---------------------------------------------------------------------------------------------------------------

    internal sealed class Body
    {
        internal string Name = "";
        internal Vector3D C0;                 // centre at t = 0 (server / local coordinates)
        internal double Radius;               // surface radius (0 = star)
        internal double Mu;                   // m³/s² (planets × GravityMultiplier; star = exact StarMu)
        internal double Range;                // gravity reach (engine generator cut-off); infinity for the star
        internal Bubbles.Bubble? Bubble;      // moving bubble (OrbitAround != null) or null = fixed at C0
        internal Bubbles.Bubble? Frame;       // its own bubble (spin frame) if any
        internal Body? Parent;
        internal double Soi;                  // sphere of influence radius
        internal int Depth;
    }

    private static readonly object BodiesGate = new();
    private static List<Body> _bodies = new();
    private static long _bodiesBuiltTick = -1000;
    private static int _bodiesPlanetCount = -1;
    private static bool _bodiesHadStar;
    private static int _bodiesBubbleCount = -1;

    /// <summary>Star + every registered planet with a known surface gravity; rebuilt when planets/bubbles/star change (cheap).</summary>
    internal static List<Body> Bodies
    {
        get
        {
            var planets = RealPhysics.Planets;
            bool hasStar = Star != null && StarMu > 0;
            int bubbleCount = Bubbles.All.Count;
            lock (BodiesGate)
            {
                if (planets.Length == _bodiesPlanetCount && hasStar == _bodiesHadStar &&
                    bubbleCount == _bodiesBubbleCount && Profiler.Ticks - _bodiesBuiltTick < 600)
                    return _bodies;
                _bodiesPlanetCount = planets.Length;
                _bodiesHadStar = hasStar;
                _bodiesBubbleCount = bubbleCount;
                _bodiesBuiltTick = Profiler.Ticks;
                var list = new List<Body>();
                Body? star = null;
                Bubbles.Bubble? starBubble = Star;
                if (starBubble != null && StarMu > 0)
                {
                    star = new Body { Name = "star", C0 = starBubble.C0, Mu = StarMu, Range = double.PositiveInfinity, Bubble = starBubble, Soi = double.PositiveInfinity };
                    list.Add(star);
                }
                foreach (var (c, r) in planets)
                {
                    if (!RealPhysics.TryGetG0(c, r, out float g0) || g0 <= 0) continue;
                    Bubbles.Bubble? frame = null;
                    foreach (Bubbles.Bubble b in Bubbles.All) if (b != starBubble && (b.C0 - c).LengthSquared() < 4.0) { frame = b; break; }
                    list.Add(new Body
                    {
                        Name = frame?.Name ?? $"planet@{c.X:0},{c.Y:0},{c.Z:0}",
                        C0 = c, Radius = r, Mu = g0 * r * r * RealRails.GravityMultiplier,
                        Range = r * Settings.RealGravityRange,
                        Bubble = frame != null && frame.OrbitAround != null ? frame : null,
                        Frame = frame,
                    });
                }
                // Parents follow the bubble chain (moon -> home -> star). Laplace SOI a (μ/μ_parent)^(2/5), capped by the
                // engine's gravity reach of the body (beyond it the planet does not pull at all in game).
                foreach (Body b in list)
                {
                    if (b.Radius == 0) continue;
                    Bubbles.Bubble? around = b.Frame?.OrbitAround;
                    b.Parent = around == null ? null : around == starBubble ? star : list.Find(x => x.Frame == around);
                    double soi = b.Range;
                    if (b.Parent != null && b.Parent.Mu > 0)
                        soi = Math.Min(soi, (b.C0 - b.Parent.C0).Length() * Math.Pow(b.Mu / b.Parent.Mu, 0.4));
                    b.Soi = soi;
                }
                foreach (Body b in list) { int d = 0; for (Body? p = b.Parent; p != null; p = p.Parent) d++; b.Depth = b.Radius == 0 ? -1 : d; }   // the star yields to any planet SOI
                _bodies = list;
                return list;
            }
        }
    }

    internal static Vector3D CentreAt(Body b, double t) => b.Bubble != null ? Bubbles.Centre(b.Bubble, t) : b.C0;
    internal static Vector3D VelocityAt(Body b, double t) => b.Bubble != null ? BubbleHandoff.OrbitalVelocity(b.Bubble, t) : Vector3D.Zero;

    /// <summary>The deepest body whose sphere of influence contains <paramref name="p"/> (global coordinates); null = none
    /// (free straight-line coasting).</summary>
    internal static Body? Dominant(in Vector3D p, double t)
    {
        Body? best = null;
        foreach (Body b in Bodies)
        {
            if ((p - CentreAt(b, t)).Length() > b.Soi) continue;
            if (best == null || b.Depth > best.Depth) best = b;
        }
        return best;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Patched conics
    // ---------------------------------------------------------------------------------------------------------------

    internal sealed class Conic
    {
        internal Body? Parent;
        internal double T0;
        internal Vector3D R0, V0;     // relative to the parent's centre / velocity at T0 (global axes)
        internal int Handoffs;

        internal void Eval(double t, out Vector3D pos, out Vector3D vel)
        {
            double dt = t - T0;
            Vector3D r, v;
            if (Parent == null || Parent.Mu <= 0) { r = R0 + V0 * dt; v = V0; }
            else RealRails.Kepler(R0, V0, Parent.Mu, dt, out r, out v);
            if (Parent == null) { pos = r; vel = v; return; }
            pos = CentreAt(Parent, t) + r;
            vel = VelocityAt(Parent, t) + v;
        }
    }

    internal static Conic Anchor(in Vector3D pos, in Vector3D vel, double t)
    {
        Body? parent = Dominant(pos, t);
        var c = new Conic { Parent = parent, T0 = t };
        c.R0 = parent == null ? pos : pos - CentreAt(parent, t);
        c.V0 = parent == null ? vel : vel - VelocityAt(parent, t);
        return c;
    }

    /// <summary>Re-anchors the conic to the body that dominates its current position (SOI entry / exit). Returns true when
    /// the parent changed. r_new = r_old + C_old - C_new, v_new = v_old + V_old - V_new: both come out of Eval in global
    /// coordinates, so re-anchoring is just Anchor at the current state.</summary>
    internal static bool Patch(ref Conic c, double t)
    {
        c.Eval(t, out Vector3D pos, out Vector3D vel);
        Body? now = Dominant(pos, t);
        if (now?.Name == c.Parent?.Name) return false;   // Bodies are rebuilt every 10 s: compare by name, not reference
        var fresh = Anchor(pos, vel, t);
        string from = c.Parent?.Name ?? "free space", to = fresh.Parent?.Name ?? "free space";
        fresh.Handoffs = c.Handoffs + 1;
        c = fresh;   // swap the reference: readers on other threads (client draw, saves) never see a half-updated conic
        ProbeLog.Write($"orbits: SOI handoff {from} -> {to} at {pos.X:0},{pos.Y:0},{pos.Z:0} v={vel.Length():0.0} m/s");
        return true;
    }

    /// <summary>
    /// Computes a realistic prograde belt-orbit velocity for a natural asteroid spawning at <paramref name="pos"/> (global)
    /// inside a planet's or star's SOI: circular speed sqrt(μ/r) along normalize(axis × r) plus subtle eccentricity
    /// (~1-3%) so belt clusters orbit together cleanly without diving into the planet's bubble.
    /// </summary>
    // Source: none (plugin-only) - v_circ = sqrt(μ/r) along prograde tangent + parent orbital velocity.
    internal static bool TryBeltVelocity(in Vector3D pos, double t, out Vector3 vel, out Body? parent)
    {
        vel = Vector3.Zero;
        parent = Dominant(pos, t);
        if (parent == null || parent.Mu <= 0) return false;
        Vector3D centre = CentreAt(parent, t);
        Vector3D rVec = pos - centre;
        double r = rVec.Length();
        double minR = Math.Max(parent.Radius * 1.5, 1000.0);
        if (r < minR) return false;

        Vector3D up = parent.Frame != null ? parent.Frame.SpinAxis
                    : parent.Bubble != null ? parent.Bubble.OrbitAxis
                    : new Vector3D(0, 1, 0);
        if (up.LengthSquared() < 1e-6) up = new Vector3D(0, 1, 0);
        else up = Vector3D.Normalize(up);

        Vector3D tang = Vector3D.Cross(up, rVec);
        if (tang.LengthSquared() < 1e-6) tang = Vector3D.Cross(new Vector3D(1, 0, 0), rVec);
        if (tang.LengthSquared() < 1e-6) tang = Vector3D.Cross(new Vector3D(0, 0, 1), rVec);
        tang = Vector3D.Normalize(tang);
        Vector3D rHat = rVec / r;
        Vector3D binorm = Vector3D.Normalize(Vector3D.Cross(tang, rHat));

        double vCirc = Math.Sqrt(parent.Mu / r);
        // Subtle belt dispersion (e ~ 0.01-0.03) so nearby cluster rocks orbit together with gentle relative drift.
        var rng = Random.Shared;
        double speedScale = 1.0 + (rng.NextDouble() - 0.5) * 0.03;   // +/- 1.5%
        double radFrac = (rng.NextDouble() - 0.5) * 0.03;           // +/- 1.5%
        double vertFrac = (rng.NextDouble() - 0.5) * 0.02;          // +/- 1.0%

        Vector3D vRel = tang * (vCirc * speedScale) + rHat * (vCirc * radFrac) + binorm * (vCirc * vertFrac);

        // Guard: never let dispersion push periapsis into the parent planet or its bubble boundary.
        double safePeri = Math.Max(parent.Radius * 1.35, (parent.Frame?.Radius ?? 0) * 1.08);
        if (safePeri > 0)
        {
            double e2 = vRel.LengthSquared() * 0.5 - parent.Mu / r;
            Vector3D h = Vector3D.Cross(rVec, vRel);
            double h2 = h.LengthSquared();
            double ecc = Math.Sqrt(Math.Max(0.0, 1.0 + 2.0 * e2 * h2 / (parent.Mu * parent.Mu)));
            double peri = (h2 / parent.Mu) / (1.0 + ecc);
            if (peri < safePeri) vRel = tang * vCirc;
        }

        Vector3D vGlobal = VelocityAt(parent, t) + vRel;
        // If spawning inside a rotating bubble's outer fringe, express in bubble-local coordinates.
        Bubbles.Bubble? b = Bubbles.BubbleAt(pos);
        if (b != null && !b.IsIdentity)
        {
            Vector3D omega = b.SpinAxis * b.SpinRate;
            vGlobal -= Vector3D.Cross(omega, pos - b.C0) + BubbleHandoff.OrbitalVelocity(b, t);
        }
        vel = (Vector3)vGlobal;
        return true;
    }

    /// <summary>Periapsis / apoapsis of a conic about its parent (m); apo = +inf for unbound or parent-less.</summary>
    internal static void Apsides(Conic c, out double peri, out double apo)
    {
        peri = 0; apo = double.PositiveInfinity;
        if (c.Parent == null || c.Parent.Mu <= 0) { peri = double.PositiveInfinity; return; }
        double mu = c.Parent.Mu, rl = c.R0.Length(), v2 = c.V0.LengthSquared();
        double energy = v2 / 2 - mu / rl;
        Vector3D h = Vector3D.Cross(c.R0, c.V0);
        double ecc = Math.Sqrt(Math.Max(0, 1 + 2 * energy * h.LengthSquared() / (mu * mu)));
        double p = h.LengthSquared() / mu;
        peri = p / (1 + ecc);
        if (energy < 0) apo = p / Math.Max(1e-9, 1 - ecc);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Classical elements (belt catalog, stage 4.2)
    // ---------------------------------------------------------------------------------------------------------------

    /// <summary>State from Keplerian elements at mean anomaly M (elliptic, e &lt; 1), relative to the parent, in the frame
    /// whose "up" is <paramref name="up"/>: a, e, inclination i, ascending node Ω, argument of periapsis ω.</summary>
    // Source: H. Curtis, Orbital Mechanics for Engineering Students, alg. 4.5 (perifocal -> inertial), Kepler's equation by Newton.
    internal static void FromElements(double mu, double a, double e, double inc, double node, double argp, double M, in Vector3D up,
        out Vector3D r, out Vector3D v)
    {
        double E = e < 0.8 ? M : Math.PI;
        for (int i = 0; i < 30; i++)
        {
            double f = E - e * Math.Sin(E) - M;
            double step = f / (1 - e * Math.Cos(E));
            E -= step;
            if (Math.Abs(step) < 1e-12) break;
        }
        double cosE = Math.Cos(E), sinE = Math.Sin(E);
        double b = a * Math.Sqrt(1 - e * e);
        double rr = a * (1 - e * cosE);
        double n = Math.Sqrt(mu / (a * a * a));
        // perifocal
        Vector3D rp = new(a * (cosE - e), b * sinE, 0);
        Vector3D vp = new(-a * n * sinE / (1 - e * cosE), b * n * cosE / (1 - e * cosE), 0);
        // frame: z = up, x any perpendicular
        Vector3D z = Vector3D.Normalize(up);
        Vector3D x = Vector3D.Normalize(Vector3D.Cross(Math.Abs(z.Y) < 0.9 ? new Vector3D(0, 1, 0) : new Vector3D(1, 0, 0), z));
        Vector3D y = Vector3D.Cross(z, x);
        r = Perifocal(rp, inc, node, argp, x, y, z);
        v = Perifocal(vp, inc, node, argp, x, y, z);
        _ = rr;
    }

    private static Vector3D Perifocal(in Vector3D q, double inc, double node, double argp, in Vector3D X, in Vector3D Y, in Vector3D Z)
    {
        double cO = Math.Cos(node), sO = Math.Sin(node), ci = Math.Cos(inc), si = Math.Sin(inc), cw = Math.Cos(argp), sw = Math.Sin(argp);
        double x = (cO * cw - sO * sw * ci) * q.X + (-cO * sw - sO * cw * ci) * q.Y;
        double y = (sO * cw + cO * sw * ci) * q.X + (-sO * sw + cO * cw * ci) * q.Y;
        double z = (sw * si) * q.X + (cw * si) * q.Y;
        return X * x + Y * y + Z * z;
    }

    internal static Vector3D Rot(in Vector3D v, in Vector3D axis, double a)
    {
        if (a == 0) return v;
        double c = Math.Cos(a), s = Math.Sin(a);
        return v * c + Vector3D.Cross(axis, v) * s + axis * (Vector3D.Dot(axis, v) * (1 - c));
    }

    internal static Quaternion RotQ(in Vector3D omega, double t)
    {
        double w = omega.Length();
        if (w < 1e-12) return Quaternion.Identity;
        return Quaternion.CreateFromAxisAngle((Vector3)(omega / w), (float)(w * t));
    }
}
