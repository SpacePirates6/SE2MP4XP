using System;
using System.Collections.Generic;
using HarmonyLib;
using Keen.Game2.Client.GameSystems;
using Keen.Game2.Client.GameSystems.PlayerControl;
using Keen.Game2.Client.GameSystems.PlayerControl.Helpers;
using Keen.Game2.Client.Input;
using Keen.Game2.Client.UI.InGame;
using Keen.Game2.Client.UI.Shared.UGC.Blueprint.Banner;
using Keen.Game2.Simulation.GameSystems.EntityDetection;
using Keen.Game2.Simulation.Replication;
using Keen.Game2.Simulation.WorldObjects.Characters;
using Keen.VRage.Core;
using Keen.VRage.Core.Game.Components;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Game.GameSystems.ProceduralGeneration;
using Keen.VRage.Core.Game.Systems;
using Keen.VRage.Core.Input;
using Keen.VRage.Core.Render;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.CoreData;
using Keen.VRage.DCS.ObjectBuilders;
using Keen.VRage.Input;
using Keen.VRage.Library.Localization;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Library.Utils;
using Keen.VRage.Physics.Data;
using Keen.VRage.Render.Utils;
using Keen.VRage.Voxels.Components;

namespace SpinProbe;

/// <summary>
/// Ctrl+X (cut), Ctrl+C (copy), and Ctrl+V (paste) support for asteroids behaving like standard SE2 blueprints:
/// <list type="bullet">
/// <item>Opens the native SE2 top paste banner + client 3D asteroid ghost preview + cyan UI3D wireframe bounding box</item>
/// <item>Single LMB click places immediately; RMB / ESC cancels; Scroll adjusts distance; Ins/Del/Home/End/PgUp/PgDn rotate</item>
/// <item>Inherits velocity exactly like vanilla BlockPlacerEntityComponent (controller velocity + forward * InitialGridVelocity)</item>
/// </list>
/// </summary>
internal static class AsteroidClipboard
{
    private sealed class PendingCapture
    {
        internal Vector3D TargetPos;
        internal double Radius;
        internal bool DeleteAfter;
        internal long Ticks;
    }

    private sealed class PendingPaste
    {
        internal WorldTransform SpawnWt;
        internal Vector3 LinearVelocity;
        internal Vector3 AngularVelocity;
        internal long Ticks;
    }

    private sealed class PendingSpawnMotion
    {
        internal Vector3D Pos;
        internal Vector3 LinVel;
        internal Vector3 AngVel;
        internal long Ticks;
    }

    private static readonly object LockObj = new();
    private static volatile PendingCapture? _pendingCapture;
    private static volatile PendingPaste? _pendingPaste;
    private static readonly List<PendingSpawnMotion> SpawnMotions = new();

    private static EntityObjectBuilder? _storedOB;
    private static EntityObjectBuilder? _storedClientOB;
    private static double _storedRadius;
    private static Vector3D _storedHalfExtents;
    private static string _storedName = "";
    private static int _storedVersion;

    // ---------- Client paste-preview state (standard SE2 blueprint placement flow) ----------
    private static volatile bool _pasteActive;
    private static volatile bool _pendingEnterPasteMode;
    private static int _previewSpawnedVersion = -1;
    private static Session? _clientSession;
    private static PlayerControllerComponent? _player;
    private static InGameUI? _inGameUI;
    private static IInputManager? _input;
    private static IObservableDisposable? _banner;
    private static Entity? _previewEntity;
    private static MeshBuilder? _lines;

    private static double _pasteDist = 60.0;
    private static Quaternion _pasteRot = Quaternion.Identity;

    // Edge tracking
    private static long _prevTickMs;
    private static bool _lmbWasDown;
    private static bool _rmbWasDown;
    private static bool _escWasDown;
    private static long _enterModeMs;
    private static int _clientErrors;

    /// <summary>True while spawning the client-local visual preview entity so physics/triggers are skipped cleanly.</summary>
    [ThreadStatic] internal static bool SpawningPreview;

    internal static bool IsPreviewEntity(Entity? e) => e != null && ReferenceEquals(e, _previewEntity);

    internal static void SetInputManager(IInputManager? im)
    {
        if (im != null) _input = im;
    }

    internal static void Clear()
    {
        lock (LockObj)
        {
            _storedOB = null;
            _storedClientOB = null;
            _storedRadius = 0;
            _storedHalfExtents = Vector3D.Zero;
            _storedName = "";
            _pendingPaste = null;
            _pendingEnterPasteMode = false;
        }
        ExitPasteMode();
    }

    internal static bool TryGet(out EntityObjectBuilder? ob, out double radius, out Vector3D halfExtents, out string name, out int version)
    {
        lock (LockObj)
        {
            ob = _storedOB;
            radius = _storedRadius;
            halfExtents = _storedHalfExtents;
            name = _storedName;
            version = _storedVersion;
            return ob != null;
        }
    }

    /// <summary>Called from MovableAsteroids.InitPatch on the server when a pasted asteroid initializes, so it
    /// bypasses PlanetExclusion and receives its inherited velocity.</summary>
    internal static bool TryConsumeSpawnMotion(in Vector3D pos, out Vector3 linVel, out Vector3 angVel)
    {
        linVel = Vector3.Zero;
        angVel = Vector3.Zero;
        long now = DateTime.UtcNow.Ticks;
        lock (SpawnMotions)
        {
            SpawnMotions.RemoveAll(x => now - x.Ticks > TimeSpan.TicksPerSecond * 10);
            for (int i = 0; i < SpawnMotions.Count; i++)
            {
                if ((SpawnMotions[i].Pos - pos).LengthSquared() < 25.0)
                {
                    linVel = SpawnMotions[i].LinVel;
                    angVel = SpawnMotions[i].AngVel;
                    SpawnMotions.RemoveAt(i);
                    return true;
                }
            }
        }
        return false;
    }

    private static void RememberClientContext(HotKeysSessionComponent hk, PlayerControllerComponent player)
    {
        _player = player;
        _inGameUI = hk._inGameUI;
        try
        {
            Session? s = hk.GetSession();
            if (s != null)
            {
                _clientSession = s;
                IInputManager? im = s.TryGet<IInputManager>();
                if (im != null) _input = im;
            }
        }
        catch { }
    }

    private static void QueueCapture(Vector3D pos, double radius, bool deleteAfter, string via)
    {
        lock (LockObj)
        {
            _pendingCapture = new PendingCapture
            {
                TargetPos = pos,
                Radius = radius,
                DeleteAfter = deleteAfter,
                Ticks = DateTime.UtcNow.Ticks,
            };
            // Enter paste mode automatically right after capture completes (just like vanilla Ctrl+C / Ctrl+X).
            _pendingEnterPasteMode = true;
        }
        ProbeLog.Write($"asteroid: clipboard {(deleteAfter ? "CUT" : "COPY")} requested at {pos.X:0},{pos.Y:0},{pos.Z:0} (r={radius:0} m, via {via})");
    }

    private static void QueuePaste(WorldTransform spawnWt, Vector3 linVel, Vector3 angVel, double dist, string name)
    {
        lock (LockObj)
        {
            _pendingPaste = new PendingPaste
            {
                SpawnWt = spawnWt,
                LinearVelocity = linVel,
                AngularVelocity = angVel,
                Ticks = DateTime.UtcNow.Ticks,
            };
        }
        ProbeLog.Write($"asteroid: clipboard PASTE requested for {name} at {spawnWt.Position.X:0},{spawnWt.Position.Y:0},{spawnWt.Position.Z:0} (dist={dist:0} m, v={linVel.Length():0.0} m/s)");
    }

    internal static void SanitizeVoxelOB(EntityObjectBuilder ob, bool newProceduralId)
    {
        int vIdx = ob.TryGetIndexOf<VoxelStorageComponentBase>();
        if (vIdx >= 0 && ob.ObjectBuilders[vIdx] is VoxelStorageComponentBaseObjectBuilder vob)
        {
            // Never share or hold a live reference to the source entity's IStorage (which is disposed on Ctrl+X
            // and would corrupt both copies on Ctrl+C). Clearing SharedStorage forces LoadStorage/CreateStorage.
            vob.SharedStorage = default;
        }

        int pIdx = ob.TryGetIndexOf<ProceduralVoxelEntityComponent>();
        if (pIdx < 0) pIdx = ob.TryGetIndexOf<ProceduralEntityComponentBase>();
        if (pIdx >= 0 && ob.ObjectBuilders[pIdx] is ProceduralEntityComponentBaseObjectBuilder pob)
        {
            pob.MarkedAsManual = true;
            if (newProceduralId)
            {
                pob.ID = new SpaceEntityId(
                    new Vector3I(
                        Random.Shared.Next(-1000000, 1000000),
                        Random.Shared.Next(-1000000, 1000000),
                        Random.Shared.Next(-1000000, 1000000)),
                    Random.Shared.Next());
            }
        }
    }

    // ---------- Server-side capture & spawn jobs ----------

    /// <summary>Runs on the server thread for EVERY non-planet VoxelPhysicsComponent (both movable and static asteroids).</summary>
    [Keen.VRage.Physics.Havok.PhysicsUpdate.OnCollectMassProperties]
    [DoesNotHave(typeof(NoAutoSync))]
    internal static void CaptureJob(VoxelPhysicsComponent physics, in WorldTransform wt)
    {
        PendingCapture? req = _pendingCapture;
        if (req == null || physics == null) return;

        if (DateTime.UtcNow.Ticks - req.Ticks > TimeSpan.TicksPerSecond * 5)
        {
            lock (LockObj)
            {
                if (ReferenceEquals(_pendingCapture, req)) _pendingCapture = null;
                _pendingEnterPasteMode = false;
            }
            ProbeLog.Write($"asteroid: clipboard capture timed out (no matching server asteroid near {req.TargetPos.X:0},{req.TargetPos.Y:0},{req.TargetPos.Z:0})");
            return;
        }

        Entity e = physics.Entity;
        if (e == null || e.TryGet<VoxelPlanetComponent>() != null) return;

        double maxMatch = Math.Max(40.0, req.Radius * 1.75);
        if ((wt.Position - req.TargetPos).LengthSquared() > maxMatch * maxMatch) return;

        lock (LockObj)
        {
            if (!ReferenceEquals(_pendingCapture, req)) return;
            _pendingCapture = null;
        }

        try
        {
            Session session = e.GetSession();
            var proc = e.TryGet<ProceduralVoxelEntityComponent>();
            if (proc != null && !proc.IsManual) proc.TurnManual();

            EntityObjectBuilder ob = e.Scene.EntitySerializer.Serialize(e);
            SanitizeVoxelOB(ob, newProceduralId: false);

            Vector3D halfExt = new Vector3D(Math.Max(8.0, req.Radius));
            try
            {
                if (physics._storage != null)
                {
                    Vector3I sz = physics._storageSize;
                    float vs = physics._storage.VoxelSize;
                    halfExt = new Vector3D(sz.X * vs * 0.42, sz.Y * vs * 0.42, sz.Z * vs * 0.42);
                }
            }
            catch { }

            string storageInfo = "";
            int vIdx = ob.TryGetIndexOf<VoxelStorageComponentBase>();
            if (vIdx >= 0 && ob.ObjectBuilders[vIdx] is VoxelStorageComponentBaseObjectBuilder vob)
            {
                storageInfo = vob is ProceduralVoxelStorageComponentObjectBuilder pvob
                    ? $", seed={pvob.Seed} size={pvob.Size:0} offloaded={vob.Offloaded.IsValid}"
                    : $", offloaded={vob.Offloaded.IsValid}";
            }

            string name = e.ToString() ?? "asteroid";
            lock (LockObj)
            {
                _storedOB = ob;
                _storedRadius = req.Radius;
                _storedHalfExtents = halfExt;
                _storedName = name;
                _storedVersion++;
            }
            ProbeLog.Write($"asteroid: clipboard captured {name} (r={req.Radius:0} m, delete={req.DeleteAfter}{storageInfo})");

            if (req.DeleteAfter)
            {
                VoxelStorageComponentBase? storage = e.TryGet<VoxelStorageComponentBase>();
                if (storage == null || !storage.TryDeleteManually())
                    session.MarkEntityForClose(e, true);
                ProbeLog.Write($"asteroid: clipboard cut deleted {name}");
            }
        }
        catch (Exception ex)
        {
            _pendingEnterPasteMode = false;
            ProbeLog.Write("asteroid: clipboard capture failed: " + ex);
        }
    }

    private static unsafe void ExecutePaste(Session session, PendingPaste req)
    {
        if (DateTime.UtcNow.Ticks - req.Ticks > TimeSpan.TicksPerSecond * 5) return;
        if (!TryGet(out EntityObjectBuilder? stored, out _, out _, out string name, out _) || stored == null) return;

        try
        {
            EntityObjectBuilder clone = stored.DeepClone();
            clone.OB<WorldTransformComponent, WorldTransformComponentObjectBuilder>().Transform = req.SpawnWt;
            SanitizeVoxelOB(clone, newProceduralId: true);

            lock (SpawnMotions)
            {
                SpawnMotions.Add(new PendingSpawnMotion
                {
                    Pos = req.SpawnWt.Position,
                    LinVel = req.LinearVelocity,
                    AngVel = req.AngularVelocity,
                    Ticks = DateTime.UtcNow.Ticks,
                });
            }

            IEntitySpawner spawner = session.Get<IEntitySpawner>();
            Vector3 halfExt;
            lock (LockObj) halfExt = (Vector3)_storedHalfExtents;
            if (halfExt.LengthSquared() < 1f) halfExt = new Vector3(32f);
            float estSize = Math.Max(halfExt.X, Math.Max(halfExt.Y, halfExt.Z)) * 2f;
            var mass = MovableAsteroids.PrototypeMass(estSize, 1f);
            var eb = MovableAsteroids.BuildDynamicSpawnData(session, halfExt, in mass, req.LinearVelocity, req.AngularVelocity);
            Entity spawned = spawner.SpawnEntity(clone, eb);

            try
            {
                if (req.LinearVelocity != Vector3.Zero || req.AngularVelocity != Vector3.Zero)
                    spawned.Data.Set(new RigidBodyData { LinearVelocity = req.LinearVelocity, AngularVelocity = req.AngularVelocity });
            }
            catch { }

            ProbeLog.Write($"asteroid: clipboard SPAWNED {spawned} (from {name}) at {req.SpawnWt.Position.X:0},{req.SpawnWt.Position.Y:0},{req.SpawnWt.Position.Z:0} v={req.LinearVelocity.Length():0.0} m/s");
        }
        catch (Exception ex)
        {
            ProbeLog.Write("asteroid: clipboard server spawn failed: " + ex);
        }
    }

    /// <summary>Spawns queued clipboard pastes at the start of the server tick (outside asteroid archetype iteration).</summary>
    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(InstanceBind<VoxelPlanetComponent>))]
    internal static void PasteJobPlanet(VoxelPlanetComponent planet)
    {
        PendingPaste? req = _pendingPaste;
        if (req == null || planet?.Entity == null) return;
        lock (LockObj)
        {
            if (!ReferenceEquals(_pendingPaste, req)) return;
            _pendingPaste = null;
        }
        ExecutePaste(planet.Entity.GetSession(), req);
    }

    /// <summary>Fallback paste runner for planet-less worlds.</summary>
    [Keen.VRage.Core.Systems.WorldUpdateBeginJobGroup]
    [MustHave(typeof(InstanceBind<CharacterComponent>))]
    internal static void PasteJobCharacter(CharacterComponent character)
    {
        PendingPaste? req = _pendingPaste;
        if (req == null || character?.Entity == null) return;
        lock (LockObj)
        {
            if (!ReferenceEquals(_pendingPaste, req)) return;
            _pendingPaste = null;
        }
        ExecutePaste(character.Entity.GetSession(), req);
    }

    // ---------- Client blueprint-style paste mode ----------

    private static void EnterPasteMode()
    {
        if (!TryGet(out EntityObjectBuilder? ob, out double radius, out _, out string name, out int ver) || ob == null)
            return;

        _pasteActive = true;
        _pasteDist = Math.Clamp(Math.Max(35.0, radius * 2.5 + 20.0), 15.0, 500.0);
        _pasteRot = Quaternion.Identity;
        _lmbWasDown = true;
        _rmbWasDown = true;
        _escWasDown = true;
        _enterModeMs = Environment.TickCount64;
        _prevTickMs = _enterModeMs;

        OpenBanner();
        EnsurePreviewEntity(ver);
        ProbeLog.Write($"asteroid: entered paste mode for {name} (r={radius:0} m, dist={_pasteDist:0} m)");
    }

    private static void ExitPasteMode()
    {
        bool was = _pasteActive;
        _pasteActive = false;
        CloseBanner();
        DestroyPreviewEntity();
        try { _lines?.Commit(); } catch { }
        if (was) ProbeLog.Write("asteroid: exited paste mode");
    }

    private static void OpenBanner()
    {
        CloseBanner();
        try
        {
            if (_inGameUI != null)
                _banner = _inGameUI.OpenBannerScreen(new BannerScreenViewModel(LocKey.FromString("ASTEROID PASTE")));
        }
        catch { }
    }

    private static void CloseBanner()
    {
        try { _banner?.Dispose(); } catch { }
        _banner = null;
    }

    private static unsafe void EnsurePreviewEntity(int version)
    {
        if (_previewSpawnedVersion == version) return;
        DestroyPreviewEntity();
        _previewSpawnedVersion = version;
        if (_clientSession == null) return;

        EntityObjectBuilder? clientOb;
        lock (LockObj) clientOb = _storedClientOB;
        if (clientOb == null) return;

        try
        {
            IEntitySpawner? spawner = _clientSession.TryGet<IEntitySpawner>();
            if (spawner == null) return;
            EntityObjectBuilder clone = clientOb.DeepClone();
            SanitizeVoxelOB(clone, newProceduralId: true);
            if (TryGetCameraTransform(_player, out WorldTransform cam))
            {
                Vector3D pos = cam.Position + (Vector3D)cam.Orientation.GetForward() * _pasteDist;
                clone.OB<WorldTransformComponent, WorldTransformComponentObjectBuilder>().Transform = new WorldTransform(pos, _pasteRot);
            }
            SpawningPreview = true;
            try
            {
                Vector3 halfExt;
                lock (LockObj) halfExt = (Vector3)_storedHalfExtents;
                if (halfExt.LengthSquared() < 1f) halfExt = new Vector3(32f);
                var eb = new Keen.VRage.DCS.Builders.DEntityBuilder();
                eb.Set(new BoundingBoxData { BoundingBox = new BoundingBox(-halfExt, halfExt) });
                _previewEntity = spawner.SpawnEntity(clone, eb);
                ProbeLog.Write($"asteroid: spawned client preview entity {_previewEntity}");
            }
            finally { SpawningPreview = false; }
        }
        catch (Exception ex)
        {
            SpawningPreview = false;
            _previewEntity = null;
            ProbeLog.Write("asteroid: preview entity spawn skipped (using wireframe box): " + ex.Message);
        }
    }

    private static void DestroyPreviewEntity()
    {
        Entity? prev = _previewEntity;
        _previewEntity = null;
        _previewSpawnedVersion = -1;
        if (prev == null) return;
        try
        {
            Session? s = _clientSession ?? prev.GetSession();
            IEntitySpawner? spawner = s?.TryGet<IEntitySpawner>();
            spawner?.DeleteEntity(prev);
        }
        catch { }
    }

    /// <summary>Matches vanilla BlockPlacerEntityComponent.GetControllerVelocity(): inherits character/ship velocity
    /// when moving faster than MinInheritedSpeed (~0.5 m/s).</summary>
    private static Vector3 GetControllerVelocity()
    {
        try
        {
            Entity? obs = _player?.GetTopObservableEntity();
            if (obs != null && obs.Data.TryGet<RigidBodyData>(out RigidBodyData rb))
            {
                Vector3 v = rb.LinearVelocity;
                if (v.IsValid() && v.LengthSquared() > 0.25f) return v;
            }
        }
        catch { }
        return Vector3.Zero;
    }

    /// <summary>Matches vanilla BlockPlacerEntityComponent: forward * BlockPlacerSessionComponent.InitialGridVelocity.</summary>
    private static float GetBlockPlacerInitialVelocity()
    {
        try
        {
            BlockPlacerSessionComponent? bp = _clientSession?.TryGet<BlockPlacerSessionComponent>();
            if (bp != null) return bp.InitialGridVelocity;
        }
        catch { }
        return 0f;
    }

    /// <summary>Called once per frame on the client main thread from AutoRun.After (GameAppComponent.GameUpdate).</summary>
    internal static void ClientTick()
    {
        if (!MovableAsteroids.On) return;
        try
        {
            if (_pendingEnterPasteMode && _pendingCapture == null)
            {
                if (TryGet(out _, out _, out _, out _, out int ver) && ver > 0)
                {
                    _pendingEnterPasteMode = false;
                    EnterPasteMode();
                }
            }

            if (!_pasteActive) return;

            if (!TryGet(out EntityObjectBuilder? stored, out double radius, out Vector3D halfExt, out string name, out int version) || stored == null)
            {
                ExitPasteMode();
                return;
            }

            if (!TryGetCameraTransform(_player, out WorldTransform cam)) return;

            EnsurePreviewEntity(version);

            long nowMs = Environment.TickCount64;
            float dt = Math.Clamp((nowMs - _prevTickMs) * 0.001f, 0.001f, 0.1f);
            _prevTickMs = nowMs;

            IInputDevice? mouse = _input?.Mouse;
            IInputDevice? kb = _input?.Keyboard;

            bool lmb = mouse != null && mouse.GetDigitalState(MouseInputs.Left);
            bool rmb = mouse != null && mouse.GetDigitalState(MouseInputs.Right);
            bool esc = kb != null && kb.GetDigitalState(KeyboardInputs.Escape);
            bool ctrl = kb != null && kb.GetDigitalState(KeyboardInputs.Control);
            float wheel = mouse != null ? mouse.GetAnalogState(MouseInputs.VerticalWheel) : 0f;

            // Cancel on RMB or ESC (just like normal SE2 blueprints).
            if ((rmb && !_rmbWasDown) || (esc && !_escWasDown))
            {
                _rmbWasDown = rmb;
                _escWasDown = esc;
                ExitPasteMode();
                return;
            }

            // Mouse wheel adjusts paste distance (just like normal SE2 blueprints).
            if (MathF.Abs(wheel) > 0.01f)
            {
                double step = Math.Max(4.0, Math.Sqrt(_pasteDist) * 1.6);
                double minDist = Math.Max(15.0, radius * 1.4 + 6.0);
                _pasteDist = Math.Clamp(_pasteDist + MathF.Sign(wheel) * step, minDist, 600.0);
            }

            // Standard SE2 rotation keys (Insert/Delete/Home/End/PageUp/PageDown).
            if (kb != null)
            {
                float rotStep = 1.6f * dt;
                Vector3 rotAxis = Vector3.Zero;
                if (kb.GetDigitalState(KeyboardInputs.Insert)) rotAxis.Y += 1f;
                if (kb.GetDigitalState(KeyboardInputs.Delete)) rotAxis.Y -= 1f;
                if (kb.GetDigitalState(KeyboardInputs.Home)) rotAxis.X += 1f;
                if (kb.GetDigitalState(KeyboardInputs.End)) rotAxis.X -= 1f;
                if (kb.GetDigitalState(KeyboardInputs.PageUp)) rotAxis.Z += 1f;
                if (kb.GetDigitalState(KeyboardInputs.PageDown)) rotAxis.Z -= 1f;
                if (rotAxis.LengthSquared() > 0.01f)
                {
                    Quaternion dq = Quaternion.CreateFromAxisAngle(Vector3.Normalize(rotAxis), rotStep);
                    _pasteRot = Quaternion.Normalize(dq * _pasteRot);
                }
            }

            Vector3D forward = (Vector3D)cam.Orientation.GetForward();
            Vector3D previewPos = cam.Position + forward * _pasteDist;
            WorldTransform previewLocalWt = new WorldTransform(previewPos, _pasteRot);

            // Exact vanilla SE2 blueprint inherited velocity: player/ship velocity + forward * InitialGridVelocity.
            Vector3 inheritedVel = GetControllerVelocity() + (Vector3)forward * GetBlockPlacerInitialVelocity();

            if (_previewEntity != null)
            {
                try { _previewEntity.Data.SetWorldTransform(in previewLocalWt); }
                catch { DestroyPreviewEntity(); }
            }

            DrawPreviewOverlay(in previewLocalWt, in halfExt, radius, inheritedVel.Length());

            // Single LMB click places immediately (exact vanilla SE2 blueprint behavior; hold Ctrl to keep pasting).
            if (lmb && !_lmbWasDown && nowMs - _enterModeMs > 150)
            {
                QueuePaste(previewLocalWt, inheritedVel, Vector3.Zero, _pasteDist, name);
                if (!ctrl) ExitPasteMode();
            }

            _lmbWasDown = lmb;
            _rmbWasDown = rmb;
            _escWasDown = esc;
        }
        catch (Exception ex)
        {
            if (_clientErrors++ < 5) ProbeLog.Write("asteroid: paste client tick error: " + ex.Message);
        }
    }

    private static void DrawPreviewOverlay(in WorldTransform localWt, in Vector3D halfExt, double radius, float inheritedSpeed)
    {
        WorldTransform wt = localWt;
        if (Bubbles.Ready && GlobalRender.CameraBubble is Bubbles.Bubble cb && !cb.IsIdentity)
            wt = Bubbles.ToGlobal(cb, localWt, Bubbles.T);

        _lines ??= new RenderMeshBuilderFactory("SpinProbeAsteroidPaste").CreateMeshBuilder(MeshBuilder.Type.UI3D, null, 2048, "SpinProbeAsteroidPaste");
        _lines.SetPrimitiveOffset(wt.Position);
        _lines.UpdateEntityTransform(new WorldTransform(wt.Position));

        ColorSRGB boxColor = new ColorSRGB(60, 230, 255, 220);
        ColorSRGB ringColor = new ColorSRGB(80, 255, 190, 140);

        // Oriented 3D Bounding Box (12 edges)
        Vector3D hx = (Vector3D)(wt.Orientation * new Vector3((float)halfExt.X, 0, 0));
        Vector3D hy = (Vector3D)(wt.Orientation * new Vector3(0, (float)halfExt.Y, 0));
        Vector3D hz = (Vector3D)(wt.Orientation * new Vector3(0, 0, (float)halfExt.Z));
        Vector3D c = wt.Position;

        Vector3D c000 = c - hx - hy - hz;
        Vector3D c100 = c + hx - hy - hz;
        Vector3D c010 = c - hx + hy - hz;
        Vector3D c110 = c + hx + hy - hz;
        Vector3D c001 = c - hx - hy + hz;
        Vector3D c101 = c + hx - hy + hz;
        Vector3D c011 = c - hx + hy + hz;
        Vector3D c111 = c + hx + hy + hz;

        _lines.AddLine(c000, c100, boxColor, depth: true);
        _lines.AddLine(c010, c110, boxColor, depth: true);
        _lines.AddLine(c001, c101, boxColor, depth: true);
        _lines.AddLine(c011, c111, boxColor, depth: true);
        _lines.AddLine(c000, c010, boxColor, depth: true);
        _lines.AddLine(c100, c110, boxColor, depth: true);
        _lines.AddLine(c001, c011, boxColor, depth: true);
        _lines.AddLine(c101, c111, boxColor, depth: true);
        _lines.AddLine(c000, c001, boxColor, depth: true);
        _lines.AddLine(c100, c101, boxColor, depth: true);
        _lines.AddLine(c010, c011, boxColor, depth: true);
        _lines.AddLine(c110, c111, boxColor, depth: true);

        // Subtle holographic wireframe rings at the asteroid's radius
        const int segs = 28;
        Vector3D rx = (Vector3D)(wt.Orientation * new Vector3((float)radius, 0, 0));
        Vector3D ry = (Vector3D)(wt.Orientation * new Vector3(0, (float)radius, 0));
        Vector3D rz = (Vector3D)(wt.Orientation * new Vector3(0, 0, (float)radius));
        DrawRing(_lines, c, rx, rz, segs, ringColor);
        DrawRing(_lines, c, rx, ry, segs, ringColor);
        DrawRing(_lines, c, ry, rz, segs, ringColor);

        Vector3D labelPos = c + hy * 1.15 + Vector3D.UnitY * 2.0;
        string velStr = inheritedSpeed > 0.5f ? $" | Vel={inheritedSpeed:0.0} m/s" : "";
        string label = $"ASTEROID PASTE | R={radius:0}m | Dist={_pasteDist:0}m{velStr}\n[LMB] Place | [Scroll] Distance | [Ins/Del/Home/End/PgUp/PgDn] Rotate | [RMB/ESC] Cancel";
        _lines.AddText(in labelPos, label, boxColor, scale: 1.0f, depth: false);

        _lines.Commit();
    }

    private static void DrawRing(MeshBuilder mb, in Vector3D center, in Vector3D axisA, in Vector3D axisB, int segs, ColorSRGB color)
    {
        Vector3D prev = center + axisA;
        for (int i = 1; i <= segs; i++)
        {
            double a = (2.0 * Math.PI * i) / segs;
            Vector3D pt = center + axisA * Math.Cos(a) + axisB * Math.Sin(a);
            mb.AddLine(prev, pt, color, depth: true);
            prev = pt;
        }
    }

    // ---------- Camera & asteroid targeting helpers ----------

    private static bool TryGetCameraTransform(PlayerControllerComponent? player, out WorldTransform cam)
    {
        if (GlobalRender.HaveCamera)
        {
            cam = GlobalRender.LocalCamera;
            return true;
        }

        try
        {
            Entity? obs = player?.GetTopObservableEntity();
            if (obs != null)
            {
                cam = obs.Data.GetWorldTransform();
                return true;
            }
        }
        catch { }

        cam = default;
        return false;
    }

    private static void CaptureClientPreviewOB(Entity? clientEntity)
    {
        if (clientEntity == null) return;
        try
        {
            EntityObjectBuilder ob = clientEntity.Scene.EntitySerializer.Serialize(clientEntity);
            SanitizeVoxelOB(ob, newProceduralId: false);
            lock (LockObj) _storedClientOB = ob;
            ProbeLog.Write($"asteroid: captured client preview OB ({ob.Definition?.DebugNameShort ?? clientEntity.ToString()})");
        }
        catch (Exception ex) { ProbeLog.Write("asteroid: client preview OB capture skipped: " + ex.Message); }
    }

    private static bool TryTargetAsteroid(PlayerControllerComponent player, StringId raycastDetectorTag,
        out Vector3D targetPos, out double radius, out string via, out Entity? clientEntity)
    {
        targetPos = default;
        radius = 0;
        via = "";
        clientEntity = null;

        // 1. Primary: exact voxel raycast from the player's detector (covers both static & movable asteroids).
        try
        {
            player.TryGetTopControllableWithComponent<RaycastEntityDetectorComponent>(raycastDetectorTag, out var detector);
            if (detector != null)
            {
                VoxelStorageComponentBase? hit = CopyPasteHelper.TryGetTargetedVoxel(detector, raycastDetectorTag);
                if (hit != null && hit.Entity != null && !IsPreviewEntity(hit.Entity) && hit.Entity.TryGet<VoxelPlanetComponent>() == null)
                {
                    targetPos = hit.Data.GetWorldTransform().Position;
                    radius = hit.Storage.Size.Max() * hit.VoxelSize * 0.35;
                    via = "raycast";
                    clientEntity = hit.Entity;
                    return true;
                }
            }
        }
        catch { }

        // 2. Fallback: camera ray-sphere test against live client movable asteroids (works in spectator & longer range up to 500 m).
        if (TryGetCameraTransform(player, out WorldTransform cam))
        {
            Vector3D origin = cam.Position;
            Vector3D dir = (Vector3D)cam.Orientation.GetForward();
            double bestT = 500.0;
            bool found = false;

            (VoxelPhysicsComponent comp, MovableAsteroids.State state)[] snap;
            lock (MovableAsteroids.LiveClient) snap = MovableAsteroids.LiveClient.ToArray();

            foreach (var (comp, st) in snap)
            {
                if (comp.Entity == null || comp.DEntity.IsEmpty || IsPreviewEntity(comp.Entity)) continue;
                try
                {
                    Vector3D c = comp.Data.GetWorldTransform().Position;
                    Vector3D oc = c - origin;
                    double proj = Vector3D.Dot(oc, dir);
                    if (proj < 0 || proj > bestT) continue;
                    double hitR = Math.Max(12.0, st.Radius * 1.15);
                    double perpSq = oc.LengthSquared() - proj * proj;
                    if (perpSq <= hitR * hitR)
                    {
                        double t = Math.Max(0.0, proj - Math.Sqrt(hitR * hitR - perpSq));
                        if (t < bestT)
                        {
                            bestT = t;
                            targetPos = c;
                            radius = st.Radius;
                            via = $"ray-sphere({bestT:0}m)";
                            clientEntity = comp.Entity;
                            found = true;
                        }
                    }
                }
                catch { }
            }

            if (found) return true;
        }

        return false;
    }

    // ---------- Harmony patches ----------

    /// <summary>Prevent the client-only preview entity from allocating Havok bodies or spatial triggers.</summary>
    [HarmonyPatch(typeof(VoxelPhysicsComponent), nameof(VoxelPhysicsComponent.OnAddedToScene))]
    internal static class PreviewVoxelAddSkipPatch
    {
        [HarmonyPrefix]
        private static bool Prefix() => !SpawningPreview;
    }

    [HarmonyPatch(typeof(VoxelPhysicsComponent), nameof(VoxelPhysicsComponent.OnBeforeRemovedFromScene))]
    internal static class PreviewVoxelRemoveSkipPatch
    {
        [HarmonyPrefix]
        private static bool Prefix(VoxelPhysicsComponent __instance) => !IsPreviewEntity(__instance.Entity);
    }

    /// <summary>Ctrl+C: copy targeted grid (clearing asteroid clipboard) or copy targeted asteroid into paste preview mode.</summary>
    [HarmonyPatch(typeof(HotKeysSessionComponent), "CopyGrid")]
    internal static class HotKeyCopyPatch
    {
        [HarmonyPrefix]
        private static bool Prefix(HotKeysSessionComponent __instance)
        {
            if (!MovableAsteroids.On) return true;
            PlayerControllerComponent? player = __instance._player?.LocalPlayerController;
            if (player == null) return true;
            RememberClientContext(__instance, player);

            StringId tag = __instance.Definition.CopyGridRaycastDetectorTag;
            if (CopyPasteHelper.TryCopyForPlayer(player, tag))
            {
                Clear();
                ProbeLog.Write("asteroid: Ctrl+C copied grid -> cleared asteroid clipboard");
                return false;
            }

            if (TryTargetAsteroid(player, tag, out Vector3D pos, out double r, out string via, out Entity? clientEntity))
            {
                CaptureClientPreviewOB(clientEntity);
                QueueCapture(pos, r, deleteAfter: false, via);
                return false;
            }

            ProbeLog.Write("asteroid: Ctrl+C pressed — no grid or asteroid targeted");
            return false;
        }
    }

    /// <summary>Ctrl+X: cut targeted grid (clearing asteroid clipboard) or cut targeted asteroid into paste preview mode.</summary>
    [HarmonyPatch(typeof(HotKeysSessionComponent), "CutGridOrDeleteVoxel")]
    internal static class HotKeyCutPatch
    {
        [HarmonyPrefix]
        private static bool Prefix(HotKeysSessionComponent __instance)
        {
            if (!MovableAsteroids.On) return true;
            PlayerControllerComponent? player = __instance._player?.LocalPlayerController;
            if (player == null) return true;
            RememberClientContext(__instance, player);

            StringId tag = __instance.Definition.CopyGridRaycastDetectorTag;
            if (CopyPasteHelper.TryCutForPlayer(player, tag))
            {
                Clear();
                ProbeLog.Write("asteroid: Ctrl+X cut grid -> cleared asteroid clipboard");
                return false;
            }

            if (TryTargetAsteroid(player, tag, out Vector3D pos, out double r, out string via, out Entity? clientEntity))
            {
                CaptureClientPreviewOB(clientEntity);
                QueueCapture(pos, r, deleteAfter: true, via);
                return false;
            }

            return true;
        }
    }

    /// <summary>Ctrl+V: if an asteroid is on our clipboard, enter blueprint-style Asteroid Paste Mode.</summary>
    [HarmonyPatch(typeof(HotKeysSessionComponent), "PasteGrid")]
    internal static class HotKeyPastePatch
    {
        [HarmonyPrefix]
        private static bool Prefix(HotKeysSessionComponent __instance)
        {
            if (!MovableAsteroids.On) return true;
            if (!TryGet(out EntityObjectBuilder? stored, out _, out _, out _, out _) || stored == null)
                return true;

            PlayerControllerComponent? player = __instance._player?.LocalPlayerController;
            if (player != null) RememberClientContext(__instance, player);

            if (_pasteActive) ExitPasteMode();
            else EnterPasteMode();
            return false;
        }
    }
}
