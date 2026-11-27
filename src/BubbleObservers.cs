using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Reflection;
using HarmonyLib;
using Keen.VRage.Core.Game.Data;
using Keen.VRage.Core.Game.GameSystems.GamePruning;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.DCS.Builders;
using Keen.VRage.DCS.Components;
using Keen.VRage.DCS.CoreData;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Multiplayer.Replications.FailureStrategies;
using Keen.VRage.Multiplayer.Replications.InProcess;
using Keen.VRage.Multiplayer.Replications.SceneSubset;
using Keen.VRage.Multiplayer;
using Keen.VRage.Core;
using Keen.VRage.Voxels.Client.Components;

namespace SpinProbe;

/// <summary>
/// Inside a planet bubble the player and the game camera sit at the planet-LOCAL position L, so everything the engine
/// spawns or sends "around the player" happens around L. The player's true global position G = ToGlobal(L) is never
/// observed, so global asteroids around G never exist on the client. Two fixes, both observing G as well as L:
/// <list type="bullet">
/// <item>Client: a second ClientTriggerTag entity at G, so client procedural generation fills sectors around G.
/// Source: VRage.Voxels.Client VoxelObserverSessionComponent.cs:43-66 (the vanilla trigger entity, moved to the
/// "VoxelObserver" camera every frame); VRage.Game.Client ProceduralGeneratorClientSessionComponent.cs:84-93 (triggers
/// on ClientTriggerTag, 11 km enter margin, sectors ref-counted, so a second trigger entity is supported).</item>
/// <item>Server: SceneSubset (per-client replication relevance, VRage.Multiplayer SceneSubsetServerSessionComponent)
/// enforcement switched on, plus G as a second observer point per client. Vanilla has one 1 m trigger per client attached
/// to its controlled entity (Subscribe :317-337); a root is a member while that point is inside the root's
/// ReplicationBoundsData sphere. We add members whose sphere contains G, keep them through the primary trigger's exits
/// (OnEntityExited :353-363) and Reconcile (:455-489), and remove them when G leaves.</item>
/// </list>
/// G is only used while L is in a moving bubble and G itself is in no bubble (G inside a bubble would be read as that
/// planet's local space and spawn things in the wrong place).
/// </summary>
internal static class BubbleObservers
{
    /// <summary>G for a planet-local pose, or null when L is not in a moving bubble or G falls in a bubble.</summary>
    internal static WorldTransform? GlobalPoint(in WorldTransform local, bool allowInBubble = false)
    {
        if (!Bubbles.Ready) return null;
        Bubbles.Bubble? b = Bubbles.BubbleAt(local.Position);
        if (b == null || b.IsIdentity) return null;
        WorldTransform g = Bubbles.ToGlobal(b, local, Bubbles.T);
        if (!allowInBubble && Bubbles.BubbleAt(g.Position) != null) return null;
        return g;
    }

    // ---------------- client: second procedural trigger at G ----------------

    private static DEntityContext _clientTrigger;
    private static bool _clientTriggerAlive;
    private static bool _loggedUpdate;
    private static bool _atG;
    private static int _clientErrors;

    private static readonly AccessTools.FieldRef<VoxelObserverSessionComponent, DEntityContext> VanillaObserver =
        AccessTools.FieldRefAccess<VoxelObserverSessionComponent, DEntityContext>("_observerEntity");

    [HarmonyPatch]
    internal static class ClientTriggerAddPatch
    {
        private static MethodBase TargetMethod() => AccessTools.FirstMethod(typeof(VoxelObserverSessionComponent),
            m => m.Name.EndsWith("OnAddedToScene", StringComparison.Ordinal));

        private static void Postfix(VoxelObserverSessionComponent __instance)
        {
            try
            {
                DEntityContext vanilla = VanillaObserver(__instance);
                WorldTransform wt = vanilla.Get<WorldTransform>();
                _clientTrigger = __instance.Session.Scene.AddEntity(default(ClientTriggerTag), wt, new BoundingBoxData
                {
                    BoundingBox = new BoundingBox(new Vector3(-1f), new Vector3(1f))
                });
                _clientTriggerAlive = true;
                _loggedUpdate = false;
                ProbeLog.Write("observers: client asteroid trigger for the global position created");
            }
            catch (Exception ex) { ProbeLog.Write("observers: client trigger create failed: " + ex); }
        }
    }

    [HarmonyPatch]
    internal static class ClientTriggerRemovePatch
    {
        private static MethodBase TargetMethod() => AccessTools.FirstMethod(typeof(VoxelObserverSessionComponent),
            m => m.Name.EndsWith("OnBeforeRemovedFromScene", StringComparison.Ordinal));

        private static void Postfix(VoxelObserverSessionComponent __instance)
        {
            if (!_clientTriggerAlive) return;
            _clientTriggerAlive = false;
            try { __instance.Session.MarkEntityForClose(_clientTrigger); }
            catch (Exception ex) { ProbeLog.Write("observers: client trigger close failed: " + ex.Message); }
        }
    }

    /// <summary>Runs right after the vanilla trigger moved to the camera (same job group, before the trigger callbacks).</summary>
    [HarmonyPatch(typeof(VoxelObserverSessionComponent), nameof(VoxelObserverSessionComponent.UpdateObserver))]
    internal static class ClientTriggerUpdatePatch
    {
        private static void Postfix(VoxelObserverSessionComponent __instance)
        {
            if (!_clientTriggerAlive) return;
            try
            {
                WorldTransform l = VanillaObserver(__instance).Get<WorldTransform>();
                // G inside the bubble is allowed here: the sectors it loads there are the planet's own (deterministic)
                // local content, just not near the player; the sectors past the bubble edge are the global asteroids.
                // A spin-only planet keeps G inside its own sphere, so without this the trigger never moved (run 76).
                WorldTransform? g = GlobalPoint(l, allowInBubble: true);
                // Not in a moving bubble: sit on the vanilla trigger (same sectors, ref-counted, no extra work).
                WorldTransform target = g ?? l;
                _clientTrigger.Set(target);
                if (!_loggedUpdate) { _loggedUpdate = true; ProbeLog.Write("observers: client trigger update running"); }
                if (g.HasValue != _atG)
                {
                    _atG = g.HasValue;
                    ProbeLog.Write(_atG
                        ? $"observers: client asteroid trigger now at global {target.Position:0} (local {l.Position:0})"
                        : "observers: client asteroid trigger back on the camera");
                }
            }
            catch (Exception ex) { if (_clientErrors++ < 5) ProbeLog.Write("observers: client trigger update failed: " + ex.Message); }
        }
    }

    // ---------------- server: SceneSubset with G as a second observer ----------------

    /// <summary>Every published replication sphere (root entity -> anchor, radius), kept by the jobs below.</summary>
    private static readonly ConcurrentDictionary<DEntity, (Vector3D Anchor, double Radius)> Bounds = new();
    /// <summary>Members we added for G, per client.</summary>
    private static readonly Dictionary<ClientId, HashSet<DEntity>> FromG = new();
    [ThreadStatic] private static bool _ourExit;
    internal static SceneSubsetServerSessionComponent? Instance;

    /// <summary>Whether any client's subset holds this root.</summary>
    internal static bool InAnySubset(DEntity root)
    {
        var inst = Instance;
        if (inst == null) return true;
        foreach (DictionaryEntry de in (IDictionary)ClientsField.GetValue(inst)!)
            if (inst.IsInSubset((ClientId)de.Key, root)) return true;
        return false;
    }
    private static long _maintainTicks;
    private static int _serverErrors;

    private static readonly Type ClientStateType = AccessTools.Inner(typeof(SceneSubsetServerSessionComponent), "ClientState");
    private static readonly FieldInfo ClientsField = AccessTools.Field(typeof(SceneSubsetServerSessionComponent), "_clients");
    private static readonly FieldInfo ConfigField = AccessTools.Field(typeof(SceneSubsetServerSessionComponent), "_configuration");
    private static readonly FieldInfo ObserverField = AccessTools.Field(ClientStateType, "Observer");
    private static readonly FieldInfo MembersField = AccessTools.Field(ClientStateType, "Members");
    private static readonly FieldInfo TriggerField = AccessTools.Field(ClientStateType, "Trigger");
    private static readonly MethodInfo EnteredMethod = AccessTools.Method(typeof(SceneSubsetServerSessionComponent), "OnEntityEntered");
    private static readonly MethodInfo ExitedMethod = AccessTools.Method(typeof(SceneSubsetServerSessionComponent), "OnEntityExited");

    private delegate bool IsSubsetManagedDel(DEntityContext ctx);
    private delegate void PublishDel(SceneSubsetServerSessionComponent inst, DEntityContext ctx, in WorldTransform wt, in BoundingBoxData bb);
    private delegate void ApplyInitialDel(SceneSubsetServerSessionComponent inst, DEntityContext ctx, DEntity entity);

    private static readonly IsSubsetManagedDel IsSubsetManagedFn = (IsSubsetManagedDel)Delegate.CreateDelegate(
        typeof(IsSubsetManagedDel), AccessTools.Method(typeof(SceneSubsetServerSessionComponent), "IsSubsetManaged"));
    private static readonly PublishDel PublishFn = (PublishDel)Delegate.CreateDelegate(
        typeof(PublishDel), AccessTools.Method(typeof(SceneSubsetServerSessionComponent), "Publish"));
    private static readonly ApplyInitialDel ApplyInitialFn = (ApplyInitialDel)Delegate.CreateDelegate(
        typeof(ApplyInitialDel), AccessTools.Method(typeof(SceneSubsetServerSessionComponent), "ApplyInitialObservedState"));

    internal static int RescuedRoots;
    private static bool _sweptInitialRoots;

    /// <summary>
    /// Publishes <see cref="ReplicationBoundsData"/> and initial <see cref="UnobservedTag"/> for any live, non-staged
    /// subset-managed root that missed <c>SceneSubsetServerSessionComponent.OnSpatialRootAdded</c>.
    /// Keen's job only listens to <c>[OnAdded(WorldTransform)]</c> and <c>[OnAdded(BoundingBoxData)]</c> with
    /// <c>[DoesNotHave(StagingTag)]</c> — it is missing <c>[OnRemoved(StagingTag)]</c>, so entities constructed in
    /// staging on tick T and added to the scene (unstaged) on tick T+k never receive bounds, never enter any client's
    /// spatial trigger, and never replicate even when right next to the player.
    /// </summary>
    internal static bool EnsurePublished(DEntityContext ctx)
    {
        var inst = Instance;
        if (inst == null || !inst.EnforcementEnabled) return false;
        if (ctx.Has<ReplicationBoundsData>() || ctx.Has<StagingTag>()) return false;
        if (!IsSubsetManagedFn(ctx)) return false;
        if (!ctx.TryGet(out WorldTransform wt) || !ctx.TryGet(out BoundingBoxData bb)) return false;
        return EnsurePublished(ctx, in wt, in bb);
    }

    internal static bool EnsurePublished(DEntityContext ctx, in WorldTransform wt, in BoundingBoxData bb)
    {
        var inst = Instance;
        if (inst == null || !inst.EnforcementEnabled) return false;
        if (ctx.Has<ReplicationBoundsData>() || ctx.Has<StagingTag>()) return false;
        if (!IsSubsetManagedFn(ctx)) return false;
        PublishFn(inst, ctx, in wt, in bb);
        ApplyInitialFn(inst, ctx, ctx.Entity);
        SubsetChildTags.Mirror(ctx.Scene, ctx.Entity);
        RescuedRoots++;
        return true;
    }

    /// <summary>Turns enforcement on before Tuning is read (Init :150-161). EnforceSceneSubset has a private setter
    /// (SceneSubsetCoreConfiguration.cs:429) and is false unless content sets it; none does.</summary>
    [HarmonyPatch(typeof(SceneSubsetServerSessionComponent), "Init")]
    internal static class EnforcePatch
    {
        private static void Prefix(SceneSubsetServerSessionComponent __instance)
        {
            Instance = __instance;
            try
            {
                object config = ConfigField.GetValue(__instance)!;
                AccessTools.Property(config.GetType(), "EnforceSceneSubset").SetValue(config, true);
                ProbeLog.Write("observers: SceneSubset enforcement ON (replication limited to each player's surroundings)");
            }
            catch (Exception ex) { ProbeLog.Write("observers: SceneSubset enable failed: " + ex); }
        }
    }

    /// <summary>The primary trigger leaving a root that G still sees must not drop it; also never evict a character
    /// (Ctrl+Space teleporting a character across a bubble boundary caused SceneSubset to drop the client's new character
    /// 36 ms after spawn mid-PossessInternal, crashing ThirdPersonCameraComponent.ReadCameraState).</summary>
    [HarmonyPatch(typeof(SceneSubsetServerSessionComponent), "OnEntityExited")]
    internal static class ExitPatch
    {
        private static bool Prefix(ClientId client, DEntity entity)
        {
            try
            {
                var scene = Instance?.Entity?.Scene;
                if (scene != null && !entity.IsEmpty && scene.IsEntityAlive(entity) &&
                    new DEntityContext(scene, entity).Has<Keen.VRage.Physics.Havok.Character.CharacterInfo>())
                    return false;
            }
            catch { }
            if (_ourExit) return true;
            lock (FromG) return !(FromG.TryGetValue(client, out var set) && set.Contains(entity));
        }
    }

    /// <summary>Reconcile removes every member the primary trigger does not hold; put G's members back.</summary>
    [HarmonyPatch(typeof(SceneSubsetServerSessionComponent), "Reconcile")]
    internal static class ReconcilePatch
    {
        private static void Postfix(SceneSubsetServerSessionComponent __instance, object __0)
        {
            try
            {
                var client = (ClientId)AccessTools.Field(ClientStateType, "Client").GetValue(__0)!;
                var members = (IDictionary)MembersField.GetValue(__0)!;
                List<DEntity> back = new();
                lock (FromG)
                    if (FromG.TryGetValue(client, out var set))
                        foreach (DEntity e in set) if (!members.Contains(e)) back.Add(e);
                foreach (DEntity e in back) EnteredMethod.Invoke(__instance, new object[] { client, e });
                if (back.Count > 0) ProbeLog.Write($"observers: reconcile dropped {back.Count} global-side member(s); restored");
            }
            catch (Exception ex) { if (_serverErrors++ < 5) ProbeLog.Write("observers: reconcile restore failed: " + ex.Message); }
        }
    }

    /// <summary>Every 30 server ticks: add roots whose sphere contains G, drop ones G left (150 m exit margin, like the
    /// vanilla trigger's MembershipExitMargin).</summary>
    [HarmonyPatch(typeof(SceneSubsetServerSessionComponent), "Maintain")]
    internal static class MaintainPatch
    {
        private static void Postfix(SceneSubsetServerSessionComponent __instance)
        {
            if (!_sweptInitialRoots && __instance.EnforcementEnabled)
            {
                _sweptInitialRoots = true;
                try
                {
                    int before = RescuedRoots;
                    foreach (Entity e in __instance.Session.Entities)
                    {
                        if (e != null && !e.DEntity.IsEmpty)
                            EnsurePublished(e.Data);
                    }
                    int added = RescuedRoots - before;
                    if (added > 0)
                        ProbeLog.Write($"observers: initial sweep published {added} unstaged subset root(s) missing ReplicationBoundsData");
                }
                catch (Exception ex) { ProbeLog.Write("observers: initial root sweep failed: " + ex.Message); }
            }
            if (++_maintainTicks % 30 != 0) return;
            if (_maintainTicks % 600 == 0)
                ProbeLog.Write($"observers: subset stats members={__instance.TotalMembers} enters={__instance.TotalEnters} exits={__instance.TotalExits} rescuedRoots={RescuedRoots} skippedSignals={SubsetEntityCloneSkipFix.Skipped} childTagged={SubsetChildTags.Tagged} childUntagged={SubsetChildTags.Untagged}");
            try
            {
                var scene = __instance.Entity.Scene;
                foreach (DictionaryEntry de in (IDictionary)ClientsField.GetValue(__instance)!)
                {
                    var client = (ClientId)de.Key;
                    object state = de.Value!;
                    var observerRef = ObserverField.GetValue(state)!;
                    var observer = (Keen.VRage.DCS.Components.Entity?)AccessTools.Property(observerRef.GetType(), "Entity").GetValue(observerRef);
                    var members = (IDictionary)MembersField.GetValue(state)!;
                    var trigger = TriggerField.GetValue(state) as Keen.VRage.Core.Game.GameSystems.SpatialTrigger.IEntityTrigger;

                    HashSet<DEntity> set;
                    lock (FromG) if (!FromG.TryGetValue(client, out set!)) FromG[client] = set = new();

                    WorldTransform? g = observer == null ? null : GlobalPoint(observer.Data.GetWorldTransform());
                    // Also observe around the active camera (spectator mode) when it is away from the character so
                    // spectator replicates real moving server asteroids instead of seeing empty space or placeholders.
                    Vector3D? camObs = null;
                    if (GlobalRender.HaveCamera)
                    {
                        Vector3D camPos = GlobalRender.LocalCamera.Position;
                        if (Bubbles.Ready)
                        {
                            Bubbles.Bubble? cb = GlobalRender.CameraBubble;
                            if (cb != null && !cb.IsIdentity) camPos = Bubbles.ToGlobal(cb, GlobalRender.LocalCamera, Bubbles.T).Position;
                        }
                        if (observer == null || (camPos - observer.Data.GetWorldTransform().Position).LengthSquared() > 500.0 * 500.0)
                            camObs = camPos;
                    }

                    List<DEntity> enter = new(), leave = new();
                    if (g.HasValue || camObs.HasValue)
                    {
                        foreach (var kv in Bounds)
                        {
                            if (!scene.IsEntityAlive(kv.Key)) { Bounds.TryRemove(kv.Key, out _); continue; }
                            double d2 = double.MaxValue;
                            if (g.HasValue) d2 = Math.Min(d2, (g.Value.Position - kv.Value.Anchor).LengthSquared());
                            if (camObs.HasValue) d2 = Math.Min(d2, (camObs.Value - kv.Value.Anchor).LengthSquared());
                            bool inside = d2 <= kv.Value.Radius * kv.Value.Radius;
                            bool keep = d2 <= (kv.Value.Radius + 150) * (kv.Value.Radius + 150);
                            lock (FromG)
                            {
                                if (inside && !set.Contains(kv.Key)) { set.Add(kv.Key); if (!members.Contains(kv.Key)) enter.Add(kv.Key); }
                                else if (!keep && set.Contains(kv.Key)) { set.Remove(kv.Key); leave.Add(kv.Key); }
                            }
                        }
                    }
                    else
                        lock (FromG) { leave.AddRange(set); set.Clear(); }

                    foreach (DEntity e in enter) EnteredMethod.Invoke(__instance, new object[] { client, e });
                    int left = 0;
                    foreach (DEntity e in leave)
                    {
                        if (trigger != null && trigger.Entities.Contains(e)) continue;   // L still sees it
                        if (observer != null && observer.DEntity == e) continue;
                        // Never evict a character (e.g. freshly teleported/spawned via Ctrl+Space while crossing a bubble
                        // boundary — evicting it mid-PossessInternal crashes ThirdPersonCameraComponent.ReadCameraState).
                        if (scene.IsEntityAlive(e) && new DEntityContext(scene, e).Has<Keen.VRage.Physics.Havok.Character.CharacterInfo>()) continue;
                        _ourExit = true;
                        try { ExitedMethod.Invoke(__instance, new object[] { client, e }); left++; }
                        finally { _ourExit = false; }
                    }
                    if (enter.Count > 0 || left > 0)
                        ProbeLog.Write($"observers: client {client} global-side subset +{enter.Count} -{left} (now {set.Count}, members {members.Count}, tracked roots {Bounds.Count})");
                }
            }
            catch (Exception ex) { if (_serverErrors++ < 5) ProbeLog.Write("observers: maintain failed: " + ex); }
        }
    }

    internal static void TrackBounds(DEntity entity, in ReplicationBoundsData rb) => Bounds[entity] = (rb.Anchor, rb.Radius);
}

/// <summary>Keeps BubbleObservers.Bounds current from the published spheres (SceneSubsetServerSessionComponent.Publish
/// :513-522 and OnSpatialRootChanged :545-564 write ReplicationBoundsData).</summary>
internal static class BubbleObserverJobs
{
    [Keen.VRage.Physics.Havok.PhysicsUpdate.OnCollectMassProperties]
    [OnAdded(typeof(ReplicationBoundsData))]
    internal static void BoundsAdded(DEntity entity, in ReplicationBoundsData rb) => BubbleObservers.TrackBounds(entity, rb);

    [Keen.VRage.Physics.Havok.PhysicsUpdate.OnCollectMassProperties]
    [OnChanged(typeof(ReplicationBoundsData))]
    internal static void BoundsChanged(DEntity entity, in ReplicationBoundsData rb) => BubbleObservers.TrackBounds(entity, rb);

    /// <summary>Catches roots whose WorldTransform + BoundingBoxData were added while StagingTag was set and unstaged later.</summary>
    [SceneSubsetPublishOrder]
    [OnRemoved(typeof(StagingTag))]
    [DoesNotHave(typeof(StagingTag))]
    internal static void OnUnstagedRoot(DEntity entity, in WorldTransform wt, in BoundingBoxData bb)
    {
        var inst = BubbleObservers.Instance;
        if (inst == null) return;
        try { BubbleObservers.EnsurePublished(new DEntityContext(inst.Entity.Scene, entity), in wt, in bb); }
        catch { }
    }
}

/// <summary>
/// SceneSubset enforcement + the in-process (single-player) client: run 75 asserted "only one replication at a time"
/// (ClientSideReplicationManager.cs:267) and then crashed replaying a signal for an entity the client never finished
/// building. Cause: ServerSceneContext.FlushReplication (:119-134) commits the frame with async allowed - the broadcast
/// bundle is built async (CommitBroadcastBundle :82-117, BuildAndReset(_allowAsync)) - and in the same CommitFrame
/// (:77-80) ReplicationServerSessionComponent.CommitSubsetReplication (:368-404) sends a second, sync bundle. The client
/// dispatches every operation fire-and-forget (InProcessClientPeerTransportSessionComponent.ProcessReplicationBatch
/// :197-200), so the subset bundle starts while the async broadcast is still awaiting, and the client's single
/// ReplicationContext is taken twice. Vanilla never sends two bundles in one frame, so it never hit this.
/// Fix: serialize LegacyClientSessionComponent.InitializeObjectsAsync(payload) (:142-149) on the client - a bundle that
/// arrives while another is in flight runs after it. Safe for signals: pairings and NotifyReplicationStarted happen in
/// ReplicationOperation.DispatchInternal (:134-143) BEFORE this call, and signals for pending objects stay buffered until
/// NotifyReplicationCompleted.
/// </summary>
internal static class SubsetReplicationSerializer
{
    private static readonly Type LegacyClient = AccessTools.TypeByName("Keen.VRage.Multiplayer.Replications.InProcess.LegacyClientSessionComponent");
    private static readonly Type PayloadType = AccessTools.TypeByName("Keen.VRage.Multiplayer.Replications.InProcess.ServerToClientEntityReplicationData");
    private static readonly MethodInfo Inner = AccessTools.Method(LegacyClient, "InitializeObjectsAsync", new[] { PayloadType });
    // Keen tasks are pooled and single-await, so we never keep or re-await one: each bundle's own task is awaited once,
    // inside Run, and queued bundles get a TaskCompletionSource the caller awaits instead.
    private static readonly Queue<(object Client, object Payload, Keen.VRage.Library.Threading.TaskCompletionSource Done)> Waiting = new();
    private static bool _inFlight;
    [ThreadStatic] private static bool _bypass;
    internal static int Chained;

    [HarmonyPatch]
    internal static class Patch
    {
        private static MethodBase TargetMethod() => Inner;

        private static bool Prefix(object __instance, object __0, ref Keen.VRage.Library.Threading.Task __result)
        {
            if (_bypass) return true;
            if (!_inFlight) { __result = Run(__instance, __0); return false; }
            var done = new Keen.VRage.Library.Threading.TaskCompletionSource("SpinProbe.SubsetReplicationQueue");
            Waiting.Enqueue((__instance, __0, done));
            __result = done.Task;
            if (Chained++ < 20) ProbeLog.Write($"observers: client replication bundle queued behind one still in flight (#{Chained}, waiting {Waiting.Count})");
            return false;
        }
    }

    private static async Keen.VRage.Library.Threading.Task Run(object client, object payload)
    {
        _inFlight = true;
        try
        {
            Keen.VRage.Library.Threading.Task t;
            _bypass = true;
            try { t = (Keen.VRage.Library.Threading.Task)Inner.Invoke(client, new[] { payload })!; }
            finally { _bypass = false; }
            await t;
        }
        finally
        {
            _inFlight = false;
            if (Waiting.Count > 0)
            {
                var next = Waiting.Dequeue();
                Forward(next.Client, next.Payload, next.Done);
            }
        }
    }

    private static async void Forward(object client, object payload, Keen.VRage.Library.Threading.TaskCompletionSource done)
    {
        try { await Run(client, payload); done.TrySetResult(); }
        catch (Exception ex) { done.TrySetException(ex); }
    }
}

/// <summary>
/// The actual fix for the run 75/78 crashes. ServerSceneContext.FlushReplication (:119-134) - run from
/// InProcessServerWorldTickComponent.BeforeTick - commits the frame with async allowed and then WAITS for those async
/// bundles before the server ticks again, so no signal is ever written for an entity the client is still building.
/// CommitSubsetReplication (:368-404) runs inside that same CommitFrame (:77-80) and sends a second, sync bundle that
/// the flush does not wait for; the client starts it while the async one is still building (run 75: "only one
/// replication at a time"), and if it is queued instead (run 78) the server ticks on and its signals reach the client
/// before the entity exists (NRE in NotifyInventoryMassChangedSignal replay). So: no subset commit inside a flush. The
/// pending subset entries stay queued (CommitSubsetReplication only clears them after sending) and go out with the next
/// normal CommitFrame, after the flush's async bundles finished - the same timing as any entity added during a tick.
/// </summary>
internal static class SubsetFlushGuard
{
    private static readonly Type ServerSceneContext = AccessTools.TypeByName("Keen.VRage.Multiplayer.Replications.InProcess.ServerSceneContext");
    [ThreadStatic] private static bool _inFlush;
    private static int _deferred;

    [HarmonyPatch]
    internal static class FlushPatch
    {
        private static MethodBase TargetMethod() => AccessTools.FirstMethod(ServerSceneContext,
            m => m.Name.EndsWith("IReplicationContext.FlushReplication", StringComparison.Ordinal));
        private static void Prefix() => _inFlush = true;
        private static void Postfix() => _inFlush = false;
        private static Exception? Finalizer(Exception? __exception) { _inFlush = false; return __exception; }
    }

    [HarmonyPatch]
    internal static class CommitPatch
    {
        private static MethodBase TargetMethod() => AccessTools.Method(
            AccessTools.TypeByName("Keen.VRage.Multiplayer.Replications.ReplicationServerSessionComponent"), "CommitSubsetReplication");
        private static bool Prefix()
        {
            if (!_inFlush) return true;
            if (_deferred++ < 10) ProbeLog.Write($"observers: subset replication deferred out of an async flush (#{_deferred})");
            return false;
        }
    }
}

/// <summary>
/// Run 79 crash: with enforcement on, the server still sent a signal (InventoryComponent.NotifyInventoryMassChanged) for
/// an entity the client never received. Signal writers drop a signal only when the entity ITSELF has UnobservedTag
/// (Game2.Simulation ReplicationCommandBufferWriterReader, e.g. the inventory trampoline: !Has&lt;ExclusiveTag&gt;() &amp;&amp;
/// !Has&lt;UnobservedTag&gt;()), but SceneSubset only tags the ROOT (OnObserverCountChanged :365-388,
/// ApplyInitialObservedState :390-396). Blocks, inventories and other children of an unobserved grid or character kept
/// sending. Fix: mirror the root's UnobservedTag onto its whole hierarchy, and onto children added or re-parented later
/// (ReplicationServerSessionComponent.OnSubsetManagedEntityAdded, SceneSubset OnRootBecameChild).
/// </summary>
internal static class SubsetChildTags
{
    private static int _errors;
    internal static int Tagged, Untagged;

    internal static void Mirror(Keen.VRage.DCS.Scenes.Scene scene, DEntity root)
    {
        try
        {
            if (!scene.IsEntityAlive(root)) return;
            var rootEntity = Keen.VRage.DCS.Components.Entity.FromDataEntity((scene, root));
            bool unobserved = rootEntity.Data.Has<UnobservedTag>();
            ApplySubtree(rootEntity, unobserved, skipSelf: true);
        }
        catch (Exception ex) { if (_errors++ < 5) ProbeLog.Write("observers: child tag mirror failed: " + ex.Message); }
    }

    internal static void ApplySubtree(Keen.VRage.DCS.Components.Entity start, bool unobserved, bool skipSelf)
    {
        var stack = new Stack<Keen.VRage.DCS.Components.Entity>();
        stack.Push(start);
        bool first = true;
        while (stack.Count > 0)
        {
            var e = stack.Pop();
            if (!(first && skipSelf))
            {
                DEntityContext d = e.Data;
                if (unobserved) { if (!d.Has<UnobservedTag>()) { d.Set(default(UnobservedTag)); Tagged++; } }
                else if (d.TryRemove<UnobservedTag>()) Untagged++;
            }
            first = false;
            var children = e.TryGet<Keen.VRage.Core.Game.Components.HierarchyComponent>()?.Children;
            if (children != null) foreach (var c in children) stack.Push(c);
        }
    }

    [HarmonyPatch(typeof(SceneSubsetServerSessionComponent), "OnObserverCountChanged")]
    internal static class CountPatch
    {
        private static void Postfix(SceneSubsetServerSessionComponent __instance, DEntity entity) => Mirror(__instance.Entity.Scene, entity);
    }

    [HarmonyPatch(typeof(SceneSubsetServerSessionComponent), "ApplyInitialObservedState")]
    internal static class InitialPatch
    {
        private static void Postfix(SceneSubsetServerSessionComponent __instance, DEntity entity) => Mirror(__instance.Entity.Scene, entity);
    }

    [HarmonyPatch]
    internal static class ChildAddedPatch
    {
        private static MethodBase TargetMethod() => AccessTools.Method(
            AccessTools.TypeByName("Keen.VRage.Multiplayer.Replications.ReplicationServerSessionComponent"), "OnSubsetManagedEntityAdded");
        private static void Postfix(Keen.VRage.DCS.Components.Entity entity)
        {
            try
            {
                if (!entity.Data.Has<ParentData>())
                {
                    BubbleObservers.EnsurePublished(entity.Data);
                    return;
                }
                DEntityContext root = entity.Data.GetTopLevelParent();
                BubbleObservers.EnsurePublished(root);
                bool unobserved = root.Has<UnobservedTag>();
                ApplySubtree(entity, unobserved, skipSelf: false);
            }
            catch (Exception ex) { if (_errors++ < 5) ProbeLog.Write("observers: child tag on add failed: " + ex.Message); }
        }
    }

    [HarmonyPatch(typeof(SceneSubsetServerSessionComponent), "OnRootBecameChild")]
    internal static class ReparentPatch
    {
        private static void Postfix(SceneSubsetServerSessionComponent subset, DEntity entity)
        {
            try
            {
                if (subset == null) return;
                var scene = subset.Entity.Scene;
                if (!scene.IsEntityAlive(entity)) return;
                var e = Keen.VRage.DCS.Components.Entity.FromDataEntity((scene, entity));
                ApplySubtree(e, e.Data.GetTopLevelParent().Has<UnobservedTag>(), skipSelf: false);
            }
            catch (Exception ex) { if (_errors++ < 5) ProbeLog.Write("observers: child tag on reparent failed: " + ex.Message); }
        }
    }
}

/// <summary>
/// Root-cause fix for the SceneSubset signal-replay NRE crashes (runs 79–82).
/// <para>When <c>EnforceSceneSubset</c> is enabled, <c>LegacyServerSessionComponent.ConfigureCloningContext</c> (:41)
/// creates <see cref="ReplicationEntityTranslationContext"/> with <c>unreplicatedToNull = true</c>. Inside
/// <see cref="ReplicationEntityTranslationContext.Clone"/> (:56-71), that flag unconditionally pushes
/// <see cref="ReplicationFailureStrategy.NullOnAnyFail"/> and returns <c>null</c> for any live unreplicated entity —
/// overriding <see cref="ReplicationFailureStrategy.Skip"/> pushed by <c>InProcessCommandBufferContext.BeforeSignal()</c>
/// for Argument 0 (the signal target) and by <c>SkipOnReplicationFail&lt;Entity&gt;</c>. Because <c>ShouldSkip</c> stays
/// <c>false</c>, the forwarder writes the signal to the client command buffer with <c>targetEntity == null</c>, crashing
/// the client with a <see cref="NullReferenceException"/> on replay.</para>
/// <para>Entity replication payloads use <c>InProcessReplicationTrackerAdapter</c> (<c>CurrentStrategy == Throw</c>), and
/// <c>NullOnReplicationFail&lt;Entity&gt;</c> pushes <c>Null</c> — both still run the original method. Only
/// <see cref="ReplicationFailureStrategy.Skip"/> honors <c>TryRemap</c> directly so signals targeting unreplicated
/// entities are cleanly skipped at forwarding time (after <c>CommitSubsetReplication</c> has registered all newly
/// entering entities in the tracker).</para>
/// </summary>
[HarmonyPatch(typeof(ReplicationEntityTranslationContext), nameof(ReplicationEntityTranslationContext.Clone))]
internal static class SubsetEntityCloneSkipFix
{
    internal static long Skipped;

    [HarmonyPrefix]
    private static bool Prefix(ReplicationEntityTranslationContext __instance, Entity entity, ref Entity? __result)
    {
        if (EntityAllocator.IsStub(entity)) return true;
        if (__instance.ReplicationContext.CurrentStrategy != ReplicationFailureStrategy.Skip) return true;

        object? mapped = __instance.ReplicationContext.TryRemap(entity);
        if (mapped == null && __instance.ReplicationContext.ShouldSkip)
        {
            long n = System.Threading.Interlocked.Increment(ref Skipped);
            if (n <= 15 || n % 500 == 0)
                ProbeLog.Write($"observers: transport skipped signal/ref for unreplicated {entity} (#{n})");
        }
        __result = (Entity?)mapped;
        return false;
    }
}
