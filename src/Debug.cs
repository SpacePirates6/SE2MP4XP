using System;
using System.Collections.Generic;
using HarmonyLib;
using Keen.Game2.Client.GameSystems.Missions;
using Keen.Game2.Client.UI.Tutorial;
using Keen.Game2.Simulation;
using Keen.Game2.Simulation.GameSystems.Colonization;
using Keen.Game2.Simulation.GameSystems.Missions;
using Keen.Game2.Simulation.GameSystems.Missions.Onboarding;
using Keen.Game2.Simulation.GameSystems.Missions.Triggers;
using Keen.Game2.Simulation.GameSystems.Missions.Triggers.Actions;
using Keen.Game2.Simulation.GameSystems.Ownership;
using Keen.Game2.Simulation.GameSystems.Player;
using Keen.Game2.Simulation.GameSystems.Progression;
using Keen.Game2.Simulation.GameSystems.Tutorial;
using Keen.Game2.Simulation.GameSystems.Video;
using Keen.VRage.Core.Render;
using Keen.VRage.DCS.Components;

namespace SpinProbe;

/// <summary>
/// Makes every loaded world genuinely Creative (not just creative tools):
/// <list type="bullet">
/// <item>Sets <c>GameModeObjectBuilder.CreativeGlobal = true</c> before <c>GameModeSessionComponent.Init</c> copies it
/// into <c>_creativeGlobal</c>, and keeps <c>CreativeGlobal</c>, <c>IsCreative</c>, and <c>IsPersonalCreative</c> true.</item>
/// <item>Disables block progression (<c>ProgressionSessionComponent</c>) and sector progression
/// (<c>ColonizationProgressSessionComponent</c>), matching Keen's Creative <c>WorldVariant</c> settings.</item>
/// </list>
/// Sources: Game2.Simulation\...\GameModeSessionComponent.cs:44-89, GameModeObjectBuilder.cs:51-54,
/// ProgressionSessionComponent.cs:45-49,131-134, ColonizationProgressSessionComponent.cs:38-42.
/// </summary>
[HarmonyPatch(typeof(GameModeSessionComponent))]
internal static class DebugCreativeToolsPatch
{
    [HarmonyPrefix]
    [HarmonyPatch("Init", new[] { typeof(GameModeObjectBuilder) })]
    private static void ForceCreativeOnInit(GameModeObjectBuilder ob)
    {
        if (!Settings.ForceCreative || ob == null) return;
        if (!ob.CreativeGlobal)
        {
            ob.CreativeGlobal = true;
            ProbeLog.Write("creative: forced GameModeObjectBuilder.CreativeGlobal = true");
        }
    }

    [HarmonyPrefix]
    [HarmonyPatch(nameof(GameModeSessionComponent.CreativeGlobal), MethodType.Setter)]
    private static void KeepCreativeGlobal(ref bool value)
    {
        if (Settings.ForceCreative) value = true;
    }

    [HarmonyPrefix]
    [HarmonyPatch(nameof(GameModeSessionComponent.IsCreative), typeof(Entity))]
    private static bool IsCreativeEntity(ref bool __result)
    {
        if (!Settings.ForceCreative) return true;
        __result = true;
        return false;
    }

    [HarmonyPrefix]
    [HarmonyPatch(nameof(GameModeSessionComponent.IsCreative), typeof(IdentityId))]
    private static bool IsCreativeIdentity(ref bool __result)
    {
        if (!Settings.ForceCreative) return true;
        __result = true;
        return false;
    }

    [HarmonyPrefix]
    [HarmonyPatch(nameof(GameModeSessionComponent.IsPersonalCreative), typeof(IdentityId))]
    private static bool IsPersonalCreative(ref bool __result)
    {
        if (!Settings.ForceCreative) return true;
        __result = true;
        return false;
    }
}

[HarmonyPatch(typeof(ProgressionSessionComponent))]
internal static class CreativeProgressionPatch
{
    [HarmonyPrefix]
    [HarmonyPatch("Init", new[] { typeof(ProgressionObjectBuilder) })]
    private static void DisableProgressionOnInit(ProgressionObjectBuilder ob)
    {
        if (!Settings.ForceCreative || ob == null) return;
        if (ob.Enabled)
        {
            ob.Enabled = false;
            ProbeLog.Write("creative: disabled block progression");
        }
    }

    [HarmonyPrefix]
    [HarmonyPatch(nameof(ProgressionSessionComponent.IsProgressionEnabled), typeof(IdentityId))]
    private static bool DisableProgressionCheck(ref bool __result)
    {
        if (!Settings.ForceCreative) return true;
        __result = false;
        return false;
    }
}

[HarmonyPatch(typeof(ColonizationProgressSessionComponent), "Init", new[] { typeof(ColonizationProgressObjectBuilder) })]
internal static class CreativeColonizationPatch
{
    [HarmonyPrefix]
    private static void DisableSectorProgression(ColonizationProgressObjectBuilder ob)
    {
        if (!Settings.ForceCreative || ob == null) return;
        if (ob.SectorProgression)
        {
            ob.SectorProgression = false;
            ProbeLog.Write("creative: disabled sector progression");
        }
    }
}

/// <summary>
/// Skips Survival / FTUE mission tutorials, intro cutscene videos, objectives HUD, onboarding popups, contextual hints,
/// and UI spotlight overlays on every loaded world.
/// Sources: MissionSessionComponent.cs:195-271,526-534,1253-1266,1554-1561; MissionSessionObjectBuilder.cs:51-54;
/// MissionUISessionComponent.cs:51-90; TutorialOverlaySessionComponent.cs:276-307.
/// </summary>
[HarmonyPatch(typeof(MissionSessionComponent))]
internal static class SkipTutorialMissionPatch
{
    [HarmonyPrefix]
    [HarmonyPatch("Init", new[] { typeof(MissionSessionObjectBuilder) })]
    private static void DisableMissionOnInit(MissionSessionObjectBuilder ob)
    {
        if (!Settings.SkipTutorial || ob == null) return;
        int triggers = ob.TriggerOBs?.Count ?? 0;
        int objectives = ob.Objectives?.Count ?? 0;
        bool wasPlaying = ob.IsPlaying;
        ob.IsPlaying = false;
        ob.Objectives?.Clear();
        ob.TriggerOBs?.Clear();
        ob.Triggers?.Clear();
        ob.ActiveTriggerOnStart?.Clear();
        ob.Progress?.Clear();
        if (wasPlaying || triggers > 0 || objectives > 0)
            ProbeLog.Write($"tutorial: skipped mission/FTUE on load (wasPlaying={wasPlaying}, cleared {triggers} trigger(s) and {objectives} objective(s))");
    }

    [HarmonyPrefix]
    [HarmonyPatch(nameof(MissionSessionComponent.Configure), new[] { typeof(bool?), typeof(bool) })]
    private static bool SkipConfigure() => !Settings.SkipTutorial;

    [HarmonyPrefix]
    [HarmonyPatch(nameof(MissionSessionComponent.SetPlaying), new[] { typeof(bool) })]
    private static bool SkipSetPlaying() => !Settings.SkipTutorial;
}

[HarmonyPatch(typeof(MissionUISessionComponent))]
internal static class SkipTutorialClientUiPatch
{
    [HarmonyPrefix]
    [HarmonyPatch(nameof(MissionUISessionComponent.PlayVideo), new[] { typeof(List<VideoDefinition>), typeof(VideoPlaybackOptions) })]
    private static bool SkipIntroVideo()
    {
        if (!Settings.SkipTutorial) return true;
        ProbeLog.Write("tutorial: blocked mission video playback");
        return false;
    }

    [HarmonyPrefix]
    [HarmonyPatch(nameof(MissionUISessionComponent.ShowOnboarding), new[] { typeof(OnboardingTooltipBaseDefinition) })]
    private static bool SkipOnboarding() => !Settings.SkipTutorial;
}

[HarmonyPatch(typeof(TutorialOverlaySessionComponent), nameof(TutorialOverlaySessionComponent.LoadState), new[] { typeof(TutorialBaseDefinition), typeof(int) })]
internal static class SkipTutorialOverlayPatch
{
    [HarmonyPrefix]
    private static bool SkipOverlay() => !Settings.SkipTutorial;
}

internal static class SkipTutorialActions
{
    internal static void Install(Harmony harmony)
    {
        if (!Settings.SkipTutorial) return;
        var skip = new HarmonyMethod(typeof(SkipTutorialActions), nameof(Skip));
        foreach (Type t in new[]
        {
            typeof(PlayVideoActionComponent),
            typeof(SetUiTutorialActionComponent),
            typeof(ShowOnboardingTooltipActionComponent),
            typeof(ShowContextualHintActionComponent),
            typeof(PlayDialogActionComponent),
        })
        {
            var m = AccessTools.Method(t, "Activate", new[] { typeof(ITriggerContext) });
            if (m != null) harmony.Patch(m, prefix: skip);
        }
        ProbeLog.Write("tutorial: action guards installed");
    }

    private static bool Skip() => !Settings.SkipTutorial;
}
