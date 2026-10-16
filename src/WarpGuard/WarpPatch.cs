using System;
using System.Reflection;
using HarmonyLib;
using KSP.Localization;
using UnityEngine;

namespace MechJebTweaks.WarpGuard
{
    /// <summary>
    /// Every MechJeb warp increase (node executor, landing autopilot, ascent, Warp Helper,
    /// rover controller, "resume warp" button) ends in the private method
    /// MuMech.MechJebModuleWarpController.SetTimeWarpRate(int rateIndex, bool instant).
    /// A prefix there vetoes increases while the vessel's engines are still burning.
    ///
    /// MechJeb is resolved by name at runtime, so this DLL has no compile-time dependency on it
    /// and does nothing (except log a warning) if MechJeb is missing or its internals changed.
    /// </summary>
    internal static class WarpPatch
    {
        private const string HarmonyId = "MechJebTweaks.WarpGuard";
        private const string WarpControllerType = "MuMech.MechJebModuleWarpController";
        private const string TargetMethod = "SetTimeWarpRate";
        private const float MessageInterval = 2.5f;

        private static MethodInfo _vesselGetter;
        private static float _lastMessageTime = float.NegativeInfinity;

        public static void Apply()
        {
            Type controller = AccessTools.TypeByName(WarpControllerType);
            if (controller == null)
            {
                Log.Warning($"{WarpControllerType} not found, MechJeb is not installed. Nothing patched.");
                return;
            }

            MethodInfo target = AccessTools.Method(controller, TargetMethod, new[] { typeof(int), typeof(bool) });
            if (target == null)
            {
                Log.Error($"{WarpControllerType}.{TargetMethod}(int, bool) not found. " +
                          "This MechJeb version is not supported. Nothing patched.");
                return;
            }

            _vesselGetter = AccessTools.PropertyGetter(controller, "Vessel");

            new Harmony(HarmonyId).Patch(target, prefix: new HarmonyMethod(typeof(WarpPatch), nameof(Prefix)));
            Log.Info($"patched {WarpControllerType}.{TargetMethod}");
        }

        /// <returns>false to skip MechJeb's original method, i.e. to refuse the warp change.</returns>
        private static bool Prefix(object __instance, int rateIndex)
        {
            try
            {
                // Dropping out of warp is always allowed.
                if (rateIndex <= TimeWarp.CurrentRateIndex)
                    return true;

                bool railsWarp = TimeWarp.WarpMode == TimeWarp.Modes.HIGH;
                if (!railsWarp && !Settings.GuardPhysicsWarp)
                    return true;

                Vessel vessel = (_vesselGetter?.Invoke(__instance, null) as Vessel) ?? FlightGlobals.ActiveVessel;
                if (ThrustTracker.IsWarpAllowed(vessel))
                    return true;

                ShowWaitingMessage();
                return false;
            }
            catch (Exception e)
            {
                // Never break MechJeb because of the guard.
                Log.Error($"prefix failed, letting MechJeb warp: {e}");
                return true;
            }
        }

        private static void ShowWaitingMessage()
        {
            if (!Settings.ShowMessages || Time.unscaledTime - _lastMessageTime < MessageInterval)
                return;

            _lastMessageTime = Time.unscaledTime;
            ScreenMessages.PostScreenMessage(Localizer.Format("#LOC_MJTweaks_WarpWaiting"), MessageInterval, ScreenMessageStyle.UPPER_CENTER);
        }
    }
}
