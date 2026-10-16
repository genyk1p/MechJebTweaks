using System;
using System.Reflection;
using HarmonyLib;
using MuMech;
using MuMech.Landing;

namespace MechJebTweaks.Landing
{
    /// <summary>
    /// MechJeb's landing step LowDeorbitBurn asks for warp towards the deorbit burn only while the vessel
    /// turns slower than 0.001 rad/s. Holding retrograde, the vessel has to turn with the orbit, and low
    /// orbits of small bodies turn faster than that (JNSQ Mun at 160 km: 0.00114 rad/s), so MechJeb never
    /// warps and coasts at 1x until the player warps by hand. A postfix asks for the same warp, under the
    /// same conditions, when the vessel turns about as fast as the orbit and points where MechJeb wants.
    /// </summary>
    internal static class LandingWarpPatch
    {
        private const string HarmonyId = "MechJebTweaks.LandingWarp";

        /// <summary>MechJeb's own angular velocity limit in LowDeorbitBurn, rad/s.</summary>
        private const double MechJebRateLimit = 0.001;

        private static AccessTools.FieldRef<AutopilotStep, MechJebCore> _core;
        private static AccessTools.FieldRef<LowDeorbitBurn, bool> _burnTriggered;
        private static LowDeorbitBurn _loggedStep;
        private static bool _errorLogged;

        public static void Apply()
        {
            try
            {
                MethodInfo target = AccessTools.Method(typeof(LowDeorbitBurn), nameof(LowDeorbitBurn.OnFixedUpdate));
                _core = AccessTools.FieldRefAccess<AutopilotStep, MechJebCore>("Core");
                _burnTriggered = AccessTools.FieldRefAccess<LowDeorbitBurn, bool>("_deorbitBurnTriggered");
                new Harmony(HarmonyId).Patch(target, postfix: new HarmonyMethod(typeof(LandingWarpPatch), nameof(Postfix)));
                Log.Info("patched MuMech.Landing.LowDeorbitBurn.OnFixedUpdate (landing warp fix)");
            }
            catch (Exception e)
            {
                Log.Error($"landing warp fix not installed, this MechJeb version is not supported: {e}");
            }
        }

        private static void Postfix(LowDeorbitBurn __instance, AutopilotStep __result)
        {
            try
            {
                // Same preconditions as MechJeb: still this step, burn not started, auto-warp on, not warping yet.
                if (!ReferenceEquals(__result, __instance) || _burnTriggered(__instance))
                    return;
                MechJebCore core = _core(__instance);
                Vessel vessel = core?.vessel;
                if (vessel == null || vessel.packed || !core.Node.Autowarp || TimeWarp.CurrentRateIndex > 0)
                    return;

                double turnRate = vessel.angularVelocity.magnitude;
                if (turnRate < MechJebRateLimit)
                    return; // MechJeb asks for warp itself

                // MechJeb's distances: warp only farther than twice the distance where the burn starts.
                VesselState state = core.VesselState;
                double horizontalSpeed = state.speedSurfaceHorizontal;
                double burnDistance = horizontalSpeed * horizontalSpeed / state.limitedMaxThrustAccel;
                double descent = (double)state.altitudeASL - core.Landing.DecelerationEndAltitude();
                if (burnDistance < descent)
                    burnDistance = descent;
                double toTarget = Vector3d.Exclude(state.up, core.Target.GetPositionTargetPosition() - state.CoM).magnitude;
                if (!(toTarget > 2 * burnDistance))
                    return;

                // Holding retrograde the vessel turns with the orbit.
                double orbitTurnRate = vessel.obt_velocity.magnitude / (vessel.CoMD - vessel.mainBody.position).magnitude;
                if (turnRate > orbitTurnRate + Settings.LandingWarpExtraRate)
                    return;
                double aim = core.Attitude.attitudeAngleFromTarget();
                if (aim > Settings.LandingWarpAimAngle)
                    return;

                core.Warp.WarpRegularAtRate((float)(vessel.orbit.period / 6));

                if (!ReferenceEquals(_loggedStep, __instance))
                {
                    _loggedStep = __instance;
                    Log.Info($"landing: warping towards the low deorbit burn for MechJeb: vessel turns {turnRate:F5} rad/s " +
                             $"(MechJeb warps only below {MechJebRateLimit}, the orbit turns {orbitTurnRate:F5}), aimed within {aim:F1}°, " +
                             $"{toTarget / 1000:F0} km to the target, burn starts at {burnDistance / 1000:F0} km");
                }
            }
            catch (Exception e)
            {
                // Never break MechJeb's landing because of the fix; runs every physics frame, so log once.
                if (!_errorLogged)
                    Log.Error($"landing warp fix failed: {e}");
                _errorLogged = true;
            }
        }
    }
}
