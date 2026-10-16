using System;
using System.Collections.Generic;

namespace MechJebTweaks.WarpGuard
{
    /// <summary>
    /// Decides whether the vessel's engines are quiet enough for on-rails warp.
    ///
    /// Two engine behaviours make a simple throttle check unreliable:
    /// - stock throttle response decays exponentially and never reaches exactly zero;
    /// - KSP Interstellar Extended engines keep thrusting through on-rails warp whenever the engine's
    ///   currentThrottle is above zero when warp starts, using the last requested mass flow; its
    ///   thermal nozzles also boost thrust at throttles below 1%.
    ///
    /// So the guard looks at actual thrust (and at the thrust a persistent-thrust engine would carry
    /// into warp), waits until the tail is negligible, then snaps the throttle of engines that are
    /// commanded off to exactly zero, and lets warp start once all engines have been silent for
    /// SettleSeconds. Only stock ModuleEngines fields are used, so this works with any engine mod.
    /// </summary>
    internal static class ThrustTracker
    {
        /// <summary>Below this, thrust counts as zero (KSPIE uses the same cut-off), kN.</summary>
        private const double ZeroThrust = 5e-7;

        /// <summary>How long after MechJeb's last warp attempt the guard may still snap throttles, s.</summary>
        private const double SnapWindow = 5.0;

        private static Vessel _vessel;
        private static List<ModuleEngines> _engines = new List<ModuleEngines>();
        private static bool _dirty = true;
        private static double _lastThrustUT = double.NegativeInfinity;
        private static double _lastWarpRequestUT = double.NegativeInfinity;
        private static bool _holding;
        private static double _holdStartUT;

        /// <summary>Engine list must be rebuilt (staging, docking, vessel switch).</summary>
        public static void Invalidate() => _dirty = true;

        /// <summary>Called every physics frame for the active vessel.</summary>
        public static void Sample(Vessel vessel)
        {
            if (vessel == null)
                return;

            if (vessel != _vessel)
            {
                _vessel = vessel;
                _dirty = true;
                _lastThrustUT = double.NegativeInfinity;
                _holding = false;
            }

            double now = Planetarium.GetUniversalTime();
            double thrust = TotalThrust(out _);
            if (thrust <= ZeroThrust)
                return;

            _lastThrustUT = now;

            // Only while MechJeb is trying to warp: cut the remaining tail once it is negligible,
            // otherwise a persistent-thrust engine would carry it through the whole warp.
            if (now - _lastWarpRequestUT < SnapWindow && TailIsNegligible(vessel, thrust))
                SnapCommandedOffEnginesToZero();
        }

        /// <summary>Called by the warp patch whenever MechJeb tries to raise on-rails warp.</summary>
        public static bool IsWarpAllowed(Vessel vessel)
        {
            if (vessel == null)
                return true;

            // MechJeb practically always drives the active vessel; for any other vessel only the
            // current state can be checked.
            if (vessel != _vessel)
                return EnginesThrust(vessel.FindPartModulesImplementing<ModuleEngines>(), out _) <= ZeroThrust;

            double now = Planetarium.GetUniversalTime();
            _lastWarpRequestUT = now;

            double thrust = TotalThrust(out string loudest);
            if (thrust > ZeroThrust)
            {
                _lastThrustUT = now;
                BeginHold(now, $"{loudest} still produces {thrust:G3} kN ({thrust / Math.Max(vessel.totalMass, 1e-6):G3} m/s²)");
                return false;
            }

            if (now - _lastThrustUT < Settings.SettleSeconds)
            {
                BeginHold(now, "engines just stopped, settling");
                return false;
            }

            if (_holding)
            {
                Log.Info($"warp released after {now - _holdStartUT:F1} s");
                _holding = false;
            }
            return true;
        }

        private static void BeginHold(double now, string reason)
        {
            if (_holding)
                return;

            _holding = true;
            _holdStartUT = now;
            Log.Info($"holding MechJeb warp: {reason}");
        }

        /// <summary>
        /// True when no engine is commanded to burn and the remaining thrust is below
        /// SnapAcceleration. A commanded burn is never cut.
        /// </summary>
        private static bool TailIsNegligible(Vessel vessel, double thrust)
        {
            if (vessel.totalMass <= 0)
                return false;

            for (int i = 0; i < _engines.Count; i++)
            {
                ModuleEngines engine = _engines[i];
                if (engine != null && engine.requestedThrottle > 0f && EngineThrust(engine) > ZeroThrust)
                    return false;
            }

            return thrust / vessel.totalMass < Settings.SnapAcceleration;
        }

        private static void SnapCommandedOffEnginesToZero()
        {
            for (int i = 0; i < _engines.Count; i++)
            {
                ModuleEngines engine = _engines[i];
                if (engine == null || engine.part == null || engine.requestedThrottle > 0f || engine.currentThrottle <= 0f)
                    continue;

                engine.currentThrottle = 0f;
                Log.Info($"snapped residual throttle of {PartTitle(engine)} to zero");
            }
        }

        private static double TotalThrust(out string loudest)
        {
            if (_dirty)
            {
                _engines = _vessel.FindPartModulesImplementing<ModuleEngines>();
                _dirty = false;
            }

            return EnginesThrust(_engines, out loudest);
        }

        private static double EnginesThrust(List<ModuleEngines> engines, out string loudest)
        {
            double total = 0;
            double max = 0;
            loudest = string.Empty;

            for (int i = 0; i < engines.Count; i++)
            {
                ModuleEngines engine = engines[i];
                if (engine == null || engine.part == null)
                {
                    _dirty = true;
                    continue;
                }

                double thrust = EngineThrust(engine);
                total += thrust;
                if (thrust > max)
                {
                    max = thrust;
                    loudest = PartTitle(engine);
                }
            }

            return total;
        }

        /// <summary>
        /// The larger of the actual thrust and the thrust implied by the last requested mass flow
        /// (what KSPIE carries into warp), kN.
        /// </summary>
        private static double EngineThrust(ModuleEngines engine)
        {
            if (!engine.EngineIgnited)
                return 0;

            double fromMassFlow = engine.requestedMassFlow * PhysicsGlobals.GravitationalAcceleration * engine.realIsp;
            return Math.Max(engine.finalThrust, fromMassFlow);
        }

        private static string PartTitle(ModuleEngines engine) => engine.part.partInfo?.title ?? engine.part.name;
    }
}
