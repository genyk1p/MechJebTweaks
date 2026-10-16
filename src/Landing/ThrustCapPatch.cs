using System;
using System.Reflection;
using System.Runtime.CompilerServices;
using HarmonyLib;
using MuMech;

namespace MechJebTweaks.Landing
{
    /// <summary>
    /// MechJeb takes an engine's available thrust the way the game computes thrust: maxFuelFlow × Isp at the
    /// current pressure × g, scaled by the thrust limiter. Exact for engines whose thrust is proportional to the
    /// throttle. KSP Interstellar Extended thermal nozzles (ThermalEngineController) rewrite maxFuelFlow and Isp
    /// every physics frame from the reactor state, so that product depends on the current throttle: for the
    /// Lightbulb MechJeb saw 438 kN at throttle 0, 308 kN at 5% and 175 kN (the real full thrust) at 100%.
    /// With MechJeb's "minimum throttle 5%" during the coast, the landing autopilot planned the deceleration burn
    /// for 308 kN, started it far too late and crashed.
    ///
    /// The fix only touches engines on parts with one of Settings.ThrustCapModules. While such an engine runs at
    /// full throttle the fix measures its thrust; afterwards MechJeb sees exactly that, at any throttle. The fix
    /// sets the thrust, not the fuel flow: at low throttle KSPIE raises the flow and lowers Isp, and capping the
    /// flow alone left MechJeb at 102 kN (the second Mun landing came down 22 km short). In an atmosphere KSPIE
    /// takes its own pressureThreshold (exit area × static pressure, kN) off the vacuum thrust, and so does the fix.
    /// Before the first full-throttle run the part's passport thrust is a cap only.
    /// The engine's own maxFuelFlow is changed only for the duration of MechJeb's call and restored right after.
    /// </summary>
    internal static class ThrustCapPatch
    {
        private const string HarmonyId = "MechJebTweaks.ThrustCap";

        /// <summary>The fix leaves the engine alone below this share of the wanted thrust (reactor off, no fuel).</summary>
        private const double MinShareToScale = 0.01;

        private class Memory
        {
            public bool Checked;
            public PartModule Controller;
            public BaseField Threshold;
            public float PassportThrust;
            public double LearnedVacuumThrust;
            public bool CapLogged;
        }

        private static readonly ConditionalWeakTable<ModuleEngines, Memory> Engines = new ConditionalWeakTable<ModuleEngines, Memory>();

        private static ModuleEngines _restoreEngine;
        private static float _restoreFlow;
        private static bool _errorLogged;

        public static void Apply()
        {
            try
            {
                MethodInfo target = AccessTools.Method(typeof(VesselState.EngineInfo), nameof(VesselState.EngineInfo.AddNewEngine));
                new Harmony(HarmonyId).Patch(target,
                    prefix: new HarmonyMethod(typeof(ThrustCapPatch), nameof(Prefix)),
                    finalizer: new HarmonyMethod(typeof(ThrustCapPatch), nameof(Finalizer)));
                Log.Info($"patched MuMech.VesselState.EngineInfo.AddNewEngine (thrust cap for {string.Join(", ", Settings.ThrustCapModules)})");
            }
            catch (Exception e)
            {
                Log.Error($"thrust cap not installed, this MechJeb version is not supported: {e}");
            }
        }

        private static void Prefix(ModuleEngines e)
        {
            _restoreEngine = null;
            try
            {
                if (e == null || e.part == null)
                    return;
                Memory memory = Engines.GetValue(e, _ => new Memory());
                if (!memory.Checked)
                    Inspect(e, memory);
                if (memory.Controller == null)
                    return;

                float flow = e.maxFuelFlow;
                if (!(flow > 0))
                    return;

                // What MechJeb is about to compute (its Isp is the curve at the current pressure; KSPIE's curve is flat).
                double isp = e.atmosphereCurve.Evaluate((float)e.part.staticPressureAtm);
                double estimate = flow * e.flowMultiplier * isp * e.g;
                if (!(estimate > 0))
                    return;
                double threshold = PressureThreshold(memory);

                if (e.isOperational && !e.flameout && e.finalThrust > 0 && e.currentThrottle >= Settings.ThrustCapLearnThrottle
                    && e.vessel != null && !e.vessel.packed)
                    Learn(e, memory, estimate + threshold);

                double wanted;
                string source;
                if (memory.LearnedVacuumThrust > 0)
                {
                    wanted = Math.Max(memory.LearnedVacuumThrust - threshold, 0);
                    source = "measured at full throttle";
                }
                else if (memory.PassportThrust > 0)
                {
                    wanted = Math.Min(estimate, Math.Max(memory.PassportThrust - threshold, 0));
                    source = "passport thrust, not yet measured";
                }
                else
                    return;

                if (estimate < MinShareToScale * wanted || Math.Abs(wanted - estimate) < 1e-3 * wanted)
                    return;

                _restoreEngine = e;
                _restoreFlow = flow;
                e.maxFuelFlow = (float)(flow * wanted / estimate);

                if (!memory.CapLogged)
                {
                    memory.CapLogged = true;
                    Log.Info($"thrust cap: {e.part.partInfo?.title} on {e.vessel?.vesselName}: MechJeb would see {estimate:F1} kN " +
                             $"at throttle {e.currentThrottle:F2}, set to {wanted:F1} kN ({source})");
                }
            }
            catch (Exception ex)
            {
                // Never break MechJeb's vessel state because of the fix; runs every physics frame, so log once.
                if (!_errorLogged)
                    Log.Error($"thrust cap failed: {ex}");
                _errorLogged = true;
            }
        }

        private static void Finalizer()
        {
            if (_restoreEngine != null)
            {
                _restoreEngine.maxFuelFlow = _restoreFlow;
                _restoreEngine = null;
            }
        }

        private static void Inspect(ModuleEngines e, Memory memory)
        {
            memory.Checked = true;
            foreach (string name in Settings.ThrustCapModules)
                if (e.part.Modules.Contains(name))
                {
                    memory.Controller = e.part.Modules[name];
                    break;
                }
            if (memory.Controller == null)
                return;
            memory.Threshold = memory.Controller.Fields["pressureThreshold"];

            Part prefab = e.part.partInfo?.partPrefab;
            int index = e.part.Modules.IndexOf(e);
            if (prefab != null && index >= 0 && index < prefab.Modules.Count && prefab.Modules[index] is ModuleEngines passport)
                memory.PassportThrust = passport.maxThrust;
            Log.Info($"thrust cap: watching {e.part.partInfo?.title} ({e.engineID}) on {e.vessel?.vesselName}, passport thrust {memory.PassportThrust:F1} kN" +
                     (memory.Threshold == null ? ", no pressureThreshold field (atmosphere not accounted)" : string.Empty));
        }

        /// <summary>KSPIE's thrust loss to the outside pressure, kN (0 in vacuum or when unknown).</summary>
        private static double PressureThreshold(Memory memory)
        {
            if (memory.Threshold == null)
                return 0;
            object value = memory.Threshold.GetValue(memory.Controller);
            double threshold = value is double d ? d : value is float f ? f : 0;
            return threshold > 0 && !double.IsInfinity(threshold) ? threshold : 0;
        }

        private static void Learn(ModuleEngines e, Memory memory, double vacuumThrust)
        {
            double previous = memory.LearnedVacuumThrust;
            memory.LearnedVacuumThrust = vacuumThrust;
            if (previous <= 0 || Math.Abs(vacuumThrust - previous) > 0.05 * previous)
                Log.Info($"thrust cap: {e.part.partInfo?.title} on {e.vessel?.vesselName} measured at throttle {e.currentThrottle:F2}: " +
                         $"full thrust {vacuumThrust:F1} kN in vacuum ({e.part.staticPressureAtm:F3} atm now)");
        }
    }
}
