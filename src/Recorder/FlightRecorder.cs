using System;
using System.Collections.Generic;
using System.Linq;
using MechJebTweaks.Docking;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.Recorder
{
    /// <summary>
    /// Flight recorder for debugging MechJeb's autopilots (and this mod's own): while switched on in
    /// the "Flight recorder" window it writes to KSP.log a status line every LogInterval real seconds
    /// plus events — warp, vessel switches, situation and SOI changes, staging, SAS mode and RCS on
    /// either vessel, target changes, docking/undocking, MechJeb modules switching on or off.
    /// Lines start with "[MechJebTweaks] rec:"; near a loaded target vessel a docking line
    /// ("[MechJebTweaks] docking: d=...") follows each status line.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class FlightRecorder : MonoBehaviour
    {
        /// <summary>Switched from the recorder window. Also turns on the docking autopilot's detail lines.</summary>
        public static bool Recording;

        private const float ModulePollInterval = 0.5f;

        private bool _wasRecording;
        private float _nextLine;
        private float _nextModulePoll;

        private Vessel _lastActive;
        private Vessel _lastOther;
        private string _lastActiveSas;
        private string _lastOtherSas;
        private bool _lastRcs;
        private string _lastTarget;
        private HashSet<string> _lastModules = new HashSet<string>();

        private float _maxTranslation;
        private double _lastPropellant = double.NaN;

        private float _lastWarpRate = -1f;
        private TimeWarp.Modes _lastWarpMode;

        private void OnEnable()
        {
            GameEvents.onTimeWarpRateChanged.Add(OnWarpChanged);
            GameEvents.onVesselChange.Add(OnVesselChange);
            GameEvents.onPartCouple.Add(OnPartCouple);
            GameEvents.onPartUndock.Add(OnPartUndock);
            GameEvents.onVesselSituationChange.Add(OnSituationChange);
            GameEvents.onVesselSOIChanged.Add(OnSoiChange);
            GameEvents.onStageActivate.Add(OnStage);
        }

        private void OnDisable()
        {
            GameEvents.onTimeWarpRateChanged.Remove(OnWarpChanged);
            GameEvents.onVesselChange.Remove(OnVesselChange);
            GameEvents.onPartCouple.Remove(OnPartCouple);
            GameEvents.onPartUndock.Remove(OnPartUndock);
            GameEvents.onVesselSituationChange.Remove(OnSituationChange);
            GameEvents.onVesselSOIChanged.Remove(OnSoiChange);
            GameEvents.onStageActivate.Remove(OnStage);
        }

        private void Update()
        {
            if (Recording != _wasRecording)
            {
                _wasRecording = Recording;
                Write(Recording ? "recording started" : "recording stopped");
                _lastPropellant = double.NaN;
                _lastModules.Clear();
                _lastActive = null;
            }

            Vessel active = FlightGlobals.ActiveVessel;
            if (!Recording || active == null)
                return;

            PollChanges(active);

            if (Time.unscaledTime >= _nextModulePoll)
            {
                _nextModulePoll = Time.unscaledTime + ModulePollInterval;
                PollMechJebModules(active);
            }

            _maxTranslation = Math.Max(_maxTranslation, Math.Abs(active.ctrlState.X));
            _maxTranslation = Math.Max(_maxTranslation, Math.Abs(active.ctrlState.Y));
            _maxTranslation = Math.Max(_maxTranslation, Math.Abs(active.ctrlState.Z));

            if (Time.unscaledTime >= _nextLine)
            {
                _nextLine = Time.unscaledTime + (float)Math.Max(0.1, Settings.RecorderLogInterval);
                WriteStatusLine(active);
                WriteDockingLine(active);
                _maxTranslation = 0;
            }
        }

        // ---------------------------------------------------------------- status lines

        private void WriteStatusLine(Vessel v)
        {
            string warp = TimeWarp.CurrentRate <= 1f ? "1x" : $"{TimeWarp.CurrentRate:0.#}x {(TimeWarp.WarpMode == TimeWarp.Modes.HIGH ? "rails" : "phys")}";
            double thrust = 0;
            foreach (ModuleEngines engine in v.FindPartModulesImplementing<ModuleEngines>())
                if (engine.EngineIgnited)
                    thrust += engine.finalThrust;

            string orbit = v.situation == Vessel.Situations.ORBITING || v.situation == Vessel.Situations.ESCAPING || v.situation == Vessel.Situations.SUB_ORBITAL
                ? $" Ap={v.orbit.ApA / 1000:F1}km Pe={v.orbit.PeA / 1000:F1}km"
                : string.Empty;

            // MechJeb's own idea of the available thrust (what its landing and burn planning use).
            MechJebCore core = v.GetMasterMechJeb();
            VesselState mjState = core?.VesselState;
            string mjThrust = mjState != null ? $" mjmax={mjState.thrustAvailable:F1}kN" : string.Empty;
            // MechJeb's aim and how fast the vessel turns: its node executor warps only below 1° and 0.001 rad/s.
            if (core != null && core.Attitude != null && core.Attitude.Enabled)
                mjThrust += $" | aim off={core.Attitude.attitudeAngleFromTarget():F2}° w={v.angularVelocity.magnitude:F4}rad/s";
            if (core?.Node != null && core.Node.Enabled)
                mjThrust += $" node={core.Node.State}";

            string line = $"{v.vesselName} {v.situation} {v.mainBody.bodyName} alt={v.altitude:F0}m{orbit} " +
                          $"v={v.obt_speed:F1} srf={v.srfSpeed:F1} vs={v.verticalSpeed:F1}m/s | warp {warp}{(v.packed ? " packed" : "")} | " +
                          $"SAS {SasText(v)} RCS {(v.ActionGroups[KSPActionGroup.RCS] ? "on" : "off")} in={_maxTranslation:F2} | " +
                          $"throttle={v.ctrlState.mainThrottle:F2} thrust={thrust:F1}kN{mjThrust} | mj=[{string.Join(",", _lastModules.OrderBy(n => n))}]";

            if (v.targetObject != null && v.ReferenceTransform != null && v.targetObject.GetTransform() != null)
            {
                Vessel other = v.targetObject.GetVessel();
                Vector3d toTarget = VesselDockingState.ToTarget(v, v.targetObject, v.ReferenceTransform);
                Vector3d relative = v.obt_velocity - v.targetObject.GetObtVelocity();
                Vector3d direction = toTarget.normalized;
                double closing = Vector3d.Dot(relative, direction);
                double lateral = (relative - closing * direction).magnitude;
                line += $" | target {v.targetObject.GetDisplayName()}: d={toTarget.magnitude:F1}m vc={closing:F3} vlat={lateral:F3}m/s" +
                        (other != null && other != v ? (other.loaded ? (other.packed ? " packed" : " physics") : " unloaded") : string.Empty);
            }

            Write(line);
        }

        /// <summary>Docking detail near a loaded target vessel: aiming of both vessels, RCS capability and use.</summary>
        private void WriteDockingLine(Vessel active)
        {
            Vessel other = active.targetObject?.GetVessel();
            if (other == null || other == active || !other.loaded)
                return;

            DockingPair pair = DockingPair.Measure(active);
            if (pair.Active.Distance > Settings.DockingMaxRange)
                return;

            VesselDockingState a = pair.Active;
            VesselDockingState b = pair.Other;
            double used = double.IsNaN(_lastPropellant) ? 0 : _lastPropellant - pair.RcsPropellantAmount;
            _lastPropellant = pair.RcsPropellantAmount;

            Log.Info($"docking: d={a.Distance:F1}m vc={a.ClosingSpeed:F3} vlat={a.LateralSpeed:F3}m/s | " +
                     $"A err={a.PointingError:F2}° w={a.AngularRate:F3}°/s {SasText(a)} | " +
                     $"B err={b.PointingError:F2}° w={b.AngularRate:F3}°/s {SasText(b)}{(other.packed ? " packed" : "")} | " +
                     $"{pair.RcsPropellant}={pair.RcsPropellantAmount:F2} (-{used:F3}) | parts2.5km={pair.PartsNearby} mass={pair.TotalMass:F1}t | " +
                     $"rcs fwd={pair.Rcs.Forward:F3} back={pair.Rcs.Back:F3} lat={pair.Rcs.Lateral:F3}m/s² vFar={pair.Rcs.FarSpeed:F2} vTouch={pair.Rcs.TouchSpeed:F2} brake={pair.Rcs.BrakingDistance(a.ClosingSpeed):F0}m");
        }

        // ---------------------------------------------------------------- polled changes

        private void PollChanges(Vessel active)
        {
            Vessel other = active.targetObject?.GetVessel();
            if (other == active)
                other = null;

            string activeSas = SasText(active);
            if (active == _lastActive && activeSas != _lastActiveSas)
                Write($"SAS {active.vesselName}: {_lastActiveSas} -> {activeSas}");

            string otherSas = other != null && other.loaded ? SasText(other) : "not loaded";
            if (other != null && other == _lastOther && otherSas != _lastOtherSas)
                Write($"SAS {other.vesselName}: {_lastOtherSas} -> {otherSas}");

            bool rcs = active.ActionGroups[KSPActionGroup.RCS];
            if (active == _lastActive && rcs != _lastRcs)
                Write($"RCS {(rcs ? "on" : "off")}");

            string target = active.targetObject?.GetDisplayName() ?? "none";
            if (active == _lastActive && target != _lastTarget)
                Write($"target of {active.vesselName}: {_lastTarget} -> {target}");

            _lastActive = active;
            _lastOther = other;
            _lastActiveSas = activeSas;
            _lastOtherSas = otherSas;
            _lastRcs = rcs;
            _lastTarget = target;
        }

        /// <summary>MechJeb computer modules (autopilots, controllers) that are running, by class name.</summary>
        private void PollMechJebModules(Vessel active)
        {
            var now = new HashSet<string>();
            MechJebCore core = active.GetMasterMechJeb();
            if (core != null)
            {
                foreach (ComputerModule module in core.GetComputerModules<ComputerModule>())
                    if (!(module is DisplayModule) && module.Enabled)
                        now.Add(ShortName(module));
            }

            var started = now.Where(n => !_lastModules.Contains(n)).OrderBy(n => n).ToList();
            var stopped = _lastModules.Where(n => !now.Contains(n)).OrderBy(n => n).ToList();
            if (started.Count > 0 || stopped.Count > 0)
                Write("MechJeb modules:" + string.Concat(started.Select(n => " +" + n)) + string.Concat(stopped.Select(n => " -" + n)));
            _lastModules = now;
        }

        private static string ShortName(ComputerModule module)
        {
            string name = module.GetType().Name;
            return name.StartsWith("MechJebModule") ? name.Substring("MechJebModule".Length) : name;
        }

        // ---------------------------------------------------------------- events

        private void OnWarpChanged()
        {
            // KSP fires this event twice per change; log each actual change once.
            if (TimeWarp.CurrentRate == _lastWarpRate && TimeWarp.WarpMode == _lastWarpMode)
                return;
            _lastWarpRate = TimeWarp.CurrentRate;
            _lastWarpMode = TimeWarp.WarpMode;

            if (Recording)
                Write($"warp -> {TimeWarp.CurrentRate:0.##}x {(TimeWarp.WarpMode == TimeWarp.Modes.HIGH ? "rails" : "physics")}");
        }

        private void OnVesselChange(Vessel vessel)
        {
            if (!Recording)
                return;

            Vessel other = vessel.targetObject?.GetVessel();
            Write($"switched to {vessel.vesselName}: SAS {SasText(vessel)}" +
                  (other != null && other != vessel && other.loaded ? $"; {other.vesselName}: SAS {SasText(other)}" : string.Empty));
        }

        private void OnPartCouple(GameEvents.FromToAction<Part, Part> action)
        {
            if (Recording)
                Write($"DOCKED: {action.from?.vessel?.vesselName} -> {action.to?.vessel?.vesselName}");
        }

        private void OnPartUndock(Part part)
        {
            if (Recording)
                Write($"UNDOCKED: {part?.vessel?.vesselName} at {part?.partInfo?.title}");
        }

        private void OnSituationChange(GameEvents.HostedFromToAction<Vessel, Vessel.Situations> change)
        {
            if (Recording && change.host == FlightGlobals.ActiveVessel)
                Write($"situation of {change.host.vesselName}: {change.from} -> {change.to}");
        }

        private void OnSoiChange(GameEvents.HostedFromToAction<Vessel, CelestialBody> change)
        {
            if (Recording && change.host == FlightGlobals.ActiveVessel)
                Write($"SOI of {change.host.vesselName}: {change.from?.bodyName} -> {change.to?.bodyName}");
        }

        private void OnStage(int stage)
        {
            if (Recording)
                Write($"stage {stage} activated");
        }

        // ---------------------------------------------------------------- helpers

        private static string SasText(Vessel vessel) =>
            vessel.ActionGroups[KSPActionGroup.SAS] ? vessel.Autopilot.Mode.ToString() : "SAS off";

        private static string SasText(VesselDockingState state) =>
            state.SasOn ? state.SasMode.ToString() : "SAS off";

        private static void Write(string message) => Log.Info("rec: " + message);
    }
}
