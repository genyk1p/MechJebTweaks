using System;
using MechJebTweaks.Recorder;
using UnityEngine;

namespace MechJebTweaks.Docking
{
    /// <summary>
    /// Keeps SAS target hold engaged on both vessels on request (docking window checkbox) and writes
    /// the docking lines of the window and the autopilot to KSP.log. The flight recording itself
    /// (status lines and events) moved to <see cref="FlightRecorder"/>.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class DockingTelemetry : MonoBehaviour
    {
        /// <summary>Re-engage SAS target hold on both vessels whenever it drops (window checkbox).</summary>
        public static bool KeepTargetHold;

        private const float ReengageInterval = 0.5f;

        private float _nextReengage;

        private void OnEnable()
        {
            GameEvents.onPartCouple.Add(OnPartCouple);
        }

        private void OnDisable()
        {
            GameEvents.onPartCouple.Remove(OnPartCouple);
        }

        private void Update()
        {
            Vessel active = FlightGlobals.ActiveVessel;
            if (active == null || !KeepTargetHold || Time.unscaledTime < _nextReengage)
                return;

            _nextReengage = Time.unscaledTime + ReengageInterval;
            EngageTargetHold(active, "keep");
            Vessel other = active.targetObject?.GetVessel();
            if (other != null && other != active && other.loaded && !other.packed)
                EngageTargetHold(other, "keep");
        }

        /// <summary>
        /// Turns SAS on and selects target hold on a loaded vessel that has a target.
        /// Returns true if the mode had to be (re)selected.
        /// </summary>
        public static bool EngageTargetHold(Vessel vessel, string reason)
        {
            if (vessel == null || vessel.targetObject == null || vessel.Autopilot == null)
                return false;

            bool sasOn = vessel.ActionGroups[KSPActionGroup.SAS];
            if (sasOn && vessel.Autopilot.Mode == VesselAutopilot.AutopilotMode.Target)
                return false;

            VesselAutopilot.AutopilotMode previous = vessel.Autopilot.Mode;
            if (!sasOn)
                vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
            bool ok = vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.Target);

            // A refusal repeats every re-engage tick; log it at most once a minute per vessel.
            if (ok || !_lastRefusalLog.TryGetValue(vessel.id, out float last) || Time.unscaledTime - last > 60f)
            {
                if (!ok)
                    _lastRefusalLog[vessel.id] = Time.unscaledTime;
                Write($"{reason}: target hold on {vessel.vesselName} " +
                      (ok ? $"engaged (was SAS {(sasOn ? previous.ToString() : "off")})" : "refused by KSP (no SAS level 3 or no target)"));
            }
            return ok;
        }

        private static readonly System.Collections.Generic.Dictionary<Guid, float> _lastRefusalLog =
            new System.Collections.Generic.Dictionary<Guid, float>();

        private void OnPartCouple(GameEvents.FromToAction<Part, Part> action)
        {
            // The docking is over: switch "keep target hold" off. Left on, it turned the station to
            // the next vessel the player merely selected as target. The flight recorder is a general
            // tool and stays as the player set it.
            if (KeepTargetHold)
                Write("docked: target hold keeping switched off");
            KeepTargetHold = false;
        }

        private static void Write(string message) => Log.Info("docking: " + message);

        /// <summary>Writes a docking line regardless of the recorder (window actions, autopilot events).</summary>
        public static void Note(string message) => Write(message);
    }
}
