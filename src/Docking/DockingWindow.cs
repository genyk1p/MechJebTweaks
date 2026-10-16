using KSP.Localization;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.Docking
{
    /// <summary>
    /// "Docking" window in MechJeb's window list: target, distance and closing speed, the docking
    /// autopilot (what stands in the way of starting, start/stop, its state), and a "Details"
    /// section with both vessels, all start conditions, RCS speeds and the SAS target hold buttons.
    /// </summary>
    public class DockingWindow : DisplayModule
    {
        private static GUIStyle _red;
        private static GUIStyle _green;
        private static GUIStyle _hint;

        private DockingPair _pair;
        private float _nextMeasure;
        private string _buttonNote = string.Empty;

        public DockingWindow(MechJebCore core) : base(core)
        {
            Hidden = !Settings.DockingEnabled;
        }

        public override string GetName() => Localizer.Format("#LOC_MJTweaks_DK_Title");

        public override string IconName() => "Docking Assist";

        // MechJeb hides a window whose unlock check fails (career/science; sandbox has everything).
        protected override bool IsSpaceCenterUpgradeUnlocked() => Settings.TechUnlocked(Settings.DockingUnlockTech);

        protected override GUILayoutOption[] WindowOptions() => new[] { GUILayout.Width(340f), GUILayout.Height(150f) };

        /// <summary>The "Details" section is open (remembered between flights).</summary>
        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool ShowDetails;

        protected override void WindowGUI(int windowID)
        {
            InitStyles();
            GUILayout.BeginVertical();

            // How to prepare both vessels; hidden while the autopilot runs, its status matters more then.
            DockingAutopilot running = Core.GetComputerModule<DockingAutopilot>();
            if (running == null || !running.Enabled)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_HowTo"), _hint);
                GUILayout.Space(4);
            }

            if (Vessel.targetObject == null)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_NoTarget"));
                GUILayout.EndVertical();
                base.WindowGUI(windowID);
                return;
            }

            // Measuring walks every part of both vessels; once per 0.2 s is plenty for a status window.
            if (_pair == null || Time.unscaledTime >= _nextMeasure)
            {
                _pair = DockingPair.Measure(Vessel);
                _nextMeasure = Time.unscaledTime + 0.2f;
            }
            DockingPair pair = _pair;
            VesselDockingState a = pair.Active;

            // Always on top: target, distance and closing speed; then the autopilot (state, start/stop).
            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_DK_Target"), Vessel.targetObject.GetDisplayName());
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_Summary",
                a.Distance < 100 ? a.Distance.ToString("F1") : a.Distance.ToString("F0"), a.ClosingSpeed.ToString("F2")));

            DrawAutopilot(pair);

            GUILayout.Space(6);
            if (GUILayout.Button((ShowDetails ? "▾ " : "▸ ") + Localizer.Format("#LOC_MJTweaks_DK_Details")))
                ShowDetails = !ShowDetails;
            if (ShowDetails)
                DrawDetails(pair);

            GUILayout.EndVertical();
            base.WindowGUI(windowID);
        }

        /// <summary>Everything for manual docking and debugging: both vessels, all conditions, RCS speeds, buttons.</summary>
        private void DrawDetails(DockingPair pair)
        {
            VesselDockingState a = pair.Active;
            VesselDockingState b = pair.Other;
            bool otherLoaded = pair.OtherVessel != null && pair.OtherVessel.loaded;

            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_DK_Lateral"), $"{a.LateralSpeed:F2} m/s");

            GUILayout.Space(4);
            DrawVesselRow(Localizer.Format("#LOC_MJTweaks_DK_Ours"), a, true);
            DrawVesselRow(Localizer.Format("#LOC_MJTweaks_DK_Theirs"), b, otherLoaded);

            GUILayout.Space(4);
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_Conditions"));
            Condition(pair.TargetsOk, Localizer.Format("#LOC_MJTweaks_DK_CondMutual"));
            Condition(pair.Loaded, Localizer.Format("#LOC_MJTweaks_DK_CondRange", Settings.DockingMaxRange.ToString("F0")));
            Condition(pair.TargetHoldOk, Localizer.Format("#LOC_MJTweaks_DK_CondHold", YesNo(a.TargetHoldAvailable && a.Controllable), YesNo(otherLoaded && b.TargetHoldAvailable && b.Controllable)));
            Condition(pair.RotationOk, Localizer.Format("#LOC_MJTweaks_DK_CondTurn", Seconds(a.TurnTime90), otherLoaded ? Seconds(b.TurnTime90) : "—", Settings.DockingMaxTurnTime90.ToString("F0")));
            Condition(pair.RcsOk, Localizer.Format("#LOC_MJTweaks_DK_CondRcs", pair.RcsBlocks, pair.RcsPropellant.Length > 0 ? pair.RcsPropellant : "—", pair.RcsPropellantAmount.ToString("F1")));

            // Not a start condition: in orbit KSP only simulates the other vessel within ~200 m, it aims itself once we get there.
            if (otherLoaded)
                GUILayout.Label(pair.Simulated
                    ? Localizer.Format("#LOC_MJTweaks_DK_Simulated")
                    : Localizer.Format("#LOC_MJTweaks_DK_Packed", pair.UnpackRange.ToString("F0")),
                    pair.Simulated ? _green : GUI.skin.label);

            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_DK_PartsNearby", Settings.DockingMaxRange.ToString("F0")),
                pair.PartsNearby + " -> " + (pair.WarpLimit > 1 ? Localizer.Format("#LOC_MJTweaks_DK_WarpUpTo", pair.WarpLimit) : Localizer.Format("#LOC_MJTweaks_DK_NoWarp")));
            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_DK_Mass"), $"{pair.TotalMass:F1} t");

            DrawSpeeds(pair);

            GUILayout.Space(4);
            if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_DK_PointBoth")))
            {
                DockingTelemetry.EngageTargetHold(Vessel, "button");
                if (otherLoaded && !pair.OtherVessel.packed)
                {
                    DockingTelemetry.EngageTargetHold(pair.OtherVessel, "button");
                    _buttonNote = string.Empty;
                }
                else
                {
                    _buttonNote = Localizer.Format("#LOC_MJTweaks_DK_PointPacked", pair.UnpackRange.ToString("F0"));
                    DockingTelemetry.Note($"button: {pair.OtherVessel?.vesselName} is packed by KSP (farther than ~{pair.UnpackRange:F0} m), cannot turn it yet");
                }
            }
            if (!string.IsNullOrEmpty(_buttonNote) && !pair.Simulated)
                GUILayout.Label(_buttonNote);

            DockingTelemetry.KeepTargetHold = GUILayout.Toggle(DockingTelemetry.KeepTargetHold, Localizer.Format("#LOC_MJTweaks_DK_KeepHold"));
        }

        private void DrawAutopilot(DockingPair pair)
        {
            DockingAutopilot autopilot = Core.GetComputerModule<DockingAutopilot>();
            if (autopilot == null)
                return;

            GUILayout.Space(6);
            if (autopilot.Enabled)
            {
                string phase = Localizer.Format(autopilot.Status);
                string next = autopilot.NextCheckpoint > 0 ? $"{autopilot.NextCheckpoint:F0} m" : Localizer.Format("#LOC_MJTweaks_DK_AP_TheTarget");
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_AP_StatusLine", phase, next, autopilot.LegSpeed.ToString("F2")), _green);
                if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_DK_AP_StopButton"), GUILayout.Height(30f)))
                    autopilot.Stop("#LOC_MJTweaks_DK_AP_Stopped");
                return;
            }

            if (autopilot.State == DockingAutopilot.Phase.Docked || autopilot.State == DockingAutopilot.Phase.Aborted)
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_AP_LastResult", Localizer.Format(autopilot.Status)),
                    autopilot.State == DockingAutopilot.Phase.Docked ? _green : _red);

            // Only what stands in the way, each with what to do about it; one green line when nothing does.
            string reason = DockingAutopilot.CannotStartReason(pair);
            if (!pair.AllConditionsOk)
            {
                VesselDockingState a = pair.Active;
                VesselDockingState b = pair.Other;
                bool otherLoaded = pair.OtherVessel != null && pair.OtherVessel.loaded;
                if (!pair.TargetsOk)
                    Blocker(Localizer.Format("#LOC_MJTweaks_DK_CondMutual"), Localizer.Format("#LOC_MJTweaks_DK_HintMutual"));
                if (!pair.Loaded)
                    Blocker(Localizer.Format("#LOC_MJTweaks_DK_CondRange", Settings.DockingMaxRange.ToString("F0")),
                        Localizer.Format("#LOC_MJTweaks_DK_HintRange", Settings.DockingMaxRange.ToString("F0")));
                if (!pair.TargetHoldOk)
                    Blocker(Localizer.Format("#LOC_MJTweaks_DK_CondHold", YesNo(a.TargetHoldAvailable && a.Controllable), YesNo(otherLoaded && b.TargetHoldAvailable && b.Controllable)),
                        Localizer.Format("#LOC_MJTweaks_DK_HintHold"));
                if (!pair.RotationOk)
                    Blocker(Localizer.Format("#LOC_MJTweaks_DK_CondTurn", Seconds(a.TurnTime90), otherLoaded ? Seconds(b.TurnTime90) : "—", Settings.DockingMaxTurnTime90.ToString("F0")),
                        Localizer.Format("#LOC_MJTweaks_DK_HintTurn"));
                if (!pair.RcsOk)
                    Blocker(Localizer.Format("#LOC_MJTweaks_DK_CondRcs", pair.RcsBlocks, pair.RcsPropellant.Length > 0 ? pair.RcsPropellant : "—", pair.RcsPropellantAmount.ToString("F1")),
                        Localizer.Format("#LOC_MJTweaks_DK_HintRcs"));
            }
            else if (reason != null)
                GUILayout.Label(Localizer.Format(reason, Settings.DockingFarStartMax.ToString("F0")), _red);
            else
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_Ready"), _green);

            GUI.enabled = reason == null;
            if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_DK_AP_StartButton"), GUILayout.Height(30f)))
                autopilot.Start(this);
            GUI.enabled = true;
        }

        private static void Blocker(string condition, string hint)
        {
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_No") + "  " + condition, _red);
            GUILayout.Label("    " + hint);
        }

        /// <summary>RCS acceleration and the approach speeds computed from it — a hint for manual docking.</summary>
        private void DrawSpeeds(DockingPair pair)
        {
            RcsCapability rcs = pair.Rcs;
            GUILayout.Space(4);
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_SpeedsHeader"));

            if (!rcs.Any)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_NoRcsThrust"), _red);
                return;
            }

            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_DK_RcsAccel"),
                $"{rcs.Forward:F3} / {rcs.Back:F3} / {rcs.Lateral:F3} m/s²");

            double far = rcs.FarSpeed;
            double touch = rcs.TouchSpeed;
            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_DK_FarSpeed"), $"{far:F2} m/s");
            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_DK_TouchSpeed"), $"{touch:F2} m/s");

            double distance = pair.Active.Distance;
            double closing = pair.Active.ClosingSpeed;
            if (closing > far + 0.05)
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_TooFast", closing.ToString("F2"), far.ToString("F2")), _red);

            // Where to start braking to the touchdown speed: before the 100 m checkpoint while farther,
            // before the target itself once inside it.
            double brake = rcs.BrakingDistance(closing);
            if (brake > 0)
            {
                double checkpoint = distance > Settings.DockingBrakeDistance ? Settings.DockingBrakeDistance : 0;
                double startAt = checkpoint + brake;
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_BrakeAt", touch.ToString("F2"), startAt.ToString("F0")),
                    distance <= startAt ? _red : GUI.skin.label);
            }

            if (rcs.Lateral > 0 && pair.Active.LateralSpeed > 0.01)
                GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_DK_LateralTime"),
                    $"{pair.Active.LateralSpeed / rcs.Lateral:F0} s");
        }

        private void DrawVesselRow(string label, VesselDockingState state, bool loaded)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(110f));
            if (!loaded)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_NotLoaded"), _red);
            }
            else
            {
                GUIStyle aim = state.PointingError <= Settings.DockingAlignedAngle ? _green
                    : state.PointingError <= Settings.DockingRoughAngle ? GUI.skin.label : _red;
                GUILayout.Label($"{state.PointingError:F2}°", aim, GUILayout.Width(60f));
                GUILayout.Label($"{state.AngularRate:F2}°/s", GUILayout.Width(65f));
                GUILayout.Label(state.SasOn ? SasModeName(state.SasMode) : Localizer.Format("#LOC_MJTweaks_DK_SasOff"),
                    state.SasOn && state.SasMode == VesselAutopilot.AutopilotMode.Target ? _green : GUI.skin.label);
                if (state.Vessel != null && state.Vessel.packed)
                    GUILayout.Label(Localizer.Format("#LOC_MJTweaks_DK_FrozenTag"));
            }
            GUILayout.EndHorizontal();
        }

        private static string SasModeName(VesselAutopilot.AutopilotMode mode) =>
            mode == VesselAutopilot.AutopilotMode.Target ? Localizer.Format("#LOC_MJTweaks_DK_SasTarget")
            : mode == VesselAutopilot.AutopilotMode.StabilityAssist ? Localizer.Format("#LOC_MJTweaks_DK_SasHold")
            : mode.ToString();

        private static void Condition(bool ok, string text) =>
            GUILayout.Label(Localizer.Format(ok ? "#LOC_MJTweaks_DK_Ok" : "#LOC_MJTweaks_DK_No") + "  " + text, ok ? _green : _red);

        private static string YesNo(bool ok) => Localizer.Format(ok ? "#LOC_MJTweaks_DK_Ok" : "#LOC_MJTweaks_DK_No");

        private static string Seconds(double s) => double.IsInfinity(s) ? "—" : $"{s:F0}";

        private static void InitStyles()
        {
            if (_red != null)
                return;

            _red = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 0.35f, 0.3f) } };
            _green = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(0.4f, 1f, 0.4f) } };
            _hint = new GUIStyle(GUI.skin.label) { wordWrap = true, normal = { textColor = new Color(0.75f, 0.75f, 0.75f) } };
        }
    }
}
