using System.Collections.Generic;
using KSP.Localization;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.ControlCheck
{
    /// <summary>
    /// "RCS and reaction wheel check" window in MechJeb's window list, in the editor (the ship being
    /// built) and in flight (the active vessel): reaction wheel turn times, RCS acceleration and
    /// approach speeds, RCS delta-v, each with a grade and how much is missing.
    /// </summary>
    public class ControlCheckWindow : DisplayModule
    {
        private static GUIStyle _red, _yellow, _green, _wrap;

        private ControlCheck _check;
        private float _nextMeasure;

        public ControlCheckWindow(MechJebCore core) : base(core)
        {
            Hidden = !Settings.ControlCheckEnabled;
            ShowInEditor = true;
            ShowInFlight = true;
        }

        public override string GetName() => Localizer.Format("#LOC_MJTweaks_CC_Title");

        public override string IconName() => "RCS and reaction wheel check";

        protected override GUILayoutOption[] WindowOptions() => new[] { GUILayout.Width(340f), GUILayout.Height(150f) };

        protected override void WindowGUI(int windowID)
        {
            InitStyles();
            GUILayout.BeginVertical();

            // Walks every part; twice a second is plenty, also while parts are being added in the editor.
            if (_check == null || Time.unscaledTime >= _nextMeasure)
            {
                _check = Measure();
                _nextMeasure = Time.unscaledTime + 0.5f;
            }

            if (_check == null || _check.Mass <= 0)
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_NoShip"));
            else
                Draw(_check);

            GUILayout.EndVertical();
            base.WindowGUI(windowID);
        }

        /// <summary>Editor: the ship being built, nose = root part's up. Flight: the active vessel and its control point.</summary>
        private ControlCheck Measure()
        {
            if (HighLogic.LoadedSceneIsEditor)
            {
                List<Part> parts = EditorLogic.fetch?.ship?.parts;
                Part root = EditorLogic.RootPart;
                return root == null ? null : ControlCheck.Compute(parts, root.transform);
            }
            return Vessel == null ? null : ControlCheck.Compute(Vessel.parts, Vessel.ReferenceTransform);
        }

        private void Draw(ControlCheck c)
        {
            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_CC_Mass"), $"{c.Mass:F2} t");
            if (HighLogic.LoadedSceneIsEditor)
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_EditorNose"), _wrap);

            // ---- reaction wheels ----
            GUILayout.Space(6);
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_Wheels"));
            if (c.WheelGrade == Grade.None)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_NoWheels"), _red);
            }
            else
            {
                GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_CC_Torque"),
                    $"{c.Torque.x:F1} / {c.Torque.z:F1} / {c.Torque.y:F1} kN·m");
                GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_CC_Turn90"),
                    $"{Seconds(c.PitchTime)} / {Seconds(c.YawTime)} / {Seconds(c.RollTime)} s");
                GradeLine(c.WheelGrade, WheelVerdict(c.WheelGrade));
                if (c.MissingPitchTorque > 0 || c.MissingYawTorque > 0)
                    GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_WheelsMissing",
                        c.MissingPitchTorque.ToString("F1"), c.MissingYawTorque.ToString("F1"), Settings.ControlTurnOk.ToString("F0")), _wrap);
            }

            // ---- RCS ----
            GUILayout.Space(6);
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_Rcs"));
            if (c.RcsGrade == Grade.None)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_NoRcs"), _red);
            }
            else
            {
                GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_CC_RcsAccel"),
                    $"{c.Rcs.Forward:F3} / {c.Rcs.Back:F3} / {c.Rcs.Lateral:F3} m/s²");
                GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_CC_Approach"),
                    $"{c.Rcs.FarSpeed:F2} / {c.Rcs.TouchSpeed:F2} m/s");
                GradeLine(c.RcsGrade, RcsVerdict(c));
                if (c.MissingRcsThrust > 0)
                    GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_RcsMissing",
                        c.MissingRcsThrust.ToString("F2"), Localizer.Format(c.RcsWeakestName), Settings.ControlRcsOk.ToString("F2")), _wrap);
            }

            // ---- RCS propellant ----
            if (c.RcsGrade != Grade.None)
            {
                GUILayout.Space(6);
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_Fuel"));
                if (c.FuelGrade == Grade.None)
                {
                    GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_NoFuelData"));
                }
                else
                {
                    GuiUtils.SimpleLabel(c.Propellants, $"{c.PropellantMass:F3} t");
                    GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_CC_RcsDv"), $"{c.RcsDeltaV:F0} m/s (Isp {c.Isp:F0} s)");
                    GradeLine(c.FuelGrade, FuelVerdict(c.FuelGrade));
                    if (c.SharedWithEngines)
                        GUILayout.Label(Localizer.Format("#LOC_MJTweaks_CC_FuelShared"), _wrap);
                }
            }
        }

        private static string WheelVerdict(Grade grade)
        {
            switch (grade)
            {
                case Grade.Good: return Localizer.Format("#LOC_MJTweaks_CC_WheelsGood");
                case Grade.Ok: return Localizer.Format("#LOC_MJTweaks_CC_WheelsOk");
                case Grade.Weak: return Localizer.Format("#LOC_MJTweaks_CC_WheelsSlow");
                default: return Localizer.Format("#LOC_MJTweaks_CC_WheelsBad", Settings.DockingMaxTurnTime90.ToString("F0"));
            }
        }

        private static string RcsVerdict(ControlCheck c)
        {
            switch (c.RcsGrade)
            {
                case Grade.Ok: return Localizer.Format("#LOC_MJTweaks_CC_RcsOk");
                case Grade.Weak: return Localizer.Format("#LOC_MJTweaks_CC_RcsWeak");
                case Grade.TooMuch: return Localizer.Format("#LOC_MJTweaks_CC_RcsStrong");
                default: return Localizer.Format("#LOC_MJTweaks_CC_RcsBad");
            }
        }

        private static string FuelVerdict(Grade grade)
        {
            switch (grade)
            {
                case Grade.Ok: return Localizer.Format("#LOC_MJTweaks_CC_FuelOk");
                case Grade.Weak: return Localizer.Format("#LOC_MJTweaks_CC_FuelTight");
                case Grade.TooMuch: return Localizer.Format("#LOC_MJTweaks_CC_FuelMuch", Settings.ControlRcsDvHigh.ToString("F0"));
                default: return Localizer.Format("#LOC_MJTweaks_CC_FuelBad");
            }
        }

        private void GradeLine(Grade grade, string text)
        {
            GUIStyle style = grade == Grade.Good || grade == Grade.Ok ? _green
                : grade == Grade.Bad ? _red
                : _yellow;
            GUILayout.Label(text, style);
        }

        private static string Seconds(double s) => double.IsInfinity(s) ? "—" : s.ToString("F0");

        private static void InitStyles()
        {
            if (_red != null)
                return;

            _red = new GUIStyle(GUI.skin.label) { wordWrap = true, normal = { textColor = new Color(1f, 0.35f, 0.3f) } };
            _yellow = new GUIStyle(GUI.skin.label) { wordWrap = true, normal = { textColor = new Color(1f, 0.85f, 0.3f) } };
            _green = new GUIStyle(GUI.skin.label) { wordWrap = true, normal = { textColor = new Color(0.4f, 1f, 0.4f) } };
            _wrap = new GUIStyle(GUI.skin.label) { wordWrap = true };
        }
    }
}
