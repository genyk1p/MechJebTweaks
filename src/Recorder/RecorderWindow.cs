using KSP.Localization;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.Recorder
{
    /// <summary>"Flight recorder" window in MechJeb's window list: switches <see cref="FlightRecorder"/> on and off.</summary>
    public class RecorderWindow : DisplayModule
    {
        public RecorderWindow(MechJebCore core) : base(core)
        {
            Hidden = !Settings.RecorderEnabled;
        }

        public override string GetName() => Localizer.Format("#LOC_MJTweaks_REC_Title");

        public override string IconName() => "Flight Recorder";

        protected override bool IsSpaceCenterUpgradeUnlocked() => Settings.TechUnlocked(Settings.RecorderUnlockTech);

        protected override GUILayoutOption[] WindowOptions() => new[] { GUILayout.Width(260f), GUILayout.Height(60f) };

        protected override void WindowGUI(int windowID)
        {
            GUILayout.BeginVertical();
            FlightRecorder.Recording = GUILayout.Toggle(FlightRecorder.Recording, Localizer.Format("#LOC_MJTweaks_REC_Record"));
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_REC_Hint", Settings.RecorderLogInterval.ToString("0.#")));
            GUILayout.EndVertical();
            base.WindowGUI(windowID);
        }
    }
}
