using MechJebTweaks.WarpGuard;
using UnityEngine;

namespace MechJebTweaks
{
    /// <summary>Loads settings and installs the Harmony patch once the game database is ready.</summary>
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class Loader : MonoBehaviour
    {
        private void Start()
        {
            Settings.Load();

            if (Settings.WarpGuardEnabled)
                WarpPatch.Apply();
            else
                Log.Info("warp guard disabled in Settings.cfg");

            if (Settings.LandingWarpEnabled)
                Landing.LandingWarpPatch.Apply();
            else
                Log.Info("landing warp fix disabled in Settings.cfg");

            if (Settings.ThrustCapEnabled)
                Landing.ThrustCapPatch.Apply();
            else
                Log.Info("thrust cap disabled in Settings.cfg");

            try
            {
                PhysicsWarp.AutowarpRowPatch.Apply();
            }
            catch (System.Exception e)
            {
                Log.Warning($"auto physics warp: no row in MechJeb's windows: {e}");
            }
        }
    }

    /// <summary>Samples the active vessel's engines every physics frame while in flight.</summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class FlightMonitor : MonoBehaviour
    {
        private void OnEnable()
        {
            GameEvents.onVesselWasModified.Add(OnVesselEvent);
            GameEvents.onVesselChange.Add(OnVesselEvent);
            ThrustTracker.Invalidate();
        }

        private void OnDisable()
        {
            GameEvents.onVesselWasModified.Remove(OnVesselEvent);
            GameEvents.onVesselChange.Remove(OnVesselEvent);
        }

        private void FixedUpdate()
        {
            if (Settings.WarpGuardEnabled)
                ThrustTracker.Sample(FlightGlobals.ActiveVessel);
        }

        private void OnVesselEvent(Vessel vessel) => ThrustTracker.Invalidate();
    }
}
