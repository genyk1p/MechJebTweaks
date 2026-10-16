using System;
using System.Collections.Generic;
using System.Linq;
using FinePrint;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.Landing
{
    /// <summary>
    /// Puts the game's waypoints — custom ones (made in SCANsat or on the stock map) and contract
    /// ones — into the landing site list of MechJeb's landing guidance window. MechJeb shows that
    /// list as a drop-down with a "Set" button whenever the current body has any site; it only knew
    /// launch sites and its own config, so over the Mun the drop-down never appeared.
    ///
    /// The list is MechJeb's public static MechJebModuleLandingGuidance.LandingSites; only entries
    /// added here are ever removed again. MechJeb builds the list itself when its window starts,
    /// so nothing is added before that.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class WaypointSites : MonoBehaviour
    {
        /// <summary>Sites added by this class, kept until the waypoints change.</summary>
        private static readonly List<MechJebModuleLandingGuidance.LandingSite> Added = new List<MechJebModuleLandingGuidance.LandingSite>();

        /// <summary>The waypoint set the list was last built from; the list is only touched when it changes,
        /// so MechJeb's selected entry does not jump around.</summary>
        private static string _signature = string.Empty;

        private static bool _failed;

        private void Start()
        {
            if (Settings.WaypointSitesEnabled)
                InvokeRepeating(nameof(Sync), 1f, 2f);
        }

        private void Sync()
        {
            if (_failed)
                return;
            try
            {
                List<MechJebModuleLandingGuidance.LandingSite> sites = MechJebModuleLandingGuidance.LandingSites;
                WaypointManager manager = WaypointManager.Instance();
                if (sites == null || manager == null)
                    return;

                List<MechJebModuleLandingGuidance.LandingSite> wanted = Collect(manager.Waypoints);
                string signature = string.Join("|", wanted.Select(s => $"{s.Name};{s.Body.bodyName};{s.Latitude:F6};{s.Longitude:F6}"));
                if (signature == _signature)
                    return;

                foreach (MechJebModuleLandingGuidance.LandingSite site in Added)
                    sites.Remove(site);
                Added.Clear();

                foreach (MechJebModuleLandingGuidance.LandingSite site in wanted)
                {
                    // MechJeb's own sites (launch sites, its config) keep their name.
                    if (sites.Any(s => s.Name == site.Name && s.Body == site.Body))
                        continue;
                    sites.Add(site);
                    Added.Add(site);
                }

                _signature = signature;
                Log.Info($"landing sites from waypoints: {Added.Count} ({string.Join(", ", Added.Select(s => $"{s.Name} @ {s.Body.bodyName}"))})");
            }
            catch (Exception e)
            {
                _failed = true;
                Log.Error($"waypoint landing sites failed, MechJeb's list left as is: {e}");
            }
        }

        private static List<MechJebModuleLandingGuidance.LandingSite> Collect(IEnumerable<Waypoint> waypoints)
        {
            var result = new List<MechJebModuleLandingGuidance.LandingSite>();
            if (waypoints == null)
                return result;

            foreach (Waypoint w in waypoints)
            {
                if (w == null || string.IsNullOrEmpty(w.celestialName))
                    continue;
                CelestialBody body = FlightGlobals.Bodies.Find(b => b.bodyName == w.celestialName);
                if (body == null)
                    continue;

                // Contract waypoints share a name ("Site") and differ by FullName ("Site Alpha").
                string name = string.IsNullOrEmpty(w.FullName) ? w.name : w.FullName;
                if (string.IsNullOrEmpty(name) || result.Any(s => s.Name == name && s.Body == body))
                    continue;

                result.Add(new MechJebModuleLandingGuidance.LandingSite
                {
                    Name = name,
                    Body = body,
                    Latitude = w.latitude,
                    Longitude = w.longitude
                });
            }
            return result.OrderBy(s => s.Name, StringComparer.CurrentCulture).ToList();
        }
    }
}
