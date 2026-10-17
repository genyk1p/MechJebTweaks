using System;
using System.Collections.Generic;
using KSP.Localization;
using MechJebTweaks.PhysicsWarp;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.Rendezvous
{
    /// <summary>
    /// "Rendezvous with offset" window. MechJeb picks up every non-abstract ComputerModule subclass
    /// from all loaded assemblies, so this window shows up in MechJeb's own window list.
    /// </summary>
    public class RendezvousOffsetWindow : DisplayModule
    {
        [Persistent(pass = (int)Pass.GLOBAL)]
        public readonly EditableDouble OffsetDistance = 100.0;

        /// <summary>
        /// Altitude of the phasing orbit relative to the target's orbit (entered in km): positive above,
        /// negative below. MechJeb's rendezvous planner takes an altitude above the surface instead.
        /// </summary>
        [Persistent(pass = (int)Pass.GLOBAL)]
        public readonly EditableDoubleMult PhasingOffset = new EditableDoubleMult(200000, 1000);

        private string _status = string.Empty;
        private bool _statusIsError;

        private static GUIStyle _red;
        private static GUIStyle _green;

        private static GUIStyle _routeToggle;
        private static GUIStyle _routeToggleRed;
        private static GUIStyle _routeToggleGreen;

        private readonly List<SafeRendezvousAutopilot.Route> _routes = new List<SafeRendezvousAutopilot.Route>();
        private SafeRendezvousAutopilot.Route _chosenRoute;
        private float _routesUntil;
        /// <summary>The route the player picked in the list; null: the autopilot's own choice.</summary>
        private SafeRendezvousAutopilot.RouteKind? _pickedRoute;

        public RendezvousOffsetWindow(MechJebCore core) : base(core)
        {
            Hidden = !Settings.RendezvousOffsetEnabled;
        }

        public override string GetName() => Localizer.Format("#LOC_MJTweaks_RZ_Title");

        public override string IconName() => "Rendezvous Offset";

        protected override bool IsSpaceCenterUpgradeUnlocked() =>
            Vessel.patchedConicsUnlocked() && Settings.TechUnlocked(Settings.RendezvousOffsetUnlockTech);

        protected override GUILayoutOption[] WindowOptions() => new[] { GUILayout.Width(300f), GUILayout.Height(150f) };

        protected override void WindowGUI(int windowID)
        {
            InitStyles();

            if (!Core.Target.NormalTargetExists)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_NoTarget"));
                base.WindowGUI(windowID);
                return;
            }

            // Like MechJeb's maneuver planner: plan from the orbit after the last planned node,
            // so the window chains onto circularization / plane change nodes made elsewhere.
            List<ManeuverNode> nodes = Vessel.patchedConicSolver.maneuverNodes;
            bool chained = nodes.Count > 0;
            Orbit orbit = chained ? nodes[nodes.Count - 1].nextPatch : Orbit;
            double now = Planetarium.GetUniversalTime();
            double startUT = chained ? Math.Max(nodes[nodes.Count - 1].UT, now) : now;

            Orbit target = Core.Target.TargetOrbit;
            if (orbit == null || target.referenceBody != orbit.referenceBody)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_OtherSoi"));
                base.WindowGUI(windowID);
                return;
            }

            double closestUT = orbit.NextClosestApproachTime(target, startUT);
            double miss = orbit.Separation(target, closestUT);
            double distance = Math.Max(1.0, OffsetDistance.Val);

            GUILayout.BeginVertical();

            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_RZ_Target"), Core.Target.Name);
            if (chained)
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_AfterNodes", nodes.Count));
            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_RZ_TimeToClosest"), GuiUtils.TimeToDHMS(closestUT - now));

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_ClosestApproach"), GUILayout.ExpandWidth(true));
            GUILayout.Label(FormatDistance(miss), miss < distance ? _red : _green, GUILayout.ExpandWidth(false));
            GUILayout.EndHorizontal();

            GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_RZ_RelInclination"), orbit.RelativeInclination(target).ToString("F2") + "°");

            // While the safe rendezvous autopilot runs it owns the maneuver nodes: no manual buttons.
            if (DrawAutopilot(now))
            {
                GUILayout.EndVertical();
                base.WindowGUI(windowID);
                return;
            }

            // MechJeb's "Match planes with target".
            if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_MatchPlanes")))
                PlaceMatchPlanes(orbit, target, startUT);

            // MechJeb's "Establish new orbit at N km", but N km above (or below) the target's orbit.
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_Phasing")))
                PlacePhasingOrbit(orbit, target, startUT);
            PhasingOffset.Text = GUILayout.TextField(PhasingOffset.Text, GUILayout.Width(70f));
            GUILayout.Label("km", GUILayout.ExpandWidth(false));
            GUILayout.EndHorizontal();
            GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_PhasingHint"));

            GuiUtils.SimpleTextBox(Localizer.Format("#LOC_MJTweaks_RZ_Offset"), OffsetDistance, "m");

            if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_Hohmann")))
                Place(() => OffsetMath.HohmannWithOffset(orbit, target, startUT, distance), "#LOC_MJTweaks_RZ_PlannedMiss");

            if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_Correction")))
                Place(() => OffsetMath.CourseCorrectionWithOffset(orbit, target, startUT, distance), "#LOC_MJTweaks_RZ_PlannedMiss");

            if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_Match")))
                Place(() => OffsetMath.MatchVelocitiesAtDistance(orbit, target, startUT, distance), "#LOC_MJTweaks_RZ_StopDistance");

            if (chained)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_RemoveLast")))
                {
                    nodes[nodes.Count - 1].RemoveSelf();
                    _status = string.Empty;
                }
                if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_RemoveNodes")))
                {
                    Vessel.RemoveAllManeuverNodes();
                    _status = string.Empty;
                }
                GUILayout.EndHorizontal();
            }

            DrawExecuteControls();

            if (!string.IsNullOrEmpty(_status))
                GUILayout.Label(_status, _statusIsError ? _red : GUI.skin.label);

            GUILayout.EndVertical();
            base.WindowGUI(windowID);
        }

        /// <summary>
        /// Start/stop of the safe rendezvous autopilot, its current step, or why it cannot start and the
        /// estimated dV of the route. True while it runs.
        /// </summary>
        private bool DrawAutopilot(double now)
        {
            SafeRendezvousAutopilot autopilot = Core.GetComputerModule<SafeRendezvousAutopilot>();
            if (autopilot == null)
                return false;

            GUILayout.Space(4);
            // The global auto physics warp choice (the same row as under MechJeb's Auto-warp), also while the
            // autopilot runs (it reads the value every frame).
            AutoPhysicsWarp.DrawRow();

            if (autopilot.Enabled)
            {
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_AP_Running", autopilot.Status), _green);
                if (!double.IsNaN(autopilot.NextEventUT) && autopilot.NextEventUT > now)
                    GuiUtils.SimpleLabel(Localizer.Format("#LOC_MJTweaks_RZ_AP_NextEvent"), GuiUtils.TimeToDHMS(autopilot.NextEventUT - now));
                if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_AP_StopButton"), GUILayout.Height(30f)))
                    autopilot.Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_Stopped"), true);
                return true;
            }

            if (autopilot.Current == SafeRendezvousAutopilot.Step.Done || autopilot.Current == SafeRendezvousAutopilot.Step.Stopped)
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_AP_LastResult", autopilot.Status), autopilot.Failed ? _red : _green);

            string reason = SafeRendezvousAutopilot.CannotStartReason(Vessel);
            if (reason != null)
                GUILayout.Label(reason, _red);
            else
            {
                // Every route the autopilot considers, the one it would take marked. Recomputed twice a second
                // (a few hundred candidates), and only in the layout pass so both GUI passes draw the same lines.
                if (Event.current.type == EventType.Layout && Time.realtimeSinceStartup >= _routesUntil)
                {
                    _routesUntil = Time.realtimeSinceStartup + 0.5f;
                    VesselState state = Core.VesselState;
                    double accel = state.mass > 0 ? state.thrustAvailable / state.mass : 0;
                    _chosenRoute = SafeRendezvousAutopilot.ChooseRoute(Vessel.orbit, Core.Target.TargetOrbit, now, accel, _routes);
                    // A picked route that is no longer offered (the orbit or the target changed) falls back to the autopilot's choice.
                    if (_pickedRoute.HasValue && !_routes.Exists(r => r.Kind == _pickedRoute.Value))
                        _pickedRoute = null;
                }
                double available = Vessel.VesselDeltaV != null ? Vessel.VesselDeltaV.TotalDeltaVVac : double.NaN;
                if (_routes.Count > 1)
                {
                    GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_AP_PickHint"));
                    foreach (SafeRendezvousAutopilot.Route route in _routes)
                        RouteToggleLine(route, available);
                }
                else
                    foreach (SafeRendezvousAutopilot.Route route in _routes)
                        GUILayout.Label(RouteText(route), available < route.DeltaV ? _red : GUI.skin.label);
                GUILayout.Label(Localizer.Format("#LOC_MJTweaks_RZ_AP_VesselDv", double.IsNaN(available) ? "?" : available.ToString("F0")));
            }

            GUI.enabled = reason == null;
            if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_AP_StartButton"), GUILayout.Height(30f)))
            {
                autopilot.Start(this, OffsetDistance.Val, _routes.Count > 1 ? _pickedRoute : null);
                _status = string.Empty;
            }
            GUI.enabled = true;
            GUILayout.Space(4);
            return false;
        }

        /// <summary>
        /// One estimated route as a radio button: ticked when it is the one the autopilot will fly (the player's pick,
        /// else the autopilot's own choice, marked as recommended); red when the vessel has less dV than that
        /// (game's own figure). Ticking the recommended one goes back to the autopilot's choice.
        /// </summary>
        private void RouteToggleLine(SafeRendezvousAutopilot.Route route, double available)
        {
            bool recommended = route.Kind == _chosenRoute.Kind;
            bool ticked = _pickedRoute.HasValue ? route.Kind == _pickedRoute.Value : recommended;
            string text = RouteText(route);
            if (recommended)
                text += Localizer.Format("#LOC_MJTweaks_RZ_AP_Chosen");
            GUIStyle style = available < route.DeltaV ? _routeToggleRed : ticked ? _routeToggleGreen : _routeToggle;
            if (GUILayout.Toggle(ticked, text, style) && !ticked)
                _pickedRoute = recommended ? (SafeRendezvousAutopilot.RouteKind?)null : route.Kind;
        }

        /// <summary>One estimated route: its name, dV, time and waiting orbit.</summary>
        private string RouteText(SafeRendezvousAutopilot.Route route)
        {
            string key;
            switch (route.Kind)
            {
                case SafeRendezvousAutopilot.RouteKind.CircularizeAp: key = "#LOC_MJTweaks_RZ_AP_RouteAp"; break;
                case SafeRendezvousAutopilot.RouteKind.CircularizePe: key = "#LOC_MJTweaks_RZ_AP_RoutePe"; break;
                case SafeRendezvousAutopilot.RouteKind.AscendingNode: key = "#LOC_MJTweaks_RZ_AP_RouteAN"; break;
                case SafeRendezvousAutopilot.RouteKind.DescendingNode: key = "#LOC_MJTweaks_RZ_AP_RouteDN"; break;
                case SafeRendezvousAutopilot.RouteKind.ApoapsisBurn: key = "#LOC_MJTweaks_RZ_AP_RouteApBurn"; break;
                case SafeRendezvousAutopilot.RouteKind.PeriapsisBurn: key = "#LOC_MJTweaks_RZ_AP_RoutePeBurn"; break;
                default: key = "#LOC_MJTweaks_RZ_AP_Route"; break;
            }
            return Localizer.Format(key, route.DeltaV.ToString("F0"), GuiUtils.TimeToDHMS(route.Seconds),
                ((route.WaitingRadius - Vessel.mainBody.Radius) / 1000).ToString("F0"));
        }

        private void DrawExecuteControls()
        {
            if (Core.Node == null)
                return;

            if (!Core.Node.Enabled)
            {
                if (Vessel.patchedConicSolver.maneuverNodes.Count > 0 && GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_Execute")))
                    Core.Node.ExecuteOneNode(this);
                if (Vessel.patchedConicSolver.maneuverNodes.Count > 1 && GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_ExecuteAll")))
                    Core.Node.ExecuteAllNodes(this);
            }
            else if (GUILayout.Button(Localizer.Format("#LOC_MJTweaks_RZ_Abort")))
            {
                Core.Node.Abort();
            }

            Core.Node.Autowarp = GUILayout.Toggle(Core.Node.Autowarp, Localizer.Format("#LOC_MJTweaks_RZ_Autowarp"));
        }

        /// <summary>Computes a maneuver and appends it after the existing nodes.</summary>
        private void Place(Func<OffsetMath.Result> compute, string successKey)
        {
            OffsetMath.Result result;
            try
            {
                result = compute();
            }
            catch (Exception e)
            {
                // MechJeb's own solvers throw when they cannot converge, e.g. from a very eccentric orbit.
                Log.Warning($"rendezvous offset: MechJeb could not plan the maneuver: {e}");
                SetError(Localizer.Format("#LOC_MJTweaks_RZ_SolverFailed"));
                return;
            }

            try
            {
                if (!result.Ok)
                {
                    SetError(Localizer.Format("#LOC_MJTweaks_RZ_Failed", result.Error ?? string.Empty));
                    return;
                }

                Vessel.PlaceManeuverNode(Orbit, result.DeltaV, result.UT);
                _status = Localizer.Format(successKey, FormatDistance(result.Miss), result.DeltaV.magnitude.ToString("F1"));
                _statusIsError = false;
            }
            catch (Exception e)
            {
                // Most likely MechJeb changed one of the functions this window relies on.
                Log.Error($"rendezvous offset: {e}");
                SetError(Localizer.Format("#LOC_MJTweaks_RZ_Failed", e.Message));
            }
        }

        /// <summary>
        /// MechJeb's "Match planes with target" (MechJebModuleRendezvousGuidance), with MechJeb's own plane
        /// change math. MechJeb always takes the ascending node (the descending one only when there is no
        /// ascending node); this window takes whichever of the two comes first. Unlike MechJeb it keeps the
        /// planned nodes and plans after the last one.
        /// </summary>
        private void PlaceMatchPlanes(Orbit orbit, Orbit target, double startUT)
        {
            try
            {
                if (!OffsetMath.MatchPlanesNearest(orbit, target, startUT, out Vector3d dV, out double burnUT, out bool useAscending))
                {
                    SetError(Localizer.Format("#LOC_MJTweaks_RZ_Failed", Localizer.Format("#LOC_MJTweaks_RZ_NoNodes")));
                    return;
                }

                Vessel.PlaceManeuverNode(orbit, dV, burnUT);
                _status = Localizer.Format("#LOC_MJTweaks_RZ_PlanesPlaced",
                    Localizer.Format(useAscending ? "#LOC_MJTweaks_RZ_AscendingNode" : "#LOC_MJTweaks_RZ_DescendingNode"),
                    GuiUtils.TimeToDHMS(burnUT - Planetarium.GetUniversalTime()), dV.magnitude.ToString("F1"));
                _statusIsError = false;
            }
            catch (Exception e)
            {
                Log.Error($"rendezvous offset: match planes: {e}");
                SetError(Localizer.Format("#LOC_MJTweaks_RZ_Failed", e.Message));
            }
        }

        /// <summary>
        /// MechJeb's "Establish new orbit at N km" (MechJebModuleRendezvousGuidance): raise the
        /// apoapsis to that radius and circularize there, or lower the periapsis and circularize
        /// there, or — when the orbit already crosses that altitude — circularize where it does.
        /// Unlike MechJeb the altitude is the target's average altitude (semi-major axis) plus the
        /// offset, and an orbit below the surface is refused. Like the other buttons of this window
        /// it keeps the planned nodes and plans after the last one.
        /// </summary>
        private void PlacePhasingOrbit(Orbit orbit, Orbit target, double startUT)
        {
            double bodyRadius = orbit.referenceBody.Radius;
            double targetAltitude = target.semiMajorAxis - bodyRadius;
            double altitude = targetAltitude + PhasingOffset.Val;
            if (altitude <= 0)
            {
                SetError(Localizer.Format("#LOC_MJTweaks_RZ_PhasingBelowSurface", Km(altitude), Km(targetAltitude)));
                return;
            }

            double radius = bodyRadius + altitude;
            int before = Vessel.patchedConicSolver.maneuverNodes.Count;
            try
            {
                double dV;
                if (orbit.ApR < radius)
                {
                    double ut = startUT + 30;
                    ManeuverNode raise = Vessel.PlaceManeuverNode(orbit, OrbitalManeuverCalculator.DeltaVToChangeApoapsis(orbit, ut, radius), ut);
                    Orbit next = raise.nextPatch;
                    double apoapsisUT = next.NextApoapsisTime(ut);
                    ManeuverNode circularize = Vessel.PlaceManeuverNode(next, OrbitalManeuverCalculator.DeltaVToCircularize(next, apoapsisUT), apoapsisUT);
                    dV = raise.DeltaV.magnitude + circularize.DeltaV.magnitude;
                }
                else if (orbit.PeR > radius)
                {
                    double ut = startUT + 30;
                    ManeuverNode lower = Vessel.PlaceManeuverNode(orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, ut, radius), ut);
                    Orbit next = lower.nextPatch;
                    double periapsisUT = next.NextPeriapsisTime(ut);
                    ManeuverNode circularize = Vessel.PlaceManeuverNode(next, OrbitalManeuverCalculator.DeltaVToCircularize(next, periapsisUT), periapsisUT);
                    dV = lower.DeltaV.magnitude + circularize.DeltaV.magnitude;
                }
                else
                {
                    double ut = orbit.NextTimeOfRadius(startUT, radius);
                    ManeuverNode circularize = Vessel.PlaceManeuverNode(orbit, OrbitalManeuverCalculator.DeltaVToCircularize(orbit, ut), ut);
                    dV = circularize.DeltaV.magnitude;
                }

                _status = Localizer.Format("#LOC_MJTweaks_RZ_PhasingPlaced", Km(altitude), Km(targetAltitude),
                    Vessel.patchedConicSolver.maneuverNodes.Count - before, dV.ToString("F1"));
                _statusIsError = false;
            }
            catch (Exception e)
            {
                Log.Error($"rendezvous offset: phasing orbit: {e}");
                SetError(Localizer.Format("#LOC_MJTweaks_RZ_Failed", e.Message));
            }
        }

        private void SetError(string message)
        {
            _status = message;
            _statusIsError = true;
        }

        private static string Km(double meters) => (meters / 1000).ToString("F0");

        private static string FormatDistance(double meters) =>
            meters < 10000 ? $"{meters:F0} m" : $"{meters / 1000:F1} km";

        private static void InitStyles()
        {
            if (_red != null)
                return;

            _red = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 0.35f, 0.3f) } };
            _green = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(0.4f, 1f, 0.4f) } };
            _routeToggle = RouteToggle(GUI.skin.toggle.normal.textColor);
            _routeToggleRed = RouteToggle(_red.normal.textColor);
            _routeToggleGreen = RouteToggle(_green.normal.textColor);
        }

        /// <summary>A wrapping toggle with one text colour in every state.</summary>
        private static GUIStyle RouteToggle(Color color)
        {
            var style = new GUIStyle(GUI.skin.toggle) { wordWrap = true };
            style.normal.textColor = style.onNormal.textColor = color;
            style.hover.textColor = style.onHover.textColor = color;
            style.active.textColor = style.onActive.textColor = color;
            style.focused.textColor = style.onFocused.textColor = color;
            return style;
        }
    }
}
