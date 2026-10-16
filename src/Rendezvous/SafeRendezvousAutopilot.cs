using System;
using KSP.Localization;
using MechJebTweaks.Docking;
using MechJebTweaks.PhysicsWarp;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.Rendezvous
{
    /// <summary>
    /// Safe rendezvous autopilot, started from the "Rendezvous with offset" window. It never plans the
    /// whole route at once: each maneuver is computed from the orbit the previous one actually gave,
    /// executed by MechJeb's node executor, and only then is the next one chosen. Errors of long burns
    /// (KSPIE thermal nozzles) therefore do not pile up.
    ///
    /// Preparation, chosen anew after every burn, first matching rule wins:
    ///   1. orbit eccentric → circularize; the first time at the apsis whose whole route is better
    ///      (ChooseRoute: less dV unless much slower), later at the apsis nearest the aimed radius;
    ///   2. planes differ by more than Settings.RzPlaneAngle → match planes at the nearest node;
    ///   3. our orbit too close in size to the target's (once per run) → raise (or lower) the opposite
    ///      apsis to the waiting orbit; rule 1 then circularizes there;
    ///   4. otherwise the safe Hohmann transfer (OffsetMath), which ends the preparation.
    /// Then: mid-course correction with RCS only (skipped without RCS), then velocity matching at the
    /// offset distance. MechJeb picks this module up like its own (it scans all assemblies for
    /// ComputerModule types); the window starts and stops it through Users.
    /// </summary>
    public class SafeRendezvousAutopilot : ComputerModule
    {
        public enum Step
        {
            Idle,
            Circularize,
            MatchPlanes,
            Phasing,
            Hohmann,
            WaitCorrection,
            RcsCorrection,
            Matching,
            Done,
            Stopped
        }

        public Step Current { get; private set; } = Step.Idle;

        /// <summary>Localized status line: the current step, or the result of the last run.</summary>
        public string Status { get; private set; } = string.Empty;

        public bool Failed { get; private set; }

        /// <summary>When the next burn or check is due (NaN when nothing is scheduled).</summary>
        public double NextEventUT { get; private set; } = double.NaN;

        private Vessel _vessel;
        private ITargetable _target;
        private double _distance;
        private bool _executing;
        private bool _needPlan;
        private double _settleUntil;
        private Step _lastPlanned;
        private int _attempts;
        private double _phasingRadius;
        /// <summary>Radius the circularization aims at (the chosen apsis, later the waiting orbit); NaN = not chosen yet.</summary>
        private double _aimRadius = double.NaN;
        private bool _oldAutowarp;

        // The burn being executed: to tell a finished burn from a cancelled one.
        private Orbit _orbitBefore;
        private double _plannedDv;

        // Mid-course correction
        private double _correctionCheckUT;
        private Orbit _desiredOrbit;
        private double _correctionStartUT;
        private double _correctionDeadline;
        private double _convergedSince;
        private RcsCapability _rcs;
        private Vector3d? _rcsCommand;
        private bool _controlsTaken;
        private bool _oldRcs;
        private bool _oldSas;
        private VesselAutopilot.AutopilotMode _oldSasMode;
        private double _nextRcsLogUT;

        // Braking guidance
        private double _burnStartUT;
        private double _burnDeadline;
        private double _burnThrottle;
        private double _nextTrimUT;
        private float _nextTrimReal;
        private BrakePhase _brakePhase;
        private double _alignedSince;
        /// <summary>Direction of the relative velocity at the closest approach (orbit frame): "along" for the burn plan.</summary>
        private Vector3d _approachDir;
        private bool _engineTaken;
        private double _cleanupDeadline;

        // Sideways RCS nudges during the burn
        private bool _nudging;
        private Vector3d _nudgeCommand;
        private double _nudgeEndUT;
        private double _nextNudgeUT;
        private bool _nudgeRcsOn;
        private bool _oldRcsNudge;

        private const double SettleSeconds = 1.0;
        private const double BurnLead = 30.0;

        public SafeRendezvousAutopilot(MechJebCore core) : base(core)
        {
        }

        // ---------------------------------------------------------------- start / stop

        /// <summary>Why the autopilot cannot start now (localized), or null.</summary>
        public static string CannotStartReason(Vessel vessel)
        {
            ITargetable target = vessel.targetObject;
            Orbit targetOrbit = target?.GetOrbit();
            if (targetOrbit == null)
                return Localizer.Format("#LOC_MJTweaks_RZ_NoTarget");
            if (targetOrbit.referenceBody != vessel.orbit.referenceBody)
                return Localizer.Format("#LOC_MJTweaks_RZ_OtherSoi");
            Vessel targetVessel = target.GetVessel();
            if (targetVessel != null && targetVessel.LandedOrSplashed)
                return Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrTargetLanded");
            if (targetOrbit.eccentricity > Settings.RzTargetMaxEccentricity)
                return Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrTargetEcc",
                    targetOrbit.eccentricity.ToString("F3"), Settings.RzTargetMaxEccentricity.ToString("F3"));

            CelestialBody body = vessel.orbit.referenceBody;
            bool inAtmosphere = body.atmosphere && vessel.altitude < body.atmosphereDepth;
            if (vessel.LandedOrSplashed || inAtmosphere || vessel.orbit.eccentricity >= 1)
                return Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrNotOrbiting");
            return null;
        }

        public void Start(object user, double distance)
        {
            _distance = Math.Max(1.0, distance);
            Users.Add(user);
        }

        public void Stop(string status, bool failed)
        {
            if (Current != Step.Done)
            {
                Current = Step.Stopped;
                Status = status;
                Failed = failed;
            }
            Users.Clear();
            Enabled = false;
        }

        protected override void OnModuleEnabled()
        {
            _vessel = Vessel;
            _target = Vessel.targetObject;
            _executing = false;
            _needPlan = false;
            _settleUntil = 0;
            _lastPlanned = Step.Idle;
            _attempts = 0;
            _phasingRadius = 0;
            _aimRadius = double.NaN;
            _rcsCommand = null;
            _controlsTaken = false;
            Failed = false;
            NextEventUT = double.NaN;

            _oldAutowarp = Core.Node.Autowarp;
            Core.Node.Autowarp = true;

            int removed = Vessel.patchedConicSolver.maneuverNodes.Count;
            Vessel.RemoveAllManeuverNodes();

            Orbit orbit = Vessel.orbit;
            Orbit target = _target.GetOrbit();
            Log.Info($"rendezvous autopilot: started for {_target.GetDisplayName()} at offset {_distance:F0} m; " +
                     $"ours {OrbitText(orbit)}, target {OrbitText(target)}, relative inclination {orbit.RelativeInclination(target):F2}°" +
                     (removed > 0 ? $"; removed {removed} planned node(s)" : ""));
            PlanNext();
        }

        protected override void OnModuleDisabled()
        {
            if (_executing && Core.Node.Enabled)
                Core.Node.Abort();
            _executing = false;
            ReleaseControls();
            Core.Node.Autowarp = _oldAutowarp;
            NextEventUT = double.NaN;
            if (Current != Step.Done && Current != Step.Stopped)
            {
                Current = Step.Stopped;
                Status = Localizer.Format("#LOC_MJTweaks_RZ_AP_Stopped");
                Failed = true;
            }
            Log.Info($"rendezvous autopilot: stopped: {Current} ({Status})");
        }

        // ---------------------------------------------------------------- main loop

        public override void OnFixedUpdate()
        {
            if (Vessel != _vessel)
            {
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_VesselChanged"), true);
                return;
            }
            if (Vessel.targetObject == null || Vessel.targetObject != _target || _target.GetOrbit() == null)
            {
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_TargetLost"), true);
                return;
            }

            double now = Planetarium.GetUniversalTime();
            if (_executing)
            {
                if (Core.Node.Enabled)
                {
                    // Physics warp during MechJeb's turns and burns: PhysicsWarp.ExecutorPhysicsWarp, as for any node.
                    var nodes = Vessel.patchedConicSolver.maneuverNodes;
                    NextEventUT = nodes.Count > 0 ? nodes[0].UT : double.NaN;
                    return;
                }

                _executing = false;
                // The executor also ends when the player aborts it or deletes the node; then the orbit has
                // not changed (2026-10-12: a cancelled circularization was taken as done and planned twice more).
                double change = (Vessel.orbit.WorldOrbitalVelocityAtUT(now) - _orbitBefore.WorldOrbitalVelocityAtUT(now)).magnitude;
                if (Vessel.patchedConicSolver.maneuverNodes.Count > 0 || change < Math.Max(0.5, 0.05 * _plannedDv))
                {
                    Log.Info($"rendezvous autopilot: {Current} not executed (velocity changed by {change:F2} of {_plannedDv:F2} m/s), stopping");
                    Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrNodeAborted"), true);
                    return;
                }
                Log.Info($"rendezvous autopilot: {Current} done: ours {OrbitText(Vessel.orbit)}, " +
                         $"relative inclination {Vessel.orbit.RelativeInclination(_target.GetOrbit()):F2}°");
                _settleUntil = now + SettleSeconds;
                AfterBurn();
                return;
            }

            if (now < _settleUntil)
                return;

            if (_needPlan)
            {
                PlanNext();
                return;
            }

            switch (Current)
            {
                case Step.WaitCorrection:
                    WaitForCorrection(now);
                    break;
                case Step.RcsCorrection:
                    RcsCorrect(now);
                    break;
                case Step.Matching:
                    Brake(now);
                    break;
            }
        }

        public override void Drive(FlightCtrlState s)
        {
            if (_rcsCommand.HasValue)
                RcsDrive.Apply(s, _rcsCommand.Value);
        }

        /// <summary>What follows a finished burn.</summary>
        private void AfterBurn()
        {
            switch (Current)
            {
                case Step.Circularize:
                case Step.MatchPlanes:
                case Step.Phasing:
                    _needPlan = true;
                    break;
                case Step.Hohmann:
                    BeginTransfer();
                    break;
            }
        }

        // ---------------------------------------------------------------- preparation

        private void PlanNext()
        {
            _needPlan = false;
            Orbit orbit = Vessel.orbit;
            Orbit target = _target.GetOrbit();
            double now = Planetarium.GetUniversalTime();

            if (orbit.eccentricity >= 1)
            {
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrNotOrbiting"), true);
                return;
            }

            Step step;
            Vector3d dV;
            double ut;
            try
            {
                if (orbit.eccentricity > Settings.RzRoundEccentricity)
                {
                    step = Step.Circularize;
                    if (double.IsNaN(_aimRadius))
                    {
                        // First circularization: at the apsis whose whole route is better (see ChooseRoute).
                        Route chosen = ChooseRoute(orbit, target, now, out Route other);
                        _aimRadius = chosen.AtPeriapsis ? orbit.PeR : orbit.ApR;
                        Log.Info($"rendezvous autopilot: route via {RouteText(chosen)} chosen over {RouteText(other)}");
                    }
                    ut = CircularizeUT(orbit, now);
                    dV = OrbitalManeuverCalculator.DeltaVToCircularize(orbit, ut);
                }
                else if (orbit.RelativeInclination(target) > Settings.RzPlaneAngle)
                {
                    step = Step.MatchPlanes;
                    if (!OffsetMath.MatchPlanesNearest(orbit, target, now + BurnLead, out dV, out ut, out _))
                    {
                        Stop(Localizer.Format("#LOC_MJTweaks_RZ_Failed", Localizer.Format("#LOC_MJTweaks_RZ_NoNodes")), true);
                        return;
                    }
                }
                else if (_phasingRadius <= 0 && NeedsPhasing(orbit.semiMajorAxis, target.semiMajorAxis) && PickWaitingOrbit(orbit, target, now))
                {
                    step = Step.Phasing;
                    if (_phasingRadius <= 0)
                    {
                        Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrNoPhasing"), true);
                        return;
                    }
                    _aimRadius = _phasingRadius;
                    ut = LeadTime(now, t => ChangeRadius(orbit, t, _phasingRadius));
                    dV = ChangeRadius(orbit, ut, _phasingRadius);
                }
                else
                {
                    step = Step.Hohmann;
                    OffsetMath.Result hohmann = OffsetMath.HohmannWithOffset(orbit, target, now + BurnLead, _distance);
                    if (!hohmann.Ok)
                    {
                        Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrSolver", hohmann.Error ?? string.Empty), true);
                        return;
                    }
                    dV = hohmann.DeltaV;
                    ut = hohmann.UT;
                }
            }
            catch (Exception e)
            {
                Log.Warning($"rendezvous autopilot: planning failed: {e}");
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrSolver", e.Message), true);
                return;
            }

            _attempts = step == _lastPlanned ? _attempts + 1 : 1;
            _lastPlanned = step;
            if (_attempts > Settings.RzMaxAttempts)
            {
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrRepeat", StepName(step), Settings.RzMaxAttempts), true);
                return;
            }

            Execute(step, orbit, dV, ut);
        }

        /// <summary>Circularize at the apsis nearest the aimed radius (the chosen apsis, or the waiting orbit).</summary>
        private double CircularizeUT(Orbit orbit, double now)
        {
            if (Math.Abs(orbit.PeR - _aimRadius) < Math.Abs(orbit.ApR - _aimRadius))
                return orbit.NextPeriapsisTime(now);
            return orbit.NextApoapsisTime(now);
        }

        /// <summary>From a round orbit: raise the apoapsis to the radius, or lower the periapsis to it.</summary>
        private static Vector3d ChangeRadius(Orbit orbit, double ut, double radius) =>
            radius > orbit.semiMajorAxis
                ? OrbitalManeuverCalculator.DeltaVToChangeApoapsis(orbit, ut, radius)
                : OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, ut, radius);

        /// <summary>A burn "now" that leaves MechJeb time to turn and to start half the burn early.</summary>
        private double LeadTime(double now, Func<double, Vector3d> dvAt)
        {
            double ut = now + BurnLead;
            double accel = MaxAcceleration();
            if (accel > 0)
                ut += 0.5 * dvAt(ut).magnitude / accel;
            return ut;
        }

        private double MaxAcceleration()
        {
            VesselState state = Core.VesselState;
            return state.mass > 0 ? state.thrustAvailable / state.mass : 0;
        }

        private static bool NeedsPhasing(double radius, double targetRadius) =>
            Math.Abs(radius / targetRadius - 1) < Settings.RzPhasingMinDifference;

        /// <summary>Waiting orbit above the target; below it only when above would leave the sphere of influence.</summary>
        private static double PhasingRadius(Orbit target)
        {
            CelestialBody body = target.referenceBody;
            double above = target.semiMajorAxis * Settings.RzPhasingRatio;
            if (above < 0.9 * body.sphereOfInfluence)
                return above;
            double below = target.semiMajorAxis / Settings.RzPhasingRatio;
            return below >= LowestRadius(body) ? below : 0;
        }

        /// <summary>
        /// Chooses the waiting orbit (sets _phasingRadius). True: go to it; false: the direct transfer from the
        /// current orbit was chosen (_phasingRadius is then our own radius, so no waiting orbit is picked again).
        /// No usable candidate at all: true with _phasingRadius 0.
        /// </summary>
        private bool PickWaitingOrbit(Orbit orbit, Orbit target, double now)
        {
            PhasingChoice choice = ChoosePhasing(orbit, target, now);
            if (!choice.Chosen.Valid)
            {
                _phasingRadius = 0;
                return true;
            }
            Log.Info($"rendezvous autopilot: waiting orbit: {PhasingText(choice.Chosen, orbit)} chosen; " +
                     $"standard {PhasingText(choice.Baseline, orbit)}, time limit {choice.Budget / 3600:F1} h, " +
                     $"cheapest within it out of {choice.Candidates} candidates");
            if (choice.Chosen.Direct)
            {
                _phasingRadius = orbit.semiMajorAxis;
                return false;
            }
            _phasingRadius = choice.Chosen.Radius;
            return true;
        }

        /// <summary>A way to the transfer window: via a round waiting orbit, or the direct transfer from our orbit.</summary>
        public struct PhasingPlan
        {
            public bool Valid;
            /// <summary>No waiting orbit: wait on our own orbit for the transfer window.</summary>
            public bool Direct;
            public double Radius;
            /// <summary>Raise/lower, circularize, transfer and match velocities: all impulsive.</summary>
            public double DeltaV;
            /// <summary>From now to the arrival at the target.</summary>
            public double Seconds;
        }

        public struct PhasingChoice
        {
            public PhasingPlan Chosen;
            /// <summary>The waiting orbit PhasingRatio away from the target (the old fixed choice).</summary>
            public PhasingPlan Baseline;
            public double Budget;
            public int Candidates;
        }

        /// <summary>
        /// History (test 11, 2026-10-13): the waiting orbit was always 1.25 x the target's radius; with a bad phase
        /// that cost 527 m/s for one revolution of waiting. A waiting orbit closer to the target is much cheaper
        /// but the target drifts past more slowly, so the wait is longer. Now candidates from PhasingStep to
        /// PhasingRatio away from the target, above and below, plus the direct transfer, are each timed from the
        /// actual phase of the target; the cheapest one that takes at most SlowFactor x as long as the
        /// PhasingRatio orbit is taken.
        /// </summary>
        public static PhasingChoice ChoosePhasing(Orbit orbit, Orbit target, double now)
        {
            CelestialBody body = orbit.referenceBody;
            double burnUT = now + BurnLead;
            double targetRadius = target.semiMajorAxis;
            var choice = new PhasingChoice();

            double standard = PhasingRadius(target);
            choice.Baseline = standard > 0 ? PlanPhasing(orbit, target, now, burnUT, standard, false) : default;

            var plans = new System.Collections.Generic.List<PhasingPlan> { PlanPhasing(orbit, target, now, now, orbit.semiMajorAxis, true) };
            if (choice.Baseline.Valid)
                plans.Add(choice.Baseline);
            double step = Math.Max(0.001, Settings.RzPhasingStep);
            for (double share = step; share < Settings.RzPhasingRatio - 1 + 0.5 * step; share += step)
            {
                double above = targetRadius * (1 + share);
                if (above < 0.9 * body.sphereOfInfluence)
                    plans.Add(PlanPhasing(orbit, target, now, burnUT, above, false));
                double below = targetRadius / (1 + share);
                if (below >= LowestRadius(body))
                    plans.Add(PlanPhasing(orbit, target, now, burnUT, below, false));
            }

            double fastest = double.MaxValue;
            foreach (PhasingPlan p in plans)
                if (p.Valid)
                {
                    choice.Candidates++;
                    fastest = Math.Min(fastest, p.Seconds);
                }
            choice.Budget = Settings.RzSlowFactor * (choice.Baseline.Valid ? choice.Baseline.Seconds : fastest);
            foreach (PhasingPlan p in plans)
                if (p.Valid && p.Seconds <= choice.Budget && (!choice.Chosen.Valid || p.DeltaV < choice.Chosen.DeltaV))
                    choice.Chosen = p;
            return choice;
        }

        /// <summary>
        /// Coplanar circular-orbit model: Hohmann to the waiting radius (burn at burnUT), wait there until the
        /// target is where a Hohmann transfer from that radius meets it, the transfer. Direct: the same from our
        /// own orbit without the first transfer. Invalid when a burn would be too small to execute reliably or
        /// the target never drifts relative to us.
        /// </summary>
        private static PhasingPlan PlanPhasing(Orbit orbit, Orbit target, double now, double burnUT, double radius, bool direct)
        {
            double mu = orbit.referenceBody.gravParameter;
            double ours = orbit.semiMajorAxis;
            double targetRadius = target.semiMajorAxis;
            var plan = new PhasingPlan { Direct = direct, Radius = radius };

            if (FirstBurn(mu, ours, direct ? targetRadius : radius) < MinPlanBurn)
                return plan;
            if (!direct && FirstBurn(mu, radius, targetRadius) < MinPlanBurn)
                return plan;

            // How far the target is ahead of us along our direction of motion at burnUT, 0..2 pi.
            Vector3d position = orbit.getRelativePositionAtUT(burnUT);
            Vector3d normal = Vector3d.Cross(position, orbit.getOrbitalVelocityAtUT(burnUT)).normalized;
            Vector3d targetPosition = target.getRelativePositionAtUT(burnUT);
            double ahead = Math.Atan2(Vector3d.Dot(normal, Vector3d.Cross(position, targetPosition)), Vector3d.Dot(position, targetPosition));

            double targetRate = 2 * Math.PI / target.period;
            double seconds = burnUT - now;
            double dv = 0;
            if (!direct)
            {
                double first = HalfTransfer(mu, ours, radius);
                seconds += first;
                ahead += targetRate * first - Math.PI;
                dv += Hohmann(mu, ours, radius);
            }

            double waitRate = Math.Sqrt(mu / (radius * radius * radius));
            double transfer = HalfTransfer(mu, radius, targetRadius);
            double needed = Math.PI - targetRate * transfer;
            double drift = targetRate - waitRate;
            if (Math.Abs(drift) < 1e-12)
                return plan;
            double wait = drift > 0 ? Mod2Pi(needed - ahead) / drift : Mod2Pi(ahead - needed) / -drift;

            plan.Valid = true;
            plan.DeltaV = dv + Hohmann(mu, radius, targetRadius);
            plan.Seconds = seconds + wait + transfer;
            return plan;
        }

        /// <summary>Smallest first burn of a planned transfer: smaller ones are hard to execute and to tell from a cancelled node, m/s.</summary>
        private const double MinPlanBurn = 2.0;

        private static double FirstBurn(double mu, double r1, double r2) =>
            Math.Abs(Math.Sqrt(mu / r1) * (Math.Sqrt(2 * r2 / (r1 + r2)) - 1));

        private static double HalfTransfer(double mu, double r1, double r2) => 0.5 * Period(mu, 0.5 * (r1 + r2));

        private static double Mod2Pi(double angle)
        {
            double twoPi = 2 * Math.PI;
            angle %= twoPi;
            return angle < 0 ? angle + twoPi : angle;
        }

        private static string PhasingText(PhasingPlan p, Orbit orbit) =>
            !p.Valid ? "(none)"
            : $"{(p.Direct ? "direct transfer" : $"{(p.Radius - orbit.referenceBody.Radius) / 1000:F0} km")}: dV {p.DeltaV:F0} m/s, {p.Seconds / 3600:F1} h";

        /// <summary>Lowest radius for a circular orbit of ours: above the atmosphere (or sea level) plus a margin for terrain.</summary>
        private static double LowestRadius(CelestialBody body) =>
            body.Radius + (body.atmosphere ? body.atmosphereDepth : 0) + Settings.RzTerrainMargin;

        private void Execute(Step step, Orbit orbit, Vector3d dV, double ut)
        {
            Vessel.RemoveAllManeuverNodes();
            Vessel.PlaceManeuverNode(orbit, dV, ut);
            _orbitBefore = new Orbit(orbit.inclination, orbit.eccentricity, orbit.semiMajorAxis, orbit.LAN,
                orbit.argumentOfPeriapsis, orbit.meanAnomalyAtEpoch, orbit.epoch, orbit.referenceBody);
            _plannedDv = dV.magnitude;
            Core.Node.ExecuteOneNode(this);
            _executing = true;
            Current = step;
            Status = StepName(step);
            NextEventUT = ut;
            Log.Info($"rendezvous autopilot: {step}{(_attempts > 1 ? $" (attempt {_attempts})" : "")}: dV {dV.magnitude:F1} m/s " +
                     $"in {ut - Planetarium.GetUniversalTime():F0} s; ours {OrbitText(orbit)}" +
                     (step == Step.Phasing ? $", waiting orbit at {(_phasingRadius - orbit.referenceBody.Radius) / 1000:F1} km" : ""));
        }

        // ---------------------------------------------------------------- transfer and correction

        private void BeginTransfer()
        {
            Orbit orbit = Vessel.orbit;
            Orbit target = _target.GetOrbit();
            double now = Planetarium.GetUniversalTime();
            double arrival = orbit.NextClosestApproachTime(target, now);
            double miss = orbit.Separation(target, arrival);
            Log.Info($"rendezvous autopilot: on the transfer, closest approach {miss:F0} m in {arrival - now:F0} s");

            _rcs = RcsCapability.Measure(Vessel);
            if (!_rcs.Any || RcsDrive.PropellantAmount(Vessel) < 0.01)
            {
                Log.Info("rendezvous autopilot: no usable RCS, mid-course correction skipped");
                BeginMatching();
                return;
            }
            if (arrival - now < Settings.RzCorrectionMinFlight)
            {
                Log.Info($"rendezvous autopilot: flight shorter than {Settings.RzCorrectionMinFlight:F0} s, mid-course correction skipped");
                BeginMatching();
                return;
            }

            _correctionCheckUT = now + 0.5 * (arrival - now);
            Current = Step.WaitCorrection;
            Status = StepName(Step.WaitCorrection);
            NextEventUT = _correctionCheckUT;
        }

        /// <summary>Halfway to the target: compute the correction (from the orbit actually flown) and when to fire RCS.</summary>
        private void WaitForCorrection(double now)
        {
            if (now < _correctionCheckUT - 1)
            {
                Core.Warp.WarpToUT(_correctionCheckUT);
                return;
            }
            if (!AtPhysicsSpeed())
                return;

            Orbit orbit = Vessel.orbit;
            OffsetMath.Result r;
            try
            {
                r = OffsetMath.CourseCorrectionWithOffset(orbit, _target.GetOrbit(), now, _distance);
            }
            catch (Exception e)
            {
                r = new OffsetMath.Result { Error = e.Message };
            }

            if (!r.Ok)
            {
                Log.Warning($"rendezvous autopilot: no mid-course correction found ({r.Error}), going on");
                BeginMatching();
                return;
            }
            if (r.DeltaV.magnitude < Settings.RzCorrectionMinDv)
            {
                Log.Info($"rendezvous autopilot: mid-course correction {r.DeltaV.magnitude:F3} m/s, not needed (miss {r.Miss:F0} m)");
                BeginMatching();
                return;
            }

            _rcs = RcsCapability.Measure(Vessel);
            double burnTime = RcsBurnTime(r.DeltaV);
            _desiredOrbit = orbit.PerturbedOrbit(r.UT, r.DeltaV);
            _correctionStartUT = Math.Max(now, r.UT - 0.5 * burnTime);
            _correctionDeadline = _correctionStartUT + 3 * burnTime + 60;
            _convergedSince = double.NaN;
            _nextRcsLogUT = 0;
            Current = Step.RcsCorrection;
            Status = StepName(Step.RcsCorrection);
            NextEventUT = _correctionStartUT;
            Log.Info($"rendezvous autopilot: mid-course correction {r.DeltaV.magnitude:F2} m/s by RCS, about {burnTime:F0} s, " +
                     $"starts in {_correctionStartUT - now:F0} s, planned miss {r.Miss:F0} m");
        }

        /// <summary>
        /// Time for RCS to give dV: every reference axis works in parallel, the slowest one decides;
        /// plus a few time constants for the proportional tail.
        /// </summary>
        private double RcsBurnTime(Vector3d dV)
        {
            Vector3 local = Quaternion.Inverse(Vessel.ReferenceTransform.rotation) * (Vector3)dV;
            double longest = 0;
            for (int axis = 0; axis < 3; axis++)
            {
                double available = local[axis] > 0 ? _rcs.Positive[axis] : _rcs.Negative[axis];
                if (Math.Abs(local[axis]) < 1e-4)
                    continue;
                longest = Math.Max(longest, available > 0 ? Math.Abs(local[axis]) / available : 600);
            }
            return longest + 3 * Settings.DockingRcsTimeConstant;
        }

        /// <summary>
        /// RCS steers the vessel's velocity onto the corrected orbit. The error is measured against that
        /// orbit at the same moment, so gravity is the same on both sides and the burn may start early.
        /// </summary>
        private void RcsCorrect(double now)
        {
            if (now < _correctionStartUT - 1)
            {
                if (_correctionStartUT - now > 3)
                    Core.Warp.WarpToUT(_correctionStartUT - 1);
                else if (TimeWarp.CurrentRateIndex > 0)
                    Core.Warp.MinimumWarp(true);
                return;
            }
            if (!AtPhysicsSpeed())
                return;

            if (RcsDrive.PropellantAmount(Vessel) < 0.01)
            {
                Log.Warning("rendezvous autopilot: RCS propellant ran out during the correction, going on");
                EndCorrection(now);
                return;
            }

            Vector3d error = Vessel.orbit.WorldOrbitalVelocityAtUT(now) - _desiredOrbit.WorldOrbitalVelocityAtUT(now);
            double size = error.magnitude;
            if (size <= Settings.RzCorrectionTolerance)
            {
                SetRcs(null);
                if (double.IsNaN(_convergedSince))
                    _convergedSince = now;
                else if (now - _convergedSince >= 1.0)
                {
                    EndCorrection(now);
                    return;
                }
            }
            else
            {
                _convergedSince = double.NaN;
                SetRcs(RcsDrive.Command(Vessel.ReferenceTransform, error, _rcs, Settings.DockingDeadband));
            }

            if (now >= _nextRcsLogUT)
            {
                _nextRcsLogUT = now + 5;
                Log.Info($"rendezvous autopilot: correcting, velocity error {size:F3} m/s, RCS {(_rcsCommand.HasValue ? "on" : "off")}");
            }

            if (now > _correctionDeadline)
            {
                Log.Warning($"rendezvous autopilot: correction time limit, {size:F3} m/s left, going on");
                EndCorrection(now);
            }
        }

        private void EndCorrection(double now)
        {
            ReleaseControls();
            Orbit orbit = Vessel.orbit;
            Orbit target = _target.GetOrbit();
            double arrival = orbit.NextClosestApproachTime(target, now);
            Log.Info($"rendezvous autopilot: correction done, closest approach {orbit.Separation(target, arrival):F0} m in {arrival - now:F0} s");
            BeginMatching();
        }

        // ---------------------------------------------------------------- final braking

        /// <summary>
        /// The final braking is one steady burn, tail-first against the relative velocity, planned by
        /// simulating the whole finite burn (thrust, mass flow, gravity, the target moving on its orbit).
        /// History (2026-10-12): a MechJeb "match velocities" node is burned around the node time as if the
        /// approach were straight; at ~2 m/s² against 550 m/s the burn lasts 4.5 min over ~23° of orbit, the
        /// relative path curves, and the vessel passed the target at 92 m still doing 55 m/s and stopped 1 km
        /// behind it. Closed-loop guidance stopped it right (test 5) but turned and shook the vessel all the
        /// time. Now the start is found so that the simulated burn ends the offset distance from the target on
        /// the near side; during the burn the simulation is rerun from the actual state every few seconds and
        /// only the throttle is trimmed, the vessel stays retrograde. The last metre per second goes with RCS.
        /// </summary>
        private void BeginMatching()
        {
            double now = Planetarium.GetUniversalTime();
            Orbit orbit = Vessel.orbit;
            Orbit target = _target.GetOrbit();
            if (!EngineModel(out double thrust, out double massFlow))
            {
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrNoThrust"), true);
                return;
            }

            double arrival = orbit.NextClosestApproachTime(target, now);
            Vector3d relativeAtArrival = orbit.getOrbitalVelocityAtUT(arrival) - target.getOrbitalVelocityAtUT(arrival);
            _approachDir = relativeAtArrival.normalized;
            double mass = Vessel.totalMass;
            double throttle = Settings.RzBurnThrottle;
            double accel = throttle * thrust / mass;

            // The start: bisection between "far too early" (stops short) and the closest approach (passes).
            double early = Math.Max(now + 1, arrival - 3 * relativeAtArrival.magnitude / accel - 120);
            Func<double, BurnPlan> planAt = t0 => Simulate(orbit.getRelativePositionAtUT(t0), orbit.getOrbitalVelocityAtUT(t0), t0,
                mass, throttle * thrust, throttle * massFlow, target);
            double lo = early, hi = arrival;
            BurnPlan plan = planAt(lo);
            if (Shortfall(plan) > 0)
            {
                for (int i = 0; i < 40; i++)
                {
                    double mid = 0.5 * (lo + hi);
                    BurnPlan p = planAt(mid);
                    if (Shortfall(p) > 0)
                    {
                        lo = mid;
                        plan = p;
                    }
                    else
                        hi = mid;
                }
            }
            else
                Log.Warning("rendezvous autopilot: even braking right now does not stop the vessel short of the target");

            // A burn long against the orbit (weak engine) curves far off the approach line: in a test simulation
            // at 0.5 m/s² against 550 m/s it ended 97 km aside. One retrograde burn is no use then.
            if (plan.Lateral > Settings.RzBurnMaxLateral)
            {
                Log.Warning($"rendezvous autopilot: braking burn would end {plan.Lateral:F0} m aside of the target ({plan.Seconds:F0} s, {thrust / mass:F2} m/s²)");
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrWeakEngine", (plan.Lateral / 1000).ToString("F1"), plan.Seconds.ToString("F0")), true);
                return;
            }

            _burnStartUT = lo;
            _burnThrottle = throttle;
            _brakePhase = BrakePhase.Approach;
            _attempts = 1;
            _lastPlanned = Step.Matching;
            Current = Step.Matching;
            Status = StepName(Step.Matching);
            NextEventUT = _burnStartUT;
            _burnDeadline = _burnStartUT + 2 * plan.Seconds + 120;
            Log.Info($"rendezvous autopilot: braking burn planned: closest approach {orbit.Separation(target, arrival):F0} m at " +
                     $"{relativeAtArrival.magnitude:F1} m/s in {arrival - now:F0} s; burn at {throttle:P0} of {thrust / mass:F2} m/s² " +
                     $"(mass flow {massFlow * 1000:F1} kg/s at full) starts in {_burnStartUT - now:F0} s, lasts {plan.Seconds:F0} s, dV {plan.DeltaV:F0} m/s, " +
                     $"ends {plan.Along:F0} m short / {plan.Lateral:F0} m aside of the target, closest {plan.MinDistance:F0} m");
        }

        private enum BrakePhase
        {
            /// <summary>On-rails warp until BrakeTurnLead seconds before the burn.</summary>
            Approach,
            /// <summary>1x: turn tail-first against the relative velocity.</summary>
            Align,
            /// <summary>Turned: on-rails warp again (attitude stays) until BrakeWarpStop seconds before the burn.</summary>
            Coast,
            /// <summary>The burn, throttle trimmed by re-simulation; physics warp allowed until the last kilometres.</summary>
            Burn,
            /// <summary>Main engine off, the rest of the relative velocity with RCS.</summary>
            Cleanup
        }

        private void Brake(double now)
        {
            Orbit target = _target.GetOrbit();
            Relative(Vessel.orbit, target, now, out Vector3d toTarget, out Vector3d velocity);
            double distance = toTarget.magnitude;
            double speed = velocity.magnitude;
            Vector3d retrograde = speed > 1e-6 ? -velocity / speed : -toTarget.normalized;

            switch (_brakePhase)
            {
                case BrakePhase.Approach:
                    if (now < _burnStartUT - Settings.RzBrakeTurnLead - 1)
                    {
                        Core.Warp.WarpToUT(_burnStartUT - Settings.RzBrakeTurnLead);
                        return;
                    }
                    if (!OutOfRailsWarp())
                        return;
                    _engineTaken = true;
                    Core.Thrust.Users.Add(this);
                    Core.Thrust.TargetThrottle = 0;
                    _brakePhase = BrakePhase.Align;
                    _alignedSince = double.NaN;
                    Log.Info($"rendezvous autopilot: braking: turning tail-first, burn in {_burnStartUT - now:F0} s, d {distance:F0} m, {speed:F1} m/s");
                    return;

                case BrakePhase.Align:
                    if (!OutOfRailsWarp())
                        return;
                    Core.Attitude.attitudeTo(retrograde, AttitudeReference.INERTIAL_COT, this);
                    double off = Core.Attitude.attitudeAngleFromTarget();
                    // Turn in physics warp, the last degrees at 1x so the vessel settles exactly.
                    PhysicsWarp(off > Settings.AutoPhysicsWarpAimAngle);
                    if (off <= Settings.RzBrakeAlignedAngle)
                    {
                        if (double.IsNaN(_alignedSince))
                            _alignedSince = now;
                    }
                    else
                        _alignedSince = double.NaN;
                    bool aligned = !double.IsNaN(_alignedSince) && now - _alignedSince >= 3;
                    if (aligned || now >= _burnStartUT - Settings.RzBrakeWarpStop)
                    {
                        _brakePhase = _burnStartUT - now > Settings.RzBrakeWarpStop + 10 ? BrakePhase.Coast : BrakePhase.Burn;
                        _nextTrimUT = _burnStartUT + Settings.RzBurnTrimInterval;
                        Log.Info($"rendezvous autopilot: braking: {(aligned ? "turned" : "not turned yet")} ({off:F1}° off), " +
                                 $"{(_brakePhase == BrakePhase.Coast ? "warping on" : "waiting for the burn")}, burn in {_burnStartUT - now:F0} s");
                    }
                    return;

                case BrakePhase.Coast:
                    if (now < _burnStartUT - Settings.RzBrakeWarpStop - 1)
                    {
                        Core.Warp.WarpToUT(_burnStartUT - Settings.RzBrakeWarpStop);
                        return;
                    }
                    if (!OutOfRailsWarp())
                        return;
                    _brakePhase = BrakePhase.Burn;
                    Log.Info($"rendezvous autopilot: braking: out of warp, burn in {_burnStartUT - now:F0} s, d {distance:F0} m, {speed:F1} m/s");
                    return;

                case BrakePhase.Cleanup:
                    Cleanup(now, velocity, distance);
                    return;
            }

            // BrakePhase.Burn. Physics warp (the window's checkbox) while burning far out or turning a lot;
            // always 1x in the last kilometres.
            if (!OutOfRailsWarp())
                return;
            Core.Attitude.attitudeTo(retrograde, AttitudeReference.INERTIAL_COT, this);
            double angle = Core.Attitude.attitudeAngleFromTarget();
            if (distance < Settings.RzBrakeNoWarpDistance)
            {
                if (TimeWarp.CurrentRateIndex > 0)
                    TimeWarp.SetRate(0, true);
            }
            else
                PhysicsWarp(angle > Settings.AutoPhysicsWarpAimAngle || now >= _burnStartUT);

            if (now >= _burnStartUT && speed <= Settings.RzBurnCutSpeed)
            {
                Core.Thrust.TargetThrottle = 0;
                Log.Info($"rendezvous autopilot: braking burn done: d {distance:F0} m, {speed:F2} m/s left, took {now - _burnStartUT:F0} s");
                BeginCleanup(now);
                return;
            }
            if (now > _burnDeadline)
            {
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrBrakeTimeout"), true);
                return;
            }

            if (!EngineModel(out double thrust, out double massFlow))
            {
                Stop(Localizer.Format("#LOC_MJTweaks_RZ_AP_ErrNoThrust"), true);
                return;
            }
            // Every BurnTrimInterval game seconds, but at most twice a real second (physics warp).
            if (now >= _nextTrimUT && Time.realtimeSinceStartup >= _nextTrimReal)
            {
                _nextTrimReal = Time.realtimeSinceStartup + 0.5f;
                Trim(now, thrust, massFlow, distance, speed, angle);
            }

            Core.Thrust.TargetThrottle = now >= _burnStartUT && angle <= Settings.RzBrakeAimAngle ? (float)_burnThrottle : 0f;
            UpdateNudge(now);
        }

        // ---------------------------------------------------------------- sideways RCS nudges during the burn

        /// <summary>
        /// The retrograde burn itself drifts sideways (test 6, 2026-10-13: planned and got ~200-230 m aside of the
        /// target instead of 100); the player's idea is to push the end point back with RCS across the engine axis,
        /// without turning. Test 7 showed two traps: (1) the relative velocity of the two vessels is no measure of
        /// what RCS gave — 80 km apart the difference in gravity alone changes it by ~0.2 m/s², more than the
        /// vessel's sideways RCS, so nudges ran long and grew to 1 m/s; (2) on an orbit a sideways push does not
        /// move the end point by "dv x time left". Now the effect of a push is taken from the burn simulation itself
        /// (two extra runs with a small push along the two axes across the engine), the push that moves the end
        /// point to BurnLateralShare x the offset aside is solved from that, NudgeGain of it is fired open loop for
        /// the time the RCS needs, and the next nudge waits until a re-simulation has seen the result.
        /// </summary>
        private void PlanNudge(double now, Func<Vector3d, BurnPlan> simulateWithPush)
        {
            if (_nudging || now < _nextNudgeUT)
                return;
            BurnPlan plan = simulateWithPush(Vector3d.zero);
            if (plan.Seconds < Settings.RzNudgeMinTime || plan.Lateral < 1)
                return;
            double wanted = Settings.RzBurnLateralShare * _distance;
            if (Math.Abs(plan.Lateral - wanted) < Settings.RzNudgeMinMiss)
                return;

            // Two directions across the engine axis (orbit frame; the orbit frame is the world frame with y and z swapped).
            Vector3d forward = Core.VesselState.forward.xzy;
            Vector3d e1 = Vector3d.Cross(forward, _approachDir);
            if (e1.magnitude < 1e-6)
                e1 = Vector3d.Cross(forward, new Vector3d(0, 0, 1));
            e1 = e1.normalized;
            Vector3d e2 = Vector3d.Cross(forward, e1).normalized;

            // Sensitivity of the end point (sideways part) to a push along e1 and e2.
            const double probe = 0.1;
            Vector3d j1 = (simulateWithPush(probe * e1).LateralVec - plan.LateralVec) / probe;
            Vector3d j2 = (simulateWithPush(probe * e2).LateralVec - plan.LateralVec) / probe;
            Vector3d want = (wanted / plan.Lateral - 1) * plan.LateralVec;

            // Least squares for x1, x2: j1 x1 + j2 x2 = want.
            double a11 = Vector3d.Dot(j1, j1), a12 = Vector3d.Dot(j1, j2), a22 = Vector3d.Dot(j2, j2);
            double b1 = Vector3d.Dot(j1, want), b2 = Vector3d.Dot(j2, want);
            double det = a11 * a22 - a12 * a12;
            if (Math.Abs(det) < 1e-9)
                return;
            double x1 = (b1 * a22 - b2 * a12) / det;
            double x2 = (a11 * b2 - a12 * b1) / det;
            Vector3d dv = (Settings.RzNudgeGain * (x1 * e1 + x2 * e2)).xzy; // world frame
            if (dv.magnitude < 0.02)
                return;
            if (dv.magnitude > Settings.RzNudgeMaxDv)
                dv = Settings.RzNudgeMaxDv / dv.magnitude * dv;

            _rcs = RcsCapability.Measure(Vessel);
            if (!_rcs.Any || RcsDrive.PropellantAmount(Vessel) < 0.01)
                return;

            // Open loop: every sideways axis fires for its share so that all end together.
            Vector3 local = Quaternion.Inverse(Vessel.ReferenceTransform.rotation) * (Vector3)dv;
            double seconds = 0;
            foreach (int axis in new[] { 0, 2 })
            {
                double available = local[axis] > 0 ? _rcs.Positive[axis] : _rcs.Negative[axis];
                if (Math.Abs(local[axis]) > 1e-4 && available > 0)
                    seconds = Math.Max(seconds, Math.Abs(local[axis]) / available);
            }
            if (!(seconds > 0))
                return;
            var command = new Vector3d();
            foreach (int axis in new[] { 0, 2 })
            {
                double available = local[axis] > 0 ? _rcs.Positive[axis] : _rcs.Negative[axis];
                if (Math.Abs(local[axis]) <= 1e-4 || available <= 0)
                    continue;
                // RcsDrive convention: a positive command pushes towards -axis.
                double share = Math.Max(Settings.DockingMinRcsCommand, Math.Abs(local[axis]) / available / seconds);
                command[axis] = -Math.Sign(local[axis]) * share;
            }

            _nudging = true;
            _nudgeCommand = command;
            _nudgeEndUT = now + seconds;
            Log.Info($"rendezvous autopilot: braking: sideways RCS nudge {dv.magnitude:F2} m/s for {seconds:F1} s, end point {plan.Lateral:F0} m aside -> {wanted:F0} m " +
                     $"(1 m/s moves it {j1.magnitude:F0} / {j2.magnitude:F0} m)");
        }

        private void UpdateNudge(double now)
        {
            if (!_nudging)
                return;
            if (now >= _nudgeEndUT || RcsDrive.PropellantAmount(Vessel) < 0.01)
            {
                StopNudge();
                // Let two re-simulations see the result before the next one.
                _nextNudgeUT = now + 2 * Settings.RzBurnTrimInterval;
                return;
            }
            if (!_nudgeRcsOn)
            {
                _oldRcsNudge = Vessel.ActionGroups[KSPActionGroup.RCS];
                Vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
                _nudgeRcsOn = true;
            }
            _rcsCommand = _nudgeCommand;
        }

        private void StopNudge()
        {
            if (_nudging)
                _rcsCommand = null;
            _nudging = false;
            if (_nudgeRcsOn && Vessel != null)
                Vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, _oldRcsNudge);
            _nudgeRcsOn = false;
        }

        /// <summary>
        /// Re-simulates the rest of the burn from the actual state and picks the throttle (within
        /// BurnTrimMin..1) that ends it the offset distance from the target. Only the throttle changes.
        /// </summary>
        private void Trim(double now, double thrust, double massFlow, double distance, double speed, double angle)
        {
            _nextTrimUT = now + Settings.RzBurnTrimInterval;
            Orbit orbit = Vessel.orbit;
            Orbit target = _target.GetOrbit();
            Vector3d r = orbit.getRelativePositionAtUT(now);
            Vector3d v = orbit.getOrbitalVelocityAtUT(now);
            double mass = Vessel.totalMass;
            Func<double, BurnPlan> planWith = th => Simulate(r, v, now, mass, th * thrust, th * massFlow, target);

            double lo = Settings.RzBurnTrimMin, hi = 1.0;
            BurnPlan full = planWith(hi);
            BurnPlan plan;
            if (Shortfall(full) < 0)
            {
                _burnThrottle = hi;
                plan = full;
            }
            else if (Shortfall(plan = planWith(lo)) > 0)
                _burnThrottle = lo;
            else
            {
                // More throttle stops earlier: the shortfall grows with the throttle.
                for (int i = 0; i < 20; i++)
                {
                    double mid = 0.5 * (lo + hi);
                    BurnPlan p = planWith(mid);
                    if (Shortfall(p) > 0)
                        hi = mid;
                    else
                        lo = mid;
                    plan = p;
                }
                _burnThrottle = 0.5 * (lo + hi);
            }

            Log.Info($"rendezvous autopilot: braking: d {distance:F0} m, {speed:F1} m/s, throttle {_burnThrottle:F2} -> ends " +
                     $"{plan.Along:F0} m short / {plan.Lateral:F0} m aside in {plan.Seconds:F0} s{(Shortfall(full) < 0 ? " (full throttle is not enough)" : "")}, " +
                     $"aim off {angle:F1}°, warp {TimeWarp.CurrentRate:F0}x{(_nudging ? ", nudging" : "")}");

            double throttle = _burnThrottle;
            PlanNudge(now, push => Simulate(r, v + push, now, mass, throttle * thrust, throttle * massFlow, target));
        }

        private void BeginCleanup(double now)
        {
            // MechJeb's attitude and throttle go; RCS (with SAS stability assist) takes the rest.
            StopNudge();
            Core.Thrust.ThrustOff();
            Core.Thrust.Users.Remove(this);
            Core.Attitude.attitudeDeactivate();
            _engineTaken = false;

            _rcs = RcsCapability.Measure(Vessel);
            if (!_rcs.Any || RcsDrive.PropellantAmount(Vessel) < 0.01)
            {
                Log.Info("rendezvous autopilot: no usable RCS for the rest of the speed");
                Finish();
                return;
            }
            _brakePhase = BrakePhase.Cleanup;
            _cleanupDeadline = now + Settings.RzCleanupTimeout;
            _convergedSince = double.NaN;
            Status = Localizer.Format("#LOC_MJTweaks_RZ_AP_StepCleanup");
        }

        /// <summary>RCS takes the relative velocity to zero; no turning.</summary>
        private void Cleanup(double now, Vector3d velocity, double distance)
        {
            if (!AtPhysicsSpeed())
                return;
            double speed = velocity.magnitude;
            if (speed <= Settings.RzCleanupTolerance)
            {
                SetRcs(null);
                if (double.IsNaN(_convergedSince))
                    _convergedSince = now;
                else if (now - _convergedSince >= 1.0)
                {
                    Finish();
                    return;
                }
            }
            else
            {
                _convergedSince = double.NaN;
                SetRcs(RcsDrive.Command(Vessel.ReferenceTransform, velocity, _rcs, Settings.DockingDeadband));
            }

            if (now > _cleanupDeadline || RcsDrive.PropellantAmount(Vessel) < 0.01)
            {
                Log.Warning($"rendezvous autopilot: RCS clean-up stopped with {speed:F2} m/s left at {distance:F0} m");
                Finish();
            }
        }

        // ---------------------------------------------------------------- burn simulation

        private struct BurnPlan
        {
            /// <summary>At the end of the burn: distance still to go to the target along the approach direction (negative: passed it).</summary>
            public double Along;
            /// <summary>At the end of the burn: distance aside of the approach line.</summary>
            public double Lateral;
            /// <summary>The same sideways part of "from the vessel to the target", as a vector (orbit frame).</summary>
            public Vector3d LateralVec;
            public double Seconds;
            public double DeltaV;
            /// <summary>Closest distance to the target during the burn.</summary>
            public double MinDistance;
        }

        /// <summary>Positive when the burn ends farther from the target than the offset distance (started too early).</summary>
        private double Shortfall(BurnPlan p) =>
            p.Along - Math.Sqrt(Math.Max(0, _distance * _distance - p.Lateral * p.Lateral));

        /// <summary>
        /// Integrates the vessel (RK4, body-centred, orbit frame) under gravity and a thrust of fixed size
        /// pointed against the velocity relative to the target, until that velocity drops to
        /// Settings.RzBurnCutSpeed; the target follows its own orbit exactly.
        /// </summary>
        private BurnPlan Simulate(Vector3d r, Vector3d v, double t0, double mass, double thrust, double massFlow, Orbit target)
        {
            double mu = target.referenceBody.gravParameter;
            double t = t0;
            double m = mass;
            var plan = new BurnPlan { MinDistance = double.MaxValue };

            Vector3d Acceleration(Vector3d rr, Vector3d vv, double tt, double a)
            {
                double radius = rr.magnitude;
                Vector3d gravity = -mu / (radius * radius * radius) * rr;
                Vector3d relative = vv - target.getOrbitalVelocityAtUT(tt);
                double s = relative.magnitude;
                return s > 1e-9 ? gravity - a / s * relative : gravity;
            }

            for (int i = 0; i < 20000; i++)
            {
                Vector3d relativeVelocity = v - target.getOrbitalVelocityAtUT(t);
                double speed = relativeVelocity.magnitude;
                plan.MinDistance = Math.Min(plan.MinDistance, (r - target.getRelativePositionAtUT(t)).magnitude);
                double a = thrust / Math.Max(m, 1e-3);
                if (speed <= Settings.RzBurnCutSpeed || t - t0 > MaxBurnSeconds)
                    break;

                double dt = Math.Min(1.0, Math.Max(0.05, 0.2 * speed / a));
                Vector3d k1v = Acceleration(r, v, t, a);
                Vector3d k1r = v;
                Vector3d k2v = Acceleration(r + 0.5 * dt * k1r, v + 0.5 * dt * k1v, t + 0.5 * dt, a);
                Vector3d k2r = v + 0.5 * dt * k1v;
                Vector3d k3v = Acceleration(r + 0.5 * dt * k2r, v + 0.5 * dt * k2v, t + 0.5 * dt, a);
                Vector3d k3r = v + 0.5 * dt * k2v;
                Vector3d k4v = Acceleration(r + dt * k3r, v + dt * k3v, t + dt, a);
                Vector3d k4r = v + dt * k3v;
                r += dt / 6 * (k1r + 2 * k2r + 2 * k3r + k4r);
                v += dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
                t += dt;
                m -= massFlow * dt;
                plan.DeltaV += a * dt;
            }

            Vector3d toTarget = target.getRelativePositionAtUT(t) - r;
            plan.Along = Vector3d.Dot(toTarget, _approachDir);
            plan.LateralVec = toTarget - plan.Along * _approachDir;
            plan.Lateral = plan.LateralVec.magnitude;
            plan.Seconds = t - t0;
            return plan;
        }

        private const double MaxBurnSeconds = 3000;

        /// <summary>
        /// Full thrust (MechJeb's figure, with the KSPIE thrust cap) and the mass flow at full thrust, from the
        /// vacuum Isp of the ignited engines weighted by their thrust. False when there is no thrust.
        /// </summary>
        private bool EngineModel(out double thrust, out double massFlow)
        {
            thrust = Core.VesselState.thrustAvailable;
            massFlow = 0;
            if (!(thrust > 1e-3))
                return false;

            double sumThrust = 0, sumFlow = 0;
            foreach (ModuleEngines engine in Vessel.FindPartModulesImplementing<ModuleEngines>())
            {
                if (!engine.EngineIgnited || !engine.isOperational)
                    continue;
                double f = engine.maxThrust * engine.thrustPercentage / 100.0;
                double isp = engine.atmosphereCurve.Evaluate(0f);
                if (f <= 0 || isp <= 0)
                    continue;
                sumThrust += f;
                sumFlow += f / (isp * 9.80665);
            }
            massFlow = sumThrust > 0 ? thrust * sumFlow / sumThrust : 0;
            return true;
        }

        /// <summary>Line of sight to the target and relative velocity, both from the orbits (a packed target's scene position is unreliable).</summary>
        private static void Relative(Orbit ours, Orbit target, double ut, out Vector3d toTarget, out Vector3d velocity)
        {
            toTarget = target.WorldPositionAtUT(ut) - ours.WorldPositionAtUT(ut);
            velocity = ours.WorldOrbitalVelocityAtUT(ut) - target.WorldOrbitalVelocityAtUT(ut);
        }

        private void Finish()
        {
            double now = Planetarium.GetUniversalTime();
            Orbit orbit = Vessel.orbit;
            Orbit target = _target.GetOrbit();
            double distance = orbit.Separation(target, now);
            double speed = (orbit.WorldOrbitalVelocityAtUT(now) - target.WorldOrbitalVelocityAtUT(now)).magnitude;
            Current = Step.Done;
            Failed = false;
            Status = Localizer.Format("#LOC_MJTweaks_RZ_AP_Done", distance < 10000 ? $"{distance:F0} m" : $"{distance / 1000:F1} km", speed.ToString("F2"));
            Log.Info($"rendezvous autopilot: done, {distance:F0} m from the target, relative speed {speed:F2} m/s");
            Users.Clear();
            Enabled = false;
        }

        // ---------------------------------------------------------------- helpers

        /// <summary>The global auto physics warp choice: its rate when wanted, 1x otherwise; nothing when it is off.</summary>
        private void PhysicsWarp(bool wanted) => AutoPhysicsWarp.Apply(Core, Vessel, wanted);

        /// <summary>Out of on-rails warp and in physics; physics warp (the player's choice) is left alone.</summary>
        private bool OutOfRailsWarp()
        {
            if (TimeWarp.WarpMode == TimeWarp.Modes.HIGH && TimeWarp.CurrentRateIndex > 0)
            {
                Core.Warp.MinimumWarp(true);
                return false;
            }
            return !Vessel.packed;
        }

        /// <summary>Out of warp and in physics, ready for RCS.</summary>
        private bool AtPhysicsSpeed()
        {
            if (TimeWarp.CurrentRateIndex > 0)
            {
                Core.Warp.MinimumWarp(true);
                return false;
            }
            return !Vessel.packed;
        }

        /// <summary>RCS group on and SAS in stability assist while commanding; old state back afterwards.</summary>
        private void SetRcs(Vector3d? command)
        {
            _rcsCommand = command;
            if (command.HasValue && !_controlsTaken)
            {
                _oldRcs = Vessel.ActionGroups[KSPActionGroup.RCS];
                _oldSas = Vessel.ActionGroups[KSPActionGroup.SAS];
                _oldSasMode = Vessel.Autopilot.Mode;
                Vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
                Vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
                Vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.StabilityAssist);
                _controlsTaken = true;
            }
        }

        private void ReleaseControls()
        {
            StopNudge();
            _rcsCommand = null;
            if (_engineTaken)
            {
                Core.Thrust.ThrustOff();
                Core.Thrust.Users.Remove(this);
                Core.Attitude.attitudeDeactivate();
                _engineTaken = false;
            }
            if (!_controlsTaken || Vessel == null)
                return;
            Vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, _oldRcs);
            Vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, _oldSas);
            if (_oldSas)
                Vessel.Autopilot.SetMode(_oldSasMode);
            _controlsTaken = false;
        }

        public static string StepName(Step step)
        {
            switch (step)
            {
                case Step.Circularize: return Localizer.Format("#LOC_MJTweaks_RZ_AP_StepCircularize");
                case Step.MatchPlanes: return Localizer.Format("#LOC_MJTweaks_RZ_AP_StepPlanes");
                case Step.Phasing: return Localizer.Format("#LOC_MJTweaks_RZ_AP_StepPhasing");
                case Step.Hohmann: return Localizer.Format("#LOC_MJTweaks_RZ_AP_StepHohmann");
                case Step.WaitCorrection: return Localizer.Format("#LOC_MJTweaks_RZ_AP_StepWaitCorrection");
                case Step.RcsCorrection: return Localizer.Format("#LOC_MJTweaks_RZ_AP_StepRcs");
                case Step.Matching: return Localizer.Format("#LOC_MJTweaks_RZ_AP_StepMatching");
                default: return step.ToString();
            }
        }

        private static string OrbitText(Orbit o) =>
            $"{o.PeA / 1000:F1} x {o.ApA / 1000:F1} km (e {o.eccentricity:F4})";

        /// <summary>A whole route estimated in advance: total dV and time until the velocities are matched.</summary>
        public struct Route
        {
            public bool Valid;
            /// <summary>The first circularization is at periapsis (false: at apoapsis, or the orbit is already round).</summary>
            public bool AtPeriapsis;
            /// <summary>The orbit is round already, there is no choice of apsis.</summary>
            public bool Round;
            public double DeltaV;
            public double Seconds;
        }

        /// <summary>
        /// Rough estimate of the whole route with impulsive burns between circular orbits: wait for the
        /// apsis and circularize there, plane change on that orbit (average wait: a quarter orbit), waiting
        /// orbit if needed, average wait for the transfer window (half a synodic period), Hohmann transfer.
        /// Invalid when that apsis cannot be used: periapsis too low for a circular orbit, apoapsis outside
        /// the sphere of influence.
        /// </summary>
        public static Route PlanRoute(Orbit orbit, Orbit target, double now, bool atPeriapsis)
        {
            CelestialBody body = orbit.referenceBody;
            double mu = body.gravParameter;
            var route = new Route { AtPeriapsis = atPeriapsis };
            double dv = 0;
            double seconds = 0;

            double radius = orbit.semiMajorAxis;
            if (orbit.eccentricity > Settings.RzRoundEccentricity)
            {
                if (orbit.eccentricity >= 1)
                    return route;
                if (atPeriapsis)
                {
                    if (orbit.PeR < LowestRadius(body))
                        return route;
                    radius = orbit.PeR;
                    seconds += orbit.NextPeriapsisTime(now) - now;
                    dv += Math.Sqrt(mu * (2 / radius - 1 / orbit.semiMajorAxis)) - Math.Sqrt(mu / radius);
                }
                else
                {
                    if (orbit.ApR > body.sphereOfInfluence)
                        return route;
                    radius = orbit.ApR;
                    seconds += orbit.NextApoapsisTime(now) - now;
                    dv += Math.Sqrt(mu / radius) - Math.Sqrt(mu * (2 / radius - 1 / orbit.semiMajorAxis));
                }
            }
            else
                route.Round = true;

            double inclination = orbit.RelativeInclination(target);
            if (inclination > Settings.RzPlaneAngle)
            {
                dv += 2 * Math.Sqrt(mu / radius) * Math.Sin(0.5 * inclination * Math.PI / 180);
                seconds += 0.25 * Period(mu, radius);
            }

            double targetRadius = target.semiMajorAxis;
            if (NeedsPhasing(radius, targetRadius))
            {
                // Round orbit in the target's plane: the autopilot's own choice, timed from the actual phase.
                if (route.Round && inclination <= Settings.RzPlaneAngle)
                {
                    PhasingChoice choice = ChoosePhasing(orbit, target, now);
                    if (choice.Chosen.Valid)
                    {
                        route.Valid = true;
                        route.DeltaV = choice.Chosen.DeltaV;
                        route.Seconds = choice.Chosen.Seconds;
                        return route;
                    }
                }
                double waiting = PhasingRadius(target);
                if (waiting > 0)
                {
                    dv += Hohmann(mu, radius, waiting);
                    seconds += 0.5 * Period(mu, 0.5 * (radius + waiting));
                    radius = waiting;
                }
            }

            double ours = Period(mu, radius);
            double theirs = Period(mu, targetRadius);
            if (Math.Abs(ours - theirs) > 1e-6 * theirs)
                seconds += 0.5 * ours * theirs / Math.Abs(ours - theirs);
            dv += Hohmann(mu, radius, targetRadius);
            seconds += 0.5 * Period(mu, 0.5 * (radius + targetRadius));

            route.Valid = true;
            route.DeltaV = dv;
            route.Seconds = seconds;
            return route;
        }

        /// <summary>
        /// For an eccentric orbit: the route via the apsis with less dV, unless it takes more than
        /// Settings.RzSlowFactor times longer while the faster one costs at most Settings.RzDvMargin more.
        /// For a round orbit the only route (other is invalid).
        /// </summary>
        public static Route ChooseRoute(Orbit orbit, Orbit target, double now, out Route other)
        {
            Route apoapsis = PlanRoute(orbit, target, now, false);
            if (apoapsis.Round)
            {
                other = default;
                return apoapsis;
            }
            Route periapsis = PlanRoute(orbit, target, now, true);
            if (!periapsis.Valid || !apoapsis.Valid)
            {
                other = periapsis.Valid ? apoapsis : periapsis;
                return periapsis.Valid ? periapsis : apoapsis;
            }

            Route cheap = apoapsis.DeltaV <= periapsis.DeltaV ? apoapsis : periapsis;
            Route fast = apoapsis.Seconds <= periapsis.Seconds ? apoapsis : periapsis;
            Route chosen = cheap.Seconds > Settings.RzSlowFactor * fast.Seconds
                           && fast.DeltaV - cheap.DeltaV <= Settings.RzDvMargin * cheap.DeltaV
                ? fast
                : cheap;
            other = chosen.AtPeriapsis ? apoapsis : periapsis;
            return chosen;
        }

        private static string RouteText(Route r) =>
            !r.Valid ? "(unusable apsis)" : $"{(r.Round ? "round orbit" : r.AtPeriapsis ? "periapsis" : "apoapsis")}: dV {r.DeltaV:F0} m/s, {r.Seconds / 3600:F1} h";

        private static double Period(double mu, double radius) => 2 * Math.PI * Math.Sqrt(radius * radius * radius / mu);

        private static double Hohmann(double mu, double r1, double r2)
        {
            double sum = r1 + r2;
            return Math.Abs(Math.Sqrt(mu / r1) * (Math.Sqrt(2 * r2 / sum) - 1))
                   + Math.Abs(Math.Sqrt(mu / r2) * (1 - Math.Sqrt(2 * r1 / sum)));
        }
    }
}
