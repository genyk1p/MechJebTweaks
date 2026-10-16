using System;
using System.Collections.Generic;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.Docking
{
    /// <summary>
    /// Docking autopilot, RCS only.
    ///
    /// Far approach (from up to Settings.DockingFarStartMax down to the 100 m checkpoint): MechJeb's
    /// Lambert solver gives the velocity needed now to reach the target in distance / far speed
    /// seconds — orbital curvature included, so sideways drift does not build up — and RCS removes the
    /// difference until the predicted closest approach is within Settings.DockingFarMissTolerance.
    /// Then one coast, in warp, straight to the braking point for the 100 m checkpoint: on rails the
    /// orbits — and so the closest approach — do not change, all fine work is left to the near approach.
    ///
    /// Near approach (from Settings.DockingNearStartMax, or after the far approach): checkpoint to
    /// checkpoint. At each checkpoint both vessels aim at each other (SAS target hold) while RCS sets
    /// the velocity straight at the target point; then RCS goes off and the vessel coasts — in warp
    /// when the leg is long enough — to the next checkpoint. After the last checkpoint RCS stays off
    /// (except one short sideways check) and SAS alone holds the aim until the ports latch.
    ///
    /// Rules that hold in every phase:
    /// - braking is triggered by distance: once the vessel reaches the point where braking for the
    ///   next checkpoint must begin, that checkpoint counts as reached, whatever else is going on;
    /// - from the 100 m checkpoint on the wanted speed is the touchdown speed.
    ///
    /// RCS is driven by an own proportional controller (no integral term): the velocity error is
    /// divided by the available acceleration on each axis and a time constant. MechJeb's PID
    /// controller wound up and circled around the wanted direction on heavy vessels with weak RCS.
    ///
    /// MechJeb picks this module up like its own (it scans all assemblies for ComputerModule types);
    /// the docking window starts and stops it through Users.
    /// </summary>
    public class DockingAutopilot : ComputerModule
    {
        public enum Phase
        {
            Idle,
            FarCorrecting,
            FarCoasting,
            Correcting,
            Coasting,
            Final,
            Docked,
            Aborted
        }

        public Phase State { get; private set; } = Phase.Idle;
        public string Status { get; private set; } = string.Empty;

        /// <summary>Checkpoint the vessel is currently heading to, m (0 = the target itself).</summary>
        public double NextCheckpoint { get; private set; }

        /// <summary>Approach speed for the current leg, m/s.</summary>
        public double LegSpeed { get; private set; }

        private readonly List<double> _checkpoints = new List<double>();
        private double _phaseStartUT;
        private double _convergedSinceUT;
        private double _lastRcsSwitchUT = double.NegativeInfinity;
        private double _lastAxesLogUT = double.NaN;
        private Vector3 _lastRelativeLocal;
        private double _finalExpectedSeconds;
        private bool _rcsUsedThisCorrection;
        private bool _finalCheckDone;
        private bool _inFinalCheck;
        private Vector3d _lastRelativePosition;
        private double _lastClosingSpeed;
        private Vessel _vessel;

        // Far approach
        private double _nextFarSolveUT;
        private double _farClosest;
        private double _farClosestUT;
        private double _farBrakeUT;
        private double _nextFarLogUT;
        private OrbitGeometry _farGeometry;
        private Vector3d _farDeltaV;
        private double _farTime;

        /// <summary>RCS command for Drive, in reference axes (MechJeb sign convention), or null when idle.</summary>
        private Vector3d? _rcsCommand;
        private bool _rcsOwned;

        private const double ConvergedHold = 1.0;
        private const double FarSolveInterval = 0.5;

        public DockingAutopilot(MechJebCore core) : base(core)
        {
        }

        // ---------------------------------------------------------------- start / stop

        internal static string CannotStartReason(DockingPair pair)
        {
            if (!pair.AllConditionsOk)
                return "#LOC_MJTweaks_DK_AP_NotReady";
            if (pair.Active.Distance > Settings.DockingFarStartMax)
                return "#LOC_MJTweaks_DK_AP_TooFar";
            // Moving apart is fine: the first correction turns the velocity towards the target, and a
            // coast that stops closing goes back to correcting (see Coast).
            if (pair.Active.ClosingSpeed > Settings.DockingAbortSpeedFactor * pair.Rcs.FarSpeed + 0.3)
                return "#LOC_MJTweaks_DK_AP_StartTooFast";
            return null;
        }

        public void Start(object user)
        {
            Users.Add(user);
        }

        public void Stop(string reasonKey)
        {
            if (State != Phase.Docked)
            {
                State = Phase.Aborted;
                Status = reasonKey;
            }
            Users.Clear();
            Enabled = false;
        }

        protected override void OnModuleEnabled()
        {
            _vessel = Vessel;
            GameEvents.onPartCouple.Add(OnPartCouple);

            DockingPair pair = DockingPair.Measure(Vessel);
            _checkpoints.Clear();
            _finalCheckDone = false;
            foreach (double cp in new[] { 100.0, 50.0, 20.0 })
            {
                // A checkpoint needs some room ahead of us to make sense.
                if (cp < pair.Active.Distance - 5)
                    _checkpoints.Add(cp);
            }

            Log($"started at d={pair.Active.Distance:F1} m, vc={pair.Active.ClosingSpeed:F2} m/s, " +
                $"rcs back={pair.Rcs.Back:F3} lat={pair.Rcs.Lateral:F3} m/s², vFar={pair.Rcs.FarSpeed:F2} vTouch={pair.Rcs.TouchSpeed:F2}, " +
                $"checkpoints: {(_checkpoints.Count > 0 ? string.Join(" ", _checkpoints) : "none")}");

            EngageTargetHoldBoth();
            if (pair.Active.Distance > Settings.DockingNearStartMax)
                BeginFarCorrection(pair, "start (far approach)");
            else
                BeginCorrection(pair, "start");
        }

        protected override void OnModuleDisabled()
        {
            GameEvents.onPartCouple.Remove(OnPartCouple);
            SetRcs(null);
            if (TimeWarp.CurrentRateIndex > 0)
                Core.Warp.MinimumWarp();
            if (State != Phase.Docked && State != Phase.Aborted)
            {
                State = Phase.Aborted;
                Status = "#LOC_MJTweaks_DK_AP_Stopped";
            }
            Log($"stopped: {State} ({Status})");
        }

        /// <summary>
        /// Handled right in the event: after docking the vessels merge and MechJeb may stop calling
        /// this core's modules, so waiting for the next OnFixedUpdate would leave the module running.
        /// The speed is the one measured in the last physics frame before the latch — measuring it
        /// during the merge gave nonsense (-0.09 m/s).
        /// </summary>
        private void OnPartCouple(GameEvents.FromToAction<Part, Part> action)
        {
            if (action.from?.vessel != _vessel && action.to?.vessel != _vessel)
                return;

            State = Phase.Docked;
            Status = "#LOC_MJTweaks_DK_AP_Docked";
            Log($"docked at {_lastClosingSpeed:F2} m/s");
            Stop(Status);
        }

        // ---------------------------------------------------------------- main loop

        public override void OnFixedUpdate()
        {
            if (Vessel != _vessel)
            {
                Stop("#LOC_MJTweaks_DK_AP_VesselChanged");
                return;
            }

            if (Vessel.targetObject == null)
            {
                Stop("#LOC_MJTweaks_DK_AP_TargetLost");
                return;
            }

            DockingPair pair = DockingPair.Measure(Vessel);
            if (pair.OtherVessel == null || !pair.OtherVessel.loaded)
            {
                Stop("#LOC_MJTweaks_DK_AP_TargetLost");
                return;
            }
            // The contact speed for the "docked at" line: the last value before the ports touch — inside a
            // metre the latch magnets and the bump already change it (test 14 logged -0.26 at 1 m/s).
            if (pair.Active.Distance > 1.0)
                _lastClosingSpeed = pair.Active.ClosingSpeed;

            if (_rcsCommand.HasValue && pair.RcsPropellantAmount < 0.01)
            {
                Stop("#LOC_MJTweaks_DK_AP_NoPropellant");
                return;
            }

            // Too fast while coasting: the RCS may not be able to stop in time. (While correcting the
            // vessel is braking anyway; on the final approach it brakes instead, see Final.)
            // Far away the scene distance of the packed target is unreliable — use the orbits there.
            double closingNow = State == Phase.FarCoasting ? FromOrbits(Planetarium.GetUniversalTime()).Closing : pair.Active.ClosingSpeed;
            bool coasting = State == Phase.Coasting || State == Phase.FarCoasting;
            if (coasting && closingNow > Settings.DockingAbortSpeedFactor * LegSpeed + 0.3)
            {
                Stop("#LOC_MJTweaks_DK_AP_TooFast");
                return;
            }

            // KSP drops SAS target hold on vessel switches and on leaving warp: keep it on our vessel,
            // and give it to the other one as soon as it unfreezes on approach.
            if (!Vessel.packed)
                DockingTelemetry.EngageTargetHold(Vessel, "autopilot");
            if (pair.Simulated)
                DockingTelemetry.EngageTargetHold(pair.OtherVessel, "autopilot");

            switch (State)
            {
                case Phase.FarCorrecting:
                    FarCorrect(pair);
                    break;
                case Phase.FarCoasting:
                    FarCoast(pair);
                    break;
                case Phase.Correcting:
                    Correct(pair);
                    break;
                case Phase.Coasting:
                    Coast(pair);
                    break;
                case Phase.Final:
                    Final(pair);
                    break;
            }
        }

        /// <summary>Translation command, applied after the player's input.</summary>
        public override void Drive(FlightCtrlState s)
        {
            if (!_rcsCommand.HasValue)
                return;

            // The command is proportional to (current - wanted) velocity in reference axes.
            RcsDrive.Apply(s, _rcsCommand.Value);
        }

        // ---------------------------------------------------------------- far approach

        private void BeginFarCorrection(DockingPair pair, string why)
        {
            NextCheckpoint = _checkpoints.Count > 0 ? _checkpoints[0] : 0;
            LegSpeed = pair.Rcs.FarSpeed;
            State = Phase.FarCorrecting;
            Status = "#LOC_MJTweaks_DK_AP_FarCorrecting";
            _phaseStartUT = Planetarium.GetUniversalTime();
            _convergedSinceUT = double.NaN;
            _rcsUsedThisCorrection = false;
            _lastAxesLogUT = double.NaN;
            _nextFarSolveUT = double.NegativeInfinity;
            if (TimeWarp.CurrentRateIndex > 0)
                Core.Warp.MinimumWarp(true);
            double now = Planetarium.GetUniversalTime();
            SolveIntercept(pair, now);
            Log($"{why}: d(orbits)={_farGeometry.Distance:F1} m, d(scene)={pair.Active.Distance:F1} m, vc(orbits)={_farGeometry.Closing:F2}, " +
                $"intercept in {_farTime:F0} s needs dV={_farDeltaV.magnitude:F3} m/s, closest approach {_farClosest:F1} m in {_farClosestUT - now:F0} s");
        }

        private struct OrbitGeometry
        {
            public double Distance;
            public double Closing;
        }

        /// <summary>
        /// Distance and closing speed from the two orbits at a given time. Far away the other vessel
        /// is packed, and its scene position is updated out of step with ours: at ~3 km/s orbital
        /// speed that made the scene distance jump by hundreds of metres, worst in warp (test 7:
        /// 1354 vs 1625 m in the same second, then warped past the station). The orbits are exact.
        /// </summary>
        private OrbitGeometry FromOrbits(double ut)
        {
            Orbit our = _vessel.orbit;
            Orbit target = _vessel.targetObject.GetOrbit();
            Vector3d toTarget = target.WorldPositionAtUT(ut) - our.WorldPositionAtUT(ut);
            Vector3d relative = our.WorldOrbitalVelocityAtUT(ut) - target.WorldOrbitalVelocityAtUT(ut);
            double distance = toTarget.magnitude;
            return new OrbitGeometry
            {
                Distance = distance,
                Closing = distance > 0 ? Vector3d.Dot(relative, toTarget / distance) : 0
            };
        }

        /// <summary>
        /// Closest approach on the coming pass only (within window seconds), not over a whole orbit:
        /// a close pass an orbit later does not help, we would fly past the target now.
        /// </summary>
        private double ClosestApproach(double now, double window, out double closestUT)
        {
            double step = Math.Max(1.0, window / 400);
            double best = double.MaxValue;
            closestUT = now;
            for (double t = now; t <= now + window; t += step)
            {
                double d = FromOrbits(t).Distance;
                if (d < best) { best = d; closestUT = t; }
            }
            double from = Math.Max(now, closestUT - step);
            double fine = 2 * step / 40;
            for (double t = from; t <= closestUT + step; t += fine)
            {
                double d = FromOrbits(t).Distance;
                if (d < best) { best = d; closestUT = t; }
            }
            return best;
        }

        /// <summary>
        /// Velocity change needed now to meet the target (its orbit position, i.e. its centre) in
        /// distance / far speed seconds, from MechJeb's Lambert solver, in world axes; plus the
        /// predicted closest approach of the coming pass.
        /// </summary>
        private void SolveIntercept(DockingPair pair, double now)
        {
            Orbit target = _vessel.targetObject.GetOrbit();
            _farGeometry = FromOrbits(now);
            _farTime = Math.Max(Settings.DockingFarMinTime, _farGeometry.Distance / Math.Max(pair.Rcs.FarSpeed, 0.05));
            _farDeltaV = OrbitalManeuverCalculator.DeltaVToInterceptAtTime(_vessel.orbit, now, target, _farTime).v1;
            _farClosest = ClosestApproach(now, 2 * _farTime + 60, out _farClosestUT);
            _nextFarSolveUT = now + FarSolveInterval;
        }

        /// <summary>
        /// Course good enough to coast all the way to the near approach: the predicted closest
        /// approach is within the tolerance, or the intercept course is already flown (the needed
        /// velocity change is negligible). There is deliberately no closing-speed condition: on the
        /// curved relative path an exact intercept closes slower than distance / time (test 8:
        /// ~1.05 m/s against 1.25 required), and that condition kept the correction going forever.
        /// </summary>
        private bool FarCourseGood(double missTolerance, double velocityTolerance) =>
            _farClosest <= missTolerance || _farDeltaV.magnitude <= velocityTolerance;

        /// <summary>Distance rule of the far approach, from the orbits.</summary>
        private bool FarBrakingPointReached(DockingPair pair, double ut)
        {
            OrbitGeometry g = FromOrbits(ut);
            return g.Distance <= NextCheckpoint + pair.Rcs.BrakingDistance(Math.Max(g.Closing, 0), pair.Rcs.TouchSpeed);
        }

        /// <summary>
        /// RCS steers towards MechJeb's intercept velocity (re-solved twice a second) until the
        /// predicted closest approach is small enough; then one coast to the near approach.
        /// </summary>
        private void FarCorrect(DockingPair pair)
        {
            if (FarHandOverToNear(pair))
                return;

            // Physics warp skews the orbit of the active vessel against the clock: in test 9 the distance
            // by the orbits jumped by +200 m at 4x and back at 1x, and so did the intercept solution.
            // A correction finished under it would plan the braking point ~100 s off. Back to 1x.
            if (TimeWarp.WarpMode == TimeWarp.Modes.LOW && TimeWarp.CurrentRateIndex > 0)
            {
                TimeWarp.SetRate(0, true);
                Log("physics warp is not used during the far correction (it skews the orbit calculation), back to 1x");
            }

            double now = Planetarium.GetUniversalTime();
            if (now >= _nextFarSolveUT)
                SolveIntercept(pair, now);

            // Error in the controller's convention (current - wanted) is minus the needed change.
            Vector3d error = -_farDeltaV;
            double size = error.magnitude;
            double tolerance = Settings.DockingFarMissTolerance;
            double velocityTolerance = Settings.DockingFarVelocityTolerance;
            // Hysteresis: once RCS was used and switched off, it only comes back for twice the miss.
            bool courseGood = _rcsCommand.HasValue || !_rcsUsedThisCorrection
                ? FarCourseGood(tolerance, velocityTolerance)
                : FarCourseGood(2 * tolerance, 2 * velocityTolerance);
            bool wantRcs = !courseGood;
            // As in the near approach: aim first, so SAS does not turn with RCS thrusters.
            if (!_rcsCommand.HasValue && pair.Active.PointingError > Settings.DockingAimBeforeRcs)
                wantRcs = false;
            if (wantRcs != _rcsCommand.HasValue && now - _lastRcsSwitchUT < Settings.DockingRcsMinSwitch)
                wantRcs = _rcsCommand.HasValue;

            Vector3d command = CommandFor(error, pair.Rcs);
            if (wantRcs)
            {
                SetRcs(command);
                _rcsUsedThisCorrection = true;
                _convergedSinceUT = double.NaN;
            }
            else
            {
                SetRcs(null);
                // RCS off only counts as done when the course is good, not while it waits for the
                // nose (test 11: "far corrected" after 1 s with a 158 m miss and 2.7 m/s still needed).
                if (!courseGood)
                    _convergedSinceUT = double.NaN;
                else if (double.IsNaN(_convergedSinceUT))
                    _convergedSinceUT = now;
            }

            LogAxes(now, error, command, RelativeVelocity());
            if (now >= _nextFarLogUT)
            {
                _nextFarLogUT = now + 5;
                Log($"far correcting: d(orbits)={_farGeometry.Distance:F1} m, vc(orbits)={_farGeometry.Closing:F3}, " +
                    $"dV needed={size:F3} m/s, closest approach {_farClosest:F1} m in {_farClosestUT - now:F0} s, RCS {(_rcsCommand.HasValue ? "on" : "off")}");
            }

            bool converged = !double.IsNaN(_convergedSinceUT) && now - _convergedSinceUT >= ConvergedHold;
            if (converged || now - _phaseStartUT > Settings.DockingFarCorrectionTimeout)
            {
                SetRcs(null);
                SolveIntercept(pair, now);
                _farBrakeUT = FindBrakingUT(pair, now);
                Log($"far {(converged ? "corrected" : "correction time limit, going on")}: d(orbits)={_farGeometry.Distance:F1} m, " +
                    $"d(scene)={pair.Active.Distance:F1} m, vc(orbits)={_farGeometry.Closing:F3}, dV left={size:F3} m/s, " +
                    $"closest approach {_farClosest:F1} m in {_farClosestUT - now:F0} s, braking point in {_farBrakeUT - now:F0} s, took {now - _phaseStartUT:F0} s");
                State = Phase.FarCoasting;
                Status = "#LOC_MJTweaks_DK_AP_FarCoasting";
                _phaseStartUT = now;
            }
        }

        /// <summary>
        /// When the braking point for the 100 m checkpoint will be reached, found by stepping along
        /// the orbits. On rails they do not change, so the warp can go exactly there. If the pass
        /// never comes within the braking distance, stop shortly before the closest approach.
        /// </summary>
        private double FindBrakingUT(DockingPair pair, double now)
        {
            for (double t = now; t <= _farClosestUT; t += 1.0)
            {
                if (FarBrakingPointReached(pair, t))
                    return t;
            }
            return Math.Max(now, _farClosestUT - 30);
        }

        /// <summary>
        /// One coast — in warp when allowed — exactly to the precomputed braking point for the 100 m
        /// checkpoint. No checks on the way: on rails the orbits do not change; all fine corrections
        /// happen in the near approach.
        /// </summary>
        private void FarCoast(DockingPair pair)
        {
            double now = Planetarium.GetUniversalTime();
            if (now >= _farBrakeUT - 0.5)
            {
                if (TimeWarp.CurrentRateIndex > 0)
                    Core.Warp.MinimumWarp(true);
                if (TimeWarp.CurrentRateIndex > 0 || Vessel.packed)
                    return; // wait until we are back in physics
                HandOverToNear(pair);
                return;
            }

            double secondsLeft = _farBrakeUT - now;
            if (pair.WarpLimit > 1 && secondsLeft > Settings.DockingMinWarpSeconds)
                WarpTowards(_farBrakeUT, pair.WarpLimit);
            else if (TimeWarp.CurrentRateIndex > 0)
                Core.Warp.MinimumWarp(true);
        }

        /// <summary>While correcting: at the braking point (from the orbits) the near approach takes over.</summary>
        private bool FarHandOverToNear(DockingPair pair)
        {
            if (!FarBrakingPointReached(pair, Planetarium.GetUniversalTime()))
                return false;

            if (TimeWarp.CurrentRateIndex > 0)
                Core.Warp.MinimumWarp(true);
            if (TimeWarp.CurrentRateIndex > 0 || Vessel.packed)
                return true; // wait until we are back in physics

            HandOverToNear(pair);
            return true;
        }

        private void HandOverToNear(DockingPair pair)
        {
            SetRcs(null);
            OrbitGeometry g = FromOrbits(Planetarium.GetUniversalTime());
            if (_checkpoints.Count > 0 && _checkpoints[0] >= 100)
                _checkpoints.RemoveAt(0);
            BeginCorrection(pair, $"far approach done (d(orbits)={g.Distance:F1} m, vc(orbits)={g.Closing:F2}), braking for the 100 m checkpoint");
        }

        // ---------------------------------------------------------------- near approach

        private void BeginCorrection(DockingPair pair, string why)
        {
            NextCheckpoint = _checkpoints.Count > 0 ? _checkpoints[0] : 0;
            LegSpeed = SpeedForLeg(pair);

            State = Phase.Correcting;
            Status = "#LOC_MJTweaks_DK_AP_Correcting";
            _phaseStartUT = Planetarium.GetUniversalTime();
            _convergedSinceUT = double.NaN;
            _lastAxesLogUT = double.NaN; // measured acceleration starts fresh for each correction
            _rcsUsedThisCorrection = false;
            _inFinalCheck = false;
            if (TimeWarp.CurrentRateIndex > 0)
                Core.Warp.MinimumWarp(true);
            Log($"{why}: d={pair.Active.Distance:F1} m, vc={pair.Active.ClosingSpeed:F2}, correcting to {LegSpeed:F2} m/s towards {(NextCheckpoint > 0 ? NextCheckpoint + " m" : "target")}");
        }

        /// <summary>
        /// Heading for the 100 m checkpoint: the far approach speed, but no faster than still lets us
        /// brake to touchdown speed before it. Once the 100 m checkpoint is reached (or counted as
        /// reached at its braking point, which can be well beyond 100 m): touchdown speed.
        /// </summary>
        private double SpeedForLeg(DockingPair pair)
        {
            double touch = pair.Rcs.TouchSpeed;
            if (NextCheckpoint < 100 || pair.Active.Distance <= 100 + 5)
                return touch;

            double room = pair.Active.Distance - 100;
            double fitting = Math.Sqrt(touch * touch + 2 * pair.Rcs.Back * room / Settings.DockingSafetyFactor);
            return Math.Max(touch, Math.Min(pair.Rcs.FarSpeed, fitting));
        }

        /// <summary>
        /// Distance rule: has the vessel reached the point where braking for the next checkpoint must
        /// begin? Every checkpoint is followed by the touchdown speed.
        /// </summary>
        private bool ReachedBrakingPoint(DockingPair pair)
        {
            if (NextCheckpoint <= 0)
                return false;
            double stopAt = NextCheckpoint + pair.Rcs.BrakingDistance(Math.Max(pair.Active.ClosingSpeed, 0), pair.Rcs.TouchSpeed);
            return pair.Active.Distance <= stopAt;
        }

        /// <summary>RCS sets the relative velocity straight at the target point; both vessels aim meanwhile.</summary>
        private void Correct(DockingPair pair)
        {
            double now = Planetarium.GetUniversalTime();

            // Distance rule while correcting: the next checkpoint counts as reached, carry on with the
            // next leg's (touchdown) speed — braking starts right away. Not when this correction already
            // brakes to touchdown speed: then the checkpoint is kept for its own sideways correction
            // (test 9: right after the far approach, still at 1.9 m/s, it dropped the 50 m checkpoint
            // and coasted 96 m to 20 m). It only goes once the vessel actually gets there.
            bool brakingToTouch = LegSpeed <= pair.Rcs.TouchSpeed + 0.01;
            if (brakingToTouch ? NextCheckpoint > 0 && pair.Active.Distance <= NextCheckpoint + 5 : ReachedBrakingPoint(pair))
            {
                _checkpoints.RemoveAt(0);
                BeginCorrection(pair, brakingToTouch ? "checkpoint passed while correcting" : "braking point reached while correcting");
                return;
            }

            Vector3d direction = DirectionToTarget();
            // Velocities from the orbits, like MechJeb: a port on a turning vessel sweeps sideways,
            // its vessel's orbit does not — that sweep is not something to chase with RCS.
            Vector3d relative = RelativeVelocity();
            Vector3d desired = direction * LegSpeed;

            Vector3d error = relative - desired;
            double closingError = Math.Abs(Vector3d.Dot(error, direction));
            double lateralError = (error - Vector3d.Dot(error, direction) * direction).magnitude;

            double closingTolerance = Math.Max(Settings.DockingClosingTolerance, Settings.DockingClosingToleranceShare * LegSpeed);
            // On the last leg (after 20 m, RCS off for the rest of the way) sideways drift turns directly
            // into a miss at the port, so it gets a tighter tolerance.
            // The check at FinalCheckDistance is the last chance before contact: tightest of all.
            double lateralTolerance = _inFinalCheck ? Settings.DockingFinalCheckLateralTolerance
                : NextCheckpoint > 0 ? Settings.DockingLateralTolerance
                : Settings.DockingFinalLateralTolerance;
            bool withinTolerance = closingError <= closingTolerance && lateralError <= lateralTolerance;
            // Hysteresis: once off, RCS only comes back when the error doubles.
            bool withinRestart = closingError <= 2 * closingTolerance && lateralError <= 2 * lateralTolerance;

            // The relaxed restart threshold only applies after this correction already used RCS;
            // on arriving at a checkpoint the plain tolerance counts (at 20 m a 0.088 m/s drift
            // slipped through the doubled one).
            bool wantRcs = _rcsCommand.HasValue || !_rcsUsedThisCorrection ? !withinTolerance : !withinRestart;
            // Let SAS bring the nose onto the target first. Right after warp the nose can be 10-15°
            // off; with the RCS group on, SAS turns partly with RCS thrusters, and those firings push
            // the vessel sideways — that was the drift growing while the autopilot "corrected".
            // With RCS off it turns on reaction wheels only, which takes a few seconds.
            if (!_rcsCommand.HasValue && pair.Active.PointingError > Settings.DockingAimBeforeRcs)
                wantRcs = false;
            // No flicker: RCS switches on or off at most once per DockingRcsMinSwitch seconds.
            if (wantRcs != _rcsCommand.HasValue && now - _lastRcsSwitchUT < Settings.DockingRcsMinSwitch)
                wantRcs = _rcsCommand.HasValue;

            Vector3d command = CommandFor(error, pair.Rcs);
            if (wantRcs)
            {
                SetRcs(command);
                _rcsUsedThisCorrection = true;
            }
            else
                SetRcs(null);

            LogAxes(now, error, command, relative);

            if (_rcsCommand.HasValue)
                _convergedSinceUT = double.NaN;
            else if (double.IsNaN(_convergedSinceUT))
                _convergedSinceUT = now;

            bool aimed = pair.Active.PointingError <= Settings.DockingAlignedAngle
                         && (!pair.Simulated || pair.Other.PointingError <= Settings.DockingAlignedAngle);

            bool converged = !double.IsNaN(_convergedSinceUT) && now - _convergedSinceUT >= ConvergedHold;
            bool correctionTimedOut = now - _phaseStartUT > Settings.DockingCorrectionTimeout;

            if ((converged || correctionTimedOut) && aimed)
            {
                Log($"{(converged ? "corrected" : "correction time limit, going on")}: d={pair.Active.Distance:F1} m, " +
                    $"vc={pair.Active.ClosingSpeed:F3} (want {LegSpeed:F2}), vlat={pair.Active.LateralSpeed:F3}, " +
                    $"errA={pair.Active.PointingError:F2}° errB={(pair.Simulated ? pair.Other.PointingError.ToString("F2") + "°" : "frozen")}, took {now - _phaseStartUT:F0} s");
                if (NextCheckpoint > 0)
                    BeginCoast();
                else
                    BeginFinal(pair);
                return;
            }

            if (now - _phaseStartUT > Settings.DockingAlignTimeout)
                Stop("#LOC_MJTweaks_DK_AP_AimTimeout");
        }

        /// <summary>
        /// Proportional translation command: wanted acceleration = error / time constant, divided by
        /// the RCS acceleration available in the direction that reduces the error, per axis.
        /// </summary>
        private Vector3d CommandFor(Vector3d worldError, RcsCapability rcs) =>
            RcsDrive.Command(Vessel.ReferenceTransform, worldError, rcs,
                _inFinalCheck ? Settings.DockingFinalCheckDeadband : Settings.DockingDeadband);

        private void BeginCoast()
        {
            SetRcs(null);
            State = Phase.Coasting;
            Status = "#LOC_MJTweaks_DK_AP_Coasting";
            _phaseStartUT = Planetarium.GetUniversalTime();
        }

        /// <summary>Coast to the point where braking for the next checkpoint begins; warp when the leg is long.</summary>
        private void Coast(DockingPair pair)
        {
            if (ReachedBrakingPoint(pair))
            {
                if (TimeWarp.CurrentRateIndex > 0)
                    Core.Warp.MinimumWarp(true);
                if (TimeWarp.CurrentRateIndex > 0 || Vessel.packed)
                    return; // wait until we are back in physics

                _checkpoints.RemoveAt(0);
                BeginCorrection(pair, "checkpoint");
                return;
            }

            // Not closing any more (a correction that hit its time limit while still turning the
            // velocity around, e.g. after a start while moving apart): coasting would only warp away
            // from the target. Correct again instead.
            if (pair.Active.ClosingSpeed < Settings.DockingCoastMinClosing)
            {
                if (TimeWarp.CurrentRateIndex > 0)
                    Core.Warp.MinimumWarp(true);
                if (TimeWarp.CurrentRateIndex > 0 || Vessel.packed)
                    return; // wait until we are back in physics
                BeginCorrection(pair, "not closing while coasting");
                return;
            }

            double stopAt = NextCheckpoint + pair.Rcs.BrakingDistance(Math.Max(pair.Active.ClosingSpeed, 0), pair.Rcs.TouchSpeed);
            double secondsLeft = (pair.Active.Distance - stopAt) / Math.Max(pair.Active.ClosingSpeed, 0.01);
            if (pair.WarpLimit > 1 && secondsLeft > Settings.DockingMinWarpSeconds)
                WarpTowards(Planetarium.GetUniversalTime() + secondsLeft, pair.WarpLimit);
            else if (TimeWarp.CurrentRateIndex > 0)
                Core.Warp.MinimumWarp(true);
        }

        private void BeginFinal(DockingPair pair)
        {
            SetRcs(null);
            State = Phase.Final;
            Status = "#LOC_MJTweaks_DK_AP_Final";
            _phaseStartUT = Planetarium.GetUniversalTime();
            // Time limit fixed here: recomputing it from the shrinking distance made it run out
            // just before contact.
            _finalExpectedSeconds = pair.Active.Distance / Math.Max(pair.Active.ClosingSpeed, 0.05);
            Log($"final: RCS off, coasting in at {pair.Active.ClosingSpeed:F2} m/s from {pair.Active.Distance:F1} m, expected {_finalExpectedSeconds:F0} s");
        }

        /// <summary>RCS off; SAS keeps both vessels aimed until the ports latch. Brakes again if too fast.</summary>
        private void Final(DockingPair pair)
        {
            if (pair.Active.ClosingSpeed > Settings.DockingFinalOverspeed * pair.Rcs.TouchSpeed)
            {
                BeginCorrection(pair, "too fast on the final approach, braking");
                return;
            }

            // One last look shortly before contact: sideways drift keeps growing while coasting, and
            // here every 0.01 m/s means centimetres of miss at the port. A short RCS touch if needed.
            if (!_finalCheckDone && pair.Active.Distance <= Settings.DockingFinalCheckDistance)
            {
                _finalCheckDone = true;
                Vector3d direction = DirectionToTarget();
                Vector3d relative = RelativeVelocity();
                double lateral = (relative - Vector3d.Dot(relative, direction) * direction).magnitude;
                if (lateral > Settings.DockingFinalCheckLateralTolerance)
                {
                    BeginCorrection(pair, $"final check at {pair.Active.Distance:F1} m: sideways {lateral:F3} m/s");
                    _inFinalCheck = true;
                    return;
                }
                Log($"final check at {pair.Active.Distance:F1} m: sideways {lateral:F3} m/s, fine");
            }

            // Stalled (not closing any more) or taking far longer than the distance needs: give up.
            double elapsed = Planetarium.GetUniversalTime() - _phaseStartUT;
            if ((pair.Active.ClosingSpeed <= 0.01 && elapsed > Settings.DockingFinalGrace) || elapsed > 2 * _finalExpectedSeconds + Settings.DockingFinalGrace)
                Stop("#LOC_MJTweaks_DK_AP_NoContact");
        }

        // ---------------------------------------------------------------- helpers

        private Vector3d DirectionToTarget()
        {
            return VesselDockingState.ToTarget(_vessel, _vessel.targetObject, _vessel.ReferenceTransform).normalized;
        }

        /// <summary>
        /// On-rails warp towards ut, never above limit. Not MechJeb's WarpToUT: it remembers only the
        /// time, and its own OnFixedUpdate keeps calling WarpToUT(time) without the limit, so every
        /// second it stepped one rate up and this autopilot stepped it back (test 11: 10x/10.8x,
        /// test 9: 50x/51x flicker). The rate equals the seconds left, as in MechJeb, and is set
        /// every tick from here, so warp slows down on approach.
        /// </summary>
        private void WarpTowards(double ut, double limit)
        {
            double rate = MuUtils.Clamp(ut - Planetarium.GetUniversalTime(), 1, limit);
            Core.Warp.WarpRegularAtRate((float)rate);
        }

        private void EngageTargetHoldBoth()
        {
            DockingTelemetry.EngageTargetHold(Vessel, "autopilot");
            Vessel other = Vessel.targetObject?.GetVessel();
            if (other != null && other != Vessel && other.loaded && !other.packed)
                DockingTelemetry.EngageTargetHold(other, "autopilot");
        }

        /// <summary>Sets the translation command (null = RCS idle) and switches the RCS group accordingly.</summary>
        private void SetRcs(Vector3d? command)
        {
            _rcsCommand = command;
            if (command.HasValue && !_rcsOwned)
            {
                Vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
                _rcsOwned = true;
                _lastRcsSwitchUT = Planetarium.GetUniversalTime();
            }
            else if (!command.HasValue && _rcsOwned)
            {
                Vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, false);
                _rcsOwned = false;
                _lastRcsSwitchUT = Planetarium.GetUniversalTime();
            }
        }

        /// <summary>Relative velocity from the two orbits (MechJeb's Target.RelativeVelocity way).</summary>
        private Vector3d RelativeVelocity()
        {
            Orbit target = _vessel.targetObject?.GetOrbit();
            return target != null ? _vessel.orbit.GetVel() - target.GetVel() : _vessel.obt_velocity - _vessel.targetObject.GetObtVelocity();
        }

        /// <summary>
        /// Once a second while correcting: velocity error, RCS command and the acceleration actually
        /// obtained, per reference axis (x = right, y = forward, z = down), plus the direction to the
        /// target in those axes. If an axis responds with the wrong sign, or the controller's idea of
        /// "straight ahead" differs from where the nose points, it shows up here.
        /// </summary>
        private void LogAxes(double now, Vector3d worldError, Vector3d command, Vector3d relative)
        {
            // Detail lines only while the flight recorder is on; the autopilot's events always go to the log.
            if (!Recorder.FlightRecorder.Recording)
                return;

            Transform reference = _vessel.ReferenceTransform;
            Quaternion toLocal = Quaternion.Inverse(reference.rotation);
            Vector3 relativeLocal = toLocal * (Vector3)relative;
            if (now - _lastAxesLogUT < 1.0)
                return;

            string accel = "-";
            if (!double.IsNaN(_lastAxesLogUT))
            {
                Vector3 a = (relativeLocal - _lastRelativeLocal) / (float)(now - _lastAxesLogUT);
                accel = $"({a.x:F3},{a.y:F3},{a.z:F3})";
            }
            Vector3 e = toLocal * (Vector3)worldError;
            Vector3d direction = DirectionToTarget();
            Vector3 los = toLocal * (Vector3)direction;
            Log($"axes: err=({e.x:F3},{e.y:F3},{e.z:F3}) cmd={(_rcsCommand.HasValue ? $"({command.x:F2},{command.y:F2},{command.z:F2})" : "off")} acc={accel} " +
                $"los=({los.x:F3},{los.y:F3},{los.z:F3}) noseToTarget={Vector3d.Angle(reference.up, direction):F2}° part={reference.name}");

            // Three ways to get the relative velocity, as closing / sideways speed: from the orbits
            // (what the controller uses), from obt_velocity (what the navball shows), and from how
            // the relative position actually changed since the last line.
            Vector3d relativePosition = -VesselDockingState.ToTarget(_vessel, _vessel.targetObject, reference);
            string geometric = "-";
            if (!double.IsNaN(_lastAxesLogUT))
                geometric = SplitText((relativePosition - _lastRelativePosition) / (now - _lastAxesLogUT), direction);
            Vector3d navball = _vessel.obt_velocity - _vessel.targetObject.GetObtVelocity();
            Vessel other = _vessel.targetObject.GetVessel();
            Log($"velocity check (closing/sideways): orbits={SplitText(relative, direction)} navball={SplitText(navball, direction)} " +
                $"geometry={geometric} other={(other != null && other.packed ? "packed" : "physics")}");

            _lastAxesLogUT = now;
            _lastRelativeLocal = relativeLocal;
            _lastRelativePosition = relativePosition;
        }

        private static string SplitText(Vector3d velocity, Vector3d direction)
        {
            double closing = Vector3d.Dot(velocity, direction);
            double sideways = (velocity - closing * direction).magnitude;
            return $"{closing:F3}/{sideways:F3}";
        }

        private static void Log(string message) => DockingTelemetry.Note("autopilot: " + message);
    }
}
