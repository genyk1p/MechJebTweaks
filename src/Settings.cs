namespace MechJebTweaks
{
    /// <summary>Values from the MECHJEB_TWEAKS node in Settings.cfg.</summary>
    internal static class Settings
    {
        // ---- WARP_GUARD ----

        public static bool WarpGuardEnabled = true;

        /// <summary>
        /// Once no engine is commanded to burn and the remaining thrust tail is below this
        /// acceleration, the guard snaps those engines' throttle to exactly zero, m/s².
        /// Engine throttle decays exponentially and never reaches zero by itself.
        /// </summary>
        public static double SnapAcceleration = 0.001;

        /// <summary>Extra seconds to wait after the last engine stopped burning.</summary>
        public static double SettleSeconds = 2.0;

        /// <summary>
        /// Also hold back physics warp. Off by default: physics warp keeps simulating thrust,
        /// and MechJeb's classic ascent intentionally throttles during physics-warped coasts.
        /// </summary>
        public static bool GuardPhysicsWarp = false;

        public static bool ShowMessages = true;

        // ---- LANDING_WARP ----

        /// <summary>
        /// Let MechJeb's landing autopilot warp towards a low-orbit deorbit burn while the vessel turns
        /// with the orbit (MechJeb only warps below 0.001 rad/s, slower than a low orbit turns).
        /// </summary>
        public static bool LandingWarpEnabled = true;

        /// <summary>The nose must point within this of MechJeb's wanted direction, degrees.</summary>
        public static double LandingWarpAimAngle = 2;

        /// <summary>Allowed angular velocity above the rate at which the orbit turns, rad/s.</summary>
        public static double LandingWarpExtraRate = 0.002;

        // ---- THRUST_CAP ----

        /// <summary>
        /// Cap MechJeb's available-thrust estimate of engines whose thrust is not proportional to the throttle
        /// (KSPIE thermal nozzles) at what the engine really gave at full throttle.
        /// </summary>
        public static bool ThrustCapEnabled = true;

        /// <summary>Only engines on parts with one of these part modules are capped.</summary>
        public static string[] ThrustCapModules = { "ThermalEngineController", "ThermalNozzleController" };

        /// <summary>An engine running at this throttle or above counts as full throttle for the measurement.</summary>
        public static double ThrustCapLearnThrottle = 0.95;

        // ---- LANDING_SITES ----

        /// <summary>Add the game's waypoints (SCANsat, stock map, contracts) to the landing site drop-down of MechJeb's landing guidance.</summary>
        public static bool WaypointSitesEnabled = true;

        // ---- RENDEZVOUS_OFFSET ----

        public static bool RendezvousOffsetEnabled = true;

        /// <summary>
        /// Tech tree node(s) that unlock the window in career/science, like MechJeb's own unlockTechs
        /// (any of them, separated by commas; empty = always). advUnmanned opens MechJeb's rendezvous planner.
        /// </summary>
        public static string RendezvousOffsetUnlockTech = "advUnmanned";

        // ---- RENDEZVOUS_AUTOPILOT (safe rendezvous autopilot in the same window) ----

        /// <summary>
        /// Waiting orbit candidates (when our orbit is too close in size to the target's): radius = target radius
        /// x (1 ± k x PhasingStep) for k = 1, 2, ... up to PhasingRatio. The cheapest one whose whole time to the
        /// transfer's arrival is at most SlowFactor x that of the PhasingRatio orbit is taken.
        /// </summary>
        public static double RzPhasingStep = 0.005;

        /// <summary>The autopilot refuses a target whose orbit is more eccentric than this.</summary>
        public static double RzTargetMaxEccentricity = 0.01;

        /// <summary>Our orbit is circularized while more eccentric than this.</summary>
        public static double RzRoundEccentricity = 0.01;

        /// <summary>
        /// Eccentric orbit, choice of apsis for the first circularization: the route with less dV, unless it
        /// takes more than RzSlowFactor times longer while the faster one costs at most RzDvMargin (share) more.
        /// </summary>
        public static double RzSlowFactor = 2;
        public static double RzDvMargin = 0.10;

        /// <summary>Planes are matched while the relative inclination is above this, degrees.</summary>
        public static double RzPlaneAngle = 5;

        /// <summary>Waiting orbit radius = target orbit radius × this (from the body's centre).</summary>
        public static double RzPhasingRatio = 1.25;

        /// <summary>No waiting orbit when our radius already differs from the target's by at least this share.</summary>
        public static double RzPhasingMinDifference = 0.12;

        /// <summary>A waiting orbit below the target keeps at least this altitude above the atmosphere (or sea level), m.</summary>
        public static double RzTerrainMargin = 10000;

        /// <summary>Mid-course correction: skipped when the flight to the target is shorter than this, s.</summary>
        public static double RzCorrectionMinFlight = 300;

        /// <summary>Mid-course correction: not done when smaller than this, m/s.</summary>
        public static double RzCorrectionMinDv = 0.05;

        /// <summary>Mid-course correction: done when the velocity error is below this, m/s.</summary>
        public static double RzCorrectionTolerance = 0.02;

        /// <summary>Braking burn: throttle it is planned with; trimmed during the burn within BurnTrimMin..1.</summary>
        public static double RzBurnThrottle = 0.9;
        public static double RzBurnTrimMin = 0.5;

        /// <summary>Braking burn: the rest of the burn is re-simulated and the throttle trimmed every this many seconds.</summary>
        public static double RzBurnTrimInterval = 2;

        /// <summary>Braking burn: refused when the planned burn ends farther than this aside of the target (engine too weak), m.</summary>
        public static double RzBurnMaxLateral = 2000;

        /// <summary>
        /// Route planning: a transfer straight to the target is not used when its braking at BurnThrottle would last
        /// more than this share of the target's orbital period (the burn curves off the approach line); the route
        /// goes via a waiting orbit next to the target instead. Tests: 8.6 % ended 0.8 km aside (fine), 21 % 34 km.
        /// </summary>
        public static double RzBrakeMaxOrbitShare = 0.12;

        /// <summary>
        /// A waiting orbit is at least this many expected circularization misses away from the target's orbit (a long
        /// burn of a weak engine ends tens of km off; see SafeRendezvousAutopilot.TooCloseForCircularization).
        /// </summary>
        public static double RzWaitingMissMargin = 3;

        /// <summary>
        /// Sideways RCS nudges during the burn: wanted end point BurnLateralShare x offset aside of the target;
        /// a nudge when the plan misses that by more than NudgeMinMiss m and at least NudgeMinTime s of burn are
        /// left; it gives NudgeGain of the sideways velocity that would move the end point there.
        /// </summary>
        public static double RzBurnLateralShare = 0.7;
        public static double RzNudgeMinMiss = 10;
        public static double RzNudgeMinTime = 30;
        public static double RzNudgeGain = 0.7;
        /// <summary>Largest single sideways nudge, m/s.</summary>
        public static double RzNudgeMaxDv = 1.0;

        /// <summary>Braking burn: the main engine stops at this relative speed, m/s; RCS takes the rest.</summary>
        public static double RzBurnCutSpeed = 1.0;

        /// <summary>RCS clean-up after the burn: done below this relative speed (m/s), gives up after CleanupTimeout s.</summary>
        public static double RzCleanupTolerance = 0.05;
        public static double RzCleanupTimeout = 180;

        /// <summary>Braking: on-rails warp stops this many seconds before braking starts, to turn the vessel tail-first.</summary>
        public static double RzBrakeTurnLead = 150;

        /// <summary>Braking: counts as turned within this angle (held 3 s), degrees; then on-rails warp again.</summary>
        public static double RzBrakeAlignedAngle = 3;

        /// <summary>Braking: the second on-rails warp stops this many seconds before braking starts.</summary>
        public static double RzBrakeWarpStop = 5;

        /// <summary>Braking: physics warp (the player's) is switched off closer than this to the target, m.</summary>
        public static double RzBrakeNoWarpDistance = 3000;

        /// <summary>Braking burn: the engine fires only while pointing within this angle of retrograde, degrees.</summary>
        public static double RzBrakeAimAngle = 10;

        /// <summary>The same step at most this many times in a row before the autopilot gives up.</summary>
        public static int RzMaxAttempts = 3;

        // ---- AUTO_PHYSICS_WARP (MechJeb's node executor and the rendezvous autopilot) ----

        /// <summary>Rate until the player picks one (0 = off, 2, 3, 4); the choice is kept in PluginData.</summary>
        public static int AutoPhysicsWarpDefaultRate = 4;

        /// <summary>Turns run in physics warp while more than this off; the last degrees at 1x, degrees.</summary>
        public static double AutoPhysicsWarpAimAngle = 5;

        /// <summary>A turn that dropped to 1x goes back into physics warp only beyond AimAngle + this, degrees.</summary>
        public static double AutoPhysicsWarpHysteresis = 2;

        /// <summary>
        /// Burns run in physics warp while more than max(MinBurnDv m/s, TailSeconds x full acceleration) is left;
        /// the tail at 1x.
        /// </summary>
        public static double AutoPhysicsWarpMinBurnDv = 0.5;
        public static double AutoPhysicsWarpTailSeconds = 0;

        // ---- CONTROL_CHECK ----

        public static bool ControlCheckEnabled = true;

        /// <summary>Reaction wheels: 90° turn (pitch/yaw) within this is "good", within ControlTurnOk "ok", s.
        /// Slower than DockingMaxTurnTime90 the docking autopilot refuses to start.</summary>
        public static double ControlTurnGood = 15;
        public static double ControlTurnOk = 30;

        /// <summary>RCS acceleration in the weakest direction: below Weak "too weak", below Ok "weak",
        /// above Strong "too strong" (coarse control), m/s².</summary>
        public static double ControlRcsWeak = 0.02;
        public static double ControlRcsOk = 0.05;
        public static double ControlRcsStrong = 0.5;

        /// <summary>RCS delta-v on the propellant aboard: below Low "too little", below Tight "tight",
        /// above High "more than needed" (dead weight), m/s.</summary>
        public static double ControlRcsDvLow = 10;
        public static double ControlRcsDvTight = 20;
        public static double ControlRcsDvHigh = 100;

        // ---- unlocks ----

        /// <summary>
        /// Is any of the given tech nodes researched? Empty list, sandbox or no R&amp;D yet: unlocked.
        /// Checked in code rather than through MechJeb's unlockTechs config: that one is saved with
        /// each vessel, so vessels already in flight would keep the window unlocked.
        /// </summary>
        public static bool TechUnlocked(string techs)
        {
            string[] ids = (techs ?? string.Empty).Split(new[] { ' ', ',', ';', '\t' }, System.StringSplitOptions.RemoveEmptyEntries);
            if (ids.Length == 0 || HighLogic.CurrentGame == null || HighLogic.CurrentGame.Mode == Game.Modes.SANDBOX
                || ResearchAndDevelopment.Instance == null)
                return true;
            foreach (string id in ids)
                if (ResearchAndDevelopment.GetTechnologyState(id) == RDTech.State.Available)
                    return true;
            return false;
        }

        // ---- DOCKING ----

        public static bool DockingEnabled = true;

        /// <summary>
        /// Tech node(s) that unlock the docking window: advUnmanned is where stock probe cores first
        /// get SAS level 3 (target hold, which the autopilot needs) and where MechJeb opens its own docking autopilot.
        /// </summary>
        public static string DockingUnlockTech = "advUnmanned";

        /// <summary>The other vessel must be closer than this (and loaded in physics), m.</summary>
        public static double DockingMaxRange = 2500;

        /// <summary>Each vessel must be able to turn 90° with reaction wheels within this time, s.</summary>
        public static double DockingMaxTurnTime90 = 60;

        /// <summary>Both vessels count as aimed at each other below this pointing error, degrees.</summary>
        public static double DockingAlignedAngle = 0.5;

        /// <summary>Roughly pointing at each other, degrees (status colouring only).</summary>
        public static double DockingRoughAngle = 10;

        // ---- RECORDER ----

        public static bool RecorderEnabled = true;

        /// <summary>Tech node(s) that unlock the recorder; empty = always (it is a debugging tool).</summary>
        public static string RecorderUnlockTech = "";

        /// <summary>Interval between status lines while the flight recorder writes to KSP.log, real seconds.</summary>
        public static double RecorderLogInterval = 1;

        /// <summary>Approach speed limits, m/s.</summary>
        public static double DockingFarSpeedMax = 5;
        public static double DockingTouchSpeedMax = 1;

        /// <summary>Braking from the far approach speed to the touchdown speed must fit into this distance, m.</summary>
        public static double DockingBrakeDistance = 100;

        /// <summary>Braking from the touchdown speed to zero must fit into this distance, m.</summary>
        public static double DockingTouchStopDistance = 10;

        /// <summary>Braking distances are computed with this margin (2 = assume half the RCS thrust).</summary>
        public static double DockingSafetyFactor = 2;

        /// <summary>Share of the summed RCS thrust assumed to be available for translation.</summary>
        public static double DockingRcsEfficiency = 0.8;

        /// <summary>The near-range autopilot starts only this close to the target, m.</summary>
        public static double DockingNearStartMax = 250;

        /// <summary>The autopilot starts at all only this close (the far approach takes over beyond NearStartMax), m.</summary>
        public static double DockingFarStartMax = 2250;

        /// <summary>Far approach: intercept the target in distance / far speed, but not sooner than this, s.</summary>
        public static double DockingFarMinTime = 60;

        /// <summary>
        /// Far approach: correction is done once the predicted closest approach (from the orbits,
        /// as in MechJeb's rendezvous info) is within this, m; then one coast to the near approach.
        /// </summary>
        public static double DockingFarMissTolerance = 10;

        /// <summary>Far approach: ...or once the velocity change still needed for the intercept is below this, m/s.</summary>
        public static double DockingFarVelocityTolerance = 0.02;

        /// <summary>Far approach: longest single correction, then go on with what it has, s.</summary>
        public static double DockingFarCorrectionTimeout = 240;

        /// <summary>Abort when closing faster than this many times the leg speed (+0.3 m/s) while coasting.</summary>
        public static double DockingAbortSpeedFactor = 2;

        /// <summary>Correction is done when the closing speed is within max(this, share × leg speed), m/s...</summary>
        public static double DockingClosingTolerance = 0.05;
        public static double DockingClosingToleranceShare = 0.05;

        /// <summary>...and the sideways velocity within this, m/s.</summary>
        public static double DockingLateralTolerance = 0.05;

        /// <summary>Sideways tolerance on the last leg (after the 20 m checkpoint), m/s.</summary>
        public static double DockingFinalLateralTolerance = 0.02;

        /// <summary>Last sideways check on the final approach at this distance, m.</summary>
        public static double DockingFinalCheckDistance = 8;

        /// <summary>Sideways tolerance and controller deadband of that last check, m/s.</summary>
        public static double DockingFinalCheckLateralTolerance = 0.005;
        public static double DockingFinalCheckDeadband = 0.002;

        /// <summary>Longest RCS correction at one checkpoint; then the autopilot goes on with what it has, s.</summary>
        public static double DockingCorrectionTimeout = 60;

        /// <summary>A coast closing slower than this goes back to correcting (it would never arrive), m/s.</summary>
        public static double DockingCoastMinClosing = 0.02;

        /// <summary>RCS controller: wanted acceleration = velocity error / this time constant, s.</summary>
        public static double DockingRcsTimeConstant = 2;

        /// <summary>RCS controller: velocity errors below this are ignored per axis, m/s.</summary>
        public static double DockingDeadband = 0.005;

        /// <summary>
        /// RCS controller: smallest non-zero axis command. Stock RCS thrusters do nothing at or below 5%
        /// of their thrust, so smaller commands are raised to this.
        /// </summary>
        public static double DockingMinRcsCommand = 0.1;

        /// <summary>
        /// A correction only starts RCS once the nose points within this of the target, degrees:
        /// with RCS on, SAS turns partly with RCS thrusters and those firings push the vessel sideways.
        /// </summary>
        public static double DockingAimBeforeRcs = 2;

        /// <summary>RCS is switched on or off by the autopilot at most once per this many seconds.</summary>
        public static double DockingRcsMinSwitch = 1;

        /// <summary>Final approach: brake again with RCS when faster than this × touchdown speed.</summary>
        public static double DockingFinalOverspeed = 1.5;

        /// <summary>Give up a checkpoint if correction and aiming take longer than this, s.</summary>
        public static double DockingAlignTimeout = 180;

        /// <summary>Only warp when the coast to the next checkpoint is longer than this, s.</summary>
        public static double DockingMinWarpSeconds = 10;

        /// <summary>Final approach: give up after this long without closing in, s.</summary>
        public static double DockingFinalGrace = 30;

        /// <summary>Warp limit by the number of parts of all vessels within DockingMaxRange.</summary>
        public static int DockingWarp50MaxParts = 150;
        public static int DockingWarp10MaxParts = 400;
        public static int DockingWarp5MaxParts = 800;

        public static int WarpLimitForParts(int parts) =>
            parts <= DockingWarp50MaxParts ? 50
            : parts <= DockingWarp10MaxParts ? 10
            : parts <= DockingWarp5MaxParts ? 5
            : 1;

        public static void Load()
        {
            foreach (ConfigNode root in GameDatabase.Instance.GetConfigNodes("MECHJEB_TWEAKS"))
            {
                ConfigNode warp = root.GetNode("WARP_GUARD");
                if (warp != null)
                {
                    warp.TryGetValue("Enabled", ref WarpGuardEnabled);
                    warp.TryGetValue("SnapAcceleration", ref SnapAcceleration);
                    warp.TryGetValue("SettleSeconds", ref SettleSeconds);
                    warp.TryGetValue("GuardPhysicsWarp", ref GuardPhysicsWarp);
                    warp.TryGetValue("ShowMessages", ref ShowMessages);
                }

                ConfigNode landing = root.GetNode("LANDING_WARP");
                if (landing != null)
                {
                    landing.TryGetValue("Enabled", ref LandingWarpEnabled);
                    landing.TryGetValue("AimAngle", ref LandingWarpAimAngle);
                    landing.TryGetValue("ExtraRate", ref LandingWarpExtraRate);
                }

                ConfigNode cap = root.GetNode("THRUST_CAP");
                if (cap != null)
                {
                    cap.TryGetValue("Enabled", ref ThrustCapEnabled);
                    cap.TryGetValue("LearnThrottle", ref ThrustCapLearnThrottle);
                    string modules = null;
                    if (cap.TryGetValue("Modules", ref modules))
                        ThrustCapModules = modules.Split(new[] { ' ', ',', ';', '\t' }, System.StringSplitOptions.RemoveEmptyEntries);
                }

                root.GetNode("LANDING_SITES")?.TryGetValue("FromWaypoints", ref WaypointSitesEnabled);

                ConfigNode rendezvous = root.GetNode("RENDEZVOUS_OFFSET");
                rendezvous?.TryGetValue("Enabled", ref RendezvousOffsetEnabled);
                rendezvous?.TryGetValue("UnlockTech", ref RendezvousOffsetUnlockTech);

                ConfigNode rzAuto = root.GetNode("RENDEZVOUS_AUTOPILOT");
                if (rzAuto != null)
                {
                    rzAuto.TryGetValue("TargetMaxEccentricity", ref RzTargetMaxEccentricity);
                    rzAuto.TryGetValue("RoundEccentricity", ref RzRoundEccentricity);
                    rzAuto.TryGetValue("SlowFactor", ref RzSlowFactor);
                    rzAuto.TryGetValue("DvMargin", ref RzDvMargin);
                    rzAuto.TryGetValue("PlaneAngle", ref RzPlaneAngle);
                    rzAuto.TryGetValue("PhasingRatio", ref RzPhasingRatio);
                    rzAuto.TryGetValue("PhasingMinDifference", ref RzPhasingMinDifference);
                    rzAuto.TryGetValue("TerrainMargin", ref RzTerrainMargin);
                    rzAuto.TryGetValue("CorrectionMinFlight", ref RzCorrectionMinFlight);
                    rzAuto.TryGetValue("CorrectionMinDv", ref RzCorrectionMinDv);
                    rzAuto.TryGetValue("CorrectionTolerance", ref RzCorrectionTolerance);
                    rzAuto.TryGetValue("MaxAttempts", ref RzMaxAttempts);
                    rzAuto.TryGetValue("BurnThrottle", ref RzBurnThrottle);
                    rzAuto.TryGetValue("BurnTrimMin", ref RzBurnTrimMin);
                    rzAuto.TryGetValue("BurnTrimInterval", ref RzBurnTrimInterval);
                    rzAuto.TryGetValue("BurnCutSpeed", ref RzBurnCutSpeed);
                    rzAuto.TryGetValue("BurnMaxLateral", ref RzBurnMaxLateral);
                    rzAuto.TryGetValue("BrakeMaxOrbitShare", ref RzBrakeMaxOrbitShare);
                    rzAuto.TryGetValue("WaitingMissMargin", ref RzWaitingMissMargin);
                    rzAuto.TryGetValue("BurnLateralShare", ref RzBurnLateralShare);
                    rzAuto.TryGetValue("NudgeMinMiss", ref RzNudgeMinMiss);
                    rzAuto.TryGetValue("NudgeMinTime", ref RzNudgeMinTime);
                    rzAuto.TryGetValue("NudgeGain", ref RzNudgeGain);
                    rzAuto.TryGetValue("NudgeMaxDv", ref RzNudgeMaxDv);
                    rzAuto.TryGetValue("PhasingStep", ref RzPhasingStep);
                    rzAuto.TryGetValue("CleanupTolerance", ref RzCleanupTolerance);
                    rzAuto.TryGetValue("CleanupTimeout", ref RzCleanupTimeout);
                    rzAuto.TryGetValue("BrakeTurnLead", ref RzBrakeTurnLead);
                    rzAuto.TryGetValue("BrakeAlignedAngle", ref RzBrakeAlignedAngle);
                    rzAuto.TryGetValue("BrakeWarpStop", ref RzBrakeWarpStop);
                    rzAuto.TryGetValue("BrakeNoWarpDistance", ref RzBrakeNoWarpDistance);
                    rzAuto.TryGetValue("BrakeAimAngle", ref RzBrakeAimAngle);
                }

                ConfigNode physicsWarp = root.GetNode("AUTO_PHYSICS_WARP");
                if (physicsWarp != null)
                {
                    physicsWarp.TryGetValue("DefaultRate", ref AutoPhysicsWarpDefaultRate);
                    physicsWarp.TryGetValue("AimAngle", ref AutoPhysicsWarpAimAngle);
                    physicsWarp.TryGetValue("Hysteresis", ref AutoPhysicsWarpHysteresis);
                    physicsWarp.TryGetValue("MinBurnDv", ref AutoPhysicsWarpMinBurnDv);
                    physicsWarp.TryGetValue("TailSeconds", ref AutoPhysicsWarpTailSeconds);
                }

                ConfigNode recorder = root.GetNode("RECORDER");
                if (recorder != null)
                {
                    recorder.TryGetValue("Enabled", ref RecorderEnabled);
                    recorder.TryGetValue("LogInterval", ref RecorderLogInterval);
                    recorder.TryGetValue("UnlockTech", ref RecorderUnlockTech);
                }

                ConfigNode control = root.GetNode("CONTROL_CHECK");
                if (control != null)
                {
                    control.TryGetValue("Enabled", ref ControlCheckEnabled);
                    control.TryGetValue("TurnGood", ref ControlTurnGood);
                    control.TryGetValue("TurnOk", ref ControlTurnOk);
                    control.TryGetValue("RcsWeak", ref ControlRcsWeak);
                    control.TryGetValue("RcsOk", ref ControlRcsOk);
                    control.TryGetValue("RcsStrong", ref ControlRcsStrong);
                    control.TryGetValue("RcsDvLow", ref ControlRcsDvLow);
                    control.TryGetValue("RcsDvTight", ref ControlRcsDvTight);
                    control.TryGetValue("RcsDvHigh", ref ControlRcsDvHigh);
                }

                ConfigNode docking = root.GetNode("DOCKING");
                if (docking != null)
                {
                    docking.TryGetValue("Enabled", ref DockingEnabled);
                    docking.TryGetValue("UnlockTech", ref DockingUnlockTech);
                    docking.TryGetValue("MaxRange", ref DockingMaxRange);
                    docking.TryGetValue("MaxTurnTime90", ref DockingMaxTurnTime90);
                    docking.TryGetValue("AlignedAngle", ref DockingAlignedAngle);
                    docking.TryGetValue("RoughAngle", ref DockingRoughAngle);
                    docking.TryGetValue("FarSpeedMax", ref DockingFarSpeedMax);
                    docking.TryGetValue("TouchSpeedMax", ref DockingTouchSpeedMax);
                    docking.TryGetValue("BrakeDistance", ref DockingBrakeDistance);
                    docking.TryGetValue("TouchStopDistance", ref DockingTouchStopDistance);
                    docking.TryGetValue("SafetyFactor", ref DockingSafetyFactor);
                    docking.TryGetValue("RcsEfficiency", ref DockingRcsEfficiency);
                    docking.TryGetValue("NearStartMax", ref DockingNearStartMax);
                    docking.TryGetValue("FarStartMax", ref DockingFarStartMax);
                    docking.TryGetValue("FarMinTime", ref DockingFarMinTime);
                    docking.TryGetValue("FarVelocityTolerance", ref DockingFarVelocityTolerance);
                    docking.TryGetValue("FarMissTolerance", ref DockingFarMissTolerance);
                    docking.TryGetValue("FarCorrectionTimeout", ref DockingFarCorrectionTimeout);
                    docking.TryGetValue("AbortSpeedFactor", ref DockingAbortSpeedFactor);
                    docking.TryGetValue("ClosingTolerance", ref DockingClosingTolerance);
                    docking.TryGetValue("ClosingToleranceShare", ref DockingClosingToleranceShare);
                    docking.TryGetValue("LateralTolerance", ref DockingLateralTolerance);
                    docking.TryGetValue("FinalLateralTolerance", ref DockingFinalLateralTolerance);
                    docking.TryGetValue("FinalCheckDistance", ref DockingFinalCheckDistance);
                    docking.TryGetValue("FinalCheckLateralTolerance", ref DockingFinalCheckLateralTolerance);
                    docking.TryGetValue("FinalCheckDeadband", ref DockingFinalCheckDeadband);
                    docking.TryGetValue("CorrectionTimeout", ref DockingCorrectionTimeout);
                    docking.TryGetValue("CoastMinClosing", ref DockingCoastMinClosing);
                    docking.TryGetValue("RcsTimeConstant", ref DockingRcsTimeConstant);
                    docking.TryGetValue("Deadband", ref DockingDeadband);
                    docking.TryGetValue("MinRcsCommand", ref DockingMinRcsCommand);
                    docking.TryGetValue("RcsMinSwitch", ref DockingRcsMinSwitch);
                    docking.TryGetValue("AimBeforeRcs", ref DockingAimBeforeRcs);
                    docking.TryGetValue("FinalOverspeed", ref DockingFinalOverspeed);
                    docking.TryGetValue("AlignTimeout", ref DockingAlignTimeout);
                    docking.TryGetValue("MinWarpSeconds", ref DockingMinWarpSeconds);
                    docking.TryGetValue("FinalGrace", ref DockingFinalGrace);
                    docking.TryGetValue("Warp50MaxParts", ref DockingWarp50MaxParts);
                    docking.TryGetValue("Warp10MaxParts", ref DockingWarp10MaxParts);
                    docking.TryGetValue("Warp5MaxParts", ref DockingWarp5MaxParts);
                }
            }

            Log.Info($"settings: WarpGuard Enabled={WarpGuardEnabled}, SnapAcceleration={SnapAcceleration}, " +
                     $"SettleSeconds={SettleSeconds}, GuardPhysicsWarp={GuardPhysicsWarp}, ShowMessages={ShowMessages}; " +
                     $"LandingWarp Enabled={LandingWarpEnabled}, AimAngle={LandingWarpAimAngle}, ExtraRate={LandingWarpExtraRate}; " +
                     $"ThrustCap Enabled={ThrustCapEnabled}, Modules={string.Join(",", ThrustCapModules)}, LearnThrottle={ThrustCapLearnThrottle}; " +
                     $"LandingSites FromWaypoints={WaypointSitesEnabled}; " +
                     $"RendezvousOffset Enabled={RendezvousOffsetEnabled}; " +
                     $"AutoPhysicsWarp DefaultRate={AutoPhysicsWarpDefaultRate}, AimAngle={AutoPhysicsWarpAimAngle}, " +
                     $"MinBurnDv={AutoPhysicsWarpMinBurnDv}, TailSeconds={AutoPhysicsWarpTailSeconds}; " +
                     $"Recorder Enabled={RecorderEnabled}, LogInterval={RecorderLogInterval}; " +
                     $"ControlCheck Enabled={ControlCheckEnabled}, Turn {ControlTurnGood}/{ControlTurnOk} s, " +
                     $"RCS {ControlRcsWeak}/{ControlRcsOk}/{ControlRcsStrong} m/s², RCS dV {ControlRcsDvLow}/{ControlRcsDvTight}/{ControlRcsDvHigh} m/s; " +
                     $"Docking Enabled={DockingEnabled}, MaxRange={DockingMaxRange}, MaxTurnTime90={DockingMaxTurnTime90}, " +
                     $"AlignedAngle={DockingAlignedAngle}, Warp parts 50x<={DockingWarp50MaxParts} 10x<={DockingWarp10MaxParts} 5x<={DockingWarp5MaxParts}");
        }
    }
}
