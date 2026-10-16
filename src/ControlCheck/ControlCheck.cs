using System;
using System.Collections.Generic;
using System.Linq;
using MechJebTweaks.Docking;
using UnityEngine;

namespace MechJebTweaks.ControlCheck
{
    internal enum Grade
    {
        None,      // nothing to judge (no wheels / no RCS)
        Good,
        Ok,
        Weak,      // works, but slow / tight
        Bad,       // too little: the docking autopilot refuses, or practically useless
        TooMuch    // more than useful: twitchy control or dead weight
    }

    /// <summary>
    /// Reaction wheels, RCS and RCS propellant of a ship, judged with the docking autopilot's own
    /// measures: turn time 90° on wheels, RCS acceleration and the approach speeds that follow from it.
    /// </summary>
    internal class ControlCheck
    {
        public double Mass;

        // ---- reaction wheels ----
        public Vector3 Torque;            // x = pitch, y = roll, z = yaw, kN·m
        public double PitchInertia, YawInertia, RollInertia;
        public double PitchTime, YawTime, RollTime; // 90° turn, s
        public Grade WheelGrade;
        /// <summary>Extra torque needed to turn 90° within Settings.ControlTurnOk, kN·m (0 = enough).</summary>
        public double MissingPitchTorque, MissingYawTorque;

        // ---- RCS ----
        public RcsCapability Rcs;
        public double RcsWeakest;
        public string RcsWeakestName;
        public Grade RcsGrade;
        /// <summary>Extra RCS thrust needed in the weakest direction to reach Settings.ControlRcsOk, kN.</summary>
        public double MissingRcsThrust;

        // ---- RCS propellant ----
        public string Propellants = string.Empty;
        public double PropellantMass;
        public double Isp;
        public double RcsDeltaV;
        public bool SharedWithEngines;
        public Grade FuelGrade;

        public static ControlCheck Compute(IList<Part> parts, Transform reference)
        {
            var check = new ControlCheck();
            if (parts == null || parts.Count == 0 || reference == null)
                return check;

            check.Mass = ControlMath.TotalMass(parts);
            Vector3d center = ControlMath.CenterOfMass(parts);

            check.Torque = ControlMath.WheelTorque(parts);
            check.PitchInertia = ControlMath.MomentOfInertia(parts, center, reference.right);
            check.YawInertia = ControlMath.MomentOfInertia(parts, center, reference.forward);
            check.RollInertia = ControlMath.MomentOfInertia(parts, center, reference.up);
            const double quarterTurn = Math.PI / 2;
            check.PitchTime = ControlMath.TurnTime(check.Torque.x, check.PitchInertia, quarterTurn);
            check.YawTime = ControlMath.TurnTime(check.Torque.z, check.YawInertia, quarterTurn);
            check.RollTime = ControlMath.TurnTime(check.Torque.y, check.RollInertia, quarterTurn);
            check.JudgeWheels();

            check.Rcs = RcsCapability.Measure(parts, reference, check.Mass);
            check.JudgeRcs();

            check.MeasurePropellant(parts);
            return check;
        }

        private void JudgeWheels()
        {
            double worst = Math.Max(PitchTime, YawTime);
            if (Torque.x <= 0 && Torque.z <= 0)
                WheelGrade = Grade.None;
            else if (worst <= Settings.ControlTurnGood)
                WheelGrade = Grade.Good;
            else if (worst <= Settings.ControlTurnOk)
                WheelGrade = Grade.Ok;
            else if (worst <= Settings.DockingMaxTurnTime90)
                WheelGrade = Grade.Weak;
            else
                WheelGrade = Grade.Bad;

            const double quarterTurn = Math.PI / 2;
            MissingPitchTorque = Math.Max(0, ControlMath.TorqueForTurn(PitchInertia, quarterTurn, Settings.ControlTurnOk) - Torque.x);
            MissingYawTorque = Math.Max(0, ControlMath.TorqueForTurn(YawInertia, quarterTurn, Settings.ControlTurnOk) - Torque.z);
        }

        private void JudgeRcs()
        {
            if (!Rcs.Any)
            {
                RcsGrade = Grade.None;
                return;
            }

            RcsWeakest = Rcs.Forward;
            RcsWeakestName = "#LOC_MJTweaks_CC_DirForward";
            if (Rcs.Back < RcsWeakest) { RcsWeakest = Rcs.Back; RcsWeakestName = "#LOC_MJTweaks_CC_DirBack"; }
            if (Rcs.Lateral < RcsWeakest) { RcsWeakest = Rcs.Lateral; RcsWeakestName = "#LOC_MJTweaks_CC_DirLateral"; }

            if (RcsWeakest < Settings.ControlRcsWeak)
                RcsGrade = Grade.Bad;
            else if (RcsWeakest < Settings.ControlRcsOk)
                RcsGrade = Grade.Weak;
            else if (RcsWeakest > Settings.ControlRcsStrong)
                RcsGrade = Grade.TooMuch;
            else
                RcsGrade = Grade.Ok;

            // RcsCapability already applies the efficiency; the thrust to add is the raw sum.
            MissingRcsThrust = Math.Max(0, (Settings.ControlRcsOk - RcsWeakest) * Mass / Settings.DockingRcsEfficiency);
        }

        /// <summary>
        /// Delta-v of the RCS on the propellant aboard: the propellant mix of the strongest thrusters,
        /// thrust-weighted vacuum Isp, all of that propellant on the ship counted (no flow rules).
        /// </summary>
        private void MeasurePropellant(IList<Part> parts)
        {
            List<ModuleRCS> thrusters = RcsCapability.Thrusters(parts).Where(r => r.propellants != null && r.propellants.Count > 0).ToList();
            if (thrusters.Count == 0)
            {
                FuelGrade = Grade.None;
                return;
            }

            // Group by propellant mix and take the mix with the most thrust.
            string Key(ModuleRCS r) => string.Join("+", r.propellants.Select(p => p.name).OrderBy(n => n));
            IGrouping<string, ModuleRCS> main = thrusters.GroupBy(Key)
                .OrderByDescending(g => g.Sum(r => r.thrusterPower * r.thrustPercentage * 0.01 * RcsCapability.Nozzles(r).Count()))
                .First();
            ModuleRCS sample = main.First();
            Propellants = main.Key;

            // Thrust-weighted vacuum Isp.
            double thrust = 0, flow = 0;
            foreach (ModuleRCS r in main)
            {
                double t = r.thrusterPower * r.thrustPercentage * 0.01 * RcsCapability.Nozzles(r).Count();
                double isp = r.atmosphereCurve.Evaluate(0f);
                if (t <= 0 || isp <= 0)
                    continue;
                thrust += t;
                flow += t / isp;
            }
            Isp = flow > 0 ? thrust / flow : 0;

            // Usable mix: limited by the propellant that runs out first. Massless ones (electric charge) are left out.
            double mixUnits = double.MaxValue;
            double massPerMixUnit = 0;
            foreach (Propellant p in sample.propellants)
            {
                PartResourceDefinition def = PartResourceLibrary.Instance.GetDefinition(p.name);
                if (def == null || def.density <= 0 || p.ratio <= 0)
                    continue;
                double amount = parts.Sum(part => part.Resources.Get(p.name)?.amount ?? 0);
                mixUnits = Math.Min(mixUnits, amount / p.ratio);
                massPerMixUnit += p.ratio * def.density;
            }
            PropellantMass = mixUnits < double.MaxValue ? mixUnits * massPerMixUnit : 0;

            var names = new HashSet<string>(sample.propellants.Select(p => p.name));
            SharedWithEngines = parts.Any(part => part.Modules.OfType<ModuleEngines>()
                .Any(e => e.propellants != null && e.propellants.Any(p => names.Contains(p.name))));

            RcsDeltaV = PropellantMass > 0 && PropellantMass < Mass && Isp > 0
                ? Isp * PhysicsGlobals.GravitationalAcceleration * Math.Log(Mass / (Mass - PropellantMass))
                : 0;

            if (RcsDeltaV < Settings.ControlRcsDvLow)
                FuelGrade = Grade.Bad;
            else if (RcsDeltaV < Settings.ControlRcsDvTight)
                FuelGrade = Grade.Weak;
            else if (RcsDeltaV > Settings.ControlRcsDvHigh)
                FuelGrade = Grade.TooMuch;
            else
                FuelGrade = Grade.Ok;
        }
    }
}
