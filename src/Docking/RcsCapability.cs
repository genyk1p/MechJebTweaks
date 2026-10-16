using System;
using System.Collections.Generic;
using UnityEngine;

namespace MechJebTweaks.Docking
{
    /// <summary>
    /// Translation acceleration the vessel's RCS can give along its control point axes, and the
    /// approach speeds that follow from it. Computed from the thrusters directly (not from MechJeb's
    /// VesselState), so it works while RCS is switched off.
    /// </summary>
    internal struct RcsCapability
    {
        /// <summary>Towards the control point's forward (speeding up the approach), m/s².</summary>
        public double Forward;

        /// <summary>Backwards (braking the approach), m/s².</summary>
        public double Back;

        /// <summary>Weakest of the four sideways directions (lateral corrections), m/s².</summary>
        public double Lateral;

        /// <summary>
        /// Acceleration available along each reference axis, m/s²: x = right, y = forward (nose),
        /// z = down; "Positive" pushes the vessel towards +axis, "Negative" towards -axis.
        /// </summary>
        public Vector3 Positive;
        public Vector3 Negative;

        public bool Any => Back > 0;

        public static RcsCapability Measure(Vessel vessel) =>
            Measure(vessel.parts, vessel.ReferenceTransform, vessel.totalMass);

        /// <summary>For any set of parts, e.g. the ship in the editor; reference is the control point.</summary>
        public static RcsCapability Measure(IEnumerable<Part> parts, Transform reference, double mass)
        {
            if (reference == null || mass <= 0)
                return default;

            // Thrust in the reference frame: x = right, y = forward (nose), z = down.
            Vector3 positive = Vector3.zero;
            Vector3 negative = Vector3.zero;

            foreach (ModuleRCS rcs in Thrusters(parts))
            {
                if (rcs.flameout)
                    continue;

                float power = rcs.thrusterPower * rcs.thrustPercentage * 0.01f;
                // Translation switches of the module, mapped to reference axes like MechJeb does:
                // X = sideways, Z = fore/aft, Y = up/down.
                var enabled = new Vector3(rcs.enableX ? 1f : 0f, rcs.enableZ ? 1f : 0f, rcs.enableY ? 1f : 0f);

                foreach (Transform thruster in Nozzles(rcs))
                {
                    if (thruster == null || !thruster.gameObject.activeInHierarchy)
                        continue;

                    // ModuleRCS pushes the part along -up (or -forward with useZaxis).
                    Vector3 force = -(rcs.useZaxis ? thruster.forward : thruster.up) * power;
                    Vector3 local = Vector3.Scale(reference.InverseTransformDirection(force), enabled);

                    for (int axis = 0; axis < 3; axis++)
                    {
                        if (local[axis] > 0)
                            positive[axis] += local[axis];
                        else
                            negative[axis] -= local[axis];
                    }
                }
            }

            // Not all of the summed thrust ends up as translation (thruster angles, SAS borrowing RCS for
            // rotation): in flight tests the real acceleration was ~15-40% below the sum.
            double efficiency = Settings.DockingRcsEfficiency;
            return new RcsCapability
            {
                Positive = positive * (float)(efficiency / mass),
                Negative = negative * (float)(efficiency / mass),
                Forward = efficiency * positive.y / mass,
                Back = efficiency * negative.y / mass,
                Lateral = efficiency * Math.Min(Math.Min(positive.x, negative.x), Math.Min(positive.z, negative.z)) / mass
            };
        }

        /// <summary>RCS modules that are switched on and can fire.</summary>
        public static IEnumerable<ModuleRCS> Thrusters(IEnumerable<Part> parts)
        {
            foreach (Part part in parts)
            {
                foreach (PartModule module in part.Modules)
                {
                    if (module is ModuleRCS rcs && rcs.isEnabled && rcs.rcsEnabled && !rcs.isJustForShow)
                        yield return rcs;
                }
            }
        }

        /// <summary>The module's nozzles. In flight ModuleRCS has them listed; in the editor the list may still be empty.</summary>
        public static IEnumerable<Transform> Nozzles(ModuleRCS rcs)
        {
            if (rcs.thrusterTransforms != null && rcs.thrusterTransforms.Count > 0)
                return rcs.thrusterTransforms;
            return rcs.part.FindModelTransforms(rcs.thrusterTransformName) ?? new Transform[0];
        }

        /// <summary>Touchdown speed: braking it to zero fits into Settings.DockingTouchStopDistance with the safety factor.</summary>
        public double TouchSpeed =>
            Math.Min(Settings.DockingTouchSpeedMax, Math.Sqrt(2 * Back * Settings.DockingTouchStopDistance / Settings.DockingSafetyFactor));

        /// <summary>Far approach speed: braking to touchdown speed fits into Settings.DockingBrakeDistance with the safety factor.</summary>
        public double FarSpeed
        {
            get
            {
                double touch = TouchSpeed;
                return Math.Min(Settings.DockingFarSpeedMax,
                    Math.Sqrt(touch * touch + 2 * Back * Settings.DockingBrakeDistance / Settings.DockingSafetyFactor));
            }
        }

        /// <summary>Distance needed to brake from speed to the touchdown speed, with the safety factor, m.</summary>
        public double BrakingDistance(double speed) => BrakingDistance(speed, TouchSpeed);

        /// <summary>Distance needed to brake from speed to targetSpeed, with the safety factor, m.</summary>
        public double BrakingDistance(double speed, double targetSpeed)
        {
            if (speed <= targetSpeed || Back <= 0)
                return 0;
            return Settings.DockingSafetyFactor * (speed * speed - targetSpeed * targetSpeed) / (2 * Back);
        }
    }
}
