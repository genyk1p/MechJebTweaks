using System;
using System.Collections.Generic;
using UnityEngine;

namespace MechJebTweaks.ControlCheck
{
    /// <summary>
    /// Mass, moment of inertia and reaction wheel torque of a set of parts — a vessel in flight or
    /// the ship being built in the editor. Shared by the docking autopilot and the control check
    /// window, so both judge a vessel the same way.
    /// </summary>
    internal static class ControlMath
    {
        /// <summary>
        /// Part mass with resources, t. part.mass includes mass added by modules (TweakScale,
        /// B9PartSwitch...) once Part.UpdateMass has run; in the editor that is refreshed the same
        /// way the editor's own mass readout does it (ShipConstruct.GetShipMass).
        /// </summary>
        public static double Mass(Part part)
        {
            if (HighLogic.LoadedSceneIsEditor)
                part.UpdateMass();
            return part.mass + part.GetResourceMass();
        }

        public static double TotalMass(IEnumerable<Part> parts)
        {
            double total = 0;
            foreach (Part part in parts)
                total += Mass(part);
            return total;
        }

        public static Vector3d CenterOfMass(IEnumerable<Part> parts)
        {
            Vector3d sum = Vector3d.zero;
            double total = 0;
            foreach (Part part in parts)
            {
                double m = Mass(part);
                sum += m * (Vector3d)part.transform.position;
                total += m;
            }
            return total > 0 ? sum / total : Vector3d.zero;
        }

        /// <summary>
        /// Moment of inertia about an axis through center, parts taken as point masses, t·m².
        /// Leaves out each part's own inertia, so it is a little low (turns come out a little fast).
        /// </summary>
        public static double MomentOfInertia(IEnumerable<Part> parts, Vector3d center, Vector3d axis)
        {
            double inertia = 0;
            foreach (Part part in parts)
            {
                Vector3d r = (Vector3d)part.transform.position - center;
                double alongAxis = Vector3d.Dot(r, axis);
                inertia += Mass(part) * (r.sqrMagnitude - alongAxis * alongAxis);
            }
            return inertia;
        }

        /// <summary>
        /// Reaction wheel torque, kN·m: x = pitch, y = roll, z = yaw (vessel reference frame, like
        /// ITorqueProvider). The weaker of the two directions per axis.
        /// </summary>
        public static Vector3 WheelTorque(IEnumerable<Part> parts)
        {
            Vector3 torque = Vector3.zero;
            foreach (Part part in parts)
            {
                foreach (PartModule module in part.Modules)
                {
                    if (!(module is ModuleReactionWheel wheel))
                        continue;
                    wheel.GetPotentialTorque(out Vector3 pos, out Vector3 neg);
                    torque += new Vector3(Math.Min(Math.Abs(pos.x), Math.Abs(neg.x)),
                                          Math.Min(Math.Abs(pos.y), Math.Abs(neg.y)),
                                          Math.Min(Math.Abs(pos.z), Math.Abs(neg.z)));
                }
            }
            return torque;
        }

        /// <summary>Time to turn by angle (rad) at full torque: speed up half way, brake the other half, s.</summary>
        public static double TurnTime(double torque, double inertia, double angle)
        {
            double accel = inertia > 0 ? torque / inertia : 0; // kN·m / (t·m²) = rad/s²
            return accel > 0 ? 2 * Math.Sqrt(angle / accel) : double.PositiveInfinity;
        }

        /// <summary>Torque needed to turn by angle (rad) within time, kN·m.</summary>
        public static double TorqueForTurn(double inertia, double angle, double time) => inertia * 4 * angle / (time * time);
    }
}
