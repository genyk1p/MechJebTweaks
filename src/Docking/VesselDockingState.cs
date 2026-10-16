using System;
using MechJebTweaks.ControlCheck;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.Docking
{
    /// <summary>
    /// Everything the docking window needs to know about one vessel of the pair, measured against
    /// that vessel's own target (each vessel keeps its own targetObject, even when not active).
    /// </summary>
    internal struct VesselDockingState
    {
        public Vessel Vessel;
        public ITargetable Target;

        public bool HasTarget => Target != null;

        /// <summary>Distance from this vessel's control point to its target's transform, m.</summary>
        public double Distance;

        /// <summary>Angle between the control point's forward axis and the direction to the target, degrees.</summary>
        public double PointingError;

        /// <summary>Rotation rate, degrees per second.</summary>
        public double AngularRate;

        /// <summary>Approach speed along the line of sight (positive = closing), m/s.</summary>
        public double ClosingSpeed;

        /// <summary>Speed across the line of sight, m/s.</summary>
        public double LateralSpeed;

        public bool SasOn;
        public VesselAutopilot.AutopilotMode SasMode;

        /// <summary>The vessel's pilot or probe core is good enough for SAS target hold (level 3).</summary>
        public bool TargetHoldAvailable;

        public bool Controllable;

        /// <summary>Time to turn 90° using reaction wheels only, s (infinity without wheels).</summary>
        public double TurnTime90;

        public static VesselDockingState Measure(Vessel vessel, ITargetable target)
        {
            var state = new VesselDockingState
            {
                Vessel = vessel,
                Target = target,
                SasOn = vessel.ActionGroups[KSPActionGroup.SAS],
                SasMode = vessel.Autopilot.Mode,
                TargetHoldAvailable = VesselAutopilot.AutopilotMode.Target.AvailableAtLevel(vessel),
                Controllable = vessel.IsControllable,
                AngularRate = vessel.angularVelocity.magnitude * Mathf.Rad2Deg,
                TurnTime90 = TurnTime(vessel, Math.PI / 2)
            };

            Transform reference = vessel.ReferenceTransform;
            if (target == null || target.GetTransform() == null || reference == null)
                return state;

            Vector3d toTarget = ToTarget(vessel, target, reference);
            state.Distance = toTarget.magnitude;
            state.PointingError = Vector3d.Angle(reference.up, toTarget);

            Vector3d direction = toTarget.normalized;
            Vector3d relativeVelocity = vessel.obt_velocity - target.GetObtVelocity();
            state.ClosingSpeed = Vector3d.Dot(relativeVelocity, direction);
            state.LateralSpeed = (relativeVelocity - state.ClosingSpeed * direction).magnitude;
            return state;
        }

        /// <summary>
        /// Vector from the control point to the target. While one of the two vessels is packed (in
        /// orbit: farther than ~200 m) its scene position is a physics frame out of step with ours —
        /// at ~2 km/s that is tens of metres along the track (test 9: 189 m in the scene vs 230 m by
        /// the orbits, and the nose looked 12-14° off the target while SAS held it within 0.5°,
        /// so the autopilot waited 19 s with RCS off). Then the centres of mass come from the orbits,
        /// and only the offsets of the control point and the target within their own vessels from the scene.
        /// </summary>
        public static Vector3d ToTarget(Vessel vessel, ITargetable target, Transform reference)
        {
            Vector3d scene = (Vector3d)target.GetTransform().position - (Vector3d)reference.position;
            Vessel other = target.GetVessel();
            Orbit targetOrbit = target.GetOrbit();
            if (other == null || other == vessel || targetOrbit == null || !(other.packed || vessel.packed))
                return scene;

            double now = Planetarium.GetUniversalTime();
            Vector3d centres = targetOrbit.WorldPositionAtUT(now) - vessel.orbit.WorldPositionAtUT(now);
            Vector3d targetOffset = (Vector3d)target.GetTransform().position - CentreOfMass(other);
            Vector3d ourOffset = (Vector3d)reference.position - CentreOfMass(vessel);
            return centres + targetOffset - ourOffset;
        }

        /// <summary>
        /// Where the vessel's orbit point is in the scene: CoMD in physics; a packed vessel is placed
        /// by its orbit driver at orbit position minus its rotated local centre of mass.
        /// </summary>
        private static Vector3d CentreOfMass(Vessel vessel) =>
            vessel.packed
                ? (Vector3d)vessel.transform.position + (Vector3d)(vessel.transform.rotation * vessel.localCoM)
                : vessel.CoMD;

        /// <summary>
        /// Bang-bang turn time for the given angle with reaction wheel torque only, using the weaker of
        /// the pitch and yaw axes. Moment of inertia is summed from part masses around the vessel's
        /// center of mass — an approximation, good enough to tell "minutes" from "hours".
        /// </summary>
        /// <summary>Slower of pitch and yaw; the same calculation as the control check window (ControlMath).</summary>
        private static double TurnTime(Vessel vessel, double angle)
        {
            Transform reference = vessel.ReferenceTransform;
            if (reference == null)
                return double.PositiveInfinity;

            // Torque axes of ITorqueProvider: x = pitch, y = roll, z = yaw (vessel reference frame).
            Vector3 torque = ControlMath.WheelTorque(vessel.parts);
            double pitch = ControlMath.TurnTime(torque.x, ControlMath.MomentOfInertia(vessel.parts, vessel.CoMD, reference.right), angle);
            double yaw = ControlMath.TurnTime(torque.z, ControlMath.MomentOfInertia(vessel.parts, vessel.CoMD, reference.forward), angle);
            return Math.Max(pitch, yaw);
        }
    }
}
