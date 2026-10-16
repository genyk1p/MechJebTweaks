using System;
using UnityEngine;

namespace MechJebTweaks.Docking
{
    /// <summary>
    /// Proportional RCS translation, shared by the docking autopilot and the rendezvous autopilot's
    /// course correction. No integral term: MechJeb's PID controller wound up and circled around the
    /// wanted direction on heavy vessels with weak RCS.
    /// </summary>
    internal static class RcsDrive
    {
        /// <summary>
        /// Translation command for a velocity error (current - wanted, world axes): wanted acceleration =
        /// error / time constant, divided by the RCS acceleration available in the direction that reduces
        /// the error, per reference axis. Axes with an error below the deadband get no command.
        /// </summary>
        public static Vector3d Command(Transform reference, Vector3d worldError, RcsCapability rcs, double deadband)
        {
            Vector3 local = Quaternion.Inverse(reference.rotation) * (Vector3)worldError;
            var command = new Vector3d();
            for (int axis = 0; axis < 3; axis++)
            {
                double e = local[axis];
                if (Math.Abs(e) < deadband)
                    continue;
                // A positive error is reduced by pushing towards -axis, and vice versa.
                double available = e > 0 ? rcs.Negative[axis] : rcs.Positive[axis];
                command[axis] = available > 0 ? e / (available * Settings.DockingRcsTimeConstant) : Math.Sign(e);
                // Stock ModuleRCS ignores a thruster's share at or below 5% (CalculateThrust: totalForce >
                // EPSILON = 0.05). With strong RCS the proportional command for the last few cm/s fell
                // right there (test 14: 0.05 on two axes for 20 s, acceleration 0, RCS on until contact).
                // Canted thrusters get only part of an axis command, hence a margin above 5%.
                if (Math.Abs(command[axis]) < Settings.DockingMinRcsCommand)
                    command[axis] = Math.Sign(command[axis]) * Settings.DockingMinRcsCommand;
            }
            return command;
        }

        /// <summary>
        /// Writes a command from <see cref="Command"/> into the flight controls. MechJeb's convention
        /// (MechJebModuleRCSController): X &lt;- x, Y &lt;- z, Z &lt;- y (fore/aft).
        /// </summary>
        public static void Apply(FlightCtrlState s, Vector3d c)
        {
            s.X = Mathf.Clamp((float)c.x, -1f, 1f);
            s.Y = Mathf.Clamp((float)c.z, -1f, 1f);
            s.Z = Mathf.Clamp((float)c.y, -1f, 1f);
        }

        /// <summary>Total amount of the propellants of the vessel's enabled RCS thrusters.</summary>
        public static double PropellantAmount(Vessel vessel)
        {
            double total = 0;
            var seen = new System.Collections.Generic.HashSet<string>();
            foreach (ModuleRCS rcs in RcsCapability.Thrusters(vessel.parts))
            {
                foreach (Propellant propellant in rcs.propellants)
                {
                    if (!seen.Add(propellant.name))
                        continue;
                    PartResourceDefinition definition = PartResourceLibrary.Instance.GetDefinition(propellant.name);
                    if (definition == null)
                        continue;
                    vessel.GetConnectedResourceTotals(definition.id, out double amount, out _);
                    total += amount;
                }
            }
            return total;
        }
    }
}
