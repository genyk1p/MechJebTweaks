using System;
using System.Collections.Generic;
using MuMech;
using UnityEngine;

namespace MechJebTweaks.PhysicsWarp
{
    /// <summary>
    /// Physics warp for every maneuver node MechJeb's node executor burns (its own buttons, the maneuver
    /// planner, the node editor, the safe rendezvous autopilot):
    ///   - turning to the node: the chosen rate while more than AimAngle degrees off, the last degrees at 1x;
    ///   - burning: the chosen rate while more than max(MinBurnDv, TailSeconds x full acceleration) is left;
    ///   - MechJeb's own warp to the node (on rails) is left alone.
    ///
    /// The last degrees of a turn go at 1x even when the option is off and the player switched physics warp on:
    /// MechJeb warps to the node only once the vessel has settled (less than 1 degree off and turning slower than
    /// 0.001 rad/s), and at x4 the attitude control shakes the vessel too much for that (2026-10-13 20:08,
    /// transpoer_block1: MechJeb sat on the node until the player dropped physics warp himself).
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class ExecutorPhysicsWarp : MonoBehaviour
    {
        /// <summary>Physics warp is on for the current turn (for the hysteresis).</summary>
        private bool _turnWarp;

        /// <summary>
        /// The burn has reached its tail: 1x until the executor leaves the burn. Near the threshold the
        /// remaining dV jitters (2026-10-15, transpoer_block1, Mun transfer: four times up to x4 and back,
        /// "1x rails / 1x physics" flicker).
        /// </summary>
        private bool _burnTail;
        private bool _failed;

        private void FixedUpdate()
        {
            if (_failed)
                return;
            try
            {
                Run();
            }
            catch (Exception e)
            {
                _failed = true;
                Log.Error($"auto physics warp: stopped for this scene: {e}");
            }
        }

        private void Run()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            MechJebCore core = vessel == null ? null : vessel.GetMasterMechJeb();
            if (core == null || core.Node == null || !core.Node.Enabled)
            {
                _turnWarp = false;
                _burnTail = false;
                return;
            }
            // MechJeb warps to the node itself: leave it alone, or the two switch the warp mode back and forth
            // (2026-10-13, test 8: 1x physics <-> 1x rails five times).
            if (core.Warp.warpToUT > 0)
                return;
            List<ManeuverNode> nodes = vessel.patchedConicSolver?.maneuverNodes;
            if (nodes == null || nodes.Count == 0)
                return;

            double off = core.Attitude.Enabled ? core.Attitude.attitudeAngleFromTarget() : 0;
            bool wanted;
            switch (core.Node.State)
            {
                case MechJebModuleNodeExecutor.States.WARPALIGN:
                    wanted = off > Settings.AutoPhysicsWarpAimAngle + (_turnWarp ? 0 : Settings.AutoPhysicsWarpHysteresis);
                    _turnWarp = wanted;
                    if (!AutoPhysicsWarp.On && !wanted && core.Node.Autowarp)
                        AutoPhysicsWarp.DropPhysicsWarp();
                    break;
                case MechJebModuleNodeExecutor.States.BURN:
                    double left = nodes[0].GetBurnVector(vessel.orbit).magnitude;
                    double tail = Math.Max(Settings.AutoPhysicsWarpMinBurnDv, Settings.AutoPhysicsWarpTailSeconds * core.VesselState.maxThrustAccel);
                    wanted = !_burnTail && left > tail;
                    _burnTail = !wanted;
                    _turnWarp = false;
                    break;
                default:
                    wanted = false;
                    _turnWarp = false;
                    break;
            }
            if (core.Node.State != MechJebModuleNodeExecutor.States.BURN)
                _burnTail = false;
            AutoPhysicsWarp.Apply(core, vessel, wanted);
        }
    }
}
