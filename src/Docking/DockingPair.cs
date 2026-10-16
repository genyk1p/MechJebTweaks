using System.Collections.Generic;
using UnityEngine;

namespace MechJebTweaks.Docking
{
    /// <summary>Snapshot of the active vessel, its target vessel and their surroundings.</summary>
    internal class DockingPair
    {
        public VesselDockingState Active;
        public VesselDockingState Other;
        public Vessel OtherVessel;

        /// <summary>The other vessel has the active vessel (or one of its parts) as its target.</summary>
        public bool Mutual;

        /// <summary>The other vessel is loaded (its parts exist) and within Settings.DockingMaxRange.</summary>
        public bool Loaded;

        /// <summary>
        /// The other vessel is physically simulated (not packed). In orbit KSP only unpacks vessels
        /// within ~200 m of the active one (packs again beyond ~350 m); a packed vessel cannot turn,
        /// so its SAS cannot aim it. Not a start condition: it aims itself once we get close.
        /// </summary>
        public bool Simulated;

        /// <summary>Distance at which KSP unpacks the other vessel in its current situation, m.</summary>
        public double UnpackRange;

        public int PartsNearby;
        public int WarpLimit;

        public int RcsBlocks;
        public string RcsPropellant = string.Empty;
        public double RcsPropellantAmount;

        public double TotalMass;

        /// <summary>Translation capability of the active vessel's RCS and the speeds that follow from it.</summary>
        public RcsCapability Rcs;

        public static DockingPair Measure(Vessel active)
        {
            var pair = new DockingPair();
            ITargetable target = active.targetObject;
            pair.Active = VesselDockingState.Measure(active, target);
            pair.OtherVessel = target?.GetVessel();
            pair.TotalMass = active.totalMass;

            Vessel other = pair.OtherVessel;
            if (other != null && other != active)
            {
                pair.Mutual = other.targetObject != null && other.targetObject.GetVessel() == active;
                pair.Loaded = other.loaded && pair.Active.Distance <= Settings.DockingMaxRange;
                pair.Simulated = other.loaded && !other.packed;
                if (other.loaded)
                {
                    pair.UnpackRange = other.vesselRanges.GetSituationRanges(other.situation).unpack;
                    pair.Other = VesselDockingState.Measure(other, other.targetObject);
                    pair.TotalMass += other.totalMass;
                }
            }

            pair.PartsNearby = CountPartsNearby(other ?? active);
            pair.WarpLimit = Settings.WarpLimitForParts(pair.PartsNearby);
            MeasureRcs(active, pair);
            pair.Rcs = RcsCapability.Measure(active);
            return pair;
        }

        /// <summary>Parts of every loaded vessel within range of the center vessel, bystanders included.</summary>
        private static int CountPartsNearby(Vessel center)
        {
            int parts = 0;
            Vector3d position = center.GetWorldPos3D();
            foreach (Vessel vessel in FlightGlobals.VesselsLoaded)
            {
                if ((vessel.GetWorldPos3D() - position).magnitude <= Settings.DockingMaxRange)
                    parts += vessel.parts.Count;
            }
            return parts;
        }

        private static void MeasureRcs(Vessel vessel, DockingPair pair)
        {
            var propellants = new HashSet<string>();
            foreach (ModuleRCS rcs in vessel.FindPartModulesImplementing<ModuleRCS>())
            {
                if (!rcs.rcsEnabled)
                    continue;
                pair.RcsBlocks++;
                foreach (Propellant propellant in rcs.propellants)
                    propellants.Add(propellant.name);
            }

            foreach (string name in propellants)
            {
                PartResourceDefinition definition = PartResourceLibrary.Instance.GetDefinition(name);
                if (definition == null)
                    continue;
                vessel.GetConnectedResourceTotals(definition.id, out double amount, out _);
                pair.RcsPropellant = pair.RcsPropellant.Length == 0 ? name : pair.RcsPropellant + "+" + name;
                pair.RcsPropellantAmount += amount;
            }
        }

        // ---- Conditions for the future "start docking" button ----

        public bool TargetsOk => Active.HasTarget && OtherVessel != null && Mutual;
        public bool TargetHoldOk => Active.TargetHoldAvailable && Active.Controllable && Other.TargetHoldAvailable && Other.Controllable;
        public bool RotationOk => Active.TurnTime90 <= Settings.DockingMaxTurnTime90 && Other.TurnTime90 <= Settings.DockingMaxTurnTime90;
        public bool RcsOk => RcsBlocks > 0 && RcsPropellantAmount > 0.01;

        public bool AllConditionsOk => TargetsOk && Loaded && TargetHoldOk && RotationOk && RcsOk;
    }
}
