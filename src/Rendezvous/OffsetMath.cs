using System;
using MuMech;

namespace MechJebTweaks.Rendezvous
{
    /// <summary>
    /// Maneuver math for approaching a target at a safe distance instead of aiming at its center.
    /// Everything is built on MechJeb's own orbit helpers; dV vectors are in world coordinates,
    /// the same convention as VesselExtensions.PlaceManeuverNode.
    /// </summary>
    internal static class OffsetMath
    {
        /// <summary>Probe step for measuring how strongly a burn direction moves the miss distance, m/s.</summary>
        private const double ProbeDv = 0.01;

        /// <summary>Largest correction searched in one direction, m/s.</summary>
        private const double MaxCorrectionDv = 50.0;

        private const int MaxIterations = 60;

        public struct Result
        {
            public bool Ok;
            public Vector3d DeltaV;
            public double UT;
            public double Miss;
            public string Error;
            /// <summary>Only for aimed corrections: when the target is passed, and the burn to match its velocity there.</summary>
            public double ArrivalUT;
            public double BrakeDv;
        }

        /// <summary>Closest approach distance to the target if an impulsive burn dV happens at ut.</summary>
        public static double MissAfterBurn(Orbit orbit, double ut, Vector3d dV, Orbit target)
        {
            Orbit after = orbit.PerturbedOrbit(ut, dV);
            return after.NextClosestApproachDistance(target, ut);
        }

        /// <summary>
        /// Starting from a burn that aims straight at the target (miss close to zero), adds a small
        /// correction along prograde, radial or normal — whichever moves the miss distance most for
        /// the least dV — so that the closest approach becomes <paramref name="distance"/>.
        /// A burn whose miss is already at least that large is returned unchanged.
        /// </summary>
        public static Result OffsetBurn(Orbit orbit, double ut, Vector3d baseDv, Orbit target, double distance)
        {
            double baseMiss = MissAfterBurn(orbit, ut, baseDv, target);
            if (baseMiss >= distance)
                return new Result { Ok = true, DeltaV = baseDv, UT = ut, Miss = baseMiss };

            Vector3d velocity = orbit.WorldOrbitalVelocityAtUT(ut) + baseDv;
            Vector3d[] directions =
            {
                velocity.normalized,
                orbit.RadialPlus(ut),
                orbit.NormalPlus(ut)
            };

            // Most effective direction: largest change of miss distance per m/s.
            Vector3d best = directions[0];
            double bestSensitivity = 0;
            foreach (Vector3d dir in directions)
            {
                double sensitivity = Math.Abs(MissAfterBurn(orbit, ut, baseDv + ProbeDv * dir, target) - baseMiss) / ProbeDv;
                if (sensitivity > bestSensitivity)
                {
                    bestSensitivity = sensitivity;
                    best = dir;
                }
            }

            if (bestSensitivity <= 0)
                return new Result { Error = "miss distance does not respond to small burns" };

            // Solve on both sides and keep the cheaper correction.
            Result plus = Solve(orbit, ut, baseDv, best, target, distance, bestSensitivity);
            Result minus = Solve(orbit, ut, baseDv, -best, target, distance, bestSensitivity);

            if (plus.Ok && minus.Ok)
                return (plus.DeltaV - baseDv).magnitude <= (minus.DeltaV - baseDv).magnitude ? plus : minus;
            if (plus.Ok)
                return plus;
            if (minus.Ok)
                return minus;
            return new Result { Error = "could not reach the requested miss distance" };
        }

        /// <summary>Finds eps > 0 with miss(baseDv + eps * dir) == distance: bracket, then bisect.</summary>
        private static Result Solve(Orbit orbit, double ut, Vector3d baseDv, Vector3d dir, Orbit target, double distance, double sensitivity)
        {
            double tolerance = Math.Max(1.0, 0.02 * distance);

            double lo = 0;
            double hi = distance / sensitivity;
            double missHi = MissAfterBurn(orbit, ut, baseDv + hi * dir, target);

            int iterations = 0;
            while (missHi < distance && iterations++ < MaxIterations)
            {
                lo = hi;
                hi *= 2;
                if (hi > MaxCorrectionDv)
                    return new Result();
                missHi = MissAfterBurn(orbit, ut, baseDv + hi * dir, target);
            }

            double eps = hi;
            double miss = missHi;
            while (Math.Abs(miss - distance) > tolerance && iterations++ < MaxIterations)
            {
                eps = 0.5 * (lo + hi);
                miss = MissAfterBurn(orbit, ut, baseDv + eps * dir, target);
                if (miss < distance)
                    lo = eps;
                else
                    hi = eps;
            }

            if (Math.Abs(miss - distance) > tolerance)
                return new Result();

            return new Result { Ok = true, DeltaV = baseDv + eps * dir, UT = ut, Miss = miss };
        }

        /// <summary>
        /// MechJeb's "Match planes with target" math at whichever of the ascending and descending node
        /// comes first (MechJeb always takes the ascending one). False when the planes do not cross.
        /// </summary>
        public static bool MatchPlanesNearest(Orbit orbit, Orbit target, double startUT, out Vector3d dV, out double ut, out bool ascending)
        {
            bool hasAscending = orbit.AscendingNodeExists(target);
            bool hasDescending = orbit.DescendingNodeExists(target);
            double ascendingUT = 0, descendingUT = 0;
            Vector3d ascendingDV = hasAscending
                ? OrbitalManeuverCalculator.DeltaVAndTimeToMatchPlanesAscending(orbit, target, startUT, out ascendingUT)
                : Vector3d.zero;
            Vector3d descendingDV = hasDescending
                ? OrbitalManeuverCalculator.DeltaVAndTimeToMatchPlanesDescending(orbit, target, startUT, out descendingUT)
                : Vector3d.zero;

            ascending = hasAscending && (!hasDescending || ascendingUT <= descendingUT);
            dV = ascending ? ascendingDV : descendingDV;
            ut = ascending ? ascendingUT : descendingUT;
            return hasAscending || hasDescending;
        }

        /// <summary>Hohmann transfer to the target (MechJeb's own solver), corrected to miss it by distance.</summary>
        public static Result HohmannWithOffset(Orbit orbit, Orbit target, double now, double distance)
        {
            var (dV, ut, _, _) = OrbitalManeuverCalculator.DeltaVAndTimeForHohmannTransfer(orbit, target, now, double.NaN, fixedTime: false, coplanar: false);
            return OffsetBurn(orbit, ut, dV, target, distance);
        }

        /// <summary>Cheapest course correction to intercept (MechJeb's own), corrected to miss by distance.
        /// Searched from 30 s ahead so MechJeb has time to turn the vessel.
        /// The burn is re-solved by MechJeb's Lambert intercept aimed at a point <paramref name="distance"/>
        /// off the target along its orbit normal, at the same arrival time. Searching the miss
        /// distance by small probes (OffsetBurn) fails close to the target: when the vessel is a few
        /// hundred meters away the "next closest approach" jumps between different moments and the
        /// miss distance stops being continuous. OffsetBurn stays as the fallback.</summary>
        public static Result CourseCorrectionWithOffset(Orbit orbit, Orbit target, double now, double distance)
        {
            double burnUT = now + 30.0;
            double arrivalUT = orbit.NextClosestApproachTime(target, burnUT + 2.0);

            Result aimed;
            Vector3d baseDv = Vector3d.zero;
            if (arrivalUT - burnUT >= MinFlight)
            {
                // A real closest approach lies ahead (e.g. after a Hohmann transfer): keep MechJeb's
                // burn time and arrival, only move the aim point off the target.
                baseDv = OrbitalManeuverCalculator.DeltaVAndTimeForCheapestCourseCorrection(orbit, burnUT, target, out double ut);
                aimed = AimedOffsetBurn(orbit, ut, arrivalUT, target, distance);
                Log.Info($"rendezvous offset: correction burn in {ut - now:F0} s, arrival in {arrivalUT - now:F0} s, "
                         + $"base dV {baseDv.magnitude:F2} m/s, aimed {Describe(aimed)} (wanted {distance:F0} m)");
                if (aimed.Ok)
                    return aimed;

                Result probed = OffsetBurn(orbit, ut, baseDv, target, distance);
                if (probed.Ok)
                    return probed;
                Log.Warning($"rendezvous offset: correction failed both ways: {aimed.Error}; {probed.Error}");
                return probed;
            }

            // The vessels are moving apart, so the closest approach is right now and MechJeb would
            // try to catch the target within seconds. Pick the arrival time ourselves instead.
            aimed = CheapestAimedArrival(orbit, burnUT, target, distance);
            Log.Info($"rendezvous offset: moving apart at {orbit.Separation(target, now):F0} m, own arrival search: "
                     + $"{Describe(aimed)}{(aimed.Ok ? $", arrival in {aimed.ArrivalUT - now:F0} s, braking {aimed.BrakeDv:F2} m/s" : "")}");
            return aimed;
        }

        /// <summary>Shortest flight considered for a correction, s.</summary>
        private const double MinFlight = 60.0;

        /// <summary>
        /// Scans flight times from a minute to almost one orbit and keeps the one with the smallest
        /// rendezvous cost: the correction burn plus the velocity-matching burn on arrival.
        /// </summary>
        private static Result CheapestAimedArrival(Orbit orbit, double ut, Orbit target, double distance)
        {
            const int Samples = 80;
            double longest = 0.95 * orbit.period;
            if (!(longest > MinFlight))
                return new Result { Error = "orbit too short for a correction" };

            Result best = new Result { Error = "no flight time gives a usable burn" };
            double bestCost = double.MaxValue;
            for (int i = 0; i <= Samples; i++)
            {
                double flight = MinFlight + (longest - MinFlight) * i / Samples;
                Result r = AimedOffsetBurn(orbit, ut, ut + flight, target, distance);
                if (!r.Ok)
                    continue;
                double cost = r.DeltaV.magnitude + r.BrakeDv;
                if (cost < bestCost)
                {
                    bestCost = cost;
                    best = r;
                }
            }
            return best;
        }

        private static string Describe(Result r) =>
            r.Ok ? $"dV {r.DeltaV.magnitude:F2} m/s, miss {r.Miss:F0} m" : $"failed: {r.Error}";

        /// <summary>Lambert burn at ut that passes the target at arrivalUT, distance off along its orbit normal.</summary>
        private static Result AimedOffsetBurn(Orbit orbit, double ut, double arrivalUT, Orbit target, double distance)
        {
            double flight = arrivalUT - ut;
            if (!(flight > 1.0))
                return new Result { Error = "arrival is not after the burn" };

            Vector3d dV, brake;
            try
            {
                (dV, brake) = OrbitalManeuverCalculator.DeltaVToInterceptAtTime(orbit, ut, target, flight, distance);
            }
            catch (Exception e)
            {
                return new Result { Error = "intercept solver: " + e.Message };
            }

            if (double.IsNaN(dV.x) || double.IsNaN(dV.y) || double.IsNaN(dV.z) || dV.magnitude > MaxCorrectionDv)
                return new Result { Error = "intercept solver gave no usable burn" };

            double miss = ClosestSeparation(orbit.PerturbedOrbit(ut, dV), target, ut, arrivalUT + 0.1 * flight);
            if (miss < 0.5 * distance)
                return new Result { Error = $"aimed burn passes at {miss:F0} m" };

            return new Result { Ok = true, DeltaV = dV, UT = ut, Miss = miss, ArrivalUT = arrivalUT, BrakeDv = brake.magnitude };
        }

        /// <summary>Smallest separation between two orbits in [from, to]: coarse scan, then a fine scan around the best sample.</summary>
        private static double ClosestSeparation(Orbit a, Orbit b, double from, double to)
        {
            const int Steps = 200;
            double step = (to - from) / Steps;
            double bestT = from;
            double best = double.MaxValue;
            for (int i = 0; i <= Steps; i++)
            {
                double t = from + i * step;
                double d = a.Separation(b, t);
                if (d < best) { best = d; bestT = t; }
            }
            double lo = Math.Max(from, bestT - step);
            double fine = 2 * step / Steps;
            for (int i = 0; i <= Steps; i++)
            {
                double d = a.Separation(b, lo + i * fine);
                if (d < best) best = d;
            }
            return best;
        }

        /// <summary>
        /// Velocity-matching burn timed to end at the given distance from the target.
        /// MechJeb centers a burn on its node time and a centered braking burn ends where the
        /// coasting vessel would have been at that time. So the node goes to the moment when the
        /// unbraked vessel is still <paramref name="distance"/> away on its approach — the same
        /// timing MechJeb's rendezvous autopilot uses.
        /// </summary>
        public static Result MatchVelocitiesAtDistance(Orbit orbit, Orbit target, double now, double distance)
        {
            double closestUT = orbit.NextClosestApproachTime(target, now);
            double miss = orbit.Separation(target, closestUT);
            double relativeSpeed = (orbit.WorldOrbitalVelocityAtUT(closestUT) - target.WorldOrbitalVelocityAtUT(closestUT)).magnitude;

            double ut = closestUT;
            if (miss < distance && relativeSpeed > 1e-3)
                ut -= Math.Sqrt(distance * distance - miss * miss) / relativeSpeed;

            // Never in the past; a node needs a moment to be placed and aimed at.
            ut = Math.Max(ut, now + 1.0);

            Vector3d dV = OrbitalManeuverCalculator.DeltaVToMatchVelocities(orbit, ut, target);
            return new Result { Ok = true, DeltaV = dV, UT = ut, Miss = orbit.Separation(target, ut) };
        }
    }
}
