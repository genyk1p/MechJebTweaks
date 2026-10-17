# Changelog

## 0.1.1 — 2026-10-17
- Safe automatic rendezvous, route from an eccentric orbit or one in another plane: besides circularizing at either apsis first, it now considers one burn at a node with the target's plane (at an apsis when the planes are close) that turns the plane and puts the opposite apsis at a waiting orbit next to the target, and takes the cheapest route unless it is much slower. From 3,000 x 93,400 km with planes 36° apart: 1,450 m/s instead of 2,615. Nodes are offered at any plane difference, since a few degrees left for the low orbit cost far more than turning them high up.
- From far away the transfer no longer goes straight onto the target when the braking there would be too long for one burn (more than 12% of the target's orbit, `BrakeMaxOrbitShare`): the route goes via a waiting orbit next to the target. The Hohmann transfer itself is checked before the burn, not after it.
- A waiting orbit is no closer to the target than 3 expected misses of the circularization burn that leads to it (`WaitingMissMargin`; a 12-minute burn of a weak engine ended 30 km off). If the orbit reached misses the waiting orbit anyway, a new one is chosen from the orbit reached.
- MechJeb's transfer solver is started both now and shortly before the transfer window; the cheaper transfer plus braking is taken (between close orbits the search from "now" alone gave 78 m/s instead of 22). A transfer far dearer than expected is refused before the burn.
- Tested from eccentric orbits with planes 5° and 36° apart, from a round orbit at 20,000 km and from 300 km, with a 1.8 m/s² nuclear engine and an 11.4 m/s² chemical one: 94–99 m from the target at 0.05 m/s every time, dV as planned.
- The window lists every route considered as radio buttons: the autopilot's choice is ticked and marked "recommended", ticking another one makes the autopilot fly that one instead.
- The "Flight recorder" window is now called "Black box" in English, so it no longer looks like MechJeb's own "Flight Recorder".
- The rendezvous and docking windows carry "(Geny)" in their names, so they are not taken for MechJeb's own Rendezvous / Docking Autopilot in MechJeb's window list.

## 0.1.0 — 2026-10-16
First public release.
- Warp guard: MechJeb does not raise on-rails time warp while any engine of the vessel still gives thrust, and waits 2 s after the last one goes silent; a negligible leftover throttle is set to zero, so KSP Interstellar Extended engines cannot carry it through the warp.
- Landing autopilot no longer coasts at 1x before a low-orbit deorbit burn (the vessel holding retrograde turns with the orbit, faster than MechJeb's warp threshold on low orbits of small bodies).
- "Rendezvous with offset" window: Hohmann transfer, course correction and velocity matching that keep a set distance (100 m by default) from the target; match planes at the nearest node; new orbit N km above or below the target's orbit.
- Safe automatic rendezvous in the same window: circularize, match planes, waiting orbit chosen by cost and time, safe Hohmann transfer, RCS mid-course correction and one simulated, throttle-trimmed braking burn that stops the offset distance from the target; each maneuver planned from the orbit actually reached.
- "Docking" window with a docking autopilot on RCS (start within 2250 m): far approach on MechJeb's intercept course, checkpoints at 100/50/20 m with coasting and time warp between them, final approach on SAS; start conditions, recommended speeds, both vessels aimed at each other.
- "RCS and reaction wheel check" window in the editor and in flight.
- Waypoints (custom, SCANsat, contract) as landing sites in MechJeb's landing guidance.
- MechJeb sees the real full thrust of KSP Interstellar Extended thermal nozzles at any throttle (it overestimated it at low throttle, and landings braked too late).
- Auto physics warp (off / x2 / x3 / x4) for every node MechJeb's node executor burns, under MechJeb's "Auto-warp" checkbox.
- "Flight recorder" window: status line and events to KSP.log for bug reports.
- English and Russian localization; tech tree unlocks in career/science; every feature can be switched off in Settings.cfg.
