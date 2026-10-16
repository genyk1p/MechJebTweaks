# Changelog

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
