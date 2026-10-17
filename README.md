# MechJeb Tweaks

Small additions to [MechJeb2](https://github.com/MuMech/MechJeb2). MechJeb's own files are not modified; each feature can be switched off in `Settings.cfg`.

Forum thread: https://forum.kerbalspaceprogram.com/topic/231955-112x-mechjeb-tweaks-010-safe-rendezvous-step-by-step-docking-autopilot-warp-guard-and-more-for-mechjeb/

Video (8 min): safe automatic rendezvous from a 3,000 × 93,400 km orbit 36° off the station's plane, then the docking autopilot — https://youtu.be/Z6sDFio8I0U

## 1. Warp guard

### The problem

Many engines (for example in KSP Interstellar Extended) do not shut down instantly: after the throttle is cut, their thrust decays over several seconds (`engineDecelerationSpeed`). MechJeb only commands zero throttle and never checks the actual thrust, so it can switch to on-rails warp while the engine is still pushing. The thrust tail is lost and the orbit ends up slightly off.

### What it does

Before MechJeb raises **on-rails** time warp, the guard checks the actual thrust of every engine of the vessel. Warp is held back while any engine still produces thrust, and for 2 more seconds after the last one goes silent. Lowering warp is never blocked.

Engine throttle decays exponentially and never reaches zero by itself. Once no engine is commanded to burn and the remaining tail is negligible (below 0.001 m/s² by default), the guard sets those engines' throttle to exactly zero. A commanded burn is never cut.

This matters most for KSP Interstellar Extended: its engines keep any leftover throttle as thrust through the **whole** warp, so without the guard a few percent of throttle at warp start silently changes the orbit for hours. The guard only uses stock engine data and works with any engines; KSPIE is not required.

This covers every place MechJeb warps, because all of them go through one method (`MechJebModuleWarpController.SetTimeWarpRate`):

- maneuver node executor ("execute next / all nodes"), including warp to the next node after a burn;
- landing autopilot (deorbit, coast to deceleration, deceleration burn, plane change);
- ascent autopilot (warp to launch);
- rendezvous autopilot, Warp Helper, rover autopilot;
- the "resume warp" button.

Physics warp is not blocked by default: it still simulates thrust, and MechJeb's classic ascent throttles on purpose while physics-warping through its coast. You can enable it in `Settings.cfg`.

**Landing autopilot stuck at 1x before a low-orbit deorbit burn.** In the step "moving to low deorbit burn point" MechJeb asks for warp only while the vessel turns slower than 0.001 rad/s. Holding retrograde, the vessel turns with its orbit, and low orbits of small bodies turn faster than that (JNSQ Mun below ~210 km, stock Mun below ~200 km), so MechJeb coasted at 1x until the player warped by hand. The mod asks for the same warp under MechJeb's other conditions when the vessel turns about as fast as the orbit and points where MechJeb wants.

## 2. Rendezvous with offset

A new window in MechJeb's window list, next to the Rendezvous Planner, called "Rendezvous with offset (Geny)" so it is not taken for MechJeb's own rendezvous windows.

### The problem

The Rendezvous Planner's "Hohmann transfer" aims at the target's center (a planned miss of a few meters), and "Match velocities at closest approach" places the braking burn exactly at the closest approach. MechJeb centers each burn on its node, so the braking burn ends at the closest approach point — a few meters from the target. Any execution error or a weak engine means a collision course.

### What it does

The window has an **offset distance** (100 m by default, saved by MechJeb) and three maneuvers:

| Button | What it plans |
|---|---|
| **Hohmann transfer with offset** | MechJeb's Hohmann transfer, corrected by a tiny extra burn (usually well under 1 m/s) so the planned closest approach equals the offset distance. Even if braking never happens, the trajectory passes the target at that distance. |
| **Course correction with offset** | MechJeb's cheapest course correction, corrected the same way. Use it after the transfer burn, when the real miss differs from the plan. |
| **Match velocities at offset distance** | A velocity-matching burn moved earlier so that it **ends** at the offset distance from the target instead of at the closest approach point. Same timing as MechJeb's rendezvous autopilot uses. |

All three buttons plan from the orbit after the last planned maneuver node and append the new node, the same way MechJeb's Maneuver Planner chains maneuvers — so you can plan circularization and plane change elsewhere and continue here. Use "Remove last node" to undo a step.

The correction direction (prograde, radial or normal) is chosen automatically as the one that moves the miss distance most for the least Δv. "Closest approach" in the window turns red while it is below the offset distance.

The window also shows the relative inclination and has **Match planes with target** (MechJeb's math, but at whichever of the ascending and descending node comes first — MechJeb always takes the ascending one) and **New orbit relative to target** (phasing orbit), both chained after the planned nodes like the rest. The phasing orbit is MechJeb's "Establish new orbit at N km", but N km is counted from the target's orbit (its average altitude), not from the surface: 200 puts you 200 km above the target, −200 puts you 200 km below; an orbit below the surface is refused.

Typical use: **Match planes with target** → **New orbit relative to target** → **Hohmann transfer with offset** → (optionally) **Course correction with offset** → **Match velocities at offset distance**. You end up stopped about 100 m from the target.

**Start safe automatic rendezvous** does all of that by itself (the button turns into **Stop safe automatic rendezvous** while it runs). It never plans the whole route at once: each maneuver is computed from the orbit the previous burn actually gave and executed by MechJeb's node executor (auto warp on), so the errors of long burns do not pile up. Before every burn it picks the first step that applies:

0. From an eccentric orbit, or one in another plane, the whole route is chosen once at the start, every candidate estimated in dV and time until the velocities are matched: circularize at apoapsis or at periapsis first, or **one burn at a node** with the target's plane that turns the plane and puts the opposite apsis at a waiting orbit next to the target (at an apsis, keeping the plane, when the planes are close). High up on an eccentric orbit the vessel is slow, so turning the plane there is cheap: from 3,000 × 93,400 km with the planes 36° apart the node route took 1,450 m/s where circularizing at apoapsis would take 2,615. The cheapest route wins unless it is more than 2× slower than the fastest while the fastest costs at most 10% more. The window lists every route; the autopilot's choice is ticked and marked "recommended", tick another one to fly it instead;
1. our orbit eccentric → circularize at the apsis nearest the chosen radius;
2. planes differ by more than 5° → match planes at the nearest node;
3. no waiting orbit yet and our orbit too close in size to the target's, or so far away that braking after a transfer straight onto the target would last more than 12% of the target's orbit (one burn then curves far off the approach line) → waiting orbit next to the target (raise or lower, then circularize). Candidates from 0.5% to 25% of the target's radius above and below it are each timed from the actual phase of the target; the cheapest one that takes at most 2× as long as the 25% orbit is taken. A waiting orbit closer to the target than 3× the expected miss of the circularization burn is not used (a 12-minute circularization of a weak engine ended 30 km off); if the orbit reached misses the waiting orbit anyway, a new one is chosen from the orbit reached;
4. otherwise the safe Hohmann transfer. MechJeb's solver searches from two starting times, now and shortly before the transfer window, and the cheaper transfer plus braking is taken (between close orbits the search from "now" alone found 78 m/s instead of 22). A transfer far dearer than expected, or one whose braking would be too long, is refused before the burn.

Then a mid-course correction halfway to the target **with RCS only** (skipped without RCS or propellant), and the final braking — one steady burn tail-first against the relative velocity, not a maneuver node. A node is an impulse that MechJeb spreads around its time; with a weak engine the burn lasts minutes, the approach curves and the vessel passes the target still fast (test: 550 m/s at ~2 m/s², passed at 92 m doing 55 m/s, stopped 1 km behind — MechJeb's own "match velocities" does the same). Here the start is found by simulating the whole finite burn (thrust, mass flow, gravity, the target's orbit) so that it ends the offset distance short of the target; during the burn the rest is re-simulated every 2 s from the actual state and only the throttle is trimmed (5% less thrust than expected would otherwise mean passing the target by km), so the vessel does not turn. The engine stops at 1 m/s, RCS takes the rest. A burn that would end more than 2 km aside (engine too weak) is refused. It refuses targets in eccentric orbits (above 0.01), shows the estimated dV of the whole route next to the game's dV figure, removes planned nodes when it starts, stops when a step repeats three times without result, and logs every step to KSP.log (`rendezvous autopilot:`). Thresholds in `RENDEZVOUS_AUTOPILOT` in Settings.cfg.

## 3. Docking

A "Docking (Geny)" window in MechJeb's window list with a docking autopilot that approaches in short RCS corrections with coasting and time warp between checkpoints, instead of MechJeb's continuous thrusting. The window shows:

- distance, closing and lateral speed to the target;
- for both vessels: pointing error towards each other's target, rotation rate, SAS mode;
- start conditions: the vessels target each other, the other vessel is loaded by the game (within 2.5 km), both can use SAS target hold (SAS level 3), both can turn 90° with reaction wheels within 60 s, RCS and its propellant available;
- parts of all vessels within 2.5 km and the warp limit that follows from it, total mass.
- RCS acceleration (forward / braking / sideways, computed from the thrusters even with RCS off) and the speeds that follow from it: far approach speed (braking to touchdown speed fits into 100 m), touchdown speed (stopping fits into 10 m), both with a 2x margin and capped at 5 and 1 m/s; where to start braking at the current closing speed; time to cancel sideways drift.

In orbit KSP only simulates (unpacks) another vessel within about 200 m of the active one; farther away it is loaded but frozen and its SAS cannot turn it. The window says so. With "Keep target hold on both vessels" on, the other vessel starts aiming itself as soon as it unfreezes.

**Docking autopilot** (RCS only; start within 2250 m — beyond that KSP unloads the target):

- **Far approach** (beyond 250 m): MechJeb's Lambert solver gives the velocity needed to meet the target in distance / far approach speed seconds — orbital curvature included, so sideways drift does not build up — and RCS steers towards it until the predicted closest approach (from the orbits, as in MechJeb's rendezvous info) is within 15 m. Then one coast, in warp, straight to the point where braking for 100 m must begin — on rails the closest approach does not change — and the near approach takes over.
- **Near approach** (within 250 m, or after the far approach): at the checkpoints 100, 50 and 20 m (those still ahead) both vessels aim at each other with SAS target hold while RCS sets the velocity straight at the target point; then RCS goes off and the vessel coasts — in time warp, limited by the parts nearby, when the leg is long — to where braking for the next point must begin. Beyond 100 m it flies at the far approach speed, from 100 m on at the touchdown speed. After 20 m RCS stays off except one short sideways check at 8 m, and SAS alone holds the aim until the ports latch.
- Braking is triggered by distance in every phase. KSP drops SAS target hold on vessel switches and when leaving warp; the autopilot re-engages it on both vessels.
- Stops on: player's Stop, vessel switch, target lost, out of RCS propellant, closing too fast while coasting, checkpoint not reached in time, no contact on the final approach.

Buttons: **Start docking** / **Stop docking**, **Point both vessels at each other** (SAS target hold on both vessels, the other one included, no switching needed) and **Keep target hold on both vessels** (re-engages it if KSP drops it, e.g. on vessel switch).

Before starting: on both vessels "Control From Here" on the docking port you will dock with, and each vessel targets the other one's port. The window says what is missing. In career and science mode the window opens with the tech node advUnmanned, where stock probe cores get SAS target hold.

## 4. RCS and reaction wheel check

An "RCS and reaction wheel check" window in MechJeb's window list, in the VAB/SPH (the ship being built; the nose is taken from the root part) and in flight (the active vessel). It grades, with the docking autopilot's own calculations:

- **reaction wheels**: torque and 90° turn time in pitch, yaw and roll; grade (excellent ≤ 15 s, fine ≤ 30 s, slow ≤ 60 s, too little beyond — the docking autopilot refuses); the torque missing for a 30 s turn;
- **RCS**: acceleration forward, back and sideways (from the real nozzle directions), the approach speeds that follow; grade (very weak < 0.02, weak < 0.05, strong > 0.5 m/s²); the thrust missing in the weakest direction;
- **RCS propellant**: amount and delta-v (thrust-weighted vacuum Isp); grade (too little < 10, tight < 20, a lot > 100 m/s); a note when engines burn the same propellant.

Moments of inertia treat parts as point masses, like the docking autopilot, so turns come out slightly faster than in reality. All thresholds are in `Settings.cfg`, node `CONTROL_CHECK`.

## 5. Waypoints as landing sites

MechJeb's landing guidance has a landing site drop-down with a "Set" button, shown only when the current body has a site — MechJeb knows the launch sites and its own config, so over other bodies it never appears. The game's waypoints are added to that list every couple of seconds: custom ones (made in SCANsat or on the stock map) and contract ones, under their names. MechJeb's own window is not changed; only entries added by this mod are ever removed again. Switch off with `LANDING_SITES / FromWaypoints = false`.

## 6. Thrust of KSP Interstellar Extended thermal nozzles

MechJeb takes an engine's available thrust the way the game computes thrust: maxFuelFlow × Isp at the current pressure × g, times the thrust limiter. That is exact for engines whose thrust is proportional to the throttle (stock, chemical, jets). KSPIE thermal nozzles (`ThermalEngineController`) rewrite maxFuelFlow every physics frame from the reactor state and give more than their share at low throttle: the Lightbulb gave 15.4 kN at 5% and 174.8 kN at 100%, so at MechJeb's minimum throttle of 5% MechJeb saw 308 kN. The landing autopilot planned the deceleration burn for that thrust, started it at 32 km instead of ~62 km on the JNSQ Mun and hit the ground at 280 m/s.

For engines on parts with one of the listed modules only, the mod measures the thrust while the engine runs at full throttle (≥ 95%), and MechJeb then sees exactly that thrust at any throttle. It sets the thrust, not the fuel flow: at low throttle KSPIE raises the flow and lowers Isp, and a first version that capped only the flow left MechJeb at 102 kN — the landing came down 22 km short. In an atmosphere KSPIE takes its `pressureThreshold` (exit area × static pressure) off the vacuum thrust, and so does the mod. Before the first full-throttle run the part's passport thrust is a cap only. The engine's own value is changed only during MechJeb's calculation and restored right after. KSP.log: `thrust cap:`; the flight recorder adds `mjmax=` (MechJeb's available thrust). Node `THRUST_CAP` in `Settings.cfg`.

## 7. Auto physics warp for maneuver nodes

One global choice **Auto physics warp: off / ×2 / ×3 / ×4**, shown right under MechJeb's own "Auto-warp" checkbox in the maneuver planner and the node editor (a Harmony transpiler adds the row) and in the "Rendezvous with offset" window. It is the same value everywhere and is remembered for all vessels (`PluginData/AutoPhysicsWarp.cfg`).

For every node MechJeb's node executor burns (its own buttons, the planner, the node editor, the safe rendezvous autopilot):

- the turn to the node runs in physics warp while the nose is more than 5° off; the last degrees at 1x, so MechJeb can settle the vessel and warp on rails to the node;
- the burn runs in physics warp while more than max(0.5 m/s, TailSeconds (0) × full acceleration) is left; the tail at 1x. MechJeb throttles down in proportion to what is left, so a physics step at ×4 takes only a few percent of the rest — no overshoot (with 3 m/s the 1x tail still took 12 s, it is MechJeb's exponential throttle-down);
- MechJeb's own warp to the node is left alone; mode and rate are set in the same physics tick (otherwise KSP switches back to on-rails mode in vacuum before the rate rises).

Even with the choice off, physics warp the player switched on is dropped for the last 5° of the turn: MechJeb warps to the node only once the vessel points within 1° **and** turns slower than 0.001 rad/s, which the coarse attitude control at ×4 never reaches — the executor sat on the node until the player dropped physics warp by hand. The black box logs MechJeb's aim error and the vessel's rotation rate (`aim off=… w=…`). Node `AUTO_PHYSICS_WARP` in `Settings.cfg`.

## 8. Black box

A "Black box" window in MechJeb's window list with one checkbox. While it is on, a status line of the active vessel goes to `KSP.log` every second (warp, SAS, RCS, thrust, MechJeb's available thrust and aim error, running MechJeb modules, target) plus events (warp changes, vessel switches, SAS/RCS/target changes, staging, docking). Lines start with `[MechJebTweaks] rec:`. Meant for bug reports: if an autopilot misbehaves, record the flight and attach `KSP.log`.

## Settings

`GameData/MechJebTweaks/Settings.cfg`:

| Node / key | Default | Meaning |
|---|---|---|
| `WARP_GUARD / Enabled` | `true` | Switch the warp guard on or off. |
| `WARP_GUARD / SnapAcceleration` | `0.001` | When no engine is commanded to burn and the remaining thrust is below this acceleration (m/s²), the leftover throttle is set to zero. |
| `WARP_GUARD / SettleSeconds` | `2.0` | Extra wait after the last engine stops. |
| `WARP_GUARD / GuardPhysicsWarp` | `false` | Also hold back physics warp. |
| `WARP_GUARD / ShowMessages` | `true` | On-screen message while warp is held back. |
| `LANDING_WARP / Enabled` | `true` | Landing autopilot warps towards a low-orbit deorbit burn (see below). |
| `LANDING_WARP / AimAngle`, `ExtraRate` | `2`, `0.002` | The nose must point within this many degrees of MechJeb's direction and turn at most this much (rad/s) faster than the orbit. |
| `RENDEZVOUS_OFFSET / Enabled` | `true` | Show the "Rendezvous with offset" window in MechJeb's list. |
| `AUTO_PHYSICS_WARP / DefaultRate` | `4` | Auto physics warp until the player picks one (0 = off, 2, 3, 4). |
| `AUTO_PHYSICS_WARP / AimAngle`, `Hysteresis` | `5`, `2` | Turns in physics warp while more than AimAngle degrees off; back into it only beyond AimAngle + Hysteresis. |
| `AUTO_PHYSICS_WARP / MinBurnDv`, `TailSeconds` | `0.5`, `0` | Burns in physics warp while more than max(MinBurnDv m/s, TailSeconds × full acceleration) is left. |
| `DOCKING / Enabled` | `true` | Show the "Docking" window. |
| `DOCKING / MaxRange` | `2500` | The other vessel must be closer than this, m. |
| `DOCKING / MaxTurnTime90` | `60` | Each vessel must turn 90° with reaction wheels within this time, s. |
| `DOCKING / AlignedAngle` | `0.5` | Aimed at each other below this pointing error, degrees. |
| `DOCKING / Warp50MaxParts`, `Warp10MaxParts`, `Warp5MaxParts` | `150`, `400`, `800` | Warp limit by parts within MaxRange: 50x, 10x, 5x, more — no warp. |
| `DOCKING / FarSpeedMax`, `TouchSpeedMax` | `5`, `1` | Caps of the recommended speeds, m/s. |
| `DOCKING / BrakeDistance`, `TouchStopDistance`, `SafetyFactor` | `100`, `10`, `2` | Braking room for the far approach and for touchdown, m, and the margin. |

## Requirements

- KSP 1.12.x
- MechJeb2 2.15.3.1 or newer
- Harmony 2.2.1 or newer (`000_Harmony`)

The rendezvous window is built on MechJeb's own classes, so KSP does not load this mod at all without MechJeb 2.15.3+. If a future MechJeb changes the functions the window uses, the buttons report an error instead of planning a maneuver, and the warp guard keeps working on its own. The warp guard finds MechJeb's warp controller at runtime; if that changes, it logs a warning to `KSP.log` and does nothing.

## Installation

- **CKAN:** search for "MechJeb Tweaks" (once it is indexed).
- **By hand:** download the zip from [Releases](https://github.com/genyk1p/MechJebTweaks/releases) and copy `GameData/MechJebTweaks` into the game's `GameData` folder. Install MechJeb2 and Harmony 2 as well.

Safe to add to or remove from an existing save: the mod adds no part modules. The offset distance is saved by MechJeb with its own settings, the auto physics warp choice in `GameData/MechJebTweaks/PluginData/`.

## Similar mods

MechJeb itself has a rendezvous autopilot, a rendezvous planner and a docking autopilot. This mod does not replace them, it adds windows next to them: MechJeb's rendezvous aims at the target and ends its braking at the closest approach point, its docking autopilot thrusts continuously; here the rendezvous keeps a set distance from the target all the way, and the docking autopilot flies short RCS corrections with coasting and time warp between checkpoints. No other mod with the warp guard, the KSPIE thrust fix or auto physics warp for MechJeb nodes is known.

## Building

Requires the .NET SDK (any recent version; the project targets .NET Framework 4.8 through reference assemblies, like MechJeb itself).

```
dotnet build src -c Release
```

The KSP install path defaults to the Steam location; override it with the `KSPDIR` environment variable or `-p:KSPDIR=...`. The DLL is written to `GameData/MechJebTweaks/Plugins/`.

## AI disclosure

This mod was written in collaboration with **[Claude Code](https://claude.com/claude-code)** (Anthropic), model **Claude Opus 5.5**. The code, configs and documentation were generated by the AI; the author defined the idea and requirements, reviewed the design and tested it in game.

## License

GPL v3, see [LICENSE](LICENSE) (MechJeb2 is GPL v3 too). Author: Geny.

---

## По-русски

Небольшие доработки MechJeb, файлы самого MechJeb не изменяются. Каждую часть можно выключить в `Settings.cfg`.

1. **Защита варпа.** MechJeb не включает ускорение времени (on-rails варп), пока двигатели ещё дают тягу, и ждёт ещё 2 секунды после того, как погаснет последний. Остаток дросселя, когда он уже ничтожен, обнуляет сам. Особенно важно для KSPIE: его двигатели продолжают тянуть весь варп с тем дросселем, что был при входе в варп. KSPIE для мода не обязателен. Работает во всех автопилотах MechJeb. Заодно исправлено зависание автопилота посадки на 1x перед сходом с низкой орбиты: MechJeb ждал, пока корабль перестанет вращаться, а корабль, держащий ретроград, вращается вместе с орбитой (на низких орбитах Муны быстрее порога MechJeb).
2. **Окно «Планировщик осторожного сближения (Geny)»** в списке окон MechJeb. Задаёшь дистанцию (по умолчанию 100 м), и три кнопки строят манёвры с безопасным запасом: «Гомановский переход с промахом» (траектория проходит мимо цели на этой дистанции), «Коррекция курса с промахом» и «Совместить скорости на дистанции» (торможение заканчивается на этой дистанции, а не в точке сближения). Манёвры считаются от орбиты после последнего запланированного манёвра и добавляются в конец, как в планировщике манёвров MechJeb. Ещё в окне есть относительное наклонение, кнопка «Выровнять плоскости» (в ближайшем узле) и «Новая орбита от цели на N км». Это круговая орбита на N км выше орбиты цели, а при минусе — ниже (MechJeb считает такую высоту от поверхности). Если орбита уходит под поверхность, окно выдаёт ошибку.

   Кнопка «Начать автоматическое безопасное сближение» делает всё это сама. Пока она работает, на её месте кнопка «Остановить…». Маршрут не планируется заранее целиком: каждый манёвр считается от орбиты, которая получилась после предыдущего прожига, поэтому погрешность не накапливается. Шаги:
   - выбор маршрута. С вытянутой орбиты или из другой плоскости автопилот при старте считает все варианты целиком, с Δv и временем: округление в апоцентре или в перицентре либо **один импульс в узле** с плоскостью цели, который поворачивает плоскость и опускает противоположный апсид на орбиту ожидания рядом с целью (если плоскости почти совпали — импульс в апсиде без поворота). Высоко на вытянутой орбите корабль летит медленно, и поворот плоскости там дешёвый: с 3000 × 93 400 км при наклоне 36° маршрут через узел стоил 1450 м/с против 2615 у округления в апоцентре. Берётся самый дешёвый, если он не медленнее самого быстрого вдвое при разнице в Δv до 10 %. Все варианты видны в окне, выбранный отмечен «рекомендован»; можно отметить другой, и автопилот полетит по нему;
   - округление в апсиде, ближайшем к выбранной высоте;
   - выравнивание плоскостей, если расхождение больше 5°;
   - орбита ожидания, если наша орбита слишком близка к орбите цели или так далеко, что торможение после прямого перехода длилось бы больше 12 % витка цели. Перебираются орбиты от 0,5 до 25 % радиуса цели выше и ниже её; для каждой по нынешнему положению цели считается, сколько ждать окна и сколько это стоит. Берётся самая дешёвая из тех, что не дольше вдвое, чем орбита на 25 % выше. Слишком близкие к цели (ближе трёх ожидаемых промахов длинного скругления) не берутся, а если орбита ожидания всё же не попала, она выбирается заново;
   - безопасный гомановский переход. Решатель MechJeb ищет его дважды — от «сейчас» и незадолго до окна перехода, — берётся вариант дешевле вместе с торможением. Слишком дорогой переход или такой, после которого торможение было бы слишком долгим, отклоняется до прожига;
   - коррекция курса на полпути, только на RCS (без RCS шаг пропускается);
   - торможение одним ровным прожигом кормой вперёд. Обычный манёвр «совместить скорости» на слабом двигателе проскакивает цель, MechJeb тоже. Поэтому момент старта находится расчётом всего прожига, с тягой, расходом топлива и кривизной орбиты, так чтобы корабль остановился в 100 м от цели. По ходу прожига расчёт повторяется, и подстраивается только дроссель, корабль не крутится. Остаток скорости гасится на RCS.

   Цель с вытянутой орбитой (эксцентриситет больше 0,01) автопилот не берёт. Перед стартом окно показывает оценку Δv на весь маршрут. Пороги задаются в разделе `RENDEZVOUS_AUTOPILOT` файла Settings.cfg.
3. **Окно «Автопилот ступенчатой стыковки (Geny)»** с автопилотом стыковки только на RCS (старт ближе 2250 м). Дальше 250 м — дальний подход по курсу перехвата MechJeb (задача Ламберта) с варпом кусками и проверкой промаха; ближе — контрольные точки 100, 50, 20 м: на них оба корабля наводятся друг на друга и RCS выставляет скорость точно на цель, между точками — полёт по инерции и варп, после 20 м RCS выключается и стыкует одна стабилизация. Скорости считаются от мощности RCS. Также: состояние обоих кораблей, проверка условий, подсказки скоростей для ручной стыковки, кнопка «Навести оба корабля друг на друга» и удержание режима «на цель».
4. **Окно «Экспертиза RCS и маховиков»** (название подобрано так, чтобы стоять последним среди окон MechJeb, перед инфо-окнами) — в VAB/SPH (строящийся корабль, нос — по корневой детали) и в полёте. Время разворота на 90° на маховиках, ускорение от RCS вперёд/назад/вбок и скорости подхода, ΔV на топливе RCS — каждое с оценкой и тем, сколько не хватает (кН·м момента, кН тяги). Считается теми же формулами, что и автопилот стыковки; пороги — в `Settings.cfg`, узел `CONTROL_CHECK`.
5. **Вейпоинты как места посадки.** В «Помощнике посадки» MechJeb появляется выпадающий список мест посадки с кнопкой «Set»: в него добавляются вейпоинты игры — свои (созданные в SCANsat или на карте) и контрактные, по названиям.
6. **Тяга тепловых сопел KSPIE.** MechJeb считает доступную тягу по расходу топлива, как сама игра, — это точно для двигателей, у которых тяга пропорциональна дросселю. Ядерные сопла KSPIE на малом дросселе дают больше своей доли («Лампочка»: 15,4 кН на 5 %, 174,8 кН на 100 %), и MechJeb на 5 % видел 308 кН — автопилот посадки начинал торможение слишком поздно. Только для таких двигателей мод замеряет тягу на полном дросселе, и MechJeb видит ровно её при любом дросселе (в атмосфере — за вычетом потерь по формуле самого KSPIE); до первого замера паспортная тяга детали служит потолком. Остальные двигатели не затрагиваются.
7. **Авто-физварп для манёвров.** Один общий выбор «Авто-физварп: выкл / ×2 / ×3 / ×4» под галочкой «Авто-варп» в планировщике манёвров и редакторе узлов MechJeb и в окне сближения. Значение одно на все корабли и запоминается. Для любого узла, который выполняет MechJeb: разворот на физварпе, пока нос дальше 5° от узла, последние градусы на 1x, чтобы MechJeb успокоил корабль и ушёл в обычный варп. Прожиг на физварпе, пока осталось больше 0,5 м/с, хвост на 1x. Даже при выключенном выборе физварп, включённый вручную, на последних 5° разворота сбрасывается: иначе MechJeb стоит на узле и не варпит.
8. **Окно «Чёрный ящик»** с одной галочкой: пока она включена, в KSP.log раз в секунду пишется состояние корабля (варп, SAS, RCS, тяга, работающие модули MechJeb, цель) и события. Для отчётов об ошибках автопилотов.

Мод можно ставить и убирать в идущую игру: он не добавляет модулей деталям.

Мод написан в соавторстве с **Claude Code** (Anthropic), модель **Claude Opus 5.5**: код, конфиги и документацию сгенерировал ИИ, автор задал идею и требования, проверил архитектуру и протестировал в игре.
