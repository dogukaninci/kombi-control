# Development notes: what was tried, measured and decided

This is the history behind the current control logic, for anyone (or any new
Claude Code session) picking the project up. `CLAUDE.md` describes how the code
works *now*; this file explains *why*, including the ideas that were tested and
rejected. Times are local (UTC+3); "report" means the CSV from the ESP's
`/export` page (see `CLAUDE.md`).

## How changes are evaluated

1. **Host simulator** – `tools/tpisim/TpiSim.cs`, a line-by-line C# port of the
   TPI and door-detection code in `boiler_temp_mqtt.ino`, driving a room model:
   `dT/dt = a·r − b·(T − Tout) − window + sun`, radiator `r` with one or two lag
   stages, and a DHT22 model (1.5-min lag, noise, 0.1 °C steps). Its defaults
   match the current firmware. Build and run on Windows without extra tools:

   ```
   C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe -nologo -optimize TpiSim.cs
   TpiSim.exe skip all kick down        # scenarios A–E
   TpiSim.exe skip gainfix kick down    # F: sunny afternoons + daily airing
   TpiSim.exe skip real kick down       # G/H: two-stage radiator lag, like this flat
   ```

   Scenarios: **A** typical house, cold start · **B** target changes · **C** weak
   boiler · **D** powerful boiler, fast radiators · **E** window/door openings ·
   **F** sun 10–15 h + daily airing · **G** this flat (3 °C/h, lag 8+15 min) ·
   **H** slow radiators (lag 12+25 min).
   Options via environment variables: `LAGTAU` (radiator lag in min, 0 = old duty
   weighting), `GAINRULE` (0 = old free-heat rule), `TI` (integral time, h),
   `SUNAM` (morning sun 8–12 h, °C/h), `AIR=0` (no airing in G/H), `TABLE=1`
   (6-hourly table), `WLO`/`WHI` (error-weighted learning), `LAGPRED`, `SKIPCUT`,
   `ECLIP`. Key outputs: RMS error, minutes/day above target+0.2, mean sensor
   offset, % of time the sensor reads exactly the target, boiler starts/h, and
   the learned H/L next to the true values.
2. **Real reports** from the flat, read after each change.

Rule kept throughout: a constant is changed only after the simulator shows it
is no worse in **every** scenario, never by guessing.

## Timeline

### 6 Nov – base system
- Replaced fixed offsets/schedule with a **self-learning TPI** (user only sets a
  target). Physical buttons removed. Dashboard sends the target automatically
  (step 0.1 °C, no Save button).
- MQTT buffers enlarged (1024) – the `/temp` JSON outgrew PubSubClient's
  default buffer and was silently dropped.
- Door detection made more sensitive: the old endpoint rule (−0.8 °C/min)
  never fired with the slow DHT22. Now 2-min regression slope ≤ −0.25 °C/min or
  ≥ 0.6 °C below the 5-min peak (thresholds chosen in scenario E).
- Min ON/OFF changed from the user's original 3/2 min to **5/3 min** (fewer
  starts; commercial TPI thermostats use similar minimums).
- Diagnostics: `/export` report, `reason` field and the dashboard "Why" row.

### 7 Nov – first real reports
- **Task-watchdog boot loops** in older logs: ESP32 core 3.x pre-starts the
  watchdog at 5 s, so `esp_task_wdt_init()` failed silently. Fixed with the
  `esp_task_wdt_reconfigure()` fallback and `setSocketTimeout(4)`.
- **Sunny afternoon taught "no heat loss"**: L went to 0, the boiler then waited
  ~30 min in the evening (room 21.2 vs 22). Fix: cycles that warm with the
  boiler off are not learned from (`CF_GAIN`), L floor 0.2, and an undershoot
  kick (≥ 0.3 °C below target → minimum pulse fires).
- **10-min burns from carried duty overheated the room** (+0.2 °C ~15 min after
  the burn, radiator lag). Fix: when the excess is only carry and demand is
  < 50 %, round down to a 7-min burn instead of a full 10 min.

### 7–8 Nov – radiator lag
- Symptom: H fell 2.0 → 1.06 in one evening, L hit its floor, the room sat
  0.1–0.3 °C below target. Cause: the learner used
  `u = 0.6·this duty + 0.4·previous duty`, too short for this flat's radiators,
  so heat still coming out after a burn looked like "no heat loss".
- Reproduced in the simulator with a two-stage lag (G/H): H converged to
  0.4–1.1 against a true 3.0.
- Fix: `radHeat`, a first-order lag of the boiler state (`TPI_RAD_TAU_MIN`), and
  learning against its cycle average. Above-target time: G 29 → 0, H 514 → 0
  min/day; C learned H 1.25 (true 1.2) instead of 0.30.
- Lag constant: 10 min was neutral everywhere; 15 min was best for slow
  radiators but hurt D (fast, powerful boiler); **12 min** chosen.

### 8 Nov – sun with warm radiators
- Morning sun took the room 22 → 25.7 °C with the boiler off. The first two
  cycles after the last burn still had `radHeat` 0.15–0.36, so the old
  "`u` < 0.1" free-heat test missed them and H fell 2.0 → 0.47.
- Fix: skip a cycle when **the boiler didn't run at all** and the room warmed
  more than the leftover radiator heat explains (`slope > 0.3 + H·u`). Equal or
  better in all scenarios. Confirmed in the next report: sunny cycles were
  flagged `gain` all day and H stayed at 2.0.

### 8–9 Nov – steady-state offset
- The room averaged ~22.08 °C against 22.0 (sensor time: 22.0 28 min, 22.1 33
  min, 22.2 18 min). Integral time `TPI_TI_H` 4 h → **2 h**. Simulator: time at
  exactly target G 33 → 43 %, A 37 → 49 %, ≥ 0.2 off roughly halved, no extra
  starts. 1 h oscillated with slow radiators (H: 22 % at target, 192 min/day
  above), so it was rejected. Real-world effect not yet confirmed: the next two
  reports had an unreachable target and then manual mode.

### 10 Nov – sensor location, weather model, capacity limit
- Night of 9–10 Nov, 04:10–07:10: the reading swung ~1 °C in 10 min down
  and up again; two of these were flagged as an open window although everyone
  was asleep. Cause: **the sensor sat on the window sill next to a radiator**
  (cold glass downdraft / radiator plume). Advice given: move it to an inner
  wall at ~1.5 m, away from window, radiator, door and direct sun, then
  "Reset learning". This probably also inflated the sun peaks (25–26 °C) and
  the slow-slope "door" alarms on sunny days. Relay feedback isn't available in
  reports (the relay status only goes to the dashboard), but a lost relay can't
  explain drops 3× faster than the house's normal cooling.
- 10 Nov afternoon: boiler 100 % for 7 h, room 22.6 vs 22.5 target, learned
  L ≈ H ≈ 2.2 °C/h → at capacity. With a 3-min minimum OFF, the next option
  below 100 % is 7/10 min = 70 %. Fix: when the room is **above** target, an
  OFF gap under 3 min may round down to 7 min even at high demand (alternating
  7/10 averages the real demand). Simulator: small gain near capacity, no
  extra starts. (Rounding down at high demand regardless of the error raised
  starts in C from 3.0 to 5.4/h, so it's limited to "above target".)
- **Weather model** (`tpiLearnW`, see `CLAUDE.md`): outdoor temperature from
  Open-Meteo every 15 min; heat loss `kLoss·(T − Tout)`. Simulator, current
  firmware vs weather model (sensor reads exactly target / ≥ 0.2 °C off):
  A 49 → 54 % / 4 → 2 %, G 43 → 54 % / 7 → 3 %, K (near capacity) 38 → 45 % /
  21 → 11 %, W (cold front) 48 → 51 % / 9 → 5 %. With the weather service
  2 °C off either way it stays better. Sunny scenarios got slightly worse on
  above-target time (F 400 → 447, S 449 → 495 min/day; mostly sun the boiler
  can't remove).
- New simulator scenarios: `TpiSim.exe skip extra kick down` runs **K** (near
  capacity), **W** (cold front 12 → 0 °C in 3 h) and **S** (strong morning sun
  + airing with a wall-heat rebound). Options `WEATHER=0`, `WBIAS`, `NEAR710`,
  `SUNSKIP`, `DOORSKIP`, `REBOUND`.

## Rejected ideas (tested, worse – don't retry without a new reason)

| Idea | Result in the simulator |
|---|---|
| Round every short pulse up to the 5-min minimum | Nearly doubled boiler starts |
| Variable cycle length (short ON with long OFF at low demand, ≤ 3 starts/h) | A: RMS 0.071 → 0.10, 15–30 min/day above target; D: 371 → 1040 min/day above; far fewer learning cycles |
| Lower `TPI_EXCITE` (0.15) | Sensor noise slowly drags H down over days |
| Predictive control (subtract heat still stored in the radiators) | No gain over lag-aware learning alone |
| Learn faster far from target, slower near it (0.5×/2×) | G: 0 → 94 min/day above target; A RMS 0.081 → 0.109 |
| Learn faster only far from target (1×/3×) | Sun/airing pumped H to 4.9 (true 3.1); evening undershoot with slow radiators |
| Skip overshoot-cut cycles / clip RLS errors | No consistent benefit |
| Don't learn heat loss when the boiler is off and the room is ≥ 1 °C above target (sun holding the room flat) | Alone: small gain in S; with the weather model: S 495 → 520, D 437 → 526 min/day above target |
| Skip 2 more learning cycles after a door/window closes (wall-heat rebound) | S 449 → 495, E 428 → 459 min/day above target |
| Round down to 7 min at any high demand (not only above target) | C (weak boiler) starts 3.0 → 5.4/h |
| Stretch the cycle at high demand so the 3-min OFF stays and ON/(ON+OFF) = demand (e.g. 17/3, 37/3 min); `STRETCH=<max min>`, `STRETCHFROM` | From 70 %, max 30 min: G at target 54 → 34 %, H 45 → 16 % (≥ 0.2 off 7 → 52 %), C starts 3.1 → 5.4/h. Milder (from 85 %, max 20 min): G 54 → 43 %, H 45 → 25 %, K 45 → 41 %. Long burns can't react to radiator lag; only the cold front (W) improved slightly |

## Facts learned about this flat

- Heating rate at full power **H ≈ 1.5–2.0 °C/h**; heat loss **L ≈ 0.8–1.9
  °C/h**, rising on cold nights. On 8–9 Nov night (target 22.5) the boiler ran
  100 % from midnight to 08:00 and the room still fell 22.3 → 22.0: **the boiler
  is at its capacity limit on cold nights.** Raising the boiler's own radiator
  water temperature would increase H; the learner adapts on its own.
- Strong **morning/afternoon sun**: the room reaches 25–26 °C with the boiler
  off (partly because the sensor sat on the window sill until at least 10 Nov).
- Boiler radiator water temperature set to **55 °C** (10 Nov).
- Radiators lag noticeably: room peaks ~15 min after a burn ends.
- Door alarms: several per sunny day were triggered by the "0.6 °C below the
  5-min peak" rule with a slow slope (−0.1 °C/min). Unconfirmed whether these
  were real airing or the sun going behind clouds; harmless while the room is
  above target.

## Open items

- **Sensor relocation** (10 Nov advice): check whether it was moved before
  reading a report; data from the window sill overstates sun/radiator effects.
- First report with the weather model: is `tout_c` filled in, did `kLoss`
  settle (≈ L / (22 − Tout), roughly 0.08–0.15 here), any "Outdoor temperature
  unavailable" warnings?
- Sun holding the room flat above target (9 Nov) taught L 1.29 → 0.52. The
  planned "skip when ≥ 1 °C above target" was worse in the simulator (see
  rejected ideas); re-check after the sensor has moved. A solar-radiation term
  from Open-Meteo (`shortwave_radiation`) is a possible next step.
- The 2-h integral looked right on 10 Nov 00:00–04:00 (22.51–22.53 against
  22.5).
- Possibly: relay feedback in the report (temp ESP subscribing to
  `/relay/status`, logging drops and mismatches).
- Boiler water temperature: 55 °C on 10 Nov; at capacity on cold nights.

## Practical gotchas

- Don't compile with `arduino-cli` while the Arduino IDE compiles the same
  sketch: they share a build folder and corrupt it (`cannot parse .xtensa.info
  section`). Use `--build-path <own folder>` for command-line checks.
- The user compiles and uploads from the Arduino IDE (temp ESP on COM3, relay
  on COM6). Firmware uploads keep learned values (EEPROM); use the dashboard's
  "Reset learning" only when they were learned wrongly.
