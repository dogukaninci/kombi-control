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

## Facts learned about this flat

- Heating rate at full power **H ≈ 1.5–2.0 °C/h**; heat loss **L ≈ 0.8–1.9
  °C/h**, rising on cold nights. On 8–9 Nov night (target 22.5) the boiler ran
  100 % from midnight to 08:00 and the room still fell 22.3 → 22.0: **the boiler
  is at its capacity limit on cold nights.** Raising the boiler's own radiator
  water temperature would increase H; the learner adapts on its own.
- Strong **morning/afternoon sun**: the room reaches 25–26 °C with the boiler
  off.
- Radiators lag noticeably: room peaks ~15 min after a burn ends.
- Door alarms: several per sunny day were triggered by the "0.6 °C below the
  5-min peak" rule with a slow slope (−0.1 °C/min). Unconfirmed whether these
  were real airing or the sun going behind clouds; harmless while the room is
  above target.

## Open items

- **Sun holding the room flat above target** (9 Nov, 25–26 °C): those cycles
  aren't warming, so the free-heat rule misses them, and L was taught 1.29 →
  0.52. Planned fix: don't learn loss when the boiler is off and the room is
  ≥ ~1 °C above target. Test in the simulator first (F, and G/H with `SUNAM=3`).
- Confirm the 2-h integral in a report from an evening in auto mode with a
  reachable target.
- Possibly: make the door rule ignore drops right after sun-driven rises.

## Practical gotchas

- Don't compile with `arduino-cli` while the Arduino IDE compiles the same
  sketch: they share a build folder and corrupt it (`cannot parse .xtensa.info
  section`). Use `--build-path <own folder>` for command-line checks.
- The user compiles and uploads from the Arduino IDE (temp ESP on COM3, relay
  on COM6). Firmware uploads keep learned values (EEPROM); use the dashboard's
  "Reset learning" only when they were learned wrongly.
