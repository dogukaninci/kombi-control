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

- **Chart history and log over MQTT**: on the https dashboard the chart only
  showed what that browser had seen (localStorage) and the ESP log never
  loaded, because http fetches to the ESP are blocked as mixed content. The
  dashboard now requests both over MQTT and the ESP streams them in chunks
  (see the topic table in `CLAUDE.md`). Decoding was checked in the browser
  with chunks in the firmware's format.

### 11 Nov – room held 0.2 °C high: double correction
- Learning was reset at 22:17 on 10 Nov. Overnight (outdoor 1.5–2 °C) the
  room sat at 22.0–22.4 vs 22.5: the model started with the default kLoss
  (too low), so the integral grew 0.01 → 0.155 while kLoss was learned over
  ~3 h. Once kLoss was right, both corrected the same shortfall. 11:52:
  request 86 % = base 82 % − 10 % (room 0.2 above) + 12 % integral → the room
  settled where P cancels I, ~0.24 °C high, for hours; the integral only bled
  off at ~0.05/h.
- Fix: **bumpless transfer** (learning raises/lowers the base `L/H` → the
  integral gives the same amount back, only toward zero) and `TPI_I_MAX`
  0.25 → 0.15. Simulator, current → new (time at exactly target / above or
  below target): A 54 → 56 %, B 37 → 44 % (above 297 → 249 min/day), C 56 →
  62 %, E above 437 → 193, F 27 → 31 % (above 447 → 372), G 54 → 55 %, H 45 →
  48 %, S 20 → 27 %, N (new, like this night) below 45 → 10 but above 38 → 69,
  K below 0 → 64 min/day. Options `BUMPLESS=0`, `IMAX`.
- New scenario **N** in `extra`: capacity-limited cold night (true H 1.8,
  loss 0.087/°C), day up to 9 °C with mild sun, target 22.5. The summary line
  now also prints minutes/day below target − 0.2.
- The user asked for "faster / harder" reactions. All of these were tested
  and rejected (see the table below): stronger P overall or only above
  target, faster integral unwinding, no carry repayment above target, lower
  overshoot cut. Each made the slow-radiator scenarios oscillate.

### 12 Nov – the bumpless transfer became a ratchet
- Target 22.0 since 11 Nov 14:03 (mild, outdoor 5–13 °C, demand 10–30 %).
  The room sat at 22.1–22.2 all night: hourly means 22.10–22.14, 0–30 % of
  the time at exactly 22.0. No window alarms, no swings — the sensor reading
  is much calmer than before the move.
- Cause (from every night cycle): with 5–7-min pulses the learned loss swings
  ±0.05 in a 3-cycle pattern — burn cycle (room flat), next cycle +0.6 °C/h
  with the boiler off (radiator after-heat) → L down, then −0.7 °C/h → L up.
  Each "L down" step made the bumpless rule reset the negative integral to 0
  (9 times between 02:30 and 07:30), so the integral never got past −0.04 and
  couldn't remove the +0.1 °C offset. Without the reset it would have reached
  the −0.1 it needed within ~4 h.
- Fix: transfer only while `learnN ≤ TPI_BUMP_LEARN_N` (72 cycles ≈ 12 h
  after a reset — when the 11 Nov double correction happened). The simulator
  doesn't reproduce the 3-cycle swing (its room is too clean); always / never
  / fresh-only came out within noise of each other, with and without a model
  bias. New simulator options: `GAIN0` (constant internal gains, °C/h — a
  bias the zero-intercept weather model can't represent), `DIST` (random
  free-heat disturbances, 30-min steps), `BUMPN`, `BUMPMIN`, `BUMPMODE`.
- The real burn-to-rise delay now looks longer than the 12-min radiator
  estimate (the room rises in the cycle *after* a 7-min burn). Possibly the
  new sensor spot; check `TPI_RAD_TAU_MIN` against a few more reports before
  changing it.

### 15 Nov – loss coefficient at its floor; internal-gain model tested
- The 12 Nov fix worked: nights 12–13 Nov at target 22.0 had 3-h means of
  21.94–22.05 (before: 22.10–22.14). 13–14 Nov days were 12–16 °C outside
  and sunny: 22.4–23.1 °C with the boiler off.
- 14→15 Nov night (target 21.5, outdoor ~10 °C): the boiler didn't fire for
  ~14 h and the room stayed 21.9–22.2 — this flat barely loses heat on mild
  nights (neighbours, insulation). Loss-only learning shrank kLoss by ~10 %
  per cycle and it hit `K_LOSS_MIN` (0.005 → L ≈ 0.04 °C/h at target). That
  is right for mild nights but would under-heat at the next cold spell.
- 15 Nov 10:21 a window was opened and the temp ESP was switched off until
  13:00 (the relay held the boiler off, as designed). The report couldn't be
  fetched meanwhile: no ARP/ping, and the retained `/temp` on the broker was
  the last message from before it went off (the dashboard shows retained
  values as if live — check message flow, not the values).
- Tested an **internal-gain term** (`GMODEL=1`: net loss = kLoss·(T − Tout) −
  G, 3-parameter RLS, steady cycles update kLoss and G): new scenario **M**
  (gains 0.6 °C/h, mild sunny days, cold front 10 → 0 °C) improved slightly
  (k 0.049 / G 0.50 vs true 0.06 / 0.60; below target after the front 12 → 0
  min/day), but without real gains it was clearly worse (A at target 55 →
  33 %, N 38 → 23 %, G below 0 → 88 min/day) and with gains 0.4 N fell 40 →
  12 %. A slower variant (`GLAMBDA=0.998`) was still worse (A 55 → 36 %).
  Not adopted: k and G trade off too freely.
- Raising the floor (`KMIN=0.02`) was neutral everywhere and helped M (below
  target 12 → 3 min/day). **Adopted**: `K_LOSS_MIN` 0.005 → 0.02 (the stored
  0.005 is below the new floor, so the first boot replaces it with the
  default 0.033 and learning continues from there).

### 20 Nov – capacity limit taught "weak boiler"
- 15–20 Nov (firmware with `K_LOSS_MIN` 0.02 since 15 Nov 13:17): 6-h means
  within ±0.06 °C of target except sunny middays. kLoss sat at the new floor
  (0.0202) — the floor is doing its job.
- 18 Nov 21:27 target 22.0 → 22.5: the room rose to 22.2 in 10 min, then
  stayed at 22.1–22.2 for 3 h while burns grew 50 → 100 %. Full RLS steps
  read that as a weak boiler: H 1.75 → 1.50 → 1.43 → 1.25 → 1.06 → 0.88. The
  model then over-estimated demand and the integral sat at −0.15 for days.
  Across the whole report the room never exceeded 22.3 while burning ≥ 50 %.
  The user confirmed the boiler is at its limit and doesn't want a higher flow
  temperature (radiator TRVs were considered; not confirmed).
- Fix: cycles with `u ≥ TPI_HSAT_U` (0.7) and slope < `TPI_HSAT_SLOPE` (0.3
  °C/h) are treated as steady — loss only, H kept. Simulator (lowest H after
  12 h / at target): H (slow radiators) 0.77 → 1.61 / 42 → 55 %, G 1.44 →
  1.60, N 38 → 40 % (below target 68 → 60 min/day); C (weak boiler) 56 → 51 %,
  K 47 → 44 %. Variants: threshold 0.5 kept H near true everywhere (N 1.51–
  1.80 vs 1.8) but N below target 68 → 160 min/day and C can never learn its
  low H; an H rate limit (3 %/step, `HRATE`) slowed drift but N below target
  68 → 129. New options `HSATU`, `HSATSLOPE`, `HRATE`; the summary now prints
  the lowest H after 12 h.
- Note: in N the drifted-low H acted as a gain boost (Kp = 1/(H·1.5 h)); a
  correct H made capacity-limited control slightly slower.

### 22 Nov – what commercial and open-source thermostats do; fixed band + slow H
- After the capacity rule (20 Nov) the night at target 22.5 was good (mean
  22.60, H ≈ 1.9). Then 22 Nov 08:49–09:19, boiler off (u 0.31 → 0.01) and
  the room not cooling (morning activity / walls giving back heat; solar
  radiation was only 50–150 W/m², 75 % cloud), four full-RLS steps took H
  1.94 → 0.76: with u ≈ 0 the regressor carries no H information, but the
  RLS cross-covariance moved H anyway.
- Research (sources in the 22 Nov chat):
  - Honeywell/Secure TPI: 6 cycles/h for gas boilers (3 oil, 12 electric);
    minimum ON 1–5 min as an installer setting. Siemens TPI "medium": min
    switch time > 4 min, period ~12 min. → our 10-min cycle, 5/3 min fit.
  - Optimum-start learning (Heatmiser ROC, evohome): heat-up rate adjusted in
    small steps per heating period, not per control cycle.
  - Versatile Thermostat (open source, on/off heaters): `on = coef_int·ΔT_in
    + coef_ext·ΔT_out`, defaults 0.6 / 0.01 — a **fixed** internal gain (~1.7
    K band), no integral. Its Auto TPI learns in ~48-h sessions with
    averaging / decaying EWMA (α₀ 0.08), continuous mode α 0.04, and
    **ignores cycles at 0 % and 100 % power** ("no exploitable
    information"); undersized heating is flagged, not learned.
  - SAT (open source, OpenTherm/on-off): PID with outdoor heating curve,
    3–4 PWM cycles/h for radiators, freezes the integral during solar gain.
  - tado weather adaptation uses solar intensity from online weather data;
    Nest Sunblock corrects a sensor in direct sun; Nest True Radiant /
    evohome stop early for radiator lag. Siemens: no TRVs in the reference
    room with the room sensor.
- Adopted: **fixed proportional band `TPI_BAND_K` 2 K** (Kp 0.5, was
  1/(H·1.5 h)) and **H changes ≤ 2 % per step** (`TPI_H_RATE`). Simulator,
  % at exactly target (above/below min/day): B 40 → 42 (above 304 → 216), C
  51 → 57, E 18 → 31 (above 390 → 94), F 26 → 31, S 23 → 28, M 34 → 36, A/G/K
  same, H 55 → 52, W 52 → 50, N 40 → 37 (below 60 → 146: on capacity-limited
  evenings the old drifted-low H acted as a gain boost), and **D (8 °C/h
  boiler, fast radiators) above target 437 → 773 min/day**: a fixed 2 K band
  is too sharp for a very powerful boiler; such a system needs a much wider
  `TPI_BAND_K`. "Band as a floor" (`BANDMAX`, Kp = 1/max(band, H·1.5 h)) and
  faster upward H (`HRATEUP` 0.10) only partly fixed D (620 / 547) and gave
  back the gains in B, E, F. Learned H stays near
  the start/true value everywhere (N 1.60 vs true 1.8 instead of 0.76).
- Tested and not adopted: band 1.67 K (H 55 → 45 %), band 2.5 K, ignoring H
  in cycles with u < 0.15/0.25 (`HLOWU`, neutral to worse), solar-radiation
  learning gate at 250 W/m² (`SUNGATE`, neutral — the free-heat rule already
  catches the sunny cycles, and the two real H drops had little sun).
- Open-Meteo `current=shortwave_radiation` works over plain http if a
  solar input is needed later. Hourly values are means over the preceding
  hour.

### 26 Nov – morning overshoot: derivative brake
- Fixed band + slow H since 22 Nov (kp 0.5000 confirmed). The user ran
  `manual_on` on the evenings of 24 and 25 Nov.
- Night 25→26 Nov: outdoor 0 → −3.2 °C. kLoss had sat at the 0.02 floor
  after mild days, so the base was too low; the room held 21.8–21.9 against
  22.0 while the integral climbed −0.14 → +0.14 and loss-only learning took
  L 0.43 → 1.2 — the same shortfall corrected twice again (bumpless transfer
  is only active in the first 72 cycles).
- 08:25–09:15 the room rose 21.9 → 22.5 at up to +0.9 °C/h (morning
  activity, outdoor warming), but the request stayed 63–83 % (base ~0.6 +
  I 0.14, P only −0.1 at +0.2 °C): burns at 22.2 and 22.3, the overshoot cut
  stopped the 09:05 one at 22.4. Then sun took the room to 22.9.
- New scenario **Q** (`extra`): nights to −3 °C, morning activity +0.6 °C/h
  07:30–09:30, sun 10–14 h, internal gains 0.2, lag 8+15 min. It reproduces
  the morning peaks (22.5–22.6) and evening dips (21.6).
- Adopted: **derivative brake near target** (`TPI_KD` 0.3 h,
  `TPI_KD_NEAR` 0.3 °C): duty −= 0.3 × last cycle's slope when |error| <
  0.3. Simulator vs. 22-Nov firmware (above/below min/day; % at target): Q
  224/130 → 186/52, H 52 → 57 %, K below 56 → 0, W 50 → 54 %, E above 94 →
  42, S above 485 → 441; N below 146 → 170, A 55 → 52 %, B 42 → 39 %, E 31 →
  26 %. On the real morning it would have skipped the 08:55 and 09:05 burns.
- Tested: KD 0.2 / 0.3 everywhere (slower recovery after target raises and
  windows: B below 29 → 61, E below 203 → 319), KD only for rising rooms
  (`KDPOS`, Q below 130 → 161), KD 0.4 near target (K below 0 → 144),
  bumpless with a 0.05 deadband always on (`BUMPMIN`, neutral). Options
  `KD`, `KDNEAR`, `KDPOS`.
- Still open: kLoss collapses to the floor on mild days and the integral
  then has to carry cold nights; H drifts down slowly in Q-like mornings
  (free heat during moderate burns), now bounded by the 2 % step limit.

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
| Also freeze the integral after a cycle that burned 100 % (`AWFULL=1`; 11 Nov: I rose 0.01 → 0.155 on a capacity-limited night while the request was 52–77 % but rounding burned 100 %, then held the room 0.1–0.3 °C high the next morning) | Small gains in most scenarios, but K (near capacity, the case it targets) got worse: at target 45 → 40 %, ≥ 0.2 off 11 → 15 %. Not adopted; the excess integral bleeds off in ~3 h |
| Stronger proportional gain: `TPI_HORIZON_H` 1.5 → 1.0 h (`HORIZON=1.0`) | H (slow radiators) at target 45 → 16 %, ≥ 0.2 off 7 → 51 %; D above 437 → 537 min/day |
| Stronger proportional only while above target (`KPABOVE=2`) | H at target 45 → 15 %, ≥ 0.2 off 7 → 53 % |
| Unwind a wrong-way integral 3×/5× faster (`UNWIND`) | H at target 45 → 32 % / 22 %; N above 38 → 71 / 47 min/day |
| Above target, don't repay carried duty with catch-up burns (`NOCARRYABOVE=1`) | G at target 54 → 25 %, H 45 → 16 % |
| Cut the burn / skip the cycle already at target + 0.25 (`OVER=0.25`, now 0.4) | H above 0 → 213 min/day; A 54 → 50 % |
| Internal-gain term in the weather model (`GMODEL=1`, net loss = kLoss·ΔT − G) | Without real gains: A at target 55 → 33 %, N 38 → 23 %; with gains 0.4: N 40 → 12 %. Slower variant `GLAMBDA=0.998` still A 55 → 36 % |
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
