# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A two-ESP32 home boiler ("kombi") thermostat plus a single-file web dashboard, all talking over a public MQTT broker (`broker.hivemq.com`). There is no build system, package manager or test suite — just two Arduino sketches and one HTML file.

- `boiler_temp_mqtt/boiler_temp_mqtt.ino`: the **brain**. Reads a DHT22, runs all control logic, drives a PCD8544 (Nokia 5110) LCD, persists settings in EEPROM and history/logs in LittleFS, and serves a small HTTP API.
- `boiler_relay_mqtt/boiler_relay_mqtt.ino`: the **dumb, fail-safe actuator**. Drives the relay on GPIO 23 based solely on the `boiler` field of the temp ESP's messages.
- `index.html`: the browser dashboard (English UI), using mqtt.js over WSS (port 8884). It's served by GitHub Pages from this repo, so a change goes live only after it's pushed. The user only sets a target temperature there (no offsets, min times or schedule — those were removed on purpose), and can reset the TPI learning. Chrome blocks any http request or download from the https Pages site to the ESP (mixed content). So `/history` and `/logs` are fetched only when the page is opened over http, and "Download report" opens the ESP's own page `http://<ip>/` (`httpRootHandler`), where the `/export` download link works.
## Building / flashing

Arduino IDE or `arduino-cli` with the ESP32 Arduino core **3.x** (the sketches use the `esp_task_wdt_config_t` struct API, which doesn't exist in core 2.x). Required libraries: DHT sensor library (Adafruit), PubSubClient, ArduinoJson (7.4.3 is installed; the code uses the deprecated `StaticJsonDocument<N>` name, whose `N` v7 ignores, but the sizes are also large enough for v6), Adafruit GFX, Adafruit PCD8544.

`arduino-cli` isn't on PATH; use the copy bundled with Arduino IDE 2, with the IDE's config, so it finds the installed ESP32 core (3.0.7) and the libraries in the sketchbook (`C:\Users\doguk\OneDrive\Desktop`):

```powershell
& "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" --config-file "$env:USERPROFILE\.arduinoIDE\arduino-cli.yaml" compile --fqbn esp32:esp32:esp32 boiler_temp_mqtt
```

Upload the same way with `upload -p COMx`. A full ESP32 compile takes several minutes, so run it in the background.

Debug via serial at 115200 baud; every subsystem logs with a `[TAG]` prefix. The dashboard is opened directly in a browser; there's nothing to build.

## Architecture and cross-file contracts

**Control authority lives only in the temp ESP.** TPI control, mode and door detection are all decided there; it publishes the result as `boiler: true/false`. The relay ESP must not duplicate that logic. It only adds safety gating in `setRelay()`: the relay is on only if requested **and** a heartbeat arrived within 30 s. It also forces off on door-open, `manual_off`, Wi-Fi loss and MQTT loss. Keep this fail-safe behavior intact when editing.

**MQTT topics** (prefix = `DEVICE_ID`, `silent-hearth-681`, hard-coded in both sketches):
| Topic | Publisher → Subscriber | Payload |
|---|---|---|
| `/temp` (retained, every 3 s) | temp → relay, dashboard | big JSON built with `snprintf` in `sensorUpdate()` |
| `/heartbeat` (every 15 s) | temp → relay | `{"alive":true}` |
| `/boiler/cmd` | dashboard → temp, relay | `auto` / `manual_on` / `manual_off` |
| `/settings` | dashboard → temp | `{"target":21.5}` and/or `{"tpi_reset":true}` |
| `/door` (retained) | temp | `"1"` / `"0"` |
| `/relay/status` (retained) | relay → dashboard | JSON with `relay`, `reason`, `safe`, `hb_age_ms`, … |

The `/temp` payload is ~350 bytes (fields: `t h boiler door bat mode target duty cycle_left_s heat_rate loss_rate kp integ learn_n on_min off_min ip`; the relay reads only `t boiler door bat mode target`). Both sketches define `MQTT_BUFFER_SIZE` (1024); the temp ESP formats into `TEMP_PAYLOAD_SIZE` (900) and refuses to publish a truncated payload, and the relay parses into `TEMP_JSON_SIZE` (1024). When adding fields, keep these in sync across both sketches. PubSubClient silently drops incoming messages larger than its buffer.

**Temp ESP internals worth knowing:**
- **Self-learning TPI** (`tpi*` functions). Control runs in 10-minute cycles; at each cycle start `duty = L/H + Kp·(target − T) + I` (clamped 0–1), and the boiler is ON for the first `duty × 10 min`. `H` = learned heating rate (°C/h at 100 %), `L` = learned loss rate (°C/h at 0 %), `Kp = 1/(H·1.5 h)`, `I` = slow integral with anti-windup. Pulses under 5 min are skipped and OFF gaps under 3 min become full ON (no short cycling); the duty lost to that rounding is carried to later cycles (`dutyCarry`, reset when duty is 0 or on abort), so a small demand becomes an occasional 5-min pulse rather than never firing. Exception (`tpiKick`): right after the user **raises** the target, if the room is ≥ 0.2 °C below it, the first cycle fires the minimum pulse at once instead of skipping. Simply rounding every short pulse up was tested in the simulation and nearly doubled boiler starts, so it's limited to this case. Second exception: in any cycle, if the room is ≥ `TPI_UNDERSHOOT_KICK` (0.3 °C) below target, the minimum pulse fires instead of being skipped. This is for after airing a room; in the simulation it barely added starts.
- `/temp` also carries `reason` (`heating`, `short`, `cycle_off`, `warm`, `overshoot`, `door`, `manual_on`, `manual_off`, `starting` — from `boilerReason()`), `cycle_on_s`, `carry`, `min_on_s` and `min_off_s`. The dashboard turns these into a plain-language "Why" row.
- **Door/window detection** (`doorDetect()`): open when the 2-min regression slope ≤ −0.25 °C/min or the temperature is ≥ 0.6 °C below the 5-min peak; closed after 3 min with slope > −0.05 °C/min (the buffer is then cleared so the old peak can't re-trigger). The old endpoint rule (−0.8 °C/min) was too strict for the slow DHT22 and never fired.
- **Host simulator**: a C# port of the TPI and door code with a room/radiator/sensor model was used to choose these constants (min ON/OFF 5/3, `TPI_EXCITE`, door thresholds). It lives outside the repo; if you change those constants, re-test with an equivalent simulation rather than guessing. Safety overrides: `T ≥ target+0.4` cuts the ON pulse; `T ≤ target−2` forces 100 %.
- **Learning**: during a cycle, temperature samples feed a linear regression for the slope (°C/h). At cycle end, recursive least squares (λ = 0.97) fits `slope = H·u − L`, with `u = 0.6·this duty + 0.4·previous duty` to account for radiator lag. Outlier slopes (> 6 °C/h) are rejected. Cycles where the room **warms with the boiler off** (slope > `TPI_GAIN_SLOPE` 0.3 °C/h, `u = 0`) are free heat (sun, cooking), so they aren't learned from (`CF_GAIN`), and `L` has a floor of 0.2 °C/h. On 7 Nov a sunny afternoon drove `L` to 0, and the boiler then waited ~30 min in the evening while the room fell to 21.2 against a 22 °C target. Full RLS (H and L) runs only when `u` differs from its running average by ≥ `TPI_EXCITE` (0.25); otherwise only L is nudged (`tpiLearnLossOnly`). Under steady load only the L/H ratio is observable, and lower thresholds let sensor noise drag H down over days — verified in a host simulation, so don't lower it without re-testing. `H`, `L`, `I` and the cycle count are saved to EEPROM every 3 cycles (`TPI_SAVE_EVERY`); the RLS covariance isn't saved. Any mode change, door-open or target change calls `tpiAbort()`; the next cycle starts immediately and isn't used for learning.
- Decision order in `boilerDecision()`: door open → off (overrides even `manual_on`); else mode; else TPI.
- There are no physical buttons (removed) and the LCD backlight is held off.
- EEPROM layout is fixed by the `ADDR_*` offsets. **Bump `VERSION_VALUE` whenever the layout changes**, otherwise stale bytes are read as settings. On a version mismatch, defaults are written.
- LittleFS `/history.bin`: packed 9-byte `HistRecord`s. A record is written on a ±0.1 °C change, a boiler change, a settings change or a 5-minute gap. It's compacted to 24 h every 100 records and on boot. History needs NTP; with no NTP, nothing is recorded.
- LittleFS `/log.txt`: `ts|LEVEL|msg` lines, trimmed by 25 % past 12 KB. Use the `LOG_INFO`/`LOG_WARN`/`LOG_ERROR`/`LOG_BOOT` macros for persistent logs; plain `Serial.printf` for transient debug.
- HTTP (port 80, CORS `*`): `GET /history` → `[[ts, temp, boilerFlag], ...]`, `GET /logs`, `GET /status`, and `GET /export`, which streams (chunked) one CSV diagnostics report with `[summary]`, `[cycles]`, `[temperature]` and `[log]` sections. The dashboard's "Download report" button opens it in a new tab, and the user uploads that file for analysis.
- LittleFS `/tpi.bin`: one packed 46-byte `CycleRecord` per finished or aborted TPI cycle (`cycleLog()`), with `CF_*` flags saying how the cycle was used for learning. It's trimmed to the newest 3 days. If you change the struct, old records will be misread, so delete the file or rename it.
- The task watchdog is 15 s (temp) and 30 s (relay). ESP32 core 3.x already starts it at boot with **5 s** (`CONFIG_ESP_TASK_WDT_TIMEOUT_S`), so `esp_task_wdt_init()` fails and the code falls back to `esp_task_wdt_reconfigure()`. Without that fallback the real timeout was 5 s, and the ESPs reset in loops: the Nov 2026 logs show dozens of "Task watchdog" boots. Blocking loops (Wi-Fi connect, NTP) must call `esp_task_wdt_reset()`. PubSubClient's CONNACK wait is capped with `setSocketTimeout(4)` (the default is 15 s, with no watchdog feeding), and the temp ESP retries MQTT at most every 5 s.

## Repository and credentials

This folder is the git working copy of https://github.com/dogukaninci/kombi-control (a **public** repo; `index.html` at the root is served by GitHub Pages). Push with the Git for Windows install at `C:\Program Files\Git\cmd\git.exe`; GitHub credentials are cached by Git Credential Manager.

Wi-Fi credentials live in each sketch's `secrets.h` (`SECRET_WIFI_SSID`, `SECRET_WIFI_PASSWORD`). That file is git-ignored; `secrets.example.h` is the committed template. Never put credentials back into the `.ino` files. Before pushing, check that nothing staged contains them (`git grep --cached`).
