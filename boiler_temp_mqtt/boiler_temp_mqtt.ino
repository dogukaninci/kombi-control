/*
  ============================================================
  BOILER SYSTEM — TEMPERATURE & CONTROL ESP32  v8
  ============================================================
  Features:
    - Self-learning TPI (Time Proportional & Integral) control
        The user only sets a target temperature. Every 10-minute
        cycle the boiler runs for duty × 10 min, where
          duty = L/H + Kp·(target − T) + I
          H  = learned heating rate  (°C/h at 100 % boiler)
          L  = learned heat-loss rate (°C/h at 0 % boiler)
          Kp = 1 / (H · TPI_HORIZON_H)
          I  = slow integral correction
        H and L are re-estimated after every complete cycle with
        recursive least squares (forgetting factor) from the
        measured temperature slope, and persisted to EEPROM.
    - LittleFS persistent history (/history.bin)
    - HTTP GET /history → JSON array [[ts, temp, boiler], ...]
    - NTP real UTC timestamps
    - Smart logging: ±0.1°C change | boiler transition | settings change | max 5min
    - Compact every 100 records (purge older than 24h)
    - MQTT /temp message includes "ip" field
    - No buttons; LCD backlight is kept off
  ============================================================
*/

#include "DHT.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <esp_task_wdt.h>
#include <esp_system.h>      // esp_reset_reason()
#include <LittleFS.h>
#include <WebServer.h>
#include "time.h"
#include <vector>

// ================= WIFI =================
#include "secrets.h"   // SECRET_WIFI_SSID / SECRET_WIFI_PASSWORD — git-ignored, see secrets.example.h
const char* WIFI_SSID     = SECRET_WIFI_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;

// ================= MQTT =================
const char* MQTT_SERVER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char* DEVICE_ID   = "silent-hearth-681";

// ================= TARGET =================
#define TARGET_DEFAULT      22.0f
#define TARGET_MIN          10.0f   // exclusive bounds
#define TARGET_MAX          35.0f

// ================= TPI =================
#define TPI_CYCLE_MS       600000UL  // 10 min control cycle
// Min ON/OFF: 5/3 min halves boiler starts vs 3/2 with the same accuracy in
// simulation; 6+ min ON makes powerful boilers swing ±0.5 °C.
#define TPI_MIN_ON_MS      300000UL  // ON pulse shorter than this → carried over
#define TPI_MIN_OFF_MS     180000UL  // OFF gap shorter than this → stays ON
#define TPI_HORIZON_H         1.5f   // aim to close the error in ~1.5 h
#define TPI_TI_H              4.0f   // integral time constant
#define TPI_I_MAX             0.25f  // integral clamp (± duty)
#define TPI_OVERSHOOT         0.4f   // T >= target + this → boiler off now
#define TPI_FULL_ON_ERR       2.0f   // T <= target − this → 100 % duty
#define TPI_MIN_SAMPLES       100    // samples needed to learn from a cycle
#define TPI_SAVE_EVERY        3      // persist learned model every N cycles (~30 min)

// Learned model defaults and sane bounds
#define HEAT_DEFAULT          2.0f   // °C/h at 100 %
#define LOSS_DEFAULT          0.5f   // °C/h at 0 %
#define HEAT_MIN              0.3f
#define HEAT_MAX              8.0f
#define LOSS_MIN              0.2f   // a heated home always loses some heat; 0 came from sunny days
#define LOSS_MAX              4.0f
#define RLS_LAMBDA            0.97f  // forgetting factor (~33 cycles ≈ 5.5 h memory)
#define RLS_MAX_SLOPE         6.0f   // |°C/h| — larger slopes are outliers (door, sun…)
#define TPI_EXCITE            0.25f  // duty must differ this much from its average to learn H
                                     // (lower values let sensor noise drift H — see simulation)
#define TPI_LOSS_GAIN         0.1f   // gain for loss-only updates under steady load
#define TPI_CARRY_MAX         0.5f   // max duty carried between cycles
#define TPI_KICK_ERR          0.2f   // after a target raise, heat at once if this far below
#define TPI_UNDERSHOOT_KICK   0.3f   // any cycle: this far below target → no skipping the min pulse
#define TPI_GAIN_SLOPE        0.3f   // °C/h rise with the boiler off = sun/cooking, not heat loss
#define TPI_RAD_TAU_MIN       12.0f  // radiator lag: the learner's estimate of radiator heat
                                     // follows the boiler with this time constant (see simulation)
// ================= SYSTEM CONSTANTS =================
// Door / window detection. The DHT22 reacts slowly, so look for a sustained
// fall (regression slope) or a total drop below the recent peak.
#define DHT_BUFFER_SIZE       100      // 5 min of samples (3 s each)
#define DOOR_SLOPE_SAMPLES     40      // slope window: last 2 min
#define DOOR_MIN_SAMPLES       20      // need 1 min of data before deciding
#define DOOR_SLOPE_OPEN     -0.25f     // °C/min — falling faster than this → open
#define DOOR_DROP_OPEN       0.6f      // °C below the 5-min peak → open
#define DOOR_SLOPE_STABLE   -0.05f     // °C/min — "no longer falling"
#define DOOR_CLOSE_DELAY    180000UL   // not falling for this long → closed
#define HEARTBEAT_INTERVAL     15000UL
#define SENSOR_INTERVAL         3000UL
#define LCD_INTERVAL            2000UL

// ================= HISTORY =================
#define HISTORY_FILE        "/history.bin"
#define HIST_TEMP_THRESHOLD  0.1f        // min temp change to trigger a record
#define HIST_MAX_GAP      300000UL        // max time without a record (5 min)
#define HIST_COMPACT_EVERY    100        // compact every N records
#define HTTP_PORT              80

// ================= MQTT BUFFERS =================
// Relay ESP uses the same MQTT buffer size — keep both sketches in sync.
#define MQTT_BUFFER_SIZE     1024
#define TEMP_PAYLOAD_SIZE     900
#define SETTINGS_JSON_SIZE    512

// ================= NTP =================
#define NTP_SERVER1  "pool.ntp.org"
#define NTP_SERVER2  "time.google.com"

// ================= PINS =================
#define DHTPIN      26
#define DHTTYPE     DHT22
#define PIN_BL      27   // backlight (held off)
#define PIN_BATTERY 33
#define PIN_CHARGE  32
#define LCD_CLK     18
#define LCD_DIN     23
#define LCD_DC      19
#define LCD_CE       5
#define LCD_RST     25

// ================= EEPROM =================
#define EEPROM_SIZE          40
#define ADDR_TARGET           0   // float 4B
#define ADDR_MODE             4   // uint8  0=auto 1=manual_on 2=manual_off
#define ADDR_VERSION          5   // uint8
#define ADDR_HEAT_RATE        6   // float 4B — learned H
#define ADDR_LOSS_RATE       10   // float 4B — learned L
#define ADDR_INTEG           14   // float 4B — integral term
#define ADDR_LEARN_N         18   // uint16   — learned cycle count
// Total used: 20 bytes
#define VERSION_VALUE        0xB0  // bump whenever EEPROM layout changes

// ================= LOG FILE =================
#define LOG_FILE        "/log.txt"
#define LOG_MAX_BYTES   12288   // ~12KB — trim oldest lines beyond this
#define LOG_MAX_LINES   200     // returned via HTTP

// ================= MQTT TOPICS =================
#define TOPIC_TEMP      "silent-hearth-681/temp"
#define TOPIC_CMD       "silent-hearth-681/boiler/cmd"
#define TOPIC_HEARTBEAT "silent-hearth-681/heartbeat"
#define TOPIC_DOOR      "silent-hearth-681/door"
#define TOPIC_SETTINGS  "silent-hearth-681/settings"

// ================= HISTORY RECORD =================
// 9 bytes/record → ~5.4 KB/day at 600 records/day
struct __attribute__((packed)) HistRecord {
  uint32_t ts;    // UTC unix seconds
  float    t;     // °C
  uint8_t  flags; // bit0=boiler on, bit1=door open
};

// ================= DIAGNOSTICS =================
// One record per finished or aborted TPI cycle, exported by GET /export.
#define FW_VERSION      "v8"
#define TPI_LOG_FILE    "/tpi.bin"
#define TPI_LOG_MAX     600      // when exceeded, trim to …
#define TPI_LOG_KEEP    432      // … the newest 3 days (144 cycles/day)

struct __attribute__((packed)) CycleRecord {   // 46 bytes
  uint32_t ts;              // UTC, cycle end
  float target, tStart, tEnd;
  float duty;               // requested duty
  float actual;             // actual ON fraction of the cycle
  float carry;              // carry-over after this cycle's rounding
  float slope;              // measured °C/h (NaN if not computed)
  float heat, loss, integ;  // model after this cycle's learning
  uint8_t flags;            // CF_* below
  uint8_t mode;             // 0 auto, 1 manual_on, 2 manual_off
};
#define CF_LEARN_FULL  0x01   // H and L updated
#define CF_LEARN_LOSS  0x02   // steady load: only L updated
#define CF_REJECTED    0x04   // slope outlier
#define CF_NO_LEARN    0x08   // first cycle after boot/abort, or too few samples
#define CF_ABORTED     0x10   // ended early (mode/target change, door)
#define CF_CUT         0x20   // overshoot cut the ON pulse
#define CF_DOOR        0x40   // aborted because a door/window opened
#define CF_GAIN        0x80   // not learned: room warmed with the boiler off (sun etc.)

// ================= OBJECTS =================
DHT              dht(DHTPIN, DHTTYPE);
Adafruit_PCD8544 lcd(LCD_CLK, LCD_DIN, LCD_DC, LCD_CE, LCD_RST);
WiFiClient       wifiClient;
PubSubClient     mqttClient(wifiClient);
WebServer        httpServer(HTTP_PORT);

// ================= MODE =================
enum Mode { MODE_AUTO, MODE_MANUAL_ON, MODE_MANUAL_OFF };
Mode currentMode = MODE_AUTO;

// ================= STATE =================
float temperature = 0, humidity = 0;
bool  boilerOn    = false, doorOpen = false;
float target      = TARGET_DEFAULT;

unsigned long boilerOnAt   = 0;
unsigned long boilerOffAt  = 0;

unsigned long lastSensor    = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastLcd       = 0;

bool wifiOk = false, mqttOk = false;

// NTP & network
bool ntpOk   = false;
char espIP[16] = "";

// ── TPI learned model (persisted) ─────────────────────────
float    heatRate  = HEAT_DEFAULT;   // H, °C/h at 100 %
float    lossRate  = LOSS_DEFAULT;   // L, °C/h at 0 %
float    integ     = 0.0f;           // I, duty
uint16_t learnN    = 0;              // cycles learned so far

// RLS covariance (not persisted — re-seeded moderately on boot)
float rlsP[2][2] = { {1.0f, 0.0f}, {0.0f, 0.25f} };

// ── TPI cycle state ───────────────────────────────────────
bool          cycleActive    = false;
bool          skipLearnNext  = true;   // first cycle after boot/abort has unknown history
bool          cycleLearnable = false;
unsigned long cycleStartMs   = 0;
unsigned long cycleOnMs      = 0;      // planned (and possibly cut) ON time
float         cycleDuty      = 0.0f;   // requested duty (before quantization)
float         prevActualDuty = 0.0f;   // actual duty of the previous cycle
float         dutyCarry      = 0.0f;   // duty owed (or overpaid) by quantization
float         cycleStartTemp = 0.0f;   // for the diagnostics log
bool          cycleCut       = false;  // overshoot guard fired this cycle
bool          tpiKick        = false;  // user just raised the target
float         uAvg           = -1.0f;  // running average of learning duty (<0 = unset)
double        sX = 0, sY = 0, sXX = 0, sXY = 0;  // slope regression sums
uint16_t      sN = 0;
// Radiator heat estimate (0–1): the boiler state through a first-order lag.
// Learning uses its cycle average instead of the raw duty, so heat still
// coming out of the radiators after a burn isn't mistaken for "no heat loss".
float         radHeat = 0.0f;
unsigned long radLastMs = 0;
double        radSum = 0;
uint16_t      radN = 0;
// ──────────────────────────────────────────────────────────

// History tracking
float         histLastT     = NAN;
bool          histLastBoiler = false;
unsigned long histLastMs    = 0;
int           histCount     = 0;

// DHT ring buffer (rapid-drop detection)
struct TempSample { float t; unsigned long ms; };
TempSample dhtBuf[DHT_BUFFER_SIZE];
int dhtIdx = 0, dhtFilled = 0;
unsigned long doorCloseAt = 0;

// ============================================================
// EEPROM
// ============================================================
void eepromSave() {
  EEPROM.writeFloat(ADDR_TARGET,     target);
  EEPROM.writeByte(ADDR_MODE,        (uint8_t)currentMode);
  EEPROM.writeFloat(ADDR_HEAT_RATE,  heatRate);
  EEPROM.writeFloat(ADDR_LOSS_RATE,  lossRate);
  EEPROM.writeFloat(ADDR_INTEG,      integ);
  EEPROM.writeUShort(ADDR_LEARN_N,   learnN);
  EEPROM.writeByte(ADDR_VERSION,     VERSION_VALUE);
  EEPROM.commit();
}

void eepromLoad() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.readByte(ADDR_VERSION) != VERSION_VALUE) {
    Serial.println("[EEPROM] New version — writing defaults");
    eepromSave();
    return;
  }
  float tgt = EEPROM.readFloat(ADDR_TARGET);
  float h   = EEPROM.readFloat(ADDR_HEAT_RATE);
  float l   = EEPROM.readFloat(ADDR_LOSS_RATE);
  float i   = EEPROM.readFloat(ADDR_INTEG);

  target   = (!isnan(tgt) && tgt > TARGET_MIN && tgt < TARGET_MAX) ? tgt : TARGET_DEFAULT;
  heatRate = (!isnan(h) && h >= HEAT_MIN && h <= HEAT_MAX)         ? h   : HEAT_DEFAULT;
  lossRate = (!isnan(l) && l >= LOSS_MIN && l <= LOSS_MAX)         ? l   : LOSS_DEFAULT;
  integ    = (!isnan(i) && fabsf(i) <= TPI_I_MAX)                  ? i   : 0.0f;
  learnN   = EEPROM.readUShort(ADDR_LEARN_N);
  if (learnN == 0xFFFF) learnN = 0;

  uint8_t md = EEPROM.readByte(ADDR_MODE);
  if      (md == 1) currentMode = MODE_MANUAL_ON;
  else if (md == 2) currentMode = MODE_MANUAL_OFF;
  else              currentMode = MODE_AUTO;

  // Fresh model → trust new data quickly; learned model → keep it stable
  if (learnN < 10) { rlsP[0][0] = 4.0f; rlsP[1][1] = 1.0f; }

  Serial.printf("[EEPROM] Target:%.1f  Mode:%d  H:%.2f  L:%.2f  I:%.3f  Learned:%u\n",
    target, (int)currentMode, heatRate, lossRate, integ, learnN);
}

// ============================================================
// NTP
// ============================================================
void ntpSync() {
  Serial.print("[NTP] Syncing...");
  configTime(0, 0, NTP_SERVER1, NTP_SERVER2);
  time_t now_t = 0;
  int tries = 0;
  while (now_t < 1000000000UL && tries++ < 20) {
    delay(500);
    esp_task_wdt_reset();
    time(&now_t);
  }
  ntpOk = (now_t >= 1000000000UL);
  if (ntpOk) Serial.printf(" OK (ts=%lu)\n", (unsigned long)now_t);
  else       Serial.println(" FAILED (continuing offline)");
}

// ============================================================
// WIFI
// ============================================================
void wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return;
  wifiOk = false;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int d = 0;
  while (WiFi.status() != WL_CONNECTED && d++ < 40) {
    delay(500);
    esp_task_wdt_reset();
  }
  wifiOk = (WiFi.status() == WL_CONNECTED);
  if (wifiOk) {
    WiFi.localIP().toString().toCharArray(espIP, sizeof(espIP));
    Serial.printf("[WiFi] Connected: %s\n", espIP);
    if (!ntpOk) ntpSync();
  } else {
    Serial.println("[WiFi] FAILED");
  }
}

// ============================================================
// HISTORY — purge records older than 24h
// ============================================================
void historyCompact() {
  if (!ntpOk) return;
  time_t now_t; time(&now_t);
  uint32_t cutoff = (uint32_t)(now_t > 86400 ? now_t - 86400 : 0);

  File f = LittleFS.open(HISTORY_FILE, "r");
  if (!f) { histCount = 0; return; }

  std::vector<HistRecord> keep;
  keep.reserve(800);
  HistRecord r;
  while (f.readBytes((char*)&r, sizeof(r)) == sizeof(r)) {
    if (r.ts >= cutoff) keep.push_back(r);
  }
  f.close();

  f = LittleFS.open(HISTORY_FILE, "w");
  if (f) {
    for (const auto& k : keep) f.write((const uint8_t*)&k, sizeof(k));
    f.close();
  }
  histCount = (int)keep.size();
  Serial.printf("[HISTORY] Compact done: %d records remain\n", histCount);
}

// ============================================================
// PERSISTENT LOG
// Format per line: "unix_ts|LEVEL|message\n"
// ============================================================
void logWrite(const char* level, const char* msg) {
  Serial.printf("[%s] %s\n", level, msg);

  File f = LittleFS.open(LOG_FILE, "a");
  if (!f) return;

  time_t now_t = 0; time(&now_t);
  char line[220];
  snprintf(line, sizeof(line), "%lu|%s|%s\n", (unsigned long)now_t, level, msg);
  f.print(line);
  f.close();

  // Trim if file exceeds LOG_MAX_BYTES — discard oldest 25 %
  File fc = LittleFS.open(LOG_FILE, "r");
  if (!fc) return;
  if (fc.size() <= LOG_MAX_BYTES) { fc.close(); return; }
  std::vector<String> lines;
  lines.reserve(250);
  while (fc.available()) {
    String l = fc.readStringUntil('\n');
    l.trim();                                // also heals lines saved with "\r\n"
    if (l.length()) lines.push_back(l);
  }
  fc.close();
  size_t keep = lines.size() * 3 / 4;
  File fw = LittleFS.open(LOG_FILE, "w");
  if (fw) {
    for (size_t i = lines.size() - keep; i < lines.size(); i++) {
      fw.print(lines[i]);
      fw.print('\n');      // not println(): its "\r\n" put a raw \r into /logs JSON
    }
    fw.close();
  }
}

// GET /logs — returns last LOG_MAX_LINES entries as JSON array
// Each entry: {"ts":unix, "lvl":"INFO", "msg":"..."}
void httpLogsHandler() {
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.sendHeader("Cache-Control", "no-cache");

  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) { httpServer.send(200, "application/json", "[]"); return; }

  std::vector<String> lines;
  lines.reserve(256);
  while (f.available()) {
    String l = f.readStringUntil('\n');
    if (l.length()) lines.push_back(l);
  }
  f.close();

  size_t start = lines.size() > LOG_MAX_LINES ? lines.size() - LOG_MAX_LINES : 0;

  String json = "[";
  bool first = true;
  for (size_t i = start; i < lines.size(); i++) {
    int p1 = lines[i].indexOf('|');
    int p2 = lines[i].indexOf('|', p1 + 1);
    if (p1 < 0 || p2 < 0) continue;
    String ts  = lines[i].substring(0, p1);
    String lvl = lines[i].substring(p1 + 1, p2);
    String msg = lines[i].substring(p2 + 1);
    ts.trim(); lvl.trim();
    bool tsOk = ts.length() > 0;
    for (size_t k = 0; k < ts.length(); k++) if (!isDigit(ts[k])) { tsOk = false; break; }
    if (!tsOk) ts = "0";                       // must be a valid JSON number
    for (size_t k = 0; k < lvl.length(); k++)  // level goes into a JSON string too
      if (lvl[k] == '"' || lvl[k] == '\\' || (uint8_t)lvl[k] < 0x20) lvl[k] = '?';
    // JSON-escape: backslash, quote, and control chars (e.g. a stray \r from
    // older log files) — any raw control char makes the dashboard's JSON.parse fail
    String esc;
    esc.reserve(msg.length() + 8);
    for (size_t k = 0; k < msg.length(); k++) {
      char c = msg[k];
      if      (c == '"')  esc += "\\\"";
      else if (c == '\\') esc += "\\\\";
      else if ((uint8_t)c < 0x20) { /* drop \r, \t and other control chars */ }
      else esc += c;
    }
    msg = esc;
    if (!first) json += ',';
    json += "{\"ts\":" + ts + ",\"lvl\":\"" + lvl + "\",\"msg\":\"" + msg + "\"}";
    first = false;
  }
  json += "]";

  httpServer.send(200, "application/json", json);
  Serial.printf("[HTTP] /logs sent (%d lines)\n", (int)(lines.size() - start));
}

#define LOG_INFO(msg)  logWrite("INFO",  msg)
#define LOG_WARN(msg)  logWrite("WARN",  msg)
#define LOG_ERROR(msg) logWrite("ERROR", msg)
#define LOG_BOOT(msg)  logWrite("BOOT",  msg)

// ============================================================
// BOOT REASON
// ============================================================
const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "Power-on";
    case ESP_RST_EXT:      return "External reset";
    case ESP_RST_SW:       return "Software reset";
    case ESP_RST_PANIC:    return "Panic / exception";
    case ESP_RST_INT_WDT:  return "Interrupt watchdog";
    case ESP_RST_TASK_WDT: return "Task watchdog";
    case ESP_RST_WDT:      return "Other watchdog";
    case ESP_RST_DEEPSLEEP:return "Wake from deep sleep";
    case ESP_RST_BROWNOUT: return "Brownout (low voltage)";
    case ESP_RST_SDIO:     return "SDIO reset";
    default:               return "Unknown";
  }
}

void historyWrite(float t, bool boiler, bool door, bool forced = false) {
  if (!ntpOk) return;
  unsigned long now = millis();

  bool write = forced;
  if (!write) {
    if (isnan(histLastT))                                 write = true;
    else if (fabsf(t - histLastT) >= HIST_TEMP_THRESHOLD) write = true;
    else if (boiler != histLastBoiler)                    write = true;
    else if (now - histLastMs >= HIST_MAX_GAP)            write = true;
  }
  if (!write) return;

  time_t now_t; time(&now_t);
  if (now_t < 1000000000UL) return;

  HistRecord r;
  r.ts    = (uint32_t)now_t;
  r.t     = t;
  r.flags = (boiler ? 0x01 : 0x00) | (door ? 0x02 : 0x00);

  File f = LittleFS.open(HISTORY_FILE, "a");
  if (!f) { Serial.println("[HISTORY] Write error!"); return; }
  f.write((uint8_t*)&r, sizeof(r));
  f.close();

  histLastT      = t;
  histLastBoiler = boiler;
  histLastMs     = now;
  histCount++;

  if (histCount % HIST_COMPACT_EVERY == 0) historyCompact();
}

// ============================================================
// HTTP — GET /history
// Response: [[unix_sec, degC, 0_or_1], ...]
// ============================================================
void httpHistoryHandler() {
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.sendHeader("Cache-Control", "no-cache");

  File f = LittleFS.open(HISTORY_FILE, "r");
  if (!f) { httpServer.send(200, "application/json", "[]"); return; }

  time_t now_t; time(&now_t);
  uint32_t cutoff = (uint32_t)(now_t > 86400 ? now_t - 86400 : 0);

  size_t estimate = (f.size() / sizeof(HistRecord)) * 24 + 8;
  String json;
  json.reserve(estimate);
  json = "[";

  HistRecord r;
  bool first = true;
  while (f.readBytes((char*)&r, sizeof(r)) == sizeof(r)) {
    if (r.ts < cutoff) continue;
    if (!first) json += ',';
    json += '[';
    json += (unsigned long)r.ts;
    json += ',';
    json += String(r.t, 1);
    json += ',';
    json += (r.flags & 0x01) ? '1' : '0';
    json += ']';
    first = false;
  }
  f.close();
  json += ']';

  httpServer.send(200, "application/json", json);
  Serial.printf("[HTTP] /history sent (%d bytes)\n", json.length());
}

// GET /status — debug info
void httpStatusHandler() {
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  time_t now_t; time(&now_t);
  char buf[260];
  snprintf(buf, sizeof(buf),
    "{\"ip\":\"%s\",\"ts\":%lu,\"ntp\":%s,\"records\":%d,"
    "\"rssi\":%d,\"target\":%.1f,\"duty\":%.2f,"
    "\"heat_rate\":%.2f,\"loss_rate\":%.2f,\"integ\":%.3f,\"learn_n\":%u}",
    espIP, (unsigned long)now_t, ntpOk ? "true" : "false",
    histCount, WiFi.RSSI(), target, cycleDuty,
    heatRate, lossRate, integ, learnN);
  httpServer.send(200, "application/json", buf);
}

// ============================================================
// HTTP — GET /export  →  one CSV file for offline analysis
//   [summary] current settings, learned model, constants
//   [cycles]  one row per TPI cycle (last 3 days)
//   [temperature] history.bin (last 24 h): ts,temp,boiler,door
//   [log]     log.txt events
// Streamed with chunked encoding so it never builds the whole file in RAM.
// ============================================================
static String exportBuf;
static void exportFlush(bool force) {
  if (exportBuf.length() >= 1400 || (force && exportBuf.length())) {
    httpServer.sendContent(exportBuf);
    exportBuf = "";
    esp_task_wdt_reset();
  }
}
static void exportLine(const char* s) { exportBuf += s; exportBuf += '\n'; exportFlush(false); }
static void fmtNum(char* out, size_t n, float v, int prec) {
  if (isnan(v)) out[0] = '\0'; else snprintf(out, n, "%.*f", prec, v);
}

void httpExportHandler() {
  time_t now_t = 0; time(&now_t);
  char fname[64];
  if (now_t >= 1000000000UL) {
    time_t lt = now_t + 3 * 3600;           // Turkey local time for the file name
    struct tm* t = gmtime(&lt);
    snprintf(fname, sizeof(fname), "boiler_report_%04d-%02d-%02d_%02d%02d.csv",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min);
  } else {
    snprintf(fname, sizeof(fname), "boiler_report_uptime%lu.csv", millis() / 1000);
  }
  char cd[96];
  snprintf(cd, sizeof(cd), "attachment; filename=\"%s\"", fname);
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.sendHeader("Cache-Control", "no-cache");
  httpServer.sendHeader("Content-Disposition", cd);
  httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  httpServer.send(200, "text/csv; charset=utf-8", "");
  exportBuf = ""; exportBuf.reserve(1600);

  char line[260];
  const char* modeStr = currentMode == MODE_MANUAL_ON ? "manual_on"
                      : currentMode == MODE_MANUAL_OFF ? "manual_off" : "auto";
  exportLine("[summary]");
  exportLine("key,value");
  snprintf(line, sizeof(line), "firmware,%s", FW_VERSION);                        exportLine(line);
  snprintf(line, sizeof(line), "exported_utc,%lu", (unsigned long)now_t);          exportLine(line);
  snprintf(line, sizeof(line), "uptime_min,%lu", millis() / 60000);                exportLine(line);
  snprintf(line, sizeof(line), "reset_reason,%s", resetReasonStr(esp_reset_reason())); exportLine(line);
  snprintf(line, sizeof(line), "ip,%s", espIP);                                    exportLine(line);
  snprintf(line, sizeof(line), "wifi_rssi,%d", WiFi.RSSI());                       exportLine(line);
  snprintf(line, sizeof(line), "mode,%s", modeStr);                                exportLine(line);
  snprintf(line, sizeof(line), "target,%.1f", target);                             exportLine(line);
  snprintf(line, sizeof(line), "temperature,%.1f", temperature);                   exportLine(line);
  snprintf(line, sizeof(line), "humidity,%.0f", humidity);                         exportLine(line);
  snprintf(line, sizeof(line), "door_open,%d", doorOpen ? 1 : 0);                  exportLine(line);
  snprintf(line, sizeof(line), "heat_rate_c_per_h,%.3f", heatRate);                exportLine(line);
  snprintf(line, sizeof(line), "loss_rate_c_per_h,%.3f", lossRate);                exportLine(line);
  snprintf(line, sizeof(line), "integ,%.4f", integ);                               exportLine(line);
  snprintf(line, sizeof(line), "kp,%.4f", tpiKp());                                exportLine(line);
  snprintf(line, sizeof(line), "learn_n,%u", learnN);                              exportLine(line);
  snprintf(line, sizeof(line), "cycle_min,%lu", TPI_CYCLE_MS / 60000);             exportLine(line);
  snprintf(line, sizeof(line), "min_on_s,%lu", TPI_MIN_ON_MS / 1000);              exportLine(line);
  snprintf(line, sizeof(line), "min_off_s,%lu", TPI_MIN_OFF_MS / 1000);            exportLine(line);
  snprintf(line, sizeof(line), "radiator_tau_min,%.0f", TPI_RAD_TAU_MIN);          exportLine(line);
  snprintf(line, sizeof(line), "radiator_now,%.2f", radHeat);                      exportLine(line);
  snprintf(line, sizeof(line), "overshoot_c,%.2f", TPI_OVERSHOOT);                 exportLine(line);
  snprintf(line, sizeof(line), "excite,%.2f", TPI_EXCITE);                         exportLine(line);
  snprintf(line, sizeof(line), "door_slope_open_c_per_min,%.2f", DOOR_SLOPE_OPEN); exportLine(line);
  snprintf(line, sizeof(line), "door_drop_open_c,%.2f", DOOR_DROP_OPEN);           exportLine(line);
  exportLine("flags_legend,full=H+L learned;loss=only L learned;rejected=slope outlier;nolearn=not used for learning;aborted=ended early;cut=overshoot cut ON pulse;door=aborted by door/window;gain=not learned - room warmed with boiler off");

  // ---- cycles ----
  exportLine("");
  exportLine("[cycles]");
  exportLine("ts_utc,target,t_start,t_end,duty_req,duty_actual,carry,slope_c_per_h,heat_rate,loss_rate,integ,mode,flags");
  File f = LittleFS.open(TPI_LOG_FILE, "r");
  if (f) {
    CycleRecord r;
    while (f.readBytes((char*)&r, sizeof(r)) == sizeof(r)) {
      char sl[16]; fmtNum(sl, sizeof(sl), r.slope, 3);
      char fl[64] = "";
      if (r.flags & CF_LEARN_FULL) strcat(fl, "full ");
      if (r.flags & CF_LEARN_LOSS) strcat(fl, "loss ");
      if (r.flags & CF_REJECTED)   strcat(fl, "rejected ");
      if (r.flags & CF_NO_LEARN)   strcat(fl, "nolearn ");
      if (r.flags & CF_ABORTED)    strcat(fl, "aborted ");
      if (r.flags & CF_CUT)        strcat(fl, "cut ");
      if (r.flags & CF_DOOR)       strcat(fl, "door ");
      if (r.flags & CF_GAIN)       strcat(fl, "gain ");
      size_t fll = strlen(fl); if (fll) fl[fll - 1] = '\0';
      snprintf(line, sizeof(line), "%lu,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%.4f,%u,%s",
               (unsigned long)r.ts, r.target, r.tStart, r.tEnd, r.duty, r.actual, r.carry,
               sl, r.heat, r.loss, r.integ, r.mode, fl);
      exportLine(line);
    }
    f.close();
  }

  // ---- temperature trace ----
  exportLine("");
  exportLine("[temperature]");
  exportLine("ts_utc,temp,boiler,door");
  f = LittleFS.open(HISTORY_FILE, "r");
  if (f) {
    HistRecord h;
    while (f.readBytes((char*)&h, sizeof(h)) == sizeof(h)) {
      snprintf(line, sizeof(line), "%lu,%.1f,%d,%d", (unsigned long)h.ts, h.t,
               (h.flags & 0x01) ? 1 : 0, (h.flags & 0x02) ? 1 : 0);
      exportLine(line);
    }
    f.close();
  }

  // ---- event log ----
  exportLine("");
  exportLine("[log]");
  exportLine("ts_utc,level,message");
  f = LittleFS.open(LOG_FILE, "r");
  if (f) {
    while (f.available()) {
      String l = f.readStringUntil('\n');
      l.trim();
      int p1 = l.indexOf('|'), p2 = l.indexOf('|', p1 + 1);
      if (p1 < 0 || p2 < 0) continue;
      String msg = l.substring(p2 + 1);
      msg.replace("\"", "\"\"");                 // CSV-quote the message
      String row = l.substring(0, p1) + "," + l.substring(p1 + 1, p2) + ",\"" + msg + "\"";
      exportLine(row.c_str());
    }
    f.close();
  }

  exportFlush(true);
  httpServer.sendContent("");   // end of chunked response
  exportBuf = String();         // release the buffer
  Serial.println("[HTTP] /export sent");
}

// GET /  — tiny page on the ESP itself. The https dashboard (GitHub Pages)
// can't start an http download (Chrome blocks it as mixed content), but it
// can open this http page, and a download started from here is allowed.
void httpRootHandler() {
  time_t now_t = 0; time(&now_t);
  char page[1400];
  snprintf(page, sizeof(page),
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Boiler ESP</title><link rel='icon' href='data:,'>"
    "<style>body{font-family:system-ui,sans-serif;background:#0a0e17;color:#e2e8f0;"
    "max-width:420px;margin:0 auto;padding:24px 16px}"
    "a.b{display:block;text-align:center;background:#f97316;color:#fff;text-decoration:none;"
    "padding:14px;border-radius:10px;font-weight:600;margin:16px 0}"
    "a.s{color:#38bdf8}.m{color:#4a6080;font-size:.85rem;line-height:1.5}</style></head><body>"
    "<h2>&#128293; Boiler ESP</h2>"
    "<p class='m'>Firmware %s &middot; IP %s &middot; up %lu min<br>"
    "Target %.1f &deg;C &middot; now %.1f &deg;C &middot; learned cycles %u</p>"
    "<a class='b' href='/export'>&#11015; Download report (CSV)</a>"
    "<p class='m'>If Chrome says the download is insecure, open the downloads list "
    "(Ctrl+J) and choose <b>Keep</b>. The file stays on your device.</p>"
    "<p class='m'>Raw data: <a class='s' href='/status'>/status</a> &middot; "
    "<a class='s' href='/logs'>/logs</a> &middot; <a class='s' href='/history'>/history</a></p>"
    "</body></html>",
    FW_VERSION, espIP, millis() / 60000, target, temperature, learnN);
  httpServer.send(200, "text/html; charset=utf-8", page);
}

void httpServerStart() {
  httpServer.on("/",        HTTP_GET, httpRootHandler);
  httpServer.on("/history", HTTP_GET, httpHistoryHandler);
  httpServer.on("/status",  HTTP_GET, httpStatusHandler);
  httpServer.on("/logs",    HTTP_GET, httpLogsHandler);
  httpServer.on("/export",  HTTP_GET, httpExportHandler);
  httpServer.begin();
  Serial.printf("[HTTP] Started → http://%s/  (history, logs, status, export)\n", espIP);
}

// ============================================================
// DOOR DETECTION (sustained temperature fall)
//   open  : 2-min regression slope <= DOOR_SLOPE_OPEN
//           OR temperature >= DOOR_DROP_OPEN below the 5-min peak
//   closed: slope > DOOR_SLOPE_STABLE continuously for DOOR_CLOSE_DELAY
// ============================================================
float doorSlope = 0.0f;   // last computed slope, °C/min (for logs/debug)

// i = 0 → newest sample, i = 1 → one before, …
static inline const TempSample& dhtAgo(int i) {
  return dhtBuf[(dhtIdx - 1 - i + 2 * DHT_BUFFER_SIZE) % DHT_BUFFER_SIZE];
}

void doorDetect() {
  if (dhtFilled < DOOR_MIN_SAMPLES) return;

  // Least-squares slope over the last DOOR_SLOPE_SAMPLES readings
  int n = min(dhtFilled, DOOR_SLOPE_SAMPLES);
  unsigned long ms0 = dhtAgo(n - 1).ms;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < n; i++) {
    const TempSample& s = dhtAgo(i);
    double x = (s.ms - ms0) / 60000.0;  // minutes
    sx += x; sy += s.t; sxx += x * x; sxy += x * s.t;
  }
  double den = n * sxx - sx * sx;
  if (den < 1e-9) return;
  doorSlope = (float)((n * sxy - sx * sy) / den);

  // Peak over the whole buffer (up to 5 min)
  float peak = -100.0f;
  for (int i = 0; i < dhtFilled; i++) peak = max(peak, dhtAgo(i).t);
  float cur  = dhtAgo(0).t;
  float drop = peak - cur;

  unsigned long now = millis();
  if (!doorOpen) {
    if (doorSlope <= DOOR_SLOPE_OPEN || drop >= DOOR_DROP_OPEN) {
      doorOpen = true; doorCloseAt = 0;
      mqttClient.publish(TOPIC_DOOR, "1", true);
      char msg[96];
      snprintf(msg, sizeof(msg), "Door/window OPEN — slope %.2f °C/min, %.1f °C below 5-min peak",
               doorSlope, drop);
      LOG_INFO(msg);
    }
  } else {
    // While open, only the slope decides; the old peak would keep it "open" for minutes
    if (doorSlope > DOOR_SLOPE_STABLE) {
      if (!doorCloseAt) doorCloseAt = now;
      else if (now - doorCloseAt >= DOOR_CLOSE_DELAY) {
        doorOpen = false; doorCloseAt = 0;
        dhtFilled = 0;   // forget the pre-opening peak, or it would re-trigger at once
        mqttClient.publish(TOPIC_DOOR, "0", true);
        LOG_INFO("Door/window closed — temperature stable");
      }
    } else {
      doorCloseAt = 0;
    }
  }
}

// ============================================================
// TPI — self-learning time-proportional control
// ============================================================
float tpiKp() { return 1.0f / (heatRate * TPI_HORIZON_H); }

// Duty the model asks for at the current temperature (0..1)
float tpiComputeDuty() {
  float err = target - temperature;
  if (err >= TPI_FULL_ON_ERR)        return 1.0f;
  if (err <= -TPI_OVERSHOOT)         return 0.0f;
  float base = lossRate / heatRate;          // duty that holds temperature steady
  float duty = base + tpiKp() * err + integ;
  return constrain(duty, 0.0f, 1.0f);
}

// ── Per-cycle diagnostics log (/tpi.bin, exported by GET /export) ─────
void cycleLog(float actualDuty, float slope, uint8_t flags) {
  CycleRecord r;
  time_t now_t = 0; time(&now_t);
  r.ts     = (uint32_t)(now_t >= 1000000000UL ? now_t : 0);
  r.target = target;     r.tStart = cycleStartTemp; r.tEnd = temperature;
  r.duty   = cycleDuty;  r.actual = actualDuty;     r.carry = dutyCarry;
  r.slope  = slope;      r.heat   = heatRate;       r.loss  = lossRate;
  r.integ  = integ;      r.flags  = flags;          r.mode  = (uint8_t)currentMode;

  File f = LittleFS.open(TPI_LOG_FILE, "a");
  if (!f) return;
  f.write((const uint8_t*)&r, sizeof(r));
  size_t n = f.size() / sizeof(CycleRecord);
  f.close();

  if (n > TPI_LOG_MAX) {   // keep the newest TPI_LOG_KEEP records
    File rf = LittleFS.open(TPI_LOG_FILE, "r");
    if (!rf) return;
    std::vector<CycleRecord> keep;
    keep.reserve(TPI_LOG_KEEP);
    rf.seek((n - TPI_LOG_KEEP) * sizeof(CycleRecord));
    CycleRecord c;
    while (rf.readBytes((char*)&c, sizeof(c)) == sizeof(c)) keep.push_back(c);
    rf.close();
    File wf = LittleFS.open(TPI_LOG_FILE, "w");
    if (wf) { for (auto& k : keep) wf.write((const uint8_t*)&k, sizeof(k)); wf.close(); }
  }
}

// Abandon the running cycle (mode change, door, target change).
// The next cycle starts fresh and is not used for learning.
void tpiAbortWith(uint8_t reason) {
  if (cycleActive) {
    Serial.println("[TPI] Cycle aborted");
    unsigned long el = millis() - cycleStartMs;
    float actual = (float)min(el, cycleOnMs) / (float)TPI_CYCLE_MS;
    cycleLog(actual, NAN, CF_ABORTED | reason | (cycleCut ? CF_CUT : 0));
  }
  cycleActive   = false;
  skipLearnNext = true;
  dutyCarry     = 0.0f;
}
void tpiAbort() { tpiAbortWith(0); }

// Recursive least squares on  slope = H·u − L
void tpiLearn(float u, float slope) {
  float phi[2] = { u, -1.0f };
  float Pphi[2] = {
    rlsP[0][0] * phi[0] + rlsP[0][1] * phi[1],
    rlsP[1][0] * phi[0] + rlsP[1][1] * phi[1]
  };
  float denom = RLS_LAMBDA + phi[0] * Pphi[0] + phi[1] * Pphi[1];
  float k[2]  = { Pphi[0] / denom, Pphi[1] / denom };
  float predicted = heatRate * u - lossRate;
  float e = slope - predicted;

  heatRate = constrain(heatRate + k[0] * e, HEAT_MIN, HEAT_MAX);
  lossRate = constrain(lossRate + k[1] * e, LOSS_MIN, LOSS_MAX);

  // P = (P − k·φᵀ·P) / λ
  float newP[2][2];
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      newP[i][j] = (rlsP[i][j] - k[i] * Pphi[j]) / RLS_LAMBDA;
  // Keep symmetric and bounded (prevents wind-up when u barely varies)
  float off = 0.5f * (newP[0][1] + newP[1][0]);
  newP[0][1] = newP[1][0] = off;
  float tr = newP[0][0] + newP[1][1];
  if (tr > 10.0f) {
    float s = 10.0f / tr;
    for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) newP[i][j] *= s;
  }
  memcpy(rlsP, newP, sizeof(rlsP));

  learnN++;
  char msg[120];
  snprintf(msg, sizeof(msg),
    "TPI learn #%u: u=%.2f slope=%.2f°C/h err=%.2f → H=%.2f L=%.2f",
    learnN, u, slope, e, heatRate, lossRate);
  LOG_INFO(msg);
  if (learnN % TPI_SAVE_EVERY == 0) eepromSave();
}

// Steady load: the duty barely changes, so H and L can't be told apart
// (only their ratio is observable). Keep H and the RLS covariance as they
// are and only nudge L, so the estimate doesn't drift along that line.
void tpiLearnLossOnly(float u, float slope) {
  float e = slope - (heatRate * u - lossRate);
  lossRate = constrain(lossRate - TPI_LOSS_GAIN * e, LOSS_MIN, LOSS_MAX);
  learnN++;
  Serial.printf("[TPI] learn #%u (steady): u=%.2f slope=%.2f err=%.2f → L=%.2f\n",
    learnN, u, slope, e, lossRate);
  if (learnN % TPI_SAVE_EVERY == 0) eepromSave();
}

void tpiEndCycle() {
  float actualDuty = (float)min(cycleOnMs, TPI_CYCLE_MS) / (float)TPI_CYCLE_MS;
  float slope = NAN;
  uint8_t flags = cycleCut ? CF_CUT : 0;

  if (cycleLearnable && sN >= TPI_MIN_SAMPLES) {
    double den = sN * sXX - sX * sX;
    if (den > 1e-9) {
      slope = (float)((sN * sXY - sX * sY) / den);   // °C/h
      if (fabsf(slope) <= RLS_MAX_SLOPE) {
        // Radiators lag: learn against the average radiator heat estimate of
        // this cycle, not the boiler's ON time. The old 0.6/0.4 duty weighting
        // was too short for this flat's radiators and dragged H and L down.
        float uEff = radN ? (float)(radSum / radN) : 0.6f * actualDuty + 0.4f * prevActualDuty;
        if (actualDuty <= 0.0f && slope > TPI_GAIN_SLOPE + heatRate * uEff) {
          // The boiler didn't run this cycle, yet the room warmed more than the
          // radiators' leftover heat explains: free heat (sun, cooking, people).
          // Learning from it drove L to 0 on 7 Nov, and on 8 Nov morning sun
          // right after a burn (radiator estimate still 0.15-0.36) dragged H
          // from 2.0 to 0.47 — so skip these cycles.
          Serial.printf("[TPI] +%.2f°C/h with boiler off — free heat, not learned\n", slope);
          flags |= CF_GAIN;
        } else {
          bool excited = (uAvg < 0.0f) || fabsf(uEff - uAvg) >= TPI_EXCITE;
          uAvg = (uAvg < 0.0f) ? uEff : 0.8f * uAvg + 0.2f * uEff;
          if (excited) { tpiLearn(uEff, slope);         flags |= CF_LEARN_FULL; }
          else         { tpiLearnLossOnly(uEff, slope); flags |= CF_LEARN_LOSS; }
        }
      } else {
        Serial.printf("[TPI] Slope %.2f°C/h rejected (outlier)\n", slope);
        flags |= CF_REJECTED;
      }
    } else flags |= CF_NO_LEARN;
  } else flags |= CF_NO_LEARN;

  cycleLog(actualDuty, slope, flags);
  prevActualDuty = actualDuty;
  cycleActive = false;
}

void tpiStartCycle(unsigned long now) {
  float err = target - temperature;

  // Slow integral correction with anti-windup
  float duty = tpiComputeDuty();
  bool saturatedHigh = (duty >= 1.0f && err > 0);
  bool saturatedLow  = (duty <= 0.0f && err < 0);
  if (!skipLearnNext && !saturatedHigh && !saturatedLow && fabsf(err) < TPI_FULL_ON_ERR) {
    float cycleH = TPI_CYCLE_MS / 3600000.0f;
    integ = constrain(integ + (tpiKp() / TPI_TI_H) * err * cycleH, -TPI_I_MAX, TPI_I_MAX);
    duty  = tpiComputeDuty();
  }

  // Carry-over: duty lost to the min-ON / min-OFF rounding is owed to later
  // cycles, so a 3 % demand becomes a 3-min pulse every ~100 min instead of
  // never firing. Nothing is owed while the room is warm enough.
  if (duty <= 0.0f) dutyCarry = 0.0f;
  float want = constrain(duty + dutyCarry, 0.0f, 1.0f);
  unsigned long onMs = (unsigned long)(want * TPI_CYCLE_MS);
  if (onMs < TPI_MIN_ON_MS) onMs = 0;                                 // too short → owe it
  else if (TPI_CYCLE_MS - onMs < TPI_MIN_OFF_MS) {                    // OFF gap too short:
    // If this cycle's own demand is low and the excess is only carried-over
    // debt, round DOWN to 7 min and keep owing the rest. Rounding it up to a
    // full 10-min burn overheated the room ~15 min later (radiator lag,
    // 7 Nov 19:31: 22.0 → 22.2 °C). Real high demand (≥ 50 %) or a request
    // that is nearly full still runs the whole cycle, so weak boilers keep
    // their long runs. Simulation: typical house 7.6 → 6.6 min pulses,
    // 0 min/day above target+0.2, ~12 % more starts.
    bool nearlyFull = onMs * 2 >= 2 * TPI_CYCLE_MS - TPI_MIN_OFF_MS;  // ≥ 8.5 min
    onMs = (duty >= 0.5f || nearlyFull) ? TPI_CYCLE_MS : TPI_CYCLE_MS - TPI_MIN_OFF_MS;
  }
  // The user just raised the target and the room is clearly below it: don't
  // make them wait a whole cycle for a demand below the 5-min minimum — fire
  // the minimum pulse now; the overpaid duty is taken back via the carry.
  if (tpiKick) {
    if (onMs == 0 && err >= TPI_KICK_ERR) {
      onMs = TPI_MIN_ON_MS;
      Serial.println("[TPI] Target raised — minimum pulse now");
    }
    tpiKick = false;
  }
  // Clearly below target (e.g. after airing the room): don't keep waiting for
  // the demand to reach the 5-min minimum. Rarely fires in normal operation
  // (simulation: 150 → 152 starts in 4 days).
  if (onMs == 0 && duty > 0.0f && err >= TPI_UNDERSHOOT_KICK) {
    onMs = TPI_MIN_ON_MS;
    Serial.printf("[TPI] %.1f°C below target — minimum pulse now\n", err);
  }
  dutyCarry = constrain(duty + dutyCarry - (float)onMs / TPI_CYCLE_MS,
                        -TPI_CARRY_MAX, TPI_CARRY_MAX);

  cycleActive    = true;
  cycleLearnable = !skipLearnNext;
  skipLearnNext  = false;
  cycleStartMs   = now;
  cycleOnMs      = onMs;
  cycleDuty      = duty;
  cycleStartTemp = temperature;
  cycleCut       = false;
  sX = sY = sXX = sXY = 0; sN = 0; radSum = 0; radN = 0;

  Serial.printf("[TPI] New cycle: T=%.1f target=%.1f duty=%.2f (base=%.2f Kp=%.2f I=%.3f carry=%.2f) on=%lus%s\n",
    temperature, target, duty, lossRate / heatRate, tpiKp(), integ, dutyCarry, onMs / 1000,
    cycleLearnable ? "" : " [no-learn]");
}

// Called every sensor tick in AUTO mode with the door closed
bool tpiControl() {
  unsigned long now = millis();
  if (cycleActive && now - cycleStartMs >= TPI_CYCLE_MS) tpiEndCycle();
  if (!cycleActive) tpiStartCycle(now);

  // Slope regression sample (x in hours since cycle start)
  double x = (now - cycleStartMs) / 3600000.0;
  sX += x; sY += temperature; sXX += x * x; sXY += x * temperature; sN++;
  radSum += radHeat; radN++;

  bool inOnWindow = (now - cycleStartMs) < cycleOnMs;

  // Overshoot guard: cut the ON pulse short (after its minimum length)
  if (inOnWindow && temperature >= target + TPI_OVERSHOOT &&
      (now - cycleStartMs) >= TPI_MIN_ON_MS) {
    cycleOnMs = now - cycleStartMs;
    cycleCut  = true;
    inOnWindow = false;
    Serial.printf("[TPI] Overshoot %.1f >= %.1f — ON pulse cut\n", temperature, target + TPI_OVERSHOOT);
  }
  return inOnWindow;
}

// Why the boiler is (not) running right now — shown on the dashboard
const char* boilerReason() {
  if (doorOpen)                       return "door";
  if (currentMode == MODE_MANUAL_ON)  return "manual_on";
  if (currentMode == MODE_MANUAL_OFF) return "manual_off";
  if (!cycleActive)                   return "starting";
  if (millis() - cycleStartMs < cycleOnMs) return "heating";
  if (cycleCut)                       return "overshoot";  // ON pulse cut above target
  if (cycleOnMs == 0)                 return cycleDuty <= 0.0f ? "warm"    // no heat needed
                                                               : "short";  // < min ON, carried over
  return "cycle_off";                                      // ON part of this cycle done
}

unsigned long tpiCycleLeftS() {
  if (!cycleActive) return 0;
  unsigned long el = millis() - cycleStartMs;
  return el >= TPI_CYCLE_MS ? 0 : (TPI_CYCLE_MS - el) / 1000;
}

bool boilerDecision() {
  if (doorOpen) { tpiAbortWith(CF_DOOR); return false; }
  switch (currentMode) {
    case MODE_MANUAL_ON:  tpiAbort(); return true;
    case MODE_MANUAL_OFF: tpiAbort(); return false;
    default:              return tpiControl();
  }
}

// ============================================================
// LCD
// ============================================================
void lcdUpdate() {
  lcd.clearDisplay();
  lcd.setTextColor(BLACK);
  lcd.setTextSize(1);

  lcd.setCursor(0, 0);
  if      (!wifiOk) lcd.print("NoWiFi");
  else if (!mqttOk) lcd.print("NoMQTT");
  else if (!ntpOk)  lcd.print("NoNTP ");
  else              lcd.print("OK    ");
  if (doorOpen)   { lcd.setCursor(54, 0); lcd.print("DOOR!"); }

  lcd.setTextSize(2);
  lcd.setCursor(0, 9);
  char tBuf[7];
  dtostrf(temperature > 0 ? temperature : 0.0f, 4, 1, tBuf);
  lcd.print(temperature > 0 ? tBuf : "--.-");
  lcd.setTextSize(1);
  lcd.setCursor(68,  9); lcd.print("C");
  lcd.setCursor(68, 17); lcd.print("now");

  // Target + current cycle duty
  lcd.setCursor(0, 26);
  char hBuf[6]; dtostrf(target, 4, 1, hBuf); lcd.print(hBuf);
  lcd.setCursor(36, 26);
  if (currentMode == MODE_AUTO && cycleActive) {
    char dBuf[10]; snprintf(dBuf, sizeof(dBuf), "D%3d%%", (int)(cycleDuty * 100 + 0.5f));
    lcd.print(dBuf);
  }

  lcd.setCursor(0, 35);
  switch (currentMode) {
    case MODE_AUTO:       lcd.print("TPI "); break;
    case MODE_MANUAL_ON:  lcd.print("MAN+"); break;
    case MODE_MANUAL_OFF: lcd.print("OFF "); break;
  }
  lcd.setCursor(30, 35);
  if (boilerOn) {
    int min = boilerOnAt ? (int)((millis() - boilerOnAt) / 60000) : 0;
    char buf[14]; snprintf(buf, sizeof(buf), "ON %dmin", min); lcd.print(buf);
  } else {
    lcd.print("Standby");
  }
  lcd.display();
}

// ============================================================
// SENSOR READ & PUBLISH
// ============================================================
void sensorUpdate() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) { Serial.println("[DHT] Read error!"); return; }

  temperature = t; humidity = h;
  dhtBuf[dhtIdx] = {t, millis()};
  dhtIdx = (dhtIdx + 1) % DHT_BUFFER_SIZE;
  if (dhtFilled < DHT_BUFFER_SIZE) dhtFilled++;

  doorDetect();

  // Radiator estimate: move toward the boiler state of the last interval
  unsigned long nowMs = millis();
  if (radLastMs) {
    float dtMin = (nowMs - radLastMs) / 60000.0f;
    radHeat += ((boilerOn ? 1.0f : 0.0f) - radHeat) * (1.0f - expf(-dtMin / TPI_RAD_TAU_MIN));
  }
  radLastMs = nowMs;

  bool newBoiler = boilerDecision();
  if (newBoiler != boilerOn) {
    Serial.printf("[BOILER] %s\n", newBoiler ? "ON" : "OFF");
    if (newBoiler) boilerOnAt = millis(); else boilerOffAt = millis();
  }
  boilerOn = newBoiler;

  historyWrite(t, boilerOn, doorOpen);

  int batPct = constrain(map(analogRead(PIN_BATTERY), 1500, 2600, 0, 100), 0, 100);
  const char* modeStr = (currentMode == MODE_MANUAL_ON)  ? "manual_on"
                      : (currentMode == MODE_MANUAL_OFF) ? "manual_off"
                      :                                    "auto";
  float onMin  = (boilerOn  && boilerOnAt)  ? (millis() - boilerOnAt)  / 60000.0f : 0.0f;
  float offMin = (!boilerOn && boilerOffAt) ? (millis() - boilerOffAt) / 60000.0f : 0.0f;
  bool  tpiRun = (currentMode == MODE_AUTO && cycleActive);

  char buf[TEMP_PAYLOAD_SIZE];
  int n = snprintf(buf, sizeof(buf),
    "{"
    "\"t\":%.1f,\"h\":%.0f,"
    "\"boiler\":%s,\"door\":%s,"
    "\"bat\":%d,\"mode\":\"%s\","
    "\"target\":%.1f,"
    "\"duty\":%.2f,\"cycle_left_s\":%lu,"
    "\"reason\":\"%s\",\"cycle_on_s\":%lu,\"carry\":%.2f,"
    "\"min_on_s\":%lu,\"min_off_s\":%lu,"
    "\"heat_rate\":%.2f,\"loss_rate\":%.2f,\"rad\":%.2f,"
    "\"kp\":%.3f,\"integ\":%.3f,\"learn_n\":%u,"
    "\"on_min\":%.1f,\"off_min\":%.1f,"
    "\"ip\":\"%s\""
    "}",
    temperature, humidity,
    boilerOn ? "true" : "false",
    doorOpen ? "true" : "false",
    batPct, modeStr,
    target,
    tpiRun ? cycleDuty : 0.0f, tpiRun ? tpiCycleLeftS() : 0UL,
    boilerReason(), tpiRun ? cycleOnMs / 1000 : 0UL, dutyCarry,
    TPI_MIN_ON_MS / 1000, TPI_MIN_OFF_MS / 1000,
    heatRate, lossRate, radHeat,
    tpiKp(), integ, learnN,
    onMin, offMin,
    espIP
  );
  // A truncated payload is invalid JSON — the relay would ignore it and
  // fall back to OFF on heartbeat timeout. Never publish it.
  if (n < 0 || n >= (int)sizeof(buf)) {
    Serial.printf("[MQTT] /temp payload truncated (%d bytes) — not published\n", n);
  } else if (!mqttClient.publish(TOPIC_TEMP, buf, true)) {
    Serial.printf("[MQTT] /temp publish failed (%d bytes)\n", n);
  }

  Serial.printf("[SENSOR] T:%.1f target:%.1f duty:%.2f Boiler:%s Bat:%d%% Records:%d\n",
    temperature, target, cycleDuty, boilerOn ? "ON" : "OFF", batPct, histCount);
}

// ============================================================
// HEARTBEAT
// ============================================================
void sendHeartbeat() {
  mqttClient.publish(TOPIC_HEARTBEAT, "{\"alive\":true}", false);
}

// ============================================================
// MQTT CALLBACK
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  String tp = String(topic);

  if (tp == String(TOPIC_CMD)) {
    Mode newMode = currentMode;
    if      (msg == "manual_on")  newMode = MODE_MANUAL_ON;
    else if (msg == "manual_off") newMode = MODE_MANUAL_OFF;
    else if (msg == "auto")       newMode = MODE_AUTO;
    if (newMode != currentMode) {
      currentMode = newMode;
      tpiAbort();
      eepromSave();
      String m = "Mode changed to " + msg;
      LOG_INFO(m.c_str());
    }
    Serial.println("[CMD] Mode → " + msg);
  }

  if (tp == String(TOPIC_SETTINGS)) {
    StaticJsonDocument<SETTINGS_JSON_SIZE> doc;
    if (!deserializeJson(doc, msg)) {
      bool changed = false;
      if (doc.containsKey("target")) {
        float v = doc["target"];
        if (v > TARGET_MIN && v < TARGET_MAX && fabsf(v - target) >= 0.05f) {
          tpiKick = (v > target);
          target = v; changed = true;
          tpiAbort();   // start a new cycle immediately with the new target
        }
      }
      if (doc.containsKey("tpi_reset") && (bool)doc["tpi_reset"]) {
        heatRate = HEAT_DEFAULT; lossRate = LOSS_DEFAULT; integ = 0; learnN = 0; uAvg = -1.0f;
        rlsP[0][0] = 4.0f; rlsP[0][1] = rlsP[1][0] = 0.0f; rlsP[1][1] = 1.0f;
        tpiAbort();
        changed = true;
        LOG_WARN("TPI learning reset to defaults");
      }
      if (changed) {
        eepromSave();
        historyWrite(temperature, boilerOn, doorOpen, true);
        char logMsg[80];
        snprintf(logMsg, sizeof(logMsg), "Settings saved — Target:%.1f", target);
        LOG_INFO(logMsg);
      }
    }
  }
}

// ============================================================
// MQTT CONNECT
// ============================================================
unsigned long lastMqttTry = 0;

void mqttConnect() {
  if (mqttClient.connected()) return;
  mqttOk = false;
  // Retry at most every 5 s; each attempt can block for DNS + TCP + CONNACK
  if (lastMqttTry && millis() - lastMqttTry < 5000) return;
  lastMqttTry = millis();
  if (WiFi.status() != WL_CONNECTED) return;
  esp_task_wdt_reset();
  String cid = "esp32-temp-" + String(DEVICE_ID);
  if (mqttClient.connect(cid.c_str())) {
    mqttOk = true;
    mqttClient.subscribe(TOPIC_CMD);
    mqttClient.subscribe(TOPIC_SETTINGS);
    Serial.println("[MQTT] Connected");
  } else {
    Serial.println("[MQTT] Error: " + String(mqttClient.state()));
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  esp_reset_reason_t resetReason = esp_reset_reason();

  Serial.println("\n[BOOT] Boiler Temp ESP32 v8 — self-learning TPI");

  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = 15000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic  = true
  };
  // Core 3.x already starts the task watchdog at boot with a 5 s timeout, so
  // init() fails with ESP_ERR_INVALID_STATE and the 15 s never applied —
  // anything blocking > 5 s (e.g. a slow MQTT connect) reset the ESP.
  if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(NULL);

  // Pins
  pinMode(PIN_BL,     OUTPUT); digitalWrite(PIN_BL, LOW);       // backlight off
  pinMode(PIN_CHARGE, OUTPUT); digitalWrite(PIN_CHARGE, LOW);

  // LCD splash
  lcd.begin(); lcd.setContrast(55);
  lcd.clearDisplay(); lcd.setTextSize(1); lcd.setTextColor(BLACK);
  lcd.setCursor(4, 10); lcd.print("Boiler v8");
  lcd.setCursor(0, 22); lcd.print("Learning TPI");
  lcd.display(); delay(1200);

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("[LittleFS] Mount error, formatting...");
    LittleFS.format(); LittleFS.begin();
  }
  Serial.println("[LittleFS] OK");

  // EEPROM & sensor
  eepromLoad();
  dht.begin();

  {
    char bootMsg[140];
    const char* reason = resetReasonStr(resetReason);
    const char* modeStr = (currentMode == MODE_MANUAL_ON)  ? "manual_on"
                        : (currentMode == MODE_MANUAL_OFF) ? "manual_off"
                        :                                    "auto";
    snprintf(bootMsg, sizeof(bootMsg),
      "Boot — reason: %s | mode: %s | target: %.1f | H=%.2f L=%.2f n=%u",
      reason, modeStr, target, heatRate, lossRate, learnN);
    LOG_BOOT(bootMsg);
    if (resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_INT_WDT ||
        resetReason == ESP_RST_WDT      || resetReason == ESP_RST_PANIC) {
      char warnMsg[80];
      snprintf(warnMsg, sizeof(warnMsg), "Unexpected reset: %s", reason);
      LOG_WARN(warnMsg);
    }
  }

  // WiFi (NTP called inside)
  lcd.clearDisplay(); lcd.setCursor(0, 0); lcd.print("WiFi + NTP..."); lcd.display();
  wifiConnect();

  historyCompact();

  // MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(4);   // CONNACK wait (default 15 s) — must stay under the watchdog
  if (!mqttClient.setBufferSize(MQTT_BUFFER_SIZE))
    Serial.println("[MQTT] WARNING: could not allocate MQTT buffer");
  mqttConnect();

  // HTTP
  if (wifiOk) httpServerStart();

  boilerOffAt = millis();
  lcdUpdate();

  Serial.printf("[BOOT] Ready — IP:%s  NTP:%s  Records:%d\n",
    espIP, ntpOk ? "OK" : "NO", histCount);
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  esp_task_wdt_reset();
  unsigned long now = millis();

  // ---- WiFi / MQTT ----
  if (WiFi.status() != WL_CONNECTED) {
    wifiOk = false;
    wifiConnect();
    if (wifiOk) httpServerStart();
  }
  if (!mqttClient.connected()) { mqttOk = false; mqttConnect(); }
  else                         { mqttOk = true;  mqttClient.loop(); }

  // ---- HTTP ----
  httpServer.handleClient();

  // ---- Periodic tasks ----
  if (now - lastSensor    > SENSOR_INTERVAL)    { lastSensor    = now; sensorUpdate();   }
  if (now - lastHeartbeat > HEARTBEAT_INTERVAL) { lastHeartbeat = now; sendHeartbeat();  }
  if (now - lastLcd       > LCD_INTERVAL)       { lastLcd       = now; lcdUpdate();      }

  delay(50);
}
