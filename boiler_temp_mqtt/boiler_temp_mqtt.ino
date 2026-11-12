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
#include <HTTPClient.h>
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
#define TPI_TI_H              2.0f   // integral time constant (4 h left the room ~0.1 °C off
                                     // for hours; 1 h oscillated with slow radiators — simulation)
#define TPI_I_MAX             0.15f  // integral clamp (± duty); 0.25 let a night's windup overheat the next day
#define TPI_BUMP_LEARN_N      72     // bumpless integral transfer only for the first ~12 h after a reset
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

// Outdoor-temperature model. With a recent outdoor reading the heat loss is
// modelled as L = kLoss·(T − Tout) instead of a flat L, so the base duty follows
// the weather (cold fronts, night/day) before the room drifts. Simulation: time
// at exactly target 43 → 54 % in the flat-like scenario, cold front ≥0.2 °C off
// 9 → 4 %, still better with the weather service 2 °C off.
#define K_LOSS_DEFAULT        (LOSS_DEFAULT / 15.0f)  // °C/h per °C indoor−outdoor
#define K_LOSS_MIN            0.005f
#define K_LOSS_MAX            0.4f
#define WEATHER_INTERVAL      900000UL     // fetch every 15 min (Open-Meteo updates every 15 min)
#define WEATHER_RETRY         120000UL     // after a failure
#define WEATHER_MAX_AGE      10800000UL    // older than 3 h → fall back to the flat L
#define WEATHER_HTTP_TIMEOUT   5000        // ms — must stay well under the 15 s watchdog
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
#define ADDR_K_LOSS          20   // float 4B — learned loss per °C indoor−outdoor
// Total used: 24 bytes. ADDR_K_LOSS was appended without a version bump: those
// bytes were never written before (0xFF = NaN) and are range-checked on load,
// so target, mode and the learned H/L survive the update.
#define VERSION_VALUE        0xB0  // bump whenever existing EEPROM fields move or change meaning

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
// History / log over MQTT: the https dashboard can't fetch http from the ESP
// (mixed content), so it publishes {"id":"…"} on …/get and the ESP answers
// with chunks on …/data: {"id","seq","d":[…],"end":bool}.
#define TOPIC_HIST_GET  "silent-hearth-681/history/get"
#define TOPIC_HIST_DATA "silent-hearth-681/history/data"
#define TOPIC_LOG_GET   "silent-hearth-681/log/get"
#define TOPIC_LOG_DATA  "silent-hearth-681/log/data"
#define DUMP_CHUNK_FILL      720    // start a new chunk once the payload reaches this
#define DUMP_INTERVAL_MS     150    // between chunks (be gentle with the public broker)
#define DUMP_MIN_GAP_MS     5000    // ignore a new request this soon after the last one

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
#define TPI_LOG_FILE    "/tpi2.bin"   // v2 record (adds tout); the old /tpi.bin is deleted at boot
#define TPI_LOG_OLD     "/tpi.bin"
#define TPI_LOG_MAX     600      // when exceeded, trim to …
#define TPI_LOG_KEEP    432      // … the newest 3 days (144 cycles/day)

struct __attribute__((packed)) CycleRecord {   // 50 bytes
  uint32_t ts;              // UTC, cycle end
  float target, tStart, tEnd;
  float duty;               // requested duty
  float actual;             // actual ON fraction of the cycle
  float carry;              // carry-over after this cycle's rounding
  float slope;              // measured °C/h (NaN if not computed)
  float heat, loss, integ;  // model after this cycle's learning
  uint8_t flags;            // CF_* below
  uint8_t mode;             // 0 auto, 1 manual_on, 2 manual_off
  float tout;               // outdoor °C from the weather service (NaN if unknown)
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

float    kLoss     = K_LOSS_DEFAULT; // °C/h per °C indoor−outdoor (weather model)

// RLS covariance (not persisted — re-seeded moderately on boot)
float rlsP[2][2]  = { {1.0f, 0.0f}, {0.0f, 0.25f} };
float rlsPw[2][2] = { {4.0f, 0.0f}, {0.0f, 0.01f} };   // weather model: H and kLoss

// ── Outdoor temperature (Open-Meteo, location from IP or secrets.h) ──
float         outdoorT       = NAN;
unsigned long outdoorAtMs    = 0;
unsigned long lastWeatherTry = 0;
float         geoLat = NAN, geoLon = NAN;

// ── History / log dump over MQTT (one chunk per DUMP_INTERVAL_MS) ──
struct Dump {
  bool          active = false;
  char          id[17] = "";
  uint32_t      pos    = 0;     // byte offset in the file
  uint16_t      seq    = 0;
  uint32_t      cutoff = 0;     // history: oldest timestamp to send
  unsigned long lastMs = 0;     // last chunk sent / request accepted
};
Dump histDump, logDump;

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

// (Functions start here: the Arduino builder puts its generated prototypes
//  before the first function, so every type must be declared above this.)
bool weatherValid() { return !isnan(outdoorT) && millis() - outdoorAtMs < WEATHER_MAX_AGE; }
// Heat loss (°C/h with the boiler off) at room temperature T
float lossAt(float T) { return weatherValid() ? kLoss * fmaxf(1.0f, T - outdoorT) : lossRate; }

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
  EEPROM.writeFloat(ADDR_K_LOSS,     kLoss);
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
  float kl = EEPROM.readFloat(ADDR_K_LOSS);
  kLoss    = (!isnan(kl) && kl >= K_LOSS_MIN && kl <= K_LOSS_MAX)  ? kl  : K_LOSS_DEFAULT;

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
  if (fc.size() <= LOG_MAX_BYTES || logDump.active) { fc.close(); return; }   // trim later, not mid-dump
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

  if (histCount % HIST_COMPACT_EVERY == 0 && !histDump.active) historyCompact();
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
  snprintf(line, sizeof(line), "loss_rate_c_per_h,%.3f", lossAt(target));          exportLine(line);
  snprintf(line, sizeof(line), "loss_flat_c_per_h,%.3f", lossRate);                exportLine(line);
  snprintf(line, sizeof(line), "k_loss_c_per_h_per_c,%.4f", kLoss);                exportLine(line);
  { char to[16]; fmtNum(to, sizeof(to), weatherValid() ? outdoorT : NAN, 1);
    snprintf(line, sizeof(line), "outdoor_c,%s", to);                              exportLine(line); }
  snprintf(line, sizeof(line), "outdoor_age_min,%ld", isnan(outdoorT) ? -1L : (long)((millis() - outdoorAtMs) / 60000)); exportLine(line);
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
  exportLine("ts_utc,target,t_start,t_end,duty_req,duty_actual,carry,slope_c_per_h,heat_rate,loss_rate,integ,mode,flags,tout_c");
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
      char to[16]; fmtNum(to, sizeof(to), r.tout, 1);
      snprintf(line, sizeof(line), "%lu,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%.4f,%u,%s,%s",
               (unsigned long)r.ts, r.target, r.tStart, r.tEnd, r.duty, r.actual, r.carry,
               sl, r.heat, r.loss, r.integ, r.mode, fl, to);
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
  float base = lossAt(target) / heatRate;    // duty that holds temperature steady
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
  r.tout   = weatherValid() ? outdoorT : NAN;

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

// Weather model: RLS on  slope = H·u − kLoss·d,  d = room − outdoor (°C)
void tpiLearnW(float u, float slope, float d) {
  float phi[2] = { u, -d };
  float Pphi[2] = {
    rlsPw[0][0] * phi[0] + rlsPw[0][1] * phi[1],
    rlsPw[1][0] * phi[0] + rlsPw[1][1] * phi[1]
  };
  float denom = RLS_LAMBDA + phi[0] * Pphi[0] + phi[1] * Pphi[1];
  float k[2]  = { Pphi[0] / denom, Pphi[1] / denom };
  float e = slope - (heatRate * u - kLoss * d);

  heatRate = constrain(heatRate + k[0] * e, HEAT_MIN, HEAT_MAX);
  kLoss    = constrain(kLoss    + k[1] * e, K_LOSS_MIN, K_LOSS_MAX);
  lossRate = constrain(kLoss * d, LOSS_MIN, LOSS_MAX);   // flat-L fallback stays current

  float newP[2][2];
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      newP[i][j] = (rlsPw[i][j] - k[i] * Pphi[j]) / RLS_LAMBDA;
  float off = 0.5f * (newP[0][1] + newP[1][0]);
  newP[0][1] = newP[1][0] = off;
  float tr = newP[0][0] + newP[1][1];
  if (tr > 10.0f) {
    float s = 10.0f / tr;
    for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) newP[i][j] *= s;
  }
  memcpy(rlsPw, newP, sizeof(rlsPw));

  learnN++;
  char msg[140];
  snprintf(msg, sizeof(msg),
    "TPI learn #%u: u=%.2f slope=%.2f°C/h err=%.2f out=%.1f → H=%.2f k=%.4f (L=%.2f)",
    learnN, u, slope, e, outdoorT, heatRate, kLoss, lossRate);
  LOG_INFO(msg);
  if (learnN % TPI_SAVE_EVERY == 0) eepromSave();
}

void tpiLearnLossOnlyW(float u, float slope, float d) {
  float e = slope - (heatRate * u - kLoss * d);
  kLoss    = constrain(kLoss - TPI_LOSS_GAIN * e / d, K_LOSS_MIN, K_LOSS_MAX);
  lossRate = constrain(kLoss * d, LOSS_MIN, LOSS_MAX);
  learnN++;
  Serial.printf("[TPI] learn #%u (steady, weather): u=%.2f slope=%.2f err=%.2f out=%.1f → k=%.4f\n",
    learnN, u, slope, e, outdoorT, kLoss);
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
          float base0 = lossAt(target) / heatRate;   // model base duty before learning
          if (weatherValid()) {
            float d = fmaxf(1.0f, (float)(sY / sN) - outdoorT);   // mean room − outdoor
            if (excited) { tpiLearnW(uEff, slope, d);         flags |= CF_LEARN_FULL; }
            else         { tpiLearnLossOnlyW(uEff, slope, d); flags |= CF_LEARN_LOSS; }
          } else {
            if (excited) { tpiLearn(uEff, slope);         flags |= CF_LEARN_FULL; }
            else         { tpiLearnLossOnly(uEff, slope); flags |= CF_LEARN_LOSS; }
          }
          // Bumpless transfer, only while the model is fresh (first TPI_BUMP_LEARN_N
          // learned cycles after a reset): whatever the model now covers, the
          // integral gives back. After the 10 Nov reset the integral grew to 0.155
          // while kLoss was still being learned; once the model caught up both
          // corrected the same shortfall and held the room ~0.2 °C high for hours.
          // Not later: with 5–7-min pulses the loss estimate swings ±0.05 every
          // 3 cycles (radiator after-heat), and on 12 Nov that reset a negative
          // integral to 0 nine times in one night, leaving the room +0.1 °C high.
          if (learnN <= TPI_BUMP_LEARN_N) {
            float dBase = lossAt(target) / heatRate - base0;
            if      (integ > 0.0f && dBase > 0.0f) integ = fmaxf(0.0f, integ - dBase);
            else if (integ < 0.0f && dBase < 0.0f) integ = fminf(0.0f, integ - dBase);
          }
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
    // Above target, high demand may also round down: near the boiler's limit
    // a request of ~93 % otherwise always became 100 % and the room sat 0.1 °C
    // high for hours (10 Nov, 22.6 vs 22.5). Alternating 7/10-min burns then
    // averages the real demand; no extra starts in any simulated scenario.
    bool nearlyFull = onMs * 2 >= 2 * TPI_CYCLE_MS - TPI_MIN_OFF_MS;  // ≥ 8.5 min
    onMs = ((duty >= 0.5f && err >= 0.0f) || nearlyFull) ? TPI_CYCLE_MS : TPI_CYCLE_MS - TPI_MIN_OFF_MS;
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

  char toutStr[12] = "null";
  if (weatherValid()) snprintf(toutStr, sizeof(toutStr), "%.1f", outdoorT);

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
    "\"heat_rate\":%.2f,\"loss_rate\":%.2f,\"rad\":%.2f,\"tout\":%s,"
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
    heatRate, lossAt(target), radHeat, toutStr,
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
        heatRate = HEAT_DEFAULT; lossRate = LOSS_DEFAULT; kLoss = K_LOSS_DEFAULT;
        integ = 0; learnN = 0; uAvg = -1.0f;
        rlsP[0][0] = 4.0f; rlsP[0][1] = rlsP[1][0] = 0.0f; rlsP[1][1] = 1.0f;
        rlsPw[0][0] = 4.0f; rlsPw[0][1] = rlsPw[1][0] = 0.0f; rlsPw[1][1] = 0.01f;
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

  if (tp == String(TOPIC_HIST_GET)) dumpRequest(histDump, msg, true);
  if (tp == String(TOPIC_LOG_GET))  dumpRequest(logDump,  msg, false);
}

// ============================================================
// HISTORY / LOG OVER MQTT
// ============================================================
// Request: {"id":"<1-16 letters/digits>"}. The broker is public, so requests
// are rate-limited and one dump of each kind runs at a time.
void dumpRequest(Dump& d, const String& msg, bool isHist) {
  if (d.active || (d.lastMs && millis() - d.lastMs < DUMP_MIN_GAP_MS)) return;
  StaticJsonDocument<128> req;
  if (deserializeJson(req, msg)) return;
  const char* id = req["id"] | "";
  size_t n = strlen(id);
  if (n == 0 || n >= sizeof(d.id)) return;
  for (size_t i = 0; i < n; i++) if (!isalnum((unsigned char)id[i])) return;
  strcpy(d.id, id);
  d.pos = 0; d.seq = 0; d.active = true; d.lastMs = millis();
  if (isHist) {
    time_t now_t; time(&now_t);
    d.cutoff = (uint32_t)(now_t > 86400 ? now_t - 86400 : 0);
  }
  Serial.printf("[DUMP] %s requested (id %s)\n", isHist ? "history" : "log", d.id);
}

// History chunk: "d":[ts0,t10,b, dts,t10,b, …] — the first timestamp of each
// chunk is absolute (unix s), the rest are deltas; t10 = °C × 10, b = boiler.
void histDumpStep() {
  char buf[MQTT_BUFFER_SIZE - 80];
  int n = snprintf(buf, sizeof(buf), "{\"id\":\"%s\",\"seq\":%u,\"d\":[", histDump.id, histDump.seq);
  bool eof = true;
  File f = LittleFS.open(HISTORY_FILE, "r");
  if (f) {
    f.seek(histDump.pos);
    HistRecord r;
    uint32_t prevTs = 0;
    eof = false;
    while (n < DUMP_CHUNK_FILL) {
      if (f.readBytes((char*)&r, sizeof(r)) != sizeof(r)) { eof = true; break; }
      histDump.pos += sizeof(r);
      if (r.ts < histDump.cutoff) continue;
      long t10 = lroundf(r.t * 10.0f);
      n += snprintf(buf + n, sizeof(buf) - n, prevTs ? ",%lu,%ld,%u" : "%lu,%ld,%u",
                    (unsigned long)(prevTs ? r.ts - prevTs : r.ts), t10, (unsigned)(r.flags & 0x01));
      prevTs = r.ts;
    }
    if (!eof && f.position() >= f.size()) eof = true;
    f.close();
  }
  snprintf(buf + n, sizeof(buf) - n, "],\"end\":%s}", eof ? "true" : "false");
  mqttClient.publish(TOPIC_HIST_DATA, buf);
  histDump.seq++;
  if (eof) { histDump.active = false; Serial.printf("[DUMP] history sent (%u chunks)\n", histDump.seq); }
}

// Log chunk: "d":[[ts,"LEVEL","message"], …] from /log.txt, oldest first
void logDumpStep() {
  char buf[MQTT_BUFFER_SIZE - 80];
  int n = snprintf(buf, sizeof(buf), "{\"id\":\"%s\",\"seq\":%u,\"d\":[", logDump.id, logDump.seq);
  bool eof = true, first = true;
  File f = LittleFS.open(LOG_FILE, "r");
  if (f) {
    f.seek(logDump.pos);
    eof = false;
    while (n < DUMP_CHUNK_FILL) {
      if (!f.available()) { eof = true; break; }
      String l = f.readStringUntil('\n');
      l.trim();
      int p1 = l.indexOf('|'), p2 = p1 < 0 ? -1 : l.indexOf('|', p1 + 1);
      if (p1 < 0 || p2 < 0) { logDump.pos = f.position(); continue; }
      unsigned long ts = strtoul(l.substring(0, p1).c_str(), nullptr, 10);
      char lvl[8]; size_t li = 0;
      for (int k = p1 + 1; k < p2 && li < sizeof(lvl) - 1; k++)
        if (isalpha((unsigned char)l[k])) lvl[li++] = l[k];
      lvl[li] = '\0';
      char msg[400]; size_t mi = 0;   // JSON-escaped, control chars dropped
      for (int k = p2 + 1; k < (int)l.length() && mi < sizeof(msg) - 3; k++) {
        char c = l[k];
        if (c == '"' || c == '\\') { msg[mi++] = '\\'; msg[mi++] = c; }
        else if ((uint8_t)c >= 0x20) msg[mi++] = c;
      }
      msg[mi] = '\0';
      int need = (int)mi + 32;
      if (!first && n + need >= (int)sizeof(buf) - 24) break;   // next chunk; pos not advanced
      n += snprintf(buf + n, sizeof(buf) - n, "%s[%lu,\"%s\",\"%s\"]", first ? "" : ",", ts, lvl, msg);
      first = false;
      logDump.pos = f.position();
    }
    if (!eof && logDump.pos >= f.size()) eof = true;
    f.close();
  }
  snprintf(buf + n, sizeof(buf) - n, "],\"end\":%s}", eof ? "true" : "false");
  mqttClient.publish(TOPIC_LOG_DATA, buf);
  logDump.seq++;
  if (eof) { logDump.active = false; Serial.printf("[DUMP] log sent (%u chunks)\n", logDump.seq); }
}

void dumpStep() {
  if (!mqttClient.connected()) { histDump.active = logDump.active = false; return; }
  unsigned long now = millis();
  if (histDump.active && now - histDump.lastMs >= DUMP_INTERVAL_MS) { histDump.lastMs = now; histDumpStep(); }
  else if (logDump.active && now - logDump.lastMs >= DUMP_INTERVAL_MS) { logDump.lastMs = now; logDumpStep(); }
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
    mqttClient.subscribe(TOPIC_HIST_GET);
    mqttClient.subscribe(TOPIC_LOG_GET);
    Serial.println("[MQTT] Connected");
  } else {
    Serial.println("[MQTT] Error: " + String(mqttClient.state()));
  }
}

// ============================================================
// OUTDOOR TEMPERATURE (Open-Meteo, free, no key; plain http keeps RAM low)
// ============================================================
// GET a small JSON document; false on any network/parse error.
bool httpGetJson(const char* url, JsonDocument& doc) {
  esp_task_wdt_reset();
  HTTPClient http;
  http.setConnectTimeout(WEATHER_HTTP_TIMEOUT);
  http.setTimeout(WEATHER_HTTP_TIMEOUT);
  if (!http.begin(url)) return false;
  int code = http.GET();
  bool ok = false;
  if (code == 200) ok = !deserializeJson(doc, http.getString());
  else Serial.printf("[WEATHER] HTTP %d for %s\n", code, url);
  http.end();
  esp_task_wdt_reset();
  return ok;
}

void weatherUpdate() {
  if (!wifiOk) return;
  unsigned long now = millis();
  unsigned long wait = weatherValid() ? WEATHER_INTERVAL : WEATHER_RETRY;
  if (lastWeatherTry && now - lastWeatherTry < wait) return;
  lastWeatherTry = now;

  // Location: fixed coordinates from secrets.h if given, otherwise from the
  // public IP (city level is plenty for the outdoor temperature).
  if (isnan(geoLat)) {
#if defined(SECRET_LATITUDE) && defined(SECRET_LONGITUDE)
    geoLat = SECRET_LATITUDE; geoLon = SECRET_LONGITUDE;
#else
    StaticJsonDocument<256> g;
    if (!httpGetJson("http://ip-api.com/json/?fields=status,lat,lon", g) ||
        strcmp(g["status"] | "", "success") != 0) {
      Serial.println("[WEATHER] Location lookup failed");
      return;
    }
    geoLat = g["lat"]; geoLon = g["lon"];
#endif
    Serial.printf("[WEATHER] Location %.2f, %.2f\n", geoLat, geoLon);
  }

  char url[160];
  snprintf(url, sizeof(url),
    "http://api.open-meteo.com/v1/forecast?latitude=%.3f&longitude=%.3f&current=temperature_2m",
    geoLat, geoLon);
  StaticJsonDocument<768> w;
  bool hadWeather = weatherValid();
  if (httpGetJson(url, w) && w["current"]["temperature_2m"].is<float>()) {
    outdoorT    = w["current"]["temperature_2m"];
    outdoorAtMs = millis();
    Serial.printf("[WEATHER] Outdoor %.1f °C\n", outdoorT);
    if (!hadWeather) {
      char msg[80];
      snprintf(msg, sizeof(msg), "Outdoor temperature available: %.1f °C", outdoorT);
      LOG_INFO(msg);
    }
  } else if (hadWeather && !weatherValid()) {
    LOG_WARN("Outdoor temperature unavailable — using flat heat loss");
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
  if (LittleFS.exists(TPI_LOG_OLD)) LittleFS.remove(TPI_LOG_OLD);   // old 46-byte records

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
  weatherUpdate();   // rate-limited inside (15 min, 2 min after a failure)
  dumpStep();        // history/log over MQTT, one chunk at a time

  delay(50);
}
