/*
  ============================================================
  BOILER SYSTEM — RELAY ESP32
  ============================================================
  Hardware:
    - Relay → GPIO 23  (HIGH = boiler ON)

  SAFETY: If no heartbeat is received from the temperature ESP
          within 30 seconds, or if the connection drops,
          the relay is forced OFF (fail-safe).

  Heartbeat format: {"alive":true}
  Temp message:     {"t","h","boiler","door","bat","mode","target",
                     "duty","cycle_left_s","heat_rate","loss_rate",
                     "kp","integ","learn_n","on_min","off_min","ip"}
                    (only t, boiler, door, bat, mode, target are used here)
  Command format:   "manual_on" | "manual_off" | "auto"

  NOTE: All boiler decisions (TPI control, mode) are made by
        the temperature ESP. This ESP only looks at the
        "boiler" field and applies the safety watchdog.
  ============================================================
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

// ============================================================
// SETTINGS
// ============================================================
#include "secrets.h"   // SECRET_WIFI_SSID / SECRET_WIFI_PASSWORD — git-ignored, see secrets.example.h
const char* WIFI_SSID     = SECRET_WIFI_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;

const char* MQTT_SERVER   = "broker.hivemq.com";
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "";
const char* MQTT_PASS     = "";

const char* DEVICE_ID     = "silent-hearth-681";

// ============================================================
// MQTT TOPICS
// ============================================================
#define TOPIC_HEARTBEAT  "silent-hearth-681/heartbeat"
#define TOPIC_CMD        "silent-hearth-681/boiler/cmd"
#define TOPIC_TEMP       "silent-hearth-681/temp"
#define TOPIC_STATUS     "silent-hearth-681/relay/status"

// ============================================================
// CONSTANTS
// ============================================================
#define HEARTBEAT_TIMEOUT  30000UL   // ms — relay off if exceeded
#define STATUS_INTERVAL    60000UL   // ms — periodic status publish
#define RELAY_PIN          23

// Buffer sizes — /temp payload is ~500 bytes and growing; keep headroom.
// MQTT_BUFFER_SIZE must hold topic + payload + MQTT header.
#define MQTT_BUFFER_SIZE   1024
#define TEMP_JSON_SIZE     1024      // fits ArduinoJson v6 (fixed pool) and v7 (ignored)
#define STATUS_JSON_SIZE    384

// ============================================================
// STATE
// ============================================================
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

bool relayOn        = false;
bool boilerAllowed  = false;
bool heartbeatSeen  = false;
String currentMode  = "auto";

unsigned long lastHeartbeat  = 0;
unsigned long lastWifiRetry  = 0;
unsigned long lastMqttRetry  = 0;
unsigned long lastStatusPub  = 0;

// Last received values (included in status publish)
float lastTemp       = 0;
float lastTarget     = 22.0f;

// ============================================================
// RELAY — central safety gate
// Relay only turns on if:  requested AND heartbeat is fresh
// ============================================================
void setRelay(bool on, const char* reason) {
  bool safe    = heartbeatSeen && (millis() - lastHeartbeat < HEARTBEAT_TIMEOUT);
  bool desired = on && safe;

  if (desired != relayOn) {
    relayOn = desired;
    digitalWrite(RELAY_PIN, relayOn ? HIGH : LOW);
    Serial.printf("[RELAY] %s — Reason: %s | Safe: %s\n",
                  relayOn ? "ON" : "OFF", reason, safe ? "yes" : "NO");

    StaticJsonDocument<STATUS_JSON_SIZE> doc;
    doc["relay"]        = relayOn;
    doc["reason"]       = reason;
    doc["safe"]         = safe;
    doc["hb_age_ms"]    = (long)(millis() - lastHeartbeat);
    doc["mode"]         = currentMode;
    doc["t"]            = lastTemp;
    doc["target"]       = lastTarget;
    char buf[STATUS_JSON_SIZE];
    serializeJson(doc, buf);
    mqtt.publish(TOPIC_STATUS, buf, true);
  }
}

// ============================================================
// MQTT CALLBACK
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  String tp = String(topic);

  // ---- Heartbeat ----
  if (tp == String(TOPIC_HEARTBEAT)) {
    StaticJsonDocument<64> doc;
    if (!deserializeJson(doc, msg)) {
      bool alive = doc["alive"] | false;
      if (alive) {
        lastHeartbeat = millis();
        heartbeatSeen = true;
        Serial.println("[HB] Heartbeat received");
      } else {
        Serial.println("[HB] Remote ESP offline — forcing relay OFF");
        setRelay(false, "heartbeat_offline");
      }
    } else {
      Serial.println("[HB] JSON parse error: " + msg);
    }
    return;
  }

  // ---- Command ----
  if (tp == String(TOPIC_CMD)) {
    if      (msg == "manual_on")  currentMode = "manual_on";
    else if (msg == "manual_off") currentMode = "manual_off";
    else if (msg == "auto")       currentMode = "auto";

    if (msg == "manual_off") {
      boilerAllowed = false;
      setRelay(false, "cmd_off");
    }
    Serial.println("[CMD] " + msg + " | mode=" + currentMode);
    return;
  }

  // ---- Temperature data — relay follows "boiler" field ----
  if (tp == String(TOPIC_TEMP)) {
    StaticJsonDocument<TEMP_JSON_SIZE> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (err) {
      Serial.printf("[TEMP] JSON parse error: %s (len=%u)\n", err.c_str(), length);
      return;
    }

    bool  door     = doc["door"]          | false;
    bool  boiler   = doc["boiler"]        | false;
    float t        = doc["t"]             | 0.0f;
    float tgt      = doc["target"]        | 22.0f;
    int   bat      = doc["bat"]           | 100;
    const char* md = doc["mode"]          | "auto";

    lastTemp      = t;
    lastTarget    = tgt;
    currentMode   = String(md);

    Serial.printf("[TEMP] T:%.1f target:%.1f Boiler:%s Door:%s Mode:%s Bat:%d%%\n",
      t, tgt,
      boiler ? "ON" : "OFF",
      door   ? "OPEN" : "closed",
      md, bat);

    if (bat < 15) Serial.printf("[WARN] Battery critical: %d%%\n", bat);

    if (door) {
      setRelay(false, "door_open");
    } else {
      boilerAllowed = boiler;
      setRelay(boiler, boiler ? "tpi_on" : "tpi_off");
    }
  }
}

// ============================================================
// WIFI
// ============================================================
void wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("[WiFi] Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 20) {
    delay(500); Serial.print(".");
    esp_task_wdt_reset();   // this loop can take 10 s
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println(" OK: " + WiFi.localIP().toString());
  else
    Serial.println(" FAILED");
}

// ============================================================
// MQTT
// ============================================================
void mqttConnect() {
  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.print("[MQTT] Connecting...");
  esp_task_wdt_reset();
  String cid = "esp32-relay-" + String(DEVICE_ID);
  if (mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println(" OK");
    mqtt.subscribe(TOPIC_HEARTBEAT);
    mqtt.subscribe(TOPIC_CMD);
    mqtt.subscribe(TOPIC_TEMP);
  } else {
    Serial.println(" Error: " + String(mqtt.state()));
  }
}

// ============================================================
// HEARTBEAT WATCHDOG
// ============================================================
void watchdog() {
  if (!heartbeatSeen) return;
  if (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT) {
    Serial.printf("[WATCHDOG] No heartbeat for %lums — relay OFF\n",
                  millis() - lastHeartbeat);
    setRelay(false, "heartbeat_timeout");
    heartbeatSeen = false;
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Boiler Relay ESP32 starting...");

  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = 30000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic  = true
  };
  // Core 3.x already starts the task watchdog at boot with a 5 s timeout, so
  // init() fails with ESP_ERR_INVALID_STATE and the 30 s never applied.
  if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(NULL);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("[RELAY] Initial state: OFF (fail-safe)");

  wifiConnect();

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(4);   // CONNACK wait (default 15 s) — must stay under the watchdog
  if (!mqtt.setBufferSize(MQTT_BUFFER_SIZE))
    Serial.println("[MQTT] WARNING: could not allocate MQTT buffer");
  mqttConnect();

  Serial.println("[BOOT] Ready — waiting for heartbeat...");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  esp_task_wdt_reset();
  unsigned long now = millis();

  // ---- WiFi reconnect ----
  if (WiFi.status() != WL_CONNECTED && now - lastWifiRetry > 30000) {
    lastWifiRetry = now;
    setRelay(false, "wifi_lost");
    wifiConnect();
  }

  // ---- MQTT reconnect ----
  if (!mqtt.connected()) {
    if (now - lastMqttRetry > 5000) {
      lastMqttRetry = now;
      setRelay(false, "mqtt_lost");
      mqttConnect();
    }
  } else {
    mqtt.loop();
  }

  // ---- Heartbeat watchdog ----
  watchdog();

  // ---- Periodic status publish (60s) ----
  if (now - lastStatusPub > STATUS_INTERVAL) {
    lastStatusPub = now;
    StaticJsonDocument<STATUS_JSON_SIZE> doc;
    doc["relay"]         = relayOn;
    doc["boiler_allowed"]= boilerAllowed;
    doc["mode"]          = currentMode;
    doc["hb_age_ms"]     = heartbeatSeen ? (long)(millis() - lastHeartbeat) : -1L;
    doc["wifi_rssi"]     = WiFi.RSSI();
    doc["t"]             = lastTemp;
    doc["target"]        = lastTarget;
    char buf[STATUS_JSON_SIZE];
    serializeJson(doc, buf);
    mqtt.publish(TOPIC_STATUS, buf, true);
    Serial.printf("[STATUS] Relay:%s  Mode:%s  HB:%ldms  RSSI:%d\n",
                  relayOn ? "ON" : "OFF", currentMode.c_str(),
                  heartbeatSeen ? (long)(millis() - lastHeartbeat) : -1L,
                  WiFi.RSSI());
  }

  delay(100);
}
