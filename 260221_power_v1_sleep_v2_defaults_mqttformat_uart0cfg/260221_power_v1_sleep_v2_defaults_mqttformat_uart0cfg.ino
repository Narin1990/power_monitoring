#include <WiFi.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_mac.h"

#define FW_VERSION "5.4.1"

// ================= UART0 (USB Serial) =================
#define LOG_BAUD 115200
// UART0 is used for BOTH log + JSON config from Python (newline-delimited JSON)

// ================= ADC =================
#define CT_ADC_PIN  35
#define BAT_ADC_PIN 34
#define ADC_BITS 12

#define BAT_RATIO 3.0f   // 200k/100k divider => VBAT = Vadc * 3

// SCT-013 rating (5A/1V)
#define CT_MAX_A 5.0f

// ================= Globals =================
Preferences prefs;
WiFiClient espClient;
PubSubClient mqtt(espClient);

struct Config {
  // WiFi
  char wifi_ssid[33];
  char wifi_pass[65];

  // MQTT
  char mqtt_host[65];
  uint16_t mqtt_port;
  char mqtt_user[33];
  char mqtt_pass[65];
  char mqtt_topic[129];

  // CT calibration (SCT-013 5A/1V, 50Hz)
  float ct_cal;          // A per mV_rms
  int ct_offset_mv;      // bias offset (mV)
  uint16_t ct_window_ms; // RMS window (ms)

  // Battery calibration
  float bat_cal;

  // publish interval (normal mode)
  uint32_t publish_ms;

  // power saving
  bool sleep_enable;        // deep sleep cycle mode
  uint16_t sleep_s;         // 1..600 seconds (max 10 minutes)
  uint16_t sample_gap_ms;   // gap between the 3 current samples
};

Config cfg;

uint32_t lastTick = 0;
bool streamEnabled = false;

// Robust UART0 line buffer (newline-delimited JSON)
static char cfgLine[1024];
static size_t cfgIdx = 0;

// ================= Forward decl =================
float readIrmsA_50Hz(uint16_t window_ms);
float readVbat(uint16_t samples = 16);
bool wifiConnect(uint32_t timeoutMs = 15000);
bool mqttConnect(uint32_t timeoutMs = 8000);
void publishPowerDetector(float currentA, float batteryV);
void publishPowerDetectorSleep(float avgA, float maxA, float batteryV);
float measure3_avg(float *out_max);
void enterDeepSleep(uint16_t sleep_s);

// ================= UART0 JSON output =================
// Replies are emitted as ONE-LINE JSON + "\\n" so a Python tool can parse them reliably.
void cfgSendJson(const JsonDocument &doc) {
  serializeJson(doc, Serial);
  Serial.print('\n');
  Serial.flush();
}

// ================= UART0 log =================
void logLine(const String &s) { Serial.println(s); }

// ================= MAC helpers =================
String getBaseMacHex() {
  uint64_t chipid = ESP.getEfuseMac();
  char buf[13];
  snprintf(buf, sizeof(buf), "%04X%08X",
           (uint16_t)(chipid >> 32),
           (uint32_t)chipid);
  return String(buf);
}

String getWiFiMacString() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char buf[18];
  snprintf(buf, sizeof(buf),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// ================= Replies =================
void replyError(const char* cmd, const char* err) {
  DynamicJsonDocument out(256);
  out["ok"] = false;
  out["cmd"] = cmd;
  out["fw"] = FW_VERSION;
  out["error"] = err;
  cfgSendJson(out);
}

void replyOk(const char* cmd, const char* msg = nullptr) {
  DynamicJsonDocument out(256);
  out["ok"] = true;
  out["cmd"] = cmd;
  out["fw"] = FW_VERSION;
  if (msg) out["msg"] = msg;
  cfgSendJson(out);
}

// ================= Config defaults/load/save =================
void setDefaults() {
  memset(&cfg, 0, sizeof(cfg));

  // ===== Default provisioning (factory defaults) =====
  
  // Wi-Fi
  strlcpy(cfg.wifi_ssid, "BELUKA@WIFI", sizeof(cfg.wifi_ssid));
  strlcpy(cfg.wifi_pass, "Beluka20402040", sizeof(cfg.wifi_pass));
  
  /*
  // Wi-Fi
  strlcpy(cfg.wifi_ssid, "thunderS24", sizeof(cfg.wifi_ssid));
  strlcpy(cfg.wifi_pass, "narin2533", sizeof(cfg.wifi_pass));
  */

  // MQTT
  strlcpy(cfg.mqtt_host, "13.250.13.194", sizeof(cfg.mqtt_host));
  cfg.mqtt_port = 1883;
  strlcpy(cfg.mqtt_user, "minew", sizeof(cfg.mqtt_user));
  strlcpy(cfg.mqtt_pass, "MiNew2026@#!", sizeof(cfg.mqtt_pass));
  strlcpy(cfg.mqtt_topic, "/hospital/power1", sizeof(cfg.mqtt_topic));

  cfg.ct_offset_mv = 1650;
  cfg.ct_cal = 0.005f;     // 1mVrms ~ 0.005A for 5A/1V (initial guess)
  cfg.ct_window_ms = 200;  // 10 cycles @50Hz

  cfg.bat_cal = 1.0f;

  // Publish interval
  cfg.publish_ms = 5000;  // default 5 seconds

  // Power saving defaults
  cfg.sleep_enable = true;
  cfg.sleep_s = 120;       // 5 minutes
  cfg.sample_gap_ms = 250; // ms
}

void loadConfig() {
  setDefaults();
  prefs.begin("refpower", true);

  prefs.getString("wifi_ssid", cfg.wifi_ssid, sizeof(cfg.wifi_ssid));
  prefs.getString("wifi_pass", cfg.wifi_pass, sizeof(cfg.wifi_pass));

  prefs.getString("mqtt_host", cfg.mqtt_host, sizeof(cfg.mqtt_host));
  cfg.mqtt_port = prefs.getUShort("mqtt_port", cfg.mqtt_port);
  prefs.getString("mqtt_user", cfg.mqtt_user, sizeof(cfg.mqtt_user));
  prefs.getString("mqtt_pass", cfg.mqtt_pass, sizeof(cfg.mqtt_pass));
  prefs.getString("mqtt_topic", cfg.mqtt_topic, sizeof(cfg.mqtt_topic));

  cfg.ct_cal = prefs.getFloat("ct_cal", cfg.ct_cal);
  cfg.ct_offset_mv = prefs.getInt("ct_offmv", cfg.ct_offset_mv);
  cfg.ct_window_ms = (uint16_t)prefs.getUShort("ct_win", cfg.ct_window_ms);

  cfg.bat_cal = prefs.getFloat("bat_cal", cfg.bat_cal);

  cfg.publish_ms = prefs.getUInt("pub_ms", cfg.publish_ms);

  // power saving
  cfg.sleep_enable = prefs.getBool("sl_en", cfg.sleep_enable);
  cfg.sleep_s = (uint16_t)prefs.getUShort("sl_s", cfg.sleep_s);
  cfg.sample_gap_ms = (uint16_t)prefs.getUShort("s_gap", cfg.sample_gap_ms);

  prefs.end();

  if (cfg.ct_window_ms < 40) cfg.ct_window_ms = 40;
  if (cfg.ct_window_ms > 1000) cfg.ct_window_ms = 1000;

  if (cfg.publish_ms < 200) cfg.publish_ms = 200;

  if (cfg.sleep_s < 1) cfg.sleep_s = 1;
  if (cfg.sleep_s > 600) cfg.sleep_s = 600;

  if (cfg.sample_gap_ms > 2000) cfg.sample_gap_ms = 2000;
}

void saveConfig() {
  prefs.begin("refpower", false);

  prefs.putString("wifi_ssid", cfg.wifi_ssid);
  prefs.putString("wifi_pass", cfg.wifi_pass);

  prefs.putString("mqtt_host", cfg.mqtt_host);
  prefs.putUShort("mqtt_port", cfg.mqtt_port);
  prefs.putString("mqtt_user", cfg.mqtt_user);
  prefs.putString("mqtt_pass", cfg.mqtt_pass);
  prefs.putString("mqtt_topic", cfg.mqtt_topic);

  prefs.putFloat("ct_cal", cfg.ct_cal);
  prefs.putInt("ct_offmv", cfg.ct_offset_mv);
  prefs.putUShort("ct_win", cfg.ct_window_ms);

  prefs.putFloat("bat_cal", cfg.bat_cal);

  prefs.putUInt("pub_ms", cfg.publish_ms);

  // power saving
  prefs.putBool("sl_en", cfg.sleep_enable);
  prefs.putUShort("sl_s", cfg.sleep_s);
  prefs.putUShort("s_gap", cfg.sample_gap_ms);

  prefs.end();
}

// ================= WiFi/MQTT =================
bool wifiConnect(uint32_t timeoutMs) {
  if (strlen(cfg.wifi_ssid) == 0) return false;

  // Make WiFi start cleanly after deep sleep
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

// power-save progress message over UART2 (so UI can see what's happening)
void psStep(const char* step, bool ok = true, const char* msg = nullptr) {
  DynamicJsonDocument out(360);
  out["type"] = "ps";
  out["fw"] = FW_VERSION;
  out["step"] = step;
  out["ok"] = ok;
  if (msg) out["msg"] = msg;
  out["ms"] = millis();
  cfgSendJson(out);
}

bool mqttConnect(uint32_t timeoutMs) {
  if (strlen(cfg.mqtt_host) == 0) return false;
  if (WiFi.status() != WL_CONNECTED) return false;

  mqtt.setServer(cfg.mqtt_host, cfg.mqtt_port);

  uint32_t t0 = millis();
  while (!mqtt.connected() && (millis() - t0) < timeoutMs) {
    String clientId = String("powerdet-") + getBaseMacHex();
    bool ok;

    if (strlen(cfg.mqtt_user) > 0) ok = mqtt.connect(clientId.c_str(), cfg.mqtt_user, cfg.mqtt_pass);
    else ok = mqtt.connect(clientId.c_str());

    if (ok) return true;
    delay(300);
  }
  return mqtt.connected();
}

// ================= Measurements =================
int estimateCtOffsetMv(uint16_t samples = 1500) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += (uint32_t)analogReadMilliVolts(CT_ADC_PIN);
    delayMicroseconds(200);
  }
  return (int)(sum / samples);
}

// RMS windowed for 50Hz + LIMIT to 0..5A
float readIrmsA_50Hz(uint16_t window_ms) {
  uint32_t t0 = millis();
  uint64_t sumSq = 0;
  uint32_t n = 0;

  while ((millis() - t0) < window_ms) {
    int mv = (int)analogReadMilliVolts(CT_ADC_PIN);
    int ac = mv - cfg.ct_offset_mv ;
    sumSq += (int64_t)ac * (int64_t)ac;
    n++;
    delayMicroseconds(200);
  }

  if (n == 0) return 0.0f;

  float rms_mv = sqrtf((float)sumSq / (float)n);

  // NOTE: keep as your current fixed calibration to avoid changing behavior unexpectedly
  float currentA = rms_mv * 0.005f;  // (or use cfg.ct_cal if you want)
  currentA = currentA - 0.5;
  if (currentA < 0.0f) currentA = 0.0f;
  if (currentA > CT_MAX_A) currentA = CT_MAX_A;

  return currentA;
}

float readVbat(uint16_t samples) {
  uint32_t mv_sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    mv_sum += (uint32_t)analogReadMilliVolts(BAT_ADC_PIN);
    delay(2);
  }
  float mv = mv_sum / (float)samples;
  return (mv / 1000.0f) * BAT_RATIO * cfg.bat_cal;
}

// Measure current 3 times, return AVG; output MAX through pointer
float measure3_avg(float *out_max) {
  float a1 = readIrmsA_50Hz(cfg.ct_window_ms);
  delay(cfg.sample_gap_ms);
  float a2 = readIrmsA_50Hz(cfg.ct_window_ms);
  delay(cfg.sample_gap_ms);
  float a3 = readIrmsA_50Hz(cfg.ct_window_ms);

  float mx = a1;
  if (a2 > mx) mx = a2;
  if (a3 > mx) mx = a3;

  if (out_max) *out_max = mx;
  return (a1 + a2 + a3) / 3.0f;
}

// ================= MQTT payload =================
void publishPowerDetector(float currentA, float batteryV) {
  if (!mqtt.connected()) return;

  DynamicJsonDocument doc(256);
  JsonObject pd = doc.createNestedObject("power_detector");
  pd["current(A)"] = currentA;
  pd["battery"] = batteryV;
  pd["mac"] = getBaseMacHex();   // chip-based MAC hex (no colons)

  String payload;
  serializeJson(doc, payload);
  mqtt.publish(cfg.mqtt_topic, payload.c_str());
}

// sleep mode payload (avg+max)
void publishPowerDetectorSleep(float avgA, float maxA, float batteryV) {
  (void)maxA; // kept for compatibility, not published in payload
  if (!mqtt.connected()) return;

  DynamicJsonDocument doc(256);
  JsonObject pd = doc.createNestedObject("power_detector");
  // Sleep mode: publish the averaged current as current(A) to match required MQTT schema
  pd["current(A)"] = avgA;
  pd["battery"] = batteryV;
  pd["mac"] = getBaseMacHex();   // chip-based MAC hex (no colons)

  String payload;
  serializeJson(doc, payload);
  mqtt.publish(cfg.mqtt_topic, payload.c_str());
}

// ================= UART0 stream =================
void sendDataFrameUart0(float currentA, float batteryV) {
  DynamicJsonDocument out(256);
  out["type"] = "data";
  out["irms"] = currentA;
  out["vbat"] = batteryV;
  out["ms"] = millis();
  cfgSendJson(out);
}

// ================= Deep Sleep =================
void enterDeepSleep(uint16_t sleep_s) {
  if (sleep_s < 1) sleep_s = 1;
  if (sleep_s > 600) sleep_s = 600;

  if (mqtt.connected()) mqtt.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup((uint64_t)sleep_s * 1000000ULL);
  esp_deep_sleep_start();
}

// ================= Commands =================
void handleCommandLine(const char *line) {
  DynamicJsonDocument doc(1024);
  DeserializationError e = deserializeJson(doc, line);
  if (e) { replyError("parse", "invalid_json"); return; }

  const char *cmd = doc["cmd"] | "";
  if (!cmd || cmd[0] == '\0') { replyError("parse", "missing_cmd"); return; }

  logLine(String("[CFG RX] ") + String(line));

  if (strcmp(cmd, "ping") == 0) {
    DynamicJsonDocument out(360);
    out["ok"] = true;
    out["cmd"] = "ping";
    out["fw"] = FW_VERSION;
    out["ms"] = millis();
    out["base_mac"] = getBaseMacHex();
    out["mac"] = getWiFiMacString();
    cfgSendJson(out);
    return;
  }

  if (strcmp(cmd, "get_mac") == 0) {
    DynamicJsonDocument out(360);
    out["ok"] = true;
    out["cmd"] = "get_mac";
    out["fw"] = FW_VERSION;
    out["base_mac"] = getBaseMacHex();
    out["mac"] = getWiFiMacString();
    cfgSendJson(out);
    return;
  }

  // {"cmd":"set_publish","publish_ms":10000}
  if (strcmp(cmd, "set_publish") == 0) {
    uint32_t ms = doc["publish_ms"] | cfg.publish_ms;

    if (ms < 1000) ms = 1000;
    if (ms > 3600000) ms = 3600000;

    cfg.publish_ms = ms;
    saveConfig();

    DynamicJsonDocument out(320);
    out["ok"] = true;
    out["cmd"] = "set_publish";
    out["fw"] = FW_VERSION;
    out["publish_ms"] = cfg.publish_ms;
    cfgSendJson(out);
    return;
  }

  // {"cmd":"set_sleep","enable":true,"sleep_s":120,"sample_gap_ms":250}
  if (strcmp(cmd, "set_sleep") == 0) {
    bool en = doc["enable"] | cfg.sleep_enable;
    int s = doc["sleep_s"] | (int)cfg.sleep_s;
    int gap = doc["sample_gap_ms"] | (int)cfg.sample_gap_ms;

    if (s < 1) s = 1;
    if (s > 600) s = 600;
    if (gap < 0) gap = 0;
    if (gap > 2000) gap = 2000;

    cfg.sleep_enable = en;
    cfg.sleep_s = (uint16_t)s;
    cfg.sample_gap_ms = (uint16_t)gap;
    saveConfig();

    DynamicJsonDocument out(360);
    out["ok"] = true;
    out["cmd"] = "set_sleep";
    out["fw"] = FW_VERSION;
    out["enable"] = cfg.sleep_enable;
    out["sleep_s"] = cfg.sleep_s;
    out["sample_gap_ms"] = cfg.sample_gap_ms;
    cfgSendJson(out);
    return;
  }

  // {"cmd":"sleep_now"} -> enter deep sleep immediately (if enabled)
  if (strcmp(cmd, "sleep_now") == 0) {
    replyOk("sleep_now", "entering_sleep");
    delay(50);
    enterDeepSleep(cfg.sleep_s);
    return;
  }

  if (strcmp(cmd, "set_wifi") == 0) {
    const char* ssid = doc["ssid"] | "";
    const char* pass = doc["pass"] | "";
    strlcpy(cfg.wifi_ssid, ssid, sizeof(cfg.wifi_ssid));
    strlcpy(cfg.wifi_pass, pass, sizeof(cfg.wifi_pass));
    saveConfig();
    replyOk("set_wifi", "saved");
    return;
  }

  if (strcmp(cmd, "test_wifi") == 0) {
    bool ok = (WiFi.status() == WL_CONNECTED) ? true : wifiConnect(15000);
    DynamicJsonDocument out(460);
    out["ok"] = ok;
    out["cmd"] = "test_wifi";
    out["fw"] = FW_VERSION;
    out["ssid"] = cfg.wifi_ssid;
    out["ip"] = ok ? WiFi.localIP().toString() : "";
    out["mac"] = getWiFiMacString();
    cfgSendJson(out);
    return;
  }

  if (strcmp(cmd, "set_mqtt") == 0) {
    const char* host  = doc["host"]  | "";
    int port          = doc["port"]  | 1883;
    const char* user  = doc["user"]  | "";
    const char* pass  = doc["pass"]  | "";

    const char* topic = doc["topic"] | "";
    const char* base_topic = doc["base_topic"] | "";

    strlcpy(cfg.mqtt_host, host, sizeof(cfg.mqtt_host));
    cfg.mqtt_port = (uint16_t)port;
    strlcpy(cfg.mqtt_user, user, sizeof(cfg.mqtt_user));
    strlcpy(cfg.mqtt_pass, pass, sizeof(cfg.mqtt_pass));

    if (strlen(topic) > 0) strlcpy(cfg.mqtt_topic, topic, sizeof(cfg.mqtt_topic));
    else if (strlen(base_topic) > 0) strlcpy(cfg.mqtt_topic, base_topic, sizeof(cfg.mqtt_topic));
    else strlcpy(cfg.mqtt_topic, "power_detector", sizeof(cfg.mqtt_topic));

    saveConfig();
    replyOk("set_mqtt", "saved");
    return;
  }

  if (strcmp(cmd, "test_mqtt") == 0) {
    bool w = (WiFi.status() == WL_CONNECTED) ? true : wifiConnect(15000);
    bool ok = false;
    if (w) ok = mqttConnect(8000);

    if (ok) {
      float bat = readVbat(16);
      publishPowerDetector(0.0f, bat);
      mqtt.loop();
      delay(150);
    }

    DynamicJsonDocument out(720);
    out["ok"] = ok;
    out["cmd"] = "test_mqtt";
    out["fw"] = FW_VERSION;
    out["wifi"] = (WiFi.status() == WL_CONNECTED);
    out["mqtt"] = mqtt.connected();
    out["host"] = cfg.mqtt_host;
    out["port"] = cfg.mqtt_port;
    out["topic"] = cfg.mqtt_topic;
    out["publish_ms"] = cfg.publish_ms;
    out["sleep_enable"] = cfg.sleep_enable;
    out["sleep_s"] = cfg.sleep_s;
    out["sample_gap_ms"] = cfg.sample_gap_ms;
    out["base_mac"] = getBaseMacHex();
    out["mac"] = getWiFiMacString();
    cfgSendJson(out);
    return;
  }

  if (strcmp(cmd, "get_config") == 0) {
    DynamicJsonDocument out(1600);
    out["ok"] = true;
    out["cmd"] = "get_config";
    out["fw"] = FW_VERSION;

    JsonObject w = out.createNestedObject("wifi");
    w["ssid"] = cfg.wifi_ssid;
    w["pass"] = (strlen(cfg.wifi_pass) ? "***" : "");

    JsonObject m = out.createNestedObject("mqtt");
    m["host"] = cfg.mqtt_host;
    m["port"] = cfg.mqtt_port;
    m["user"] = cfg.mqtt_user;
    m["pass"] = (strlen(cfg.mqtt_pass) ? "***" : "");
    m["topic"] = cfg.mqtt_topic;

    JsonObject ct = out.createNestedObject("ct");
    ct["cal_A_per_mVrms"] = cfg.ct_cal;
    ct["offset_mv"] = cfg.ct_offset_mv;
    ct["window_ms"] = cfg.ct_window_ms;
    ct["hz"] = 50;
    ct["max_a"] = CT_MAX_A;

    JsonObject bat = out.createNestedObject("battery");
    bat["cal"] = cfg.bat_cal;
    bat["ratio"] = BAT_RATIO;

    JsonObject ps = out.createNestedObject("power_save");
    ps["enable"] = cfg.sleep_enable;
    ps["sleep_s"] = cfg.sleep_s;
    ps["sample_gap_ms"] = cfg.sample_gap_ms;

    // compatibility for UIs that expect "sleep" instead of "power_save"
    JsonObject sl = out.createNestedObject("sleep");
    sl["enable"] = cfg.sleep_enable;
    sl["sleep_s"] = cfg.sleep_s;
    sl["sample_gap_ms"] = cfg.sample_gap_ms;

    out["publish_ms"] = cfg.publish_ms;
    out["stream"] = streamEnabled;

    out["base_mac"] = getBaseMacHex();
    out["mac"] = getWiFiMacString();

    cfgSendJson(out);
    return;
  }

  if (strcmp(cmd, "read_status") == 0) {
    float cur = readIrmsA_50Hz(cfg.ct_window_ms);
    float bat = readVbat(16);

    DynamicJsonDocument out(980);
    out["ok"] = true;
    out["cmd"] = "read_status";
    out["fw"] = FW_VERSION;
    out["wifi"] = (WiFi.status() == WL_CONNECTED);
    out["mqtt"] = mqtt.connected();

    out["irms"] = cur;
    out["vbat"] = bat;

    out["ct_offset_mv"] = cfg.ct_offset_mv;
    out["ct_cal"] = cfg.ct_cal;
    out["ct_window_ms"] = cfg.ct_window_ms;
    out["ct_max_a"] = CT_MAX_A;

    out["publish_ms"] = cfg.publish_ms;
    out["sleep_enable"] = cfg.sleep_enable;
    out["sleep_s"] = cfg.sleep_s;
    out["sample_gap_ms"] = cfg.sample_gap_ms;

    out["base_mac"] = getBaseMacHex();
    out["mac"] = getWiFiMacString();

    cfgSendJson(out);
    return;
  }

  if (strcmp(cmd, "stream_enable") == 0) {
    bool en = doc["enable"] | false;
    streamEnabled = en;

    DynamicJsonDocument out(280);
    out["ok"] = true;
    out["cmd"] = "stream_enable";
    out["fw"] = FW_VERSION;
    out["enable"] = streamEnabled;
    cfgSendJson(out);
    return;
  }

  if (strcmp(cmd, "ct_autozero") == 0) {
    int off = estimateCtOffsetMv(1500);
    cfg.ct_offset_mv = off;
    saveConfig();

    DynamicJsonDocument out(420);
    out["ok"] = true;
    out["cmd"] = "ct_autozero";
    out["fw"] = FW_VERSION;
    out["offset_mv"] = cfg.ct_offset_mv;
    cfgSendJson(out);
    return;
  }

  // {"cmd":"set_ct","cal":0.005,"window_ms":200}
  if (strcmp(cmd, "set_ct") == 0) {
    bool changed = false;

    if (doc.containsKey("cal")) {
      cfg.ct_cal = doc["cal"].as<float>();
      if (cfg.ct_cal < 0.000001f) cfg.ct_cal = 0.000001f;
      if (cfg.ct_cal > 1.0f) cfg.ct_cal = 1.0f; // safety clamp
      changed = true;
    }

    if (doc.containsKey("window_ms")) {
      int wms = doc["window_ms"].as<int>();
      if (wms < 40) wms = 40;
      if (wms > 1000) wms = 1000;
      cfg.ct_window_ms = (uint16_t)wms;
      changed = true;
    }

    if (changed) saveConfig();

    DynamicJsonDocument out(520);
    out["ok"] = true;
    out["cmd"] = "set_ct";
    out["fw"] = FW_VERSION;
    out["ct_cal"] = cfg.ct_cal;
    out["ct_offset_mv"] = cfg.ct_offset_mv;
    out["ct_window_ms"] = cfg.ct_window_ms;
    out["ct_max_a"] = CT_MAX_A;
    cfgSendJson(out);
    return;
  }

  if (strcmp(cmd, "reboot") == 0) {
    replyOk("reboot", "restarting");
    delay(200);
    ESP.restart();
  }

  replyError(cmd, "unknown_cmd");
}

// ✅ Robust UART0 receiver: ignore junk before '{', cut after last '}', drop overflow
void pollCfgUart0() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (cfgIdx >= sizeof(cfgLine) - 1) {
      cfgIdx = 0; // drop overflow garbage
    }

    if (c == '\n') {
      cfgLine[cfgIdx] = '\0';
      cfgIdx = 0;

      char *p = strchr(cfgLine, '{');
      if (!p) continue;

      char *q = strrchr(p, '}');
      if (q) *(q + 1) = '\0';

      while (*p == ' ' || *p == '\t') p++;

      if (*p) handleCommandLine(p);
    } else {
      cfgLine[cfgIdx++] = c;
    }
  }
}

// ================= setup/loop =================
void setup() {
  Serial.begin(LOG_BAUD);
  delay(120);

  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(CT_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

  loadConfig();

  Serial.println("Boot: UART0=LOG+CFG(JSON)");
  Serial.println("Base MAC: " + getBaseMacHex());
  Serial.println("WiFi  MAC: " + getWiFiMacString());
  Serial.println(String("FW=") + FW_VERSION);
  Serial.println(String("publish_ms=") + cfg.publish_ms);

  // Hello on UART0 (USB Serial)
  DynamicJsonDocument hello(720);
  hello["type"] = "hello";
  hello["fw"] = FW_VERSION;
  hello["cfg_uart"] = "UART0";  hello["hz"] = 50;
  hello["stream_default"] = false;
  hello["publish_ms"] = cfg.publish_ms;
  hello["ct_max_a"] = CT_MAX_A;
  hello["sleep_enable"] = cfg.sleep_enable;
  hello["sleep_s"] = cfg.sleep_s;
  hello["sample_gap_ms"] = cfg.sample_gap_ms;
  hello["base_mac"] = getBaseMacHex();
  hello["mac"] = getWiFiMacString();
  cfgSendJson(hello);

  // Small config window (so you can disable sleep right after waking)
  uint32_t tCfg = millis();
  while (millis() - tCfg < 3000) {
    pollCfgUart0();
    delay(5);
  }

  // If sleep mode enabled: single-cycle then deep sleep
  if (cfg.sleep_enable) {
    Serial.println("[PS] Sleep mode enabled: measure x3 -> MQTT -> deep sleep");

    psStep("wake");

    psStep("wifi_connecting");

    bool wok = (WiFi.status() == WL_CONNECTED) ? true : wifiConnect(20000);

    // report wifi status
    if (wok) {
      String ip = WiFi.localIP().toString();
      psStep("wifi_connected", true, ip.c_str());
    } else {
      psStep("wifi_connected", false, "fail");
    }

    bool mok = false;
    if (wok && strlen(cfg.mqtt_host) > 0) {
      psStep("mqtt_connecting");
      mok = mqttConnect(10000);
      psStep("mqtt_connected", mok, mok ? "ok" : "fail");
    } else {
      psStep("mqtt_skipped", false, (strlen(cfg.mqtt_host) == 0) ? "no_host" : "wifi_fail");
    }

    float batV = readVbat(16);

    float maxA = 0.0f;
    psStep("measuring");
    float avgA = measure3_avg(&maxA);
    psStep("measured");

    if (mok && mqtt.connected()) {
      psStep("publishing");
      publishPowerDetectorSleep(avgA, maxA, batV);
      mqtt.loop();
      delay(250); // let packet out before sleep
      psStep("published");
    } else {
      psStep("publish_skipped", false, "mqtt_not_connected");
    }

    DynamicJsonDocument out(520);
    out["type"] = "once";
    out["fw"] = FW_VERSION;
    out["wifi"] = (WiFi.status() == WL_CONNECTED);
    out["mqtt"] = mqtt.connected();
    out["irms_avg"] = avgA;
    out["irms_max"] = maxA;
    out["vbat"] = batV;
    out["sleep_s"] = cfg.sleep_s;
    out["base_mac"] = getBaseMacHex();
    out["mac"] = getWiFiMacString();
    cfgSendJson(out);

    Serial.printf("[PS] Sleeping %u s...\n", cfg.sleep_s);
    psStep("sleeping");
    delay(50);
    enterDeepSleep(cfg.sleep_s);
  }

  // ===== AUTO CONNECT ON BOOT (normal mode) =====
  if (strlen(cfg.wifi_ssid) > 0) {
    Serial.println("[BOOT] Auto WiFi connect...");
    bool wok = wifiConnect(15000);
    Serial.println(wok ? "[BOOT] WiFi connected" : "[BOOT] WiFi connect FAIL");
    if (wok) {
      Serial.print("[BOOT] IP: ");
      Serial.println(WiFi.localIP());
    }
  } else {
    Serial.println("[BOOT] No WiFi SSID saved.");
  }

  if (WiFi.status() == WL_CONNECTED && strlen(cfg.mqtt_host) > 0) {
    Serial.println("[BOOT] Auto MQTT connect...");
    bool mok = mqttConnect(8000);
    Serial.println(mok ? "[BOOT] MQTT connected" : "[BOOT] MQTT connect FAIL");
  } else if (strlen(cfg.mqtt_host) == 0) {
    Serial.println("[BOOT] No MQTT host saved.");
  }
}

void loop() {
  pollCfgUart0();

  // ===== WiFi reconnect =====
  static uint32_t lastWiFiTry = 0;
  if (WiFi.status() != WL_CONNECTED && strlen(cfg.wifi_ssid) > 0) {
    if (millis() - lastWiFiTry > 5000) {
      lastWiFiTry = millis();
      Serial.println("[NET] WiFi reconnect...");
      wifiConnect(10000);
    }
  }

  // ===== MQTT keep-alive / reconnect =====
  if (WiFi.status() == WL_CONNECTED && strlen(cfg.mqtt_host) > 0) {
    if (!mqtt.connected()) {
      static uint32_t lastMqttTry = 0;
      if (millis() - lastMqttTry > 3000) {
        lastMqttTry = millis();
        bool ok = mqttConnect(8000);
        Serial.println(ok ? "[NET] MQTT connected" : "[NET] MQTT connect fail");
      }
    } else {
      mqtt.loop();
    }
  }

  // periodic tick (publish) - normal mode
  if (cfg.publish_ms > 0 && millis() - lastTick >= cfg.publish_ms) {
    lastTick = millis();

    float curA = readIrmsA_50Hz(cfg.ct_window_ms);
    float batV = readVbat(16);

    if (streamEnabled) {
      sendDataFrameUart0(curA, batV);
    }

    if (mqtt.connected()) {
      publishPowerDetector(curA, batV);
    }
  }

  delay(5);
}
