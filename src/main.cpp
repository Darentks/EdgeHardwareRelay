/*
  =============================================================================
  Smart Storage FYP - Localized ESP32 Camera Hardware Relay Firmware
  Author: Daren Tan Khai Shuen (ID: 25WMR09824)
  Supervisor: Dr Tew Yiqi | TARUMT

  Architecture: Vision Capture Relay (Baseline Cache + On-Demand Comparison)
  Hardware: ESP32-CAM (AI-Thinker) + OV2640 / OV7725 Sensor
  Framework: Arduino / FreeRTOS (PlatformIO)

  Fulfills:
    - FR-05 & FR-06: Localized extraction verification via camera snapshot
    - FR-10 & NFR-08: Asynchronous 30-second MQTT health telemetry & baseline cache
    - Table 4.8 & Section 4.7: MQTT topic structure and JSON payload schemas
  =============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "camera_pins.h"

// =========================================================================
// 1. HARDWARE & FIXED COMPARTMENT CONFIGURATION
// =========================================================================
const char* DEVICE_ID    = "ESP32_R1_C1";       // Unique MQTT Client & Node Identifier
const char* RACK_ID      = "RACK_01";           // Fixed Rack Identifier
const char* SHELF_ID     = "SHELF_R1_C1";       // Fixed Mapped Compartment Identifier
const char* CAMERA_NAME  = "ESP32 Node R1-C1";  // Readable Camera Name
const char* FIRMWARE_VER = "v1.2.0-FreeRTOS";

// =========================================================================
// 2. NETWORK, HTTP SNAPSHOT SERVER & MQTT BROKER CONFIGURATION
// =========================================================================
const char* WIFI_SSID       = "B100A";     // Enter your local Wi-Fi SSID
const char* WIFI_PASSWORD   = "12345678"; // Enter your local Wi-Fi Password

// IP of the backend laptop/server running Mosquitto MQTT Broker
const char* MQTT_BROKER_IP  = "192.168.0.100";     // Change to your laptop's IP
const int   MQTT_PORT       = 1883;

// Fixed MQTT Topics conforming to Table 4.8
const char* trigger_topic   = "iotlab/storage/RACK_01/trigger";
const char* response_topic  = "iotlab/storage/SHELF_R1_C1/response";
const char* baseline_topic  = "iotlab/storage/SHELF_R1_C1/baseline";
const char* config_topic    = "iotlab/storage/node/ESP32_R1_C1/config"; // Strictly for CAPTURE_REFERENCE
const char* HEALTH_TOPIC    = "iotlab/storage/health";

// =========================================================================
// 3. HARDWARE, TIMING & PSRAM BASELINE BUFFER CONSTANTS
// =========================================================================
const unsigned long HEARTBEAT_INTERVAL_MS = 30000; // 30 seconds (NFR-08)
unsigned long lastHeartbeatTime = 0;
unsigned long lastMqttRetryTime = 0;
bool isCameraInitialized = false;

// Cached baseline JPEG frame buffer in PSRAM
uint8_t* latestBaselineBuf = nullptr;
size_t   latestBaselineLen = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer httpServer(80);

// Forward declarations
void publishHealthTelemetry(const char* event = nullptr);
void captureAndCacheBaseline();

// =========================================================================
// 4. CAMERA INITIALIZATION FUNCTION
// =========================================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality = 12;              // High quality
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320x240 fallback if no PSRAM
    config.jpeg_quality = 15;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[-] Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
  }

  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    pinMode(LED_FLASH_PIN, OUTPUT);
    digitalWrite(LED_FLASH_PIN, LOW);
  #endif

  Serial.println("[+] Camera hardware initialized successfully.");
  return true;
}

// =========================================================================
// 5. ONBOARD HTTP SNAPSHOT SERVER (PORT 80)
//    - GET /capture?mode=baseline : Serves cached 30s baseline JPEG
//    - GET /capture               : Captures fresh live JPEG image
//    - GET /status                : Returns JSON status
// =========================================================================
void setupHttpServer() {
  httpServer.on("/status", HTTP_GET, []() {
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.sendHeader("Access-Control-Allow-Private-Network", "true");
    httpServer.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
    StaticJsonDocument<256> doc;
    doc["status"]         = "ok";
    doc["device_id"]      = DEVICE_ID;
    doc["mqtt_client_id"] = DEVICE_ID;
    doc["rack_id"]        = RACK_ID;
    doc["shelf_id"]       = SHELF_ID;
    doc["ip_address"]     = WiFi.localIP().toString();
    doc["rssi_dbm"]       = WiFi.RSSI();
    String out;
    serializeJson(doc, out);
    httpServer.send(200, "application/json", out);
  });

  httpServer.on("/capture", HTTP_GET, []() {
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.sendHeader("Access-Control-Allow-Private-Network", "true");
    httpServer.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");

    // Check if mode=baseline is requested and cache is available
    if (httpServer.arg("mode") == "baseline" && latestBaselineBuf != nullptr && latestBaselineLen > 0) {
      httpServer.sendHeader("Content-Disposition", "inline; filename=esp32_baseline.jpg");
      httpServer.send_P(200, "image/jpeg", (const char*)latestBaselineBuf, latestBaselineLen);
      return;
    }

    // Otherwise, capture a fresh live JPEG frame
    bool useFlash = true;
    if (httpServer.hasArg("flash") && httpServer.arg("flash") == "0") {
      useFlash = false;
    }

    #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
      if (useFlash) {
        digitalWrite(LED_FLASH_PIN, HIGH);
        delay(40);
      }
    #endif

    camera_fb_t * fb = esp_camera_fb_get();

    #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
      if (useFlash) {
        digitalWrite(LED_FLASH_PIN, LOW);
      }
    #endif

    if (!fb) {
      httpServer.send(500, "application/json", "{\"error\":\"Snapshot capture failed\"}");
      return;
    }

    httpServer.sendHeader("Content-Disposition", "inline; filename=esp32_capture.jpg");
    httpServer.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  });

  httpServer.begin();
  Serial.println("[HTTP] Onboard ESP32 Snapshot Server started on port 80 (GET /capture).");
}

// =========================================================================
// 6. BASELINE FRAME UPDATE & PSRAM CACHE (Every 30 seconds)
// =========================================================================
void captureAndCacheBaseline() {
  if (!isCameraInitialized) return;

  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    digitalWrite(LED_FLASH_PIN, HIGH);
    delay(40);
  #endif

  camera_fb_t * fb = esp_camera_fb_get();

  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    digitalWrite(LED_FLASH_PIN, LOW);
  #endif

  if (!fb) {
    Serial.println("[-] Failed to capture baseline frame.");
    return;
  }

  // Allocate / reallocate buffer in PSRAM if available, or heap
  if (latestBaselineBuf != nullptr) {
    free(latestBaselineBuf);
    latestBaselineBuf = nullptr;
    latestBaselineLen = 0;
  }

  if (psramFound()) {
    latestBaselineBuf = (uint8_t*)ps_malloc(fb->len);
  } else {
    latestBaselineBuf = (uint8_t*)malloc(fb->len);
  }

  if (latestBaselineBuf != nullptr) {
    memcpy(latestBaselineBuf, fb->buf, fb->len);
    latestBaselineLen = fb->len;
  } else {
    Serial.println("[-] Memory allocation failed for baseline frame cache.");
  }

  // Release camera frame buffer immediately
  esp_camera_fb_return(fb);

  // Publish baseline update notification to MQTT
  StaticJsonDocument<384> doc;
  doc["device_id"]       = DEVICE_ID;
  doc["mqtt_client_id"]  = DEVICE_ID;
  doc["rack_id"]         = RACK_ID;
  doc["shelf_id"]        = SHELF_ID;
  doc["event"]           = "BASELINE_UPDATED";
  doc["ip_address"]      = WiFi.localIP().toString();
  doc["baseline_url"]    = "http://" + WiFi.localIP().toString() + "/capture?mode=baseline";
  doc["rssi_dbm"]        = WiFi.RSSI();
  doc["free_heap_bytes"] = ESP.getFreeHeap();
  doc["status"]          = "ONLINE";

  char buffer[384];
  serializeJson(doc, buffer);

  if (mqttClient.publish(baseline_topic, buffer)) {
    Serial.printf("[Baseline Updated] %s -> %s\n", baseline_topic, buffer);
  }
}

// =========================================================================
// 7. LOCALIZED EXTRACTION VERIFICATION & REFERENCE CAPTURE HANDLER
// =========================================================================
void handleCaptureTrigger(const char* expected_action, bool isReferenceCapture = false, int boxCount = 4, bool flashEnabled = true) {
  Serial.printf("\n[+] Trigger Received (%s): Localized snapshot on %s...\n",
                isReferenceCapture ? "REFERENCE_SAMPLE" : "STATE_VALIDATION", SHELF_ID);

  // Flash illumination pulse
  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    if (flashEnabled) {
      digitalWrite(LED_FLASH_PIN, HIGH);
      delay(40);
    }
  #endif

  camera_fb_t * fb = esp_camera_fb_get();

  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    if (flashEnabled) {
      digitalWrite(LED_FLASH_PIN, LOW);
    }
  #endif

  if (!fb) {
    Serial.println("[-] ERROR: Snapshot capture failed!");
    return;
  }

  // Release frame buffer immediately to preserve SRAM (bypasses 520KB constraint)
  esp_camera_fb_return(fb);

  String localIp = WiFi.localIP().toString();

  // Build JSON confirmation payload
  StaticJsonDocument<384> doc;
  doc["device_id"]       = DEVICE_ID;
  doc["shelf_id"]        = SHELF_ID;
  doc["expected_action"] = expected_action;
  doc["box_count"]       = boxCount;
  doc["status_code"]     = 200;
  doc["ip_address"]      = localIp;
  doc["baseline_url"]    = String("http://") + localIp + "/capture?mode=baseline";
  doc["latest_url"]      = String("http://") + localIp + "/capture?mode=live";

  char buffer[384];
  serializeJson(doc, buffer);

  // Publish to: iotlab/storage/SHELF_R1_C1/response
  if (mqttClient.publish(response_topic, buffer)) {
    Serial.printf("[MQTT Published] %s -> %s\n", response_topic, buffer);
  } else {
    Serial.println("[-] Failed to publish verification response to broker.");
  }
}

// =========================================================================
// 8. MQTT INCOMING MESSAGE CALLBACK
// =========================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\n[MQTT Received] Topic: %s (bytes: %u)\n", topic, length);

  StaticJsonDocument<384> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("[-] JSON parse error: %s\n", err.c_str());
    return;
  }

  const char* command = doc["command"] | "";
  const char* target_shelf = doc["shelf_id"] | "";
  const char* expected_action = doc["expected_action"] | "BORROW";

  Serial.printf("    Command: %s | Target Shelf: %s | Action: %s\n", command, target_shelf, expected_action);

  // A. Vision Capture Trigger on iotlab/storage/RACK_01/trigger
  if (strcmp(command, "CAPTURE_STATE") == 0) {
    if (strcmp(SHELF_ID, target_shelf) == 0) {
      handleCaptureTrigger(expected_action, false, 4, true);
    }
    return;
  }

  // B. Reference Sample Capture on iotlab/storage/node/ESP32_R1_C1/config
  if (strcmp(command, "CAPTURE_REFERENCE") == 0) {
    int boxCount = doc["box_count"] | 4;
    bool flashEnabled = doc["flash_enabled"] | true;
    handleCaptureTrigger("REFERENCE", true, boxCount, flashEnabled);
    return;
  }
}

// =========================================================================
// 9. ASYNCHRONOUS HEALTH & TELEMETRY PING (Every 30s - NFR-08)
// =========================================================================
void publishHealthTelemetry(const char* event) {
  String localIp = WiFi.localIP().toString();
  StaticJsonDocument<384> doc;
  doc["device_id"]        = DEVICE_ID;
  doc["mqtt_client_id"]   = DEVICE_ID;
  doc["camera_name"]      = CAMERA_NAME;
  doc["rack_id"]          = RACK_ID;
  doc["shelf_id"]         = SHELF_ID;
  doc["ip_address"]       = localIp;
  doc["snapshot_url"]     = String("http://") + localIp + "/capture";
  doc["mac_address"]      = WiFi.macAddress();
  doc["firmware_version"] = FIRMWARE_VER;
  doc["rssi_dbm"]         = WiFi.RSSI();
  doc["free_heap_bytes"]  = ESP.getFreeHeap();
  doc["status"]           = "ONLINE";
  if (event != nullptr) {
    doc["event"] = event;
  }

  char buffer[384];
  serializeJson(doc, buffer);

  if (mqttClient.publish(HEALTH_TOPIC, buffer)) {
    Serial.printf("[Health Heartbeat] %s -> %s\n", HEALTH_TOPIC, buffer);
  }
}

// =========================================================================
// 10. WI-FI & MQTT CONNECTION WATCHDOG
// =========================================================================
void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.printf("[WiFi] Connecting to %s ", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 25) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.printf("       IP: %s | Snapshot URL: http://%s/capture\n",
                  WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[-] [WiFi] Connection timed out. Will retry.");
  }
}

void ensureMQTTConnected() {
  if (mqttClient.connected()) return;

  if (WiFi.status() != WL_CONNECTED) {
    ensureWiFiConnected();
    if (WiFi.status() != WL_CONNECTED) return;
  }

  unsigned long now = millis();
  if (now - lastMqttRetryTime >= 10000 || lastMqttRetryTime == 0) {
    lastMqttRetryTime = now;
    Serial.printf("[MQTT] Connecting to broker %s:%d...\n", MQTT_BROKER_IP, MQTT_PORT);
    if (mqttClient.connect(DEVICE_ID)) {
      Serial.println("[MQTT] Connected successfully!");
      mqttClient.subscribe(trigger_topic);
      mqttClient.subscribe(config_topic);
      Serial.printf("[MQTT Subscribed] Trigger: %s | Reference Config: %s\n", trigger_topic, config_topic);
      publishHealthTelemetry("BOOT_CONNECT");
    } else {
      Serial.printf("[-] MQTT connect failed, state=%d. Will retry in 10s in background...\n", mqttClient.state());
    }
  }
}

// =========================================================================
// 11. SETUP & MAIN LOOP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=======================================================");
  Serial.println("  SMART STORAGE FYP - ESP32 CAMERA HARDWARE RELAY NODE");
  Serial.printf("  Device: %s | Shelf: %s | Rack: %s\n", DEVICE_ID, SHELF_ID, RACK_ID);
  Serial.println("=======================================================");

  isCameraInitialized = initCamera();
  if (!isCameraInitialized) {
    Serial.println("[-] WARNING: Camera sensor not responding. Check ribbon cable connection.");
  }

  ensureWiFiConnected();
  setupHttpServer();

  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(768);

  ensureMQTTConnected();
  captureAndCacheBaseline();
  lastHeartbeatTime = millis();
}

void loop() {
  httpServer.handleClient();

  if (!mqttClient.connected()) {
    ensureMQTTConnected();
  }
  mqttClient.loop();

  // 30-Second Baseline Cache Update & Health Heartbeat
  unsigned long now = millis();
  if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatTime = now;
    publishHealthTelemetry();
    captureAndCacheBaseline();
  }
}
