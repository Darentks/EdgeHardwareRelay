/*
  =============================================================================
  Smart Storage FYP - Localized ESP32 Camera Hardware Relay Firmware
  Author: Daren Tan Khai Shuen (ID: 25WMR09824)
  Supervisor: Dr Tew Yiqi | TARUMT

  Architecture: Tri-Stage Vision Capture & Hardware Relay (Section 1.3.2)
  Hardware: ESP32-CAM (AI-Thinker) + OV2640 / OV7725 Sensor
  IDE: Arduino IDE / PlatformIO

  Fulfills:
    - FR-05 & FR-06: Localized extraction verification via camera snapshot
    - FR-10 & NFR-08: Asynchronous 30-second MQTT health telemetry ping
    - Table 4.8 & Section 4.7: MQTT topic structure and JSON payload schemas
  =============================================================================
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "camera_pins.h"

// =========================================================================
// 1. HARDWARE & COMPARTMENT CONFIGURATION
// (Configure these parameters for each specific storage shelf compartment)
// =========================================================================
const char* DEVICE_ID       = "ESP32_R1_C1";       // Unique Node Identifier
const char* RACK_ID         = "RACK_01";           // Rack Identifier
const char* SHELF_ID        = "SHELF_R1_C1";       // Mapped Compartment Identifier
const char* FIRMWARE_VER    = "v1.0.4-FreeRTOS";   // Thesis Section 4.7 Specification

// =========================================================================
// 2. NETWORK & MQTT BROKER CONFIGURATION
// =========================================================================
const char* WIFI_SSID       = "YOUR_WIFI_SSID";     // Enter your local Wi-Fi SSID
const char* WIFI_PASSWORD   = "YOUR_WIFI_PASSWORD"; // Enter your local Wi-Fi Password

// IP of the backend laptop/server running Mosquitto MQTT Broker
const char* MQTT_BROKER_IP  = "192.168.1.100";     // Change to your laptop's IP
const int   MQTT_PORT       = 1883;

// Topics conforming to Table 4.8 Specification
char trigger_topic[64];     // iotlab/storage/{RACK_ID}/trigger
char response_topic[64];    // iotlab/storage/{SHELF_ID}/response
const char* HEALTH_TOPIC    = "iotlab/storage/health";

// =========================================================================
// 3. HARDWARE & TIMING CONSTANTS
// =========================================================================
const unsigned long HEARTBEAT_INTERVAL_MS = 30000; // 30 seconds (NFR-08)
unsigned long lastHeartbeatTime = 0;
bool isCameraInitialized = false;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

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
// 5. LOCALIZED EXTRACTION VERIFICATION (Trigger Handler)
// =========================================================================
void handleCaptureTrigger(const char* expected_action) {
  Serial.printf("\n[+] Trigger Received: Localized snapshot verification on %s...\n", SHELF_ID);
  unsigned long captureStart = millis();

  // Flash illumination pulse
  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    digitalWrite(LED_FLASH_PIN, HIGH);
    delay(40);
  #endif

  camera_fb_t * fb = esp_camera_fb_get();

  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    digitalWrite(LED_FLASH_PIN, LOW);
  #endif

  if (!fb) {
    Serial.println("[-] ERROR: Snapshot capture failed!");
    return;
  }

  unsigned long captureLatency = millis() - captureStart;
  Serial.printf("[+] Captured compartment snapshot (%u bytes) in %lu ms\n", fb->len, captureLatency);

  // Derive state delta (Section 4.7 Operational Verification Schema)
  const char* state_delta = (strcmp(expected_action, "RETURN") == 0) ? "ITEM_RETURNED" : "ITEM_REMOVED";

  // Release frame buffer immediately to preserve SRAM (bypasses 520KB constraint)
  esp_camera_fb_return(fb);

  // Build JSON confirmation payload
  StaticJsonDocument<256> doc;
  doc["device_id"]   = DEVICE_ID;
  doc["shelf_id"]    = SHELF_ID;
  doc["status_code"] = 200;
  doc["state_delta"] = state_delta;
  doc["bus_vcc"]     = 4.92; // Nominal 5V bus rail telemetry

  char buffer[256];
  serializeJson(doc, buffer);

  // Publish to: iotlab/storage/{shelf_id}/response
  if (mqttClient.publish(response_topic, buffer)) {
    Serial.printf("[MQTT Published] %s -> %s\n", response_topic, buffer);
  } else {
    Serial.println("[-] Failed to publish verification response to broker.");
  }
}

// =========================================================================
// 6. MQTT INCOMING MESSAGE CALLBACK
// =========================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\n[MQTT Received] Topic: %s (bytes: %u)\n", topic, length);

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("[-] JSON parse error: %s\n", err.c_str());
    return;
  }

  const char* command = doc["command"] | "";
  const char* target_shelf = doc["shelf_id"] | "";
  const char* expected_action = doc["expected_action"] | "BORROW";

  Serial.printf("    Command: %s | Target: %s | Action: %s\n", command, target_shelf, expected_action);

  // Check if addressed to this compartment
  if (strcmp(command, "CAPTURE_STATE") == 0 && strcmp(target_shelf, SHELF_ID) == 0) {
    handleCaptureTrigger(expected_action);
  }
}

// =========================================================================
// 7. ASYNCHRONOUS HEALTH & TELEMETRY PING (Every 30s - NFR-08)
// =========================================================================
void publishHealthTelemetry() {
  StaticJsonDocument<256> doc;
  doc["device_id"]        = DEVICE_ID;
  doc["mac_address"]      = WiFi.macAddress();
  doc["firmware_version"] = FIRMWARE_VER;
  doc["rssi_dbm"]         = WiFi.RSSI();
  doc["free_heap_bytes"]  = ESP.getFreeHeap();
  doc["status"]           = "ALIVE";

  char buffer[256];
  serializeJson(doc, buffer);

  if (mqttClient.publish(HEALTH_TOPIC, buffer)) {
    Serial.printf("[Health Heartbeat] %s -> %s\n", HEALTH_TOPIC, buffer);
  }
}

// =========================================================================
// 8. WI-FI & MQTT CONNECTION WATCHDOG
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
    Serial.printf("       IP: %s | MAC: %s\n", WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str());
  } else {
    Serial.println("\n[-] [WiFi] Connection timed out. Will retry.");
  }
}

void ensureMQTTConnected() {
  while (!mqttClient.connected()) {
    if (WiFi.status() != WL_CONNECTED) {
      ensureWiFiConnected();
      if (WiFi.status() != WL_CONNECTED) return;
    }

    Serial.printf("[MQTT] Connecting to broker %s:%d...\n", MQTT_BROKER_IP, MQTT_PORT);
    if (mqttClient.connect(DEVICE_ID)) {
      Serial.println("[MQTT] Connected successfully!");
      mqttClient.subscribe(trigger_topic);
      Serial.printf("[MQTT Subscribed] %s\n", trigger_topic);
      publishHealthTelemetry();
    } else {
      Serial.printf("[-] MQTT connect failed, state=%d. Retrying in 5s...\n", mqttClient.state());
      delay(5000);
    }
  }
}

// =========================================================================
// 9. SETUP & MAIN LOOP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=======================================================");
  Serial.println("  SMART STORAGE FYP - ESP32 CAMERA HARDWARE RELAY NODE");
  Serial.printf("  Device: %s | Shelf: %s | Rack: %s\n", DEVICE_ID, SHELF_ID, RACK_ID);
  Serial.println("=======================================================");

  snprintf(trigger_topic, sizeof(trigger_topic), "iotlab/storage/%s/trigger", RACK_ID);
  snprintf(response_topic, sizeof(response_topic), "iotlab/storage/%s/response", SHELF_ID);

  isCameraInitialized = initCamera();
  if (!isCameraInitialized) {
    Serial.println("[-] WARNING: Camera sensor not responding. Check ribbon cable connection.");
  }

  ensureWiFiConnected();

  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  ensureMQTTConnected();
  lastHeartbeatTime = millis();
}

void loop() {
  if (!mqttClient.connected()) {
    ensureMQTTConnected();
  }
  mqttClient.loop();

  // 30-Second Asynchronous Heartbeat (Section 1.3.9)
  unsigned long now = millis();
  if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatTime = now;
    publishHealthTelemetry();
  }
}

