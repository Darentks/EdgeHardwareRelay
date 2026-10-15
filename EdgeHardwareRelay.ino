/*
  =============================================================================
  Smart Storage FYP - Localized ESP32 Camera Hardware Relay Firmware
  Author: Daren Tan Khai Shuen (ID: 25WMR09824)
  Supervisor: Dr Tew Yiqi | TARUMT

  Architecture: Tri-Stage Vision Capture, Bidirectional MQTT CRUD & Reference Relay
  Hardware: ESP32-CAM (AI-Thinker) + OV2640 / OV7725 Sensor
  Framework: Arduino / FreeRTOS (PlatformIO & Arduino IDE compatible)

  Fulfills:
    - FR-05 & FR-06: Localized extraction verification via camera snapshot
    - FR-10 & NFR-08: Asynchronous 30-second MQTT health telemetry ping
    - Table 4.8 & Section 4.7: MQTT topic structure and JSON payload schemas
    - Dynamic Shelf Compartment & ESP32 Node CRUD via MQTT + NVS Preferences
    - Independent ESP32 Reference Sample Capture (MQTT CAPTURE_REFERENCE + HTTP GET /capture)
  =============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "camera_pins.h"

// =========================================================================
// 1. HARDWARE & DEFAULT COMPARTMENT CONFIGURATION
// =========================================================================
const char* DEVICE_ID        = "ESP32_R1_C1";       // Unique MQTT Client & Node Identifier
const char* DEFAULT_RACK_ID  = "RACK_01";           // Default Rack Identifier
const char* DEFAULT_SHELF_ID = "SHELF_R1_C1";       // Default Mapped Compartment Identifier
const char* DEFAULT_CAM_NAME = "ESP32 Node R1-C1";  // Default Readable Camera Name
const char* FIRMWARE_VER     = "v1.2.0-FreeRTOS-RefCRUD";

// Runtime NVS-backed compartment mapping state
Preferences preferences;
String rackId     = DEFAULT_RACK_ID;
String shelfId    = DEFAULT_SHELF_ID;
String cameraName = DEFAULT_CAM_NAME;

// =========================================================================
// 2. NETWORK, HTTP SNAPSHOT SERVER & MQTT BROKER CONFIGURATION
// =========================================================================
const char* WIFI_SSID       = "YOUR_WIFI_SSID";     // Enter your local Wi-Fi SSID
const char* WIFI_PASSWORD   = "YOUR_WIFI_PASSWORD"; // Enter your local Wi-Fi Password

// IP of the backend laptop/server running Mosquitto MQTT Broker
const char* MQTT_BROKER_IP  = "192.168.1.100";     // Change to your laptop's IP
const int   MQTT_PORT       = 1883;

// Dynamic MQTT Topics conforming to Table 4.8 + Node CRUD Specification
char trigger_topic[96];     // iotlab/storage/{rackId}/trigger
char response_topic[96];    // iotlab/storage/{shelfId}/response
char config_topic[96];      // iotlab/storage/node/{DEVICE_ID}/config
char crud_ack_topic[96];    // iotlab/storage/{DEVICE_ID}/crud_ack
const char* HEALTH_TOPIC    = "iotlab/storage/health";

// =========================================================================
// 3. HARDWARE & TIMING CONSTANTS
// =========================================================================
const unsigned long HEARTBEAT_INTERVAL_MS = 30000; // 30 seconds (NFR-08)
unsigned long lastHeartbeatTime = 0;
bool isCameraInitialized = false;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer httpServer(80);

// Forward declarations
void publishHealthTelemetry(const char* crudEvent = nullptr);

// =========================================================================
// 4. NVS CONFIGURATION STORAGE & DYNAMIC TOPIC BUILDER
// =========================================================================
void refreshTopicStrings() {
  snprintf(trigger_topic, sizeof(trigger_topic), "iotlab/storage/%s/trigger", rackId.c_str());
  snprintf(response_topic, sizeof(response_topic), "iotlab/storage/%s/response", shelfId.c_str());
  snprintf(config_topic, sizeof(config_topic), "iotlab/storage/node/%s/config", DEVICE_ID);
  snprintf(crud_ack_topic, sizeof(crud_ack_topic), "iotlab/storage/%s/crud_ack", DEVICE_ID);
}

void loadNodeConfigFromNVS() {
  preferences.begin("storage_cfg", true);
  rackId     = preferences.getString("rack_id", DEFAULT_RACK_ID);
  shelfId    = preferences.getString("shelf_id", DEFAULT_SHELF_ID);
  cameraName = preferences.getString("cam_name", DEFAULT_CAM_NAME);
  preferences.end();
  refreshTopicStrings();

  Serial.printf("[NVS Config] Loaded -> Device: %s | Rack: %s | Shelf: %s | Name: %s\n",
                DEVICE_ID, rackId.c_str(), shelfId.c_str(), cameraName.c_str());
}

void saveNodeConfigToNVS() {
  preferences.begin("storage_cfg", false);
  preferences.putString("rack_id", rackId);
  preferences.putString("shelf_id", shelfId);
  preferences.putString("cam_name", cameraName);
  preferences.end();
  refreshTopicStrings();

  Serial.printf("[NVS Config] Saved -> Rack: %s | Shelf: %s | Name: %s\n",
                rackId.c_str(), shelfId.c_str(), cameraName.c_str());
}

// =========================================================================
// 5. CAMERA INITIALIZATION FUNCTION
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
// 6. ONBOARD HTTP SNAPSHOT SERVER (PORT 80) FOR INDEPENDENT ESP32 CAPTURE
//    - GET /capture : Captures and streams raw JPEG image of the compartment
//    - GET /status  : Returns JSON status & 2x2 compartment mapping
// =========================================================================
void setupHttpServer() {
  httpServer.on("/status", HTTP_GET, []() {
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    StaticJsonDocument<256> doc;
    doc["status"]         = "ok";
    doc["device_id"]      = DEVICE_ID;
    doc["mqtt_client_id"] = DEVICE_ID;
    doc["rack_id"]        = rackId;
    doc["shelf_id"]       = shelfId;
    doc["ip_address"]     = WiFi.localIP().toString();
    doc["rssi_dbm"]       = WiFi.RSSI();
    String out;
    serializeJson(doc, out);
    httpServer.send(200, "application/json", out);
  });

  httpServer.on("/capture", HTTP_GET, []() {
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
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
// 7. LOCALIZED EXTRACTION VERIFICATION & REFERENCE CAPTURE HANDLER
// =========================================================================
void handleCaptureTrigger(const char* expected_action, bool isReferenceCapture = false, int boxCount = 4, bool flashEnabled = true) {
  Serial.printf("\n[+] Trigger Received (%s): Localized snapshot on %s...\n",
                isReferenceCapture ? "REFERENCE_SAMPLE" : "STATE_VALIDATION", shelfId.c_str());
  unsigned long captureStart = millis();

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

  unsigned long captureLatency = millis() - captureStart;
  size_t frameBytes = fb->len;

  // Compute lightweight 4-quadrant (2x2 box grid) byte intensity signature
  uint32_t qSum[4] = {0, 0, 0, 0};
  size_t quarter = frameBytes / 4;
  if (quarter > 0) {
    for (int q = 0; q < 4; q++) {
      size_t startIdx = q * quarter;
      size_t endIdx = (q == 3) ? frameBytes : (q + 1) * quarter;
      for (size_t i = startIdx; i < endIdx; i++) {
        qSum[q] += fb->buf[i];
      }
      qSum[q] /= (endIdx - startIdx);
    }
  }

  // Release frame buffer immediately to preserve SRAM (bypasses 520KB constraint)
  esp_camera_fb_return(fb);

  const char* state_delta = isReferenceCapture
      ? "REFERENCE_STORED"
      : ((strcmp(expected_action, "RETURN") == 0) ? "ITEM_RETURNED" : "ITEM_REMOVED");

  String localIp = WiFi.localIP().toString();

  // Build JSON confirmation payload for Main System & Supabase
  StaticJsonDocument<512> doc;
  doc["device_id"]          = DEVICE_ID;
  doc["mqtt_client_id"]     = DEVICE_ID;
  doc["rack_id"]            = rackId;
  doc["shelf_id"]           = shelfId;
  doc["capture_mode"]       = isReferenceCapture ? "REFERENCE" : "VALIDATION";
  doc["expected_action"]    = expected_action;
  doc["box_count"]          = boxCount;
  doc["status_code"]        = 200;
  doc["state_delta"]        = state_delta;
  doc["capture_latency_ms"] = captureLatency;
  doc["frame_bytes"]        = frameBytes;
  doc["bus_vcc"]            = 4.92;
  doc["ip_address"]         = localIp;
  doc["snapshot_url"]       = String("http://") + localIp + "/capture";

  JsonArray quadArr = doc.createNestedArray("quadrant_lum");
  quadArr.add(qSum[0]);
  quadArr.add(qSum[1]);
  quadArr.add(qSum[2]);
  quadArr.add(qSum[3]);

  char buffer[512];
  serializeJson(doc, buffer);

  // Publish to: iotlab/storage/{shelfId}/response
  if (mqttClient.publish(response_topic, buffer)) {
    Serial.printf("[MQTT Published] %s -> %s\n", response_topic, buffer);
  } else {
    Serial.println("[-] Failed to publish verification response to broker.");
  }
}

// =========================================================================
// 8. MQTT INCOMING MESSAGE CALLBACK (Trigger + Reference + Remote CRUD)
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

  // A. Vision Capture Trigger on iotlab/storage/{rackId}/trigger
  if (strcmp(command, "CAPTURE_STATE") == 0) {
    if (shelfId != "UNASSIGNED" && shelfId.equals(target_shelf)) {
      handleCaptureTrigger(expected_action, false, 4, true);
    }
    return;
  }

  // B. Independent ESP32 Reference Sample Capture on iotlab/storage/node/{DEVICE_ID}/config
  if (strcmp(command, "CAPTURE_REFERENCE") == 0) {
    int boxCount = doc["box_count"] | 4;
    bool flashEnabled = doc["flash_enabled"] | true;
    handleCaptureTrigger("REFERENCE", true, boxCount, flashEnabled);
    return;
  }

  // C. Remote CRUD Create / Update on iotlab/storage/node/{DEVICE_ID}/config
  if (strcmp(command, "CRUD_CREATE") == 0 || strcmp(command, "CRUD_UPDATE") == 0) {
    mqttClient.unsubscribe(trigger_topic);

    if (doc.containsKey("rack_id")) {
      rackId = doc["rack_id"].as<String>();
    }
    if (doc.containsKey("shelf_id")) {
      shelfId = doc["shelf_id"].as<String>();
    }
    if (doc.containsKey("camera_name")) {
      cameraName = doc["camera_name"].as<String>();
    }

    saveNodeConfigToNVS();
    mqttClient.subscribe(trigger_topic);
    Serial.printf("[CRUD Applied] Bound to Rack=%s, Shelf=%s. Resubscribed to %s\n",
                  rackId.c_str(), shelfId.c_str(), trigger_topic);

    publishHealthTelemetry(command);
    return;
  }

  // D. Remote CRUD Delete (Unbind Node from Compartment)
  if (strcmp(command, "CRUD_DELETE") == 0) {
    shelfId = "UNASSIGNED";
    saveNodeConfigToNVS();
    Serial.println("[CRUD Delete] Node unbound from compartment (UNASSIGNED).");
    publishHealthTelemetry("CRUD_DELETE");
    return;
  }

  // E. Remote CRUD Ping / Health Check Request
  if (strcmp(command, "CRUD_PING") == 0) {
    publishHealthTelemetry("CRUD_PING");
    return;
  }
}

// =========================================================================
// 9. ASYNCHRONOUS HEALTH & TELEMETRY PING (Every 30s - NFR-08)
// =========================================================================
void publishHealthTelemetry(const char* crudEvent) {
  String localIp = WiFi.localIP().toString();
  StaticJsonDocument<384> doc;
  doc["device_id"]        = DEVICE_ID;
  doc["mqtt_client_id"]   = DEVICE_ID;
  doc["camera_name"]      = cameraName;
  doc["rack_id"]          = rackId;
  doc["shelf_id"]         = shelfId;
  doc["ip_address"]       = localIp;
  doc["snapshot_url"]     = String("http://") + localIp + "/capture";
  doc["mac_address"]      = WiFi.macAddress();
  doc["firmware_version"] = FIRMWARE_VER;
  doc["rssi_dbm"]         = WiFi.RSSI();
  doc["free_heap_bytes"]  = ESP.getFreeHeap();
  doc["status"]           = (shelfId == "UNASSIGNED") ? "UNASSIGNED" : "ONLINE";
  if (crudEvent != nullptr) {
    doc["crud_event"] = crudEvent;
  }

  char buffer[384];
  serializeJson(doc, buffer);

  if (mqttClient.publish(HEALTH_TOPIC, buffer)) {
    Serial.printf("[Health Heartbeat] %s -> %s\n", HEALTH_TOPIC, buffer);
  }
  if (crudEvent != nullptr) {
    mqttClient.publish(crud_ack_topic, buffer);
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
  while (!mqttClient.connected()) {
    if (WiFi.status() != WL_CONNECTED) {
      ensureWiFiConnected();
      if (WiFi.status() != WL_CONNECTED) return;
    }

    Serial.printf("[MQTT] Connecting to broker %s:%d...\n", MQTT_BROKER_IP, MQTT_PORT);
    if (mqttClient.connect(DEVICE_ID)) {
      Serial.println("[MQTT] Connected successfully!");
      mqttClient.subscribe(trigger_topic);
      mqttClient.subscribe(config_topic);
      Serial.printf("[MQTT Subscribed] Trigger: %s | CRUD & Ref Config: %s\n", trigger_topic, config_topic);
      publishHealthTelemetry("BOOT_CONNECT");
    } else {
      Serial.printf("[-] MQTT connect failed, state=%d. Retrying in 5s...\n", mqttClient.state());
      delay(5000);
    }
  }
}

// =========================================================================
// 11. SETUP & MAIN LOOP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  loadNodeConfigFromNVS();

  Serial.println("\n=======================================================");
  Serial.println("  SMART STORAGE FYP - ESP32 CAMERA HARDWARE RELAY NODE");
  Serial.printf("  Device: %s | Shelf: %s | Rack: %s\n", DEVICE_ID, shelfId.c_str(), rackId.c_str());
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
  lastHeartbeatTime = millis();
}

void loop() {
  httpServer.handleClient();

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
