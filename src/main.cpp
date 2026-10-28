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
    - Dynamic NVS Provisioning (Preferences.h) + HTTP Push OTA (Update.h)
  =============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Update.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =========================================================================
// 1. HARDWARE & COMPARTMENT CONFIGURATION (Build Defaults + NVS Overrides)
// =========================================================================
#ifndef CUSTOM_DEVICE_ID
#define CUSTOM_DEVICE_ID "ESP32_R1_C1_CAM1"
#endif
#ifndef CUSTOM_SHELF_ID
#define CUSTOM_SHELF_ID "SHELF_R1_C1"
#endif
#ifndef CUSTOM_IP_OCTET
#define CUSTOM_IP_OCTET 201
#endif

Preferences prefs;
String deviceId   = CUSTOM_DEVICE_ID;
String rackId     = "RACK_01";
String shelfId    = CUSTOM_SHELF_ID;
String cameraName = CUSTOM_DEVICE_ID;
int    ipOctet    = CUSTOM_IP_OCTET;

const char* FIRMWARE_VER = "v1.5.0-NVS-HTTP-OTA";

// =========================================================================
// 2. NETWORK, HTTP SNAPSHOT SERVER & MQTT BROKER CONFIGURATION
// =========================================================================
const char* WIFI_SSID       = "B100A";     // Enter your local Wi-Fi SSID
const char* WIFI_PASSWORD   = "12345678"; // Enter your local Wi-Fi Password

// IP of the backend laptop/server running Mosquitto MQTT Broker
const char* MQTT_BROKER_IP  = "192.168.0.100";     // Change to your laptop's IP
const int   MQTT_PORT       = 1883;

// Dynamic MQTT Topics built from rackId, shelfId, and deviceId
String trigger_topic;
String response_topic;
String baseline_topic;
String config_topic;
const char* HEALTH_TOPIC    = "iotlab/storage/health";

const int FLASH_BRIGHTNESS   = 12; // PWM 0..255 (12 = ~5% low-current soft flash to prevent 5V rail voltage dips & DMA hangs)
const int FLASH_LEDC_CHANNEL = 7;  // Dedicated LEDC channel 7 (Timer 3) so flash PWM never overwrites Camera XCLK on Channel 0 (Timer 0)

// =========================================================================
// 3. HARDWARE, TIMING & PSRAM BASELINE BUFFER CONSTANTS
// =========================================================================
const unsigned long HEARTBEAT_INTERVAL_MS = 30000; // 30 seconds (NFR-08)
const unsigned long POWERBANK_KEEPALIVE_MS = 8000; // 8-second keep-alive pulse
unsigned long lastHeartbeatTime = 0;
unsigned long lastMqttRetryTime = 0;
unsigned long lastKeepAliveTime = 0;
bool isCameraInitialized = false;

// Cached baseline & triggered capture JPEG frame buffers in PSRAM
uint8_t* latestBaselineBuf = nullptr;
size_t   latestBaselineLen = 0;
uint8_t* latestCaptureBuf  = nullptr;
size_t   latestCaptureLen  = 0;

// Baseline freeze window so 30s periodic updates don't overwrite the "before" state during active operations
const unsigned long BASELINE_COOLDOWN_MS = 15000;
unsigned long baselinePauseUntil = 0;
bool isBaselineFrozen = false;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer httpServer(80);

// Forward declarations
void publishHealthTelemetry(const char* event = nullptr);
void captureAndCacheBaseline();

void rebuildMqttTopics() {
  trigger_topic  = String("iotlab/storage/") + rackId + "/trigger";
  response_topic = String("iotlab/storage/") + shelfId + "/response";
  baseline_topic = String("iotlab/storage/") + shelfId + "/baseline";
  config_topic   = String("iotlab/storage/node/") + deviceId + "/config";
}

void loadConfigFromNVS() {
  prefs.begin("cam_cfg", true);
  deviceId   = prefs.getString("device_id", CUSTOM_DEVICE_ID);
  rackId     = prefs.getString("rack_id", "RACK_01");
  shelfId    = prefs.getString("shelf_id", CUSTOM_SHELF_ID);
  cameraName = prefs.getString("camera_name", deviceId);
  ipOctet    = prefs.getInt("ip_octet", CUSTOM_IP_OCTET);
  prefs.end();

  if (ipOctet < 2 || ipOctet > 254) {
    ipOctet = CUSTOM_IP_OCTET;
  }
  rebuildMqttTopics();
}

// =========================================================================
// 4. CAMERA INITIALIZATION FUNCTION
// =========================================================================
bool initCamera() {
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  Serial.printf("[PSRAM] psramFound=%d | Total=%u bytes | Free=%u bytes\n",
                psramFound(), ESP.getPsramSize(), ESP.getFreePsram());

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
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320x240 fallback if no PSRAM
    config.jpeg_quality = 16;
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[-] Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
  }

  // Flush 3 warm-up frames so OV2640 Auto White Balance (AWB) settles on boot
  for (int i = 0; i < 3; i++) {
    camera_fb_t* warm = esp_camera_fb_get();
    if (warm) esp_camera_fb_return(warm);
    delay(80);
  }

  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    ledcSetup(FLASH_LEDC_CHANNEL, 5000, 8);
    ledcAttachPin(LED_FLASH_PIN, FLASH_LEDC_CHANNEL);
    ledcWrite(FLASH_LEDC_CHANNEL, 0);
  #endif

  Serial.println("[+] Camera hardware initialized successfully.");
  return true;
}

// Helper to flush DMA ring buffer and grab a fresh real-time frame
camera_fb_t* captureFreshFrame(bool useFlash) {
  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    if (useFlash) {
      ledcWrite(FLASH_LEDC_CHANNEL, FLASH_BRIGHTNESS);
      delay(120); // Let OV2640 auto-exposure settle under soft flash light
    }
  #endif

  // Flush stale frame from DMA buffer while LED is already illuminating the compartment
  camera_fb_t* stale = esp_camera_fb_get();
  if (stale) {
    esp_camera_fb_return(stale);
  }
  delay(30);

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    delay(100);
    fb = esp_camera_fb_get();
  }

  #if defined(LED_FLASH_PIN) && LED_FLASH_PIN >= 0
    if (useFlash) {
      ledcWrite(FLASH_LEDC_CHANNEL, 0);
    }
  #endif

  return fb;
}

// =========================================================================
// 5. ONBOARD HTTP SNAPSHOT, NVS CONFIG & OTA SERVER (PORT 80)
// =========================================================================
void sendCorsHeaders() {
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.sendHeader("Access-Control-Allow-Private-Network", "true");
  httpServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  httpServer.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
  httpServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpServer.sendHeader("Pragma", "no-cache");
}

void sendBinaryResponse(const char* contentType, const uint8_t* buf, size_t len) {
  httpServer.setContentLength(len);
  httpServer.send(200, contentType, "");
  httpServer.client().write(buf, len);
}

void handleCorsPreflight() {
  sendCorsHeaders();
  httpServer.send(204);
}

void setupHttpServer() {
  // CORS Preflight OPTIONS routes
  httpServer.on("/status", HTTP_OPTIONS, handleCorsPreflight);
  httpServer.on("/capture", HTTP_OPTIONS, handleCorsPreflight);
  httpServer.on("/config", HTTP_OPTIONS, handleCorsPreflight);
  httpServer.on("/update", HTTP_OPTIONS, handleCorsPreflight);

  httpServer.on("/status", HTTP_GET, []() {
    sendCorsHeaders();
    StaticJsonDocument<320> doc;
    doc["status"]         = "ok";
    doc["device_id"]      = deviceId;
    doc["mqtt_client_id"] = deviceId;
    doc["rack_id"]        = rackId;
    doc["shelf_id"]       = shelfId;
    doc["camera_name"]    = cameraName;
    doc["ip_octet"]       = ipOctet;
    doc["ip_address"]     = WiFi.localIP().toString();
    doc["rssi_dbm"]       = WiFi.RSSI();
    doc["firmware_ver"]   = FIRMWARE_VER;
    String out;
    serializeJson(doc, out);
    httpServer.send(200, "application/json", out);
  });

  httpServer.on("/capture", HTTP_GET, []() {
    sendCorsHeaders();

    String mode = httpServer.arg("mode");

    // 1. Return cached baseline frame if mode=baseline
    if (mode == "baseline" && latestBaselineBuf != nullptr && latestBaselineLen > 0) {
      httpServer.sendHeader("Content-Disposition", "inline; filename=esp32_baseline.jpg");
      sendBinaryResponse("image/jpeg", latestBaselineBuf, latestBaselineLen);
      return;
    }

    // 2. Return cached triggered snapshot if mode=live or mode=triggered (avoids double flash/capture)
    if ((mode == "live" || mode == "triggered") && latestCaptureBuf != nullptr && latestCaptureLen > 0) {
      httpServer.sendHeader("Content-Disposition", "inline; filename=esp32_capture.jpg");
      sendBinaryResponse("image/jpeg", latestCaptureBuf, latestCaptureLen);
      return;
    }

    // 3. Otherwise, capture a fresh live JPEG frame (soft flash on by default for dark rack)
    bool useFlash = true;
    if (httpServer.hasArg("flash") && httpServer.arg("flash") == "0") {
      useFlash = false;
    }

    camera_fb_t * fb = captureFreshFrame(useFlash);
    if (!fb) {
      delay(100);
      fb = captureFreshFrame(useFlash);
    }

    if (!fb) {
      httpServer.send(500, "application/json", "{\"error\":\"Snapshot capture failed\"}");
      return;
    }

    httpServer.sendHeader("Content-Disposition", "inline; filename=esp32_capture.jpg");
    sendBinaryResponse("image/jpeg", fb->buf, fb->len);
    esp_camera_fb_return(fb);
  });

  // Runtime NVS Configuration Endpoint (POST /config)
  httpServer.on("/config", HTTP_POST, []() {
    sendCorsHeaders();
    if (!httpServer.hasArg("plain")) {
      httpServer.send(400, "application/json", "{\"error\":\"Missing JSON body\"}");
      return;
    }

    StaticJsonDocument<384> doc;
    DeserializationError err = deserializeJson(doc, httpServer.arg("plain"));
    if (err) {
      httpServer.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }

    prefs.begin("cam_cfg", false);
    if (doc.containsKey("device_id")) {
      deviceId = doc["device_id"].as<String>();
      prefs.putString("device_id", deviceId);
    }
    if (doc.containsKey("rack_id")) {
      rackId = doc["rack_id"].as<String>();
      prefs.putString("rack_id", rackId);
    }
    if (doc.containsKey("shelf_id")) {
      shelfId = doc["shelf_id"].as<String>();
      prefs.putString("shelf_id", shelfId);
    }
    if (doc.containsKey("camera_name")) {
      cameraName = doc["camera_name"].as<String>();
      prefs.putString("camera_name", cameraName);
    }
    if (doc.containsKey("ip_octet")) {
      int newOctet = doc["ip_octet"].as<int>();
      if (newOctet >= 2 && newOctet <= 254) {
        ipOctet = newOctet;
        prefs.putInt("ip_octet", ipOctet);
      }
    }
    prefs.end();
    rebuildMqttTopics();

    StaticJsonDocument<320> res;
    res["status"]      = "ok";
    res["message"]     = "Configuration saved to NVS. Rebooting...";
    res["device_id"]   = deviceId;
    res["rack_id"]     = rackId;
    res["shelf_id"]    = shelfId;
    res["camera_name"] = cameraName;
    res["ip_octet"]    = ipOctet;
    res["new_ip"]      = String("192.168.0.") + String(ipOctet);

    String out;
    serializeJson(res, out);
    httpServer.send(200, "application/json", out);

    delay(500);
    ESP.restart();
  });

  // HTTP Push OTA Firmware Update Endpoint (POST /update)
  httpServer.on("/update", HTTP_POST, []() {
    sendCorsHeaders();
    httpServer.sendHeader("Connection", "close");
    bool ok = !Update.hasError();
    httpServer.send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FAIL");
    delay(200);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = httpServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("[HTTP OTA] Receiving firmware: %s\n", upload.filename.c_str());
      mqttClient.disconnect();
      if (latestBaselineBuf != nullptr) {
        free(latestBaselineBuf);
        latestBaselineBuf = nullptr;
        latestBaselineLen = 0;
      }
      if (latestCaptureBuf != nullptr) {
        free(latestCaptureBuf);
        latestCaptureBuf = nullptr;
        latestCaptureLen = 0;
      }
      if (isCameraInitialized) {
        esp_camera_deinit();
        isCameraInitialized = false;
      }
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("[HTTP OTA] Update Success: %u bytes. Rebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  httpServer.begin();
  Serial.println("[HTTP] Server started on port 80 (GET /status, GET /capture, POST /config, POST /update).");
}

// =========================================================================
// 6. BASELINE FRAME UPDATE & PSRAM CACHE (Every 30 seconds)
// =========================================================================
void setupOTA() {
  ArduinoOTA.setHostname(deviceId.c_str());

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Starting wireless firmware update...");
    httpServer.stop();
    mqttClient.disconnect();

    if (latestBaselineBuf != nullptr) {
      free(latestBaselineBuf);
      latestBaselineBuf = nullptr;
      latestBaselineLen = 0;
    }
    if (latestCaptureBuf != nullptr) {
      free(latestCaptureBuf);
      latestCaptureBuf = nullptr;
      latestCaptureLen = 0;
    }
    if (isCameraInitialized) {
      esp_camera_deinit();
      isCameraInitialized = false;
    }
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update complete! Rebooting...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n[OTA] Error[%u]\n", error);
    ESP.restart();
  });

  ArduinoOTA.begin();
  Serial.printf("[OTA] Wireless OTA ready at %s (%s)\n", WiFi.localIP().toString().c_str(), deviceId.c_str());
}

void captureAndCacheBaseline() {
  if (!isCameraInitialized) return;

  camera_fb_t * fb = captureFreshFrame(true);
  if (!fb) {
    delay(100);
    fb = captureFreshFrame(true);
  }

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
  doc["device_id"]       = deviceId;
  doc["mqtt_client_id"]  = deviceId;
  doc["rack_id"]         = rackId;
  doc["shelf_id"]        = shelfId;
  doc["event"]           = "BASELINE_UPDATED";
  doc["ip_address"]      = WiFi.localIP().toString();
  doc["baseline_url"]    = "http://" + WiFi.localIP().toString() + "/capture?mode=baseline";
  doc["rssi_dbm"]        = WiFi.RSSI();
  doc["free_heap_bytes"] = ESP.getFreeHeap();
  doc["status"]          = "ONLINE";

  char buffer[384];
  serializeJson(doc, buffer);

  if (mqttClient.publish(baseline_topic.c_str(), buffer)) {
    Serial.printf("[Baseline Updated] %s -> %s\n", baseline_topic.c_str(), buffer);
  }
}

// =========================================================================
// 7. LOCALIZED EXTRACTION VERIFICATION & REFERENCE CAPTURE HANDLER
// =========================================================================
void handleCaptureTrigger(const char* expected_action, bool isReferenceCapture = false, int boxCount = 4, bool flashEnabled = true) {
  Serial.printf("\n[+] Trigger Received (%s): Localized snapshot on %s...\n",
                isReferenceCapture ? "REFERENCE_SAMPLE" : "STATE_VALIDATION", shelfId.c_str());

  // Pause 30s baseline auto-updates while the backend fetches and compares the before/after frames
  baselinePauseUntil = millis() + BASELINE_COOLDOWN_MS;
  isBaselineFrozen = false;

  // Stagger CAM2 slightly on dual-camera shelves so both LEDs don't flash and blind each other simultaneously
  if (!isReferenceCapture && deviceId.endsWith("CAM2")) {
    delay(250);
  }

  camera_fb_t * fb = captureFreshFrame(flashEnabled);
  if (!fb) {
    delay(100);
    fb = captureFreshFrame(flashEnabled);
  }

  if (!fb) {
    Serial.println("[-] ERROR: Snapshot capture failed!");
    return;
  }

  // Cache the triggered frame in PSRAM/heap so HTTP GET /capture?mode=live serves the exact frame captured here
  if (latestCaptureBuf != nullptr) {
    free(latestCaptureBuf);
    latestCaptureBuf = nullptr;
    latestCaptureLen = 0;
  }

  if (psramFound()) {
    latestCaptureBuf = (uint8_t*)ps_malloc(fb->len);
  } else {
    latestCaptureBuf = (uint8_t*)malloc(fb->len);
  }

  if (latestCaptureBuf != nullptr) {
    memcpy(latestCaptureBuf, fb->buf, fb->len);
    latestCaptureLen = fb->len;
  } else {
    Serial.println("[-] Memory allocation failed for triggered frame cache.");
  }

  // Release camera DMA buffer immediately to preserve SRAM (bypasses 520KB constraint)
  esp_camera_fb_return(fb);

  String localIp = WiFi.localIP().toString();

  // Build JSON confirmation payload
  StaticJsonDocument<384> doc;
  doc["device_id"]       = deviceId;
  doc["shelf_id"]        = shelfId;
  doc["expected_action"] = expected_action;
  doc["box_count"]       = boxCount;
  doc["status_code"]     = 200;
  doc["ip_address"]      = localIp;
  doc["baseline_url"]    = String("http://") + localIp + "/capture?mode=baseline";
  doc["latest_url"]      = String("http://") + localIp + "/capture?mode=live" + (flashEnabled ? "" : "&flash=0");

  char buffer[384];
  serializeJson(doc, buffer);

  // Publish to: iotlab/storage/<shelfId>/response
  if (mqttClient.publish(response_topic.c_str(), buffer)) {
    Serial.printf("[MQTT Published] %s -> %s\n", response_topic.c_str(), buffer);
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
  const char* target_device = doc["device_id"] | "";
  const char* expected_action = doc["expected_action"] | "BORROW";

  Serial.printf("    Command: %s | Target Shelf: %s | Action: %s\n", command, target_shelf, expected_action);

  // Optional baseline freeze/unfreeze commands if backend signals when a shelf compartment is opened/closed
  if (strcmp(command, "FREEZE_BASELINE") == 0 || strcmp(command, "LOCK_BASELINE") == 0) {
    if (shelfId == target_shelf || strlen(target_shelf) == 0) {
      isBaselineFrozen = true;
      baselinePauseUntil = millis() + 60000; // Auto-expire freeze after 60s safety timeout
      Serial.println("    [Baseline] Frozen for active shelf interaction.");
    }
    return;
  }

  if (strcmp(command, "UNFREEZE_BASELINE") == 0 || strcmp(command, "UNLOCK_BASELINE") == 0) {
    if (shelfId == target_shelf || strlen(target_shelf) == 0) {
      isBaselineFrozen = false;
      baselinePauseUntil = 0;
      Serial.println("    [Baseline] Unfrozen; capturing fresh baseline.");
      captureAndCacheBaseline();
    }
    return;
  }

  // A. Vision Capture Trigger on iotlab/storage/<rackId>/trigger
  if (strcmp(command, "CAPTURE_STATE") == 0) {
    bool shelfMatch = (shelfId == target_shelf);
    bool deviceMatch = (strlen(target_device) == 0 || deviceId == target_device);
    if (shelfMatch && deviceMatch) {
      bool flashEnabled = doc["flash_enabled"] | true;
      int boxCount = doc["box_count"] | 4;
      handleCaptureTrigger(expected_action, false, boxCount, flashEnabled);
    }
    return;
  }

  // B. Reference Sample Capture on iotlab/storage/node/<deviceId>/config
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
  doc["device_id"]        = deviceId;
  doc["mqtt_client_id"]   = deviceId;
  doc["camera_name"]      = cameraName;
  doc["rack_id"]          = rackId;
  doc["shelf_id"]         = shelfId;
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
  WiFi.setSleep(false); // Disables Wi-Fi modem sleep so baseline current stays ~120mA (above power bank 80mA auto-off cutoff) and HTTP latency stays <70ms

  IPAddress local_IP(192, 168, 0, ipOctet);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);
  WiFi.config(local_IP, gateway, subnet, primaryDNS);

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

    String mdnsName = deviceId;
    mdnsName.toLowerCase();
    if (MDNS.begin(mdnsName.c_str())) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("       mDNS URL: http://%s.local/capture\n", mdnsName.c_str());
    }
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

  if (millis() - lastMqttRetryTime < 10000 && lastMqttRetryTime != 0) return;
  lastMqttRetryTime = millis();

  Serial.printf("[MQTT] Connecting to broker %s:%d...\n", MQTT_BROKER_IP, MQTT_PORT);
  if (mqttClient.connect(deviceId.c_str())) {
    Serial.println("[MQTT] Connected successfully!");
    mqttClient.subscribe(trigger_topic.c_str());
    mqttClient.subscribe(config_topic.c_str());
    Serial.printf("[MQTT Subscribed] Trigger: %s | Reference Config: %s\n", trigger_topic.c_str(), config_topic.c_str());
    publishHealthTelemetry("BOOT_CONNECT");
  } else {
    Serial.printf("[-] MQTT connect failed, state=%d. Will retry in 10s in background...\n", mqttClient.state());
  }
}

// =========================================================================
// 11. SETUP & MAIN LOOP
// =========================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector to prevent voltage-dip reboots on power banks

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  Serial.begin(115200);
  delay(1000);

  loadConfigFromNVS();

  Serial.println("\n=======================================================");
  Serial.println("  SMART STORAGE FYP - ESP32 CAMERA HARDWARE RELAY NODE");
  Serial.printf("  Device: %s | Shelf: %s | Rack: %s | IP: 192.168.0.%d\n",
                deviceId.c_str(), shelfId.c_str(), rackId.c_str(), ipOctet);
  Serial.println("=======================================================");

  isCameraInitialized = initCamera();
  if (!isCameraInitialized) {
    Serial.println("[-] WARNING: Camera sensor not responding. Check ribbon cable connection.");
  }

  ensureWiFiConnected();
  setupHttpServer();
  setupOTA();

  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(768);

  ensureMQTTConnected();

  delay(500);
  if (isCameraInitialized) {
    camera_fb_t* warmup = esp_camera_fb_get();
    if (warmup) esp_camera_fb_return(warmup);
  }
  captureAndCacheBaseline();
  lastHeartbeatTime = millis();
}

void loop() {
  ArduinoOTA.handle();
  httpServer.handleClient();

  if (!mqttClient.connected()) {
    ensureMQTTConnected();
  }
  mqttClient.loop();

  unsigned long now = millis();

  // 8-Second USB Power Bank Keep-Alive Current Pulse (~220mA briefly resets auto-off timer)
  if (now - lastKeepAliveTime >= POWERBANK_KEEPALIVE_MS) {
    lastKeepAliveTime = now;
    if (isCameraInitialized) {
      camera_fb_t* ka = esp_camera_fb_get();
      if (ka) esp_camera_fb_return(ka);
    }
  }

  // 30-Second Baseline Cache Update & Health Heartbeat
  if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatTime = now;
    publishHealthTelemetry();

    // Auto-expire baseline freeze if the pause window has elapsed
    if (baselinePauseUntil != 0 && (long)(now - baselinePauseUntil) >= 0) {
      baselinePauseUntil = 0;
      isBaselineFrozen = false;
    }

    if (!isBaselineFrozen && baselinePauseUntil == 0) {
      captureAndCacheBaseline();
    } else {
      Serial.println("[Baseline] Skipping 30s baseline update (shelf verification active).");
    }
  }
}
