/*
 * ============================================================
 *  BOX ESP-CAM — Station Camera Streamer (Full Version)
 * ============================================================
 *  This ESP-CAM sits on top of the main box body and acts as
 *  the fire station's scanning camera. It replaces the laptop
 *  webcam in the fire detection pipeline.
 *
 *  Flow:
 *    1. Box ESP-CAM streams video over WiFi.
 *    2. Python dashboard reads the stream, runs YOLO.
 *    3. If fire detected → triggers gatekeeper → deploys car.
 *
 *  Features:
 *    - MJPEG stream at /stream (port 80)
 *    - Status, capture, flash control on port 81
 *    - VGA (640x480) resolution for better YOLO range
 *    - WiFi auto-reconnect watchdog
 *    - Heartbeat logging every 30s
 *    - No motors, no pump, no ESP-NOW — camera only
 *
 *  Board: AI Thinker ESP32-CAM
 *  ESP32 Core: 2.0.17
 *
 *  Authors: Muhammad Ahmed, Ali Riaz
 * ============================================================
 */

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ============================================================
//  WIFI CONFIG — laptop hotspot
// ============================================================
const char* WIFI_SSID = "aliswork";
const char* WIFI_PASS = "12345678";

// Box ESP-CAM static IP
// Laptop gateway = 192.168.137.1
// Car 1 IP       = 192.168.137.158
// Box IP         = 192.168.137.100
IPAddress staticIP(192, 168, 137, 100);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);

// ============================================================
//  HARDWARE PINS — AI Thinker ESP32-CAM
// ============================================================
#define FLASH_LED 4
#define STATUS_LED 33

// Camera pins — identical to working car code
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============================================================
//  GLOBAL STATE
// ============================================================
httpd_handle_t stream_httpd  = NULL;
httpd_handle_t command_httpd = NULL;

unsigned long bootTime      = 0;
volatile int  streamClients = 0;

// ============================================================
//  MJPEG STREAM HANDLER — /stream on port 80
//  Copied from working car code, proven stable.
// ============================================================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[128];

  streamClients++;
  Serial.printf("🎥 [STREAM] Client #%d connected! Starting video stream...\n", streamClients);

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    streamClients--;
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("❌ Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    // Correct MJPEG order: boundary -> part header -> JPEG bytes
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));

    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) {
      break;
    }
  }

  streamClients--;
  Serial.printf("🔌 [STREAM] Client disconnected. Active clients: %d\n", streamClients);
  return res;
}

// ============================================================
//  HTTP HANDLERS — port 81
// ============================================================

// GET /status — full health report
esp_err_t status_handler(httpd_req_t *req) {
  unsigned long uptimeSec = (millis() - bootTime) / 1000;

  char text[512];
  snprintf(text, sizeof(text),
           "BOX_CAM_OK\n"
           "ROLE=STATION_CAMERA\n"
           "IP=%s\n"
           "MAC=%s\n"
           "STREAM=http://%s/stream\n"
           "STREAM_CLIENTS=%d\n"
           "UPTIME=%lu seconds\n"
           "WIFI_RSSI=%d dBm\n"
           "FREE_HEAP=%u bytes\n",
           WiFi.localIP().toString().c_str(),
           WiFi.macAddress().c_str(),
           WiFi.localIP().toString().c_str(),
           streamClients,
           uptimeSec,
           WiFi.RSSI(),
           ESP.getFreeHeap());

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// GET /capture — single JPEG frame (for testing in browser)
esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// GET /flash_on — turn flash LED on (for dark environments)
esp_err_t flash_on_handler(httpd_req_t *req) {
  digitalWrite(FLASH_LED, HIGH);
  Serial.println("💡 Flash LED ON");
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "FLASH_ON", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// GET /flash_off — turn flash LED off
esp_err_t flash_off_handler(httpd_req_t *req) {
  digitalWrite(FLASH_LED, LOW);
  Serial.println("💡 Flash LED OFF");
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "FLASH_OFF", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// ============================================================
//  START HTTP SERVERS — structure from working car code
// ============================================================
bool startStreamServer() {
  if (stream_httpd != NULL) return true;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  static httpd_uri_t stream_uri;
  stream_uri.uri = "/stream";
  stream_uri.method = HTTP_GET;
  stream_uri.handler = stream_handler;
  stream_uri.user_ctx = NULL;

  if (httpd_start(&stream_httpd, &config) != ESP_OK) {
    Serial.println("❌ Failed to start stream server.");
    return false;
  }

  httpd_register_uri_handler(stream_httpd, &stream_uri);
  Serial.println("✅ Stream server started on port 80.");
  return true;
}

bool startCommandServer() {
  if (command_httpd != NULL) return true;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 32769;
  config.max_uri_handlers = 10;

  // /status
  static httpd_uri_t status_uri;
  status_uri.uri = "/status";
  status_uri.method = HTTP_GET;
  status_uri.handler = status_handler;
  status_uri.user_ctx = NULL;

  // /capture
  static httpd_uri_t capture_uri;
  capture_uri.uri = "/capture";
  capture_uri.method = HTTP_GET;
  capture_uri.handler = capture_handler;
  capture_uri.user_ctx = NULL;

  // /flash_on
  static httpd_uri_t flash_on_uri;
  flash_on_uri.uri = "/flash_on";
  flash_on_uri.method = HTTP_GET;
  flash_on_uri.handler = flash_on_handler;
  flash_on_uri.user_ctx = NULL;

  // /flash_off
  static httpd_uri_t flash_off_uri;
  flash_off_uri.uri = "/flash_off";
  flash_off_uri.method = HTTP_GET;
  flash_off_uri.handler = flash_off_handler;
  flash_off_uri.user_ctx = NULL;

  if (httpd_start(&command_httpd, &config) != ESP_OK) {
    Serial.println("❌ Failed to start command server.");
    return false;
  }

  httpd_register_uri_handler(command_httpd, &status_uri);
  httpd_register_uri_handler(command_httpd, &capture_uri);
  httpd_register_uri_handler(command_httpd, &flash_on_uri);
  httpd_register_uri_handler(command_httpd, &flash_off_uri);

  Serial.println("✅ Command server started on port 81.");
  return true;
}

// ============================================================
//  CAMERA INIT — IDENTICAL to working car code
//  Then bumps to VGA after successful init.
// ============================================================
bool initCamera() {
  // --- Phase 1: Init with car's proven QVGA settings ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Optimized for smooth streaming over WiFi
  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;    // 320x240 — small frames = fast WiFi
    config.jpeg_quality = 15;               // more compression = smaller = smoother stream
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;  // always return freshest frame, skip stale ones
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed: 0x%x\n", err);
    return false;
  }

  // Keep QVGA — do NOT bump to VGA. VGA frames are 4x larger
  // and choke WiFi, causing the lag/stutter.
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_quality(s, 15);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);      // slight contrast boost for fire visibility
    s->set_saturation(s, 1);    // slight saturation boost for fire colors
    Serial.println("   📐 Resolution: QVGA 320x240 (optimized for smooth stream)");
  }

  Serial.println("✅ Camera initialized.");
  return true;
}

// ============================================================
//  WIFI INIT — copied from working car code
// ============================================================
bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // max power for smooth streaming

  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.println("⚠️  Static IP config failed. Trying anyway...");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to hotspot");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
    // Blink status LED while connecting
    digitalWrite(STATUS_LED, (tries % 2 == 0) ? LOW : HIGH);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n⚠️  Static IP connection failed. Retrying with DHCP...");
    WiFi.disconnect();
    delay(500);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
      delay(500);
      Serial.print(".");
      tries++;
      digitalWrite(STATUS_LED, (tries % 2 == 0) ? LOW : HIGH);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n❌ Wi-Fi connection failed.");
    return false;
  }

  Serial.println("\n✅ Connected to hotspot.");
  Serial.print("   IP:      "); Serial.println(WiFi.localIP());
  Serial.print("   MAC:     "); Serial.println(WiFi.macAddress());
  Serial.print("   Channel: "); Serial.println(WiFi.channel());
  Serial.print("   RSSI:    "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  Serial.println();
  Serial.print("   Stream:  http://"); Serial.print(WiFi.localIP()); Serial.println("/stream");
  Serial.print("   Status:  http://"); Serial.print(WiFi.localIP()); Serial.println(":81/status");
  Serial.print("   Capture: http://"); Serial.print(WiFi.localIP()); Serial.println(":81/capture");
  return true;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  bootTime = millis();

  Serial.println("\n========================================");
  Serial.println(" BOX ESP-CAM — Station Camera Streamer");
  Serial.println("========================================");
  Serial.println(" Role: Fire scanning camera (no motors)");
  Serial.println("========================================");

  // Init LEDs
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);  // ON while booting (active-low)

  // Init camera
  Serial.println("\n📷 Initializing camera...");
  if (!initCamera()) {
    Serial.println("❌ FATAL: Camera init failed. Halting.");
    // Solid ON = camera error
    while (true) { delay(1000); }
  }

  // Connect WiFi
  Serial.println("\n📡 Connecting to WiFi...");
  if (!connectWiFi()) {
    Serial.println("❌ FATAL: WiFi failed. Halting.");
    // Slow blink = WiFi error
    while (true) {
      digitalWrite(STATUS_LED, LOW); delay(500);
      digitalWrite(STATUS_LED, HIGH); delay(500);
    }
  }

  // Start servers
  Serial.println("\n🌐 Starting HTTP servers...");
  bool streamOK = startStreamServer();
  bool cmdOK    = startCommandServer();

  if (!streamOK || !cmdOK) {
    Serial.println("❌ FATAL: Server start failed. Halting.");
    // Fast blink = server error
    while (true) {
      digitalWrite(STATUS_LED, LOW); delay(100);
      digitalWrite(STATUS_LED, HIGH); delay(100);
    }
  }

  // All good — turn off status LED (active-low, HIGH = OFF)
  digitalWrite(STATUS_LED, HIGH);

  // Triple flash = ready signal
  for (int i = 0; i < 3; i++) {
    digitalWrite(FLASH_LED, HIGH); delay(100);
    digitalWrite(FLASH_LED, LOW);  delay(100);
  }

  Serial.println("\n🟢 BOX CAMERA READY.");
  Serial.println("   Dashboard can now connect to /stream.");
  Serial.println("   Waiting for Python dashboard...\n");
}

// ============================================================
//  LOOP — heartbeat + WiFi watchdog
// ============================================================
unsigned long lastHeartbeat = 0;
unsigned long lastWiFiCheck = 0;

void loop() {
  unsigned long now = millis();

  // Heartbeat every 30 seconds
  if (now - lastHeartbeat > 30000) {
    lastHeartbeat = now;
    unsigned long uptimeSec = (now - bootTime) / 1000;

    Serial.printf("💓 [HEARTBEAT] Uptime: %lus | WiFi: %s | RSSI: %d dBm | "
                  "Heap: %u bytes | Clients: %d\n",
                  uptimeSec,
                  WiFi.isConnected() ? "OK" : "DOWN",
                  WiFi.RSSI(),
                  ESP.getFreeHeap(),
                  streamClients);
  }

  // WiFi watchdog — check every 10 seconds
  if (now - lastWiFiCheck > 10000) {
    lastWiFiCheck = now;

    if (!WiFi.isConnected()) {
      Serial.println("⚠️  WiFi lost! Reconnecting...");
      digitalWrite(STATUS_LED, LOW);  // ON = problem

      WiFi.disconnect();
      delay(500);
      WiFi.begin(WIFI_SSID, WIFI_PASS);

      int tries = 0;
      while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        Serial.print(".");
        tries++;
      }

      if (WiFi.isConnected()) {
        Serial.println("\n✅ WiFi reconnected!");
        Serial.print("   IP: "); Serial.println(WiFi.localIP());
        digitalWrite(STATUS_LED, HIGH);  // OFF = OK
      } else {
        Serial.println("\n❌ Reconnect failed. Will retry in 10s...");
      }
    }
  }

  delay(10);
}
