/*
 * ============================================================
 *  DEMO CAM TEST — Minimal stream + LED blink
 * ============================================================
 *  Purpose: Test if ESP32-CAM hardware works.
 *  - Connects to WiFi
 *  - Starts MJPEG stream on port 80
 *  - Blinks flash LED every 5 seconds
 *
 *  Board: AI Thinker ESP32-CAM
 *  Partition: Huge APP (3MB No OTA / 1MB SPIFFS)
 * ============================================================
 */

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ============================================================
//  WIFI — change these to match your hotspot
// ============================================================
const char* WIFI_SSID = "aliswork";
const char* WIFI_PASS = "12345678";

IPAddress staticIP(192, 168, 137, 159);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);

// ============================================================
//  AI Thinker ESP32-CAM pins
// ============================================================
#define FLASH_LED 4
#define STATUS_LED 33

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
//  GLOBALS
// ============================================================
httpd_handle_t stream_httpd = NULL;
unsigned long lastBlink = 0;

// ============================================================
//  MJPEG STREAM — /stream on port 80
// ============================================================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[128];

  Serial.println("STREAM: Client connected!");

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("ERROR: Camera capture failed");
      res = ESP_FAIL;
      break;
    }

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

    if (res != ESP_OK) break;
  }

  Serial.println("STREAM: Client disconnected.");
  return res;
}

// ============================================================
//  CAMERA INIT
// ============================================================
bool initCamera() {
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

  if (psramFound()) {
    Serial.println("PSRAM found! Using QVGA + 2 buffers.");
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("WARNING: No PSRAM! Using QQVGA + 1 buffer.");
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Camera init failed with error 0x%x\n", err);
    return false;
  }

  Serial.println("OK: Camera initialized.");
  return true;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  DEMO CAM TEST - Stream + LED Blink");
  Serial.println("========================================");

  // LED pins only — no motor pins to avoid conflicts
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);  // ON = booting

  // Camera
  Serial.println("Step 1: Init camera...");
  if (!initCamera()) {
    Serial.println("FATAL: Camera failed. Halting.");
    while (true) { delay(1000); }
  }

  // WiFi
  Serial.println("Step 2: Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.println("WARNING: Static IP config failed.");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFATAL: WiFi failed. Halting.");
    while (true) { delay(1000); }
  }

  Serial.println("\nOK: WiFi connected!");
  Serial.print("  IP:     "); Serial.println(WiFi.localIP());
  Serial.print("  MAC:    "); Serial.println(WiFi.macAddress());
  Serial.print("  Stream: http://"); Serial.print(WiFi.localIP()); Serial.println("/stream");

  // Start stream server
  Serial.println("Step 3: Starting stream server...");
  httpd_config_t httpConfig = HTTPD_DEFAULT_CONFIG();
  httpConfig.server_port = 80;
  httpConfig.ctrl_port = 32768;

  static httpd_uri_t stream_uri;
  stream_uri.uri = "/stream";
  stream_uri.method = HTTP_GET;
  stream_uri.handler = stream_handler;
  stream_uri.user_ctx = NULL;

  if (httpd_start(&stream_httpd, &httpConfig) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("OK: Stream server on port 80.");
  } else {
    Serial.println("FATAL: Stream server failed.");
    while (true) { delay(1000); }
  }

  digitalWrite(STATUS_LED, HIGH);  // OFF = ready
  Serial.println("\n*** ALL OK — Board is working! ***");
  Serial.println("Open the stream URL in your browser.");
  Serial.println("Flash LED will blink every 5 seconds.\n");

  lastBlink = millis();
}

// ============================================================
//  LOOP — just blink LED every 5 seconds
// ============================================================
void loop() {
  if (millis() - lastBlink >= 5000) {
    lastBlink = millis();
    Serial.println("BLINK!");
    digitalWrite(FLASH_LED, HIGH);
    delay(200);
    digitalWrite(FLASH_LED, LOW);
  }
  delay(10);
}
