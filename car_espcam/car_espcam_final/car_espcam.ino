/*
 * ============================================================
 *  CAR ESP-CAM — Stream + Timed Return Motor Endpoints
 * ============================================================
 *  Correct final/demo flow:
 *    1. Car connects to laptop hotspot.
 *    2. Car starts /stream immediately at boot.
 *    3. Laptop can always open http://CAR_IP/stream.
 *    4. ESP-NOW only changes car state: DEPLOY / STOP.
 *
 *  Board: AI Thinker ESP32-CAM
 *
 *  Important demo note:
 *    - Do NOT use GPIO 1 and GPIO 3 for motor enable while debugging.
 *      They are Serial TX/RX. Keep ENA/ENB jumpers on L298N or tie enables HIGH.
 * ============================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ============================================================
//  WIFI SETTINGS
// ============================================================
const char* WIFI_SSID = "aliswork";
const char* WIFI_PASS = "12345678";

// For Windows laptop hotspot usually:
// Laptop gateway = 192.168.137.1
// Car 1 IP       = 192.168.137.158
IPAddress staticIP(192, 168, 137, 158);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);

// ============================================================
//  HARDWARE PINS
// ============================================================
// Direct Motor Control Pins
#define IN1 14
#define IN2 15
#define IN3 13
#define IN4 12
#define RELAY_PIN 2

// Motor Enables (Using U0T / U0R)
#define ENA 1
#define ENB 3

// ESP32-CAM flash LED
#define FLASH_LED 4

// Onboard red LED on AI Thinker. Active-low: LOW = ON, HIGH = OFF
#define STATUS_LED 33

// AI Thinker ESP32-CAM camera pins
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
//  MESSAGE STRUCT — must match gatekeeper_esp32_final.ino
// ============================================================
typedef struct {
  char command[32];
} CommandMessage;

// ============================================================
//  GLOBAL STATE
// ============================================================
httpd_handle_t stream_httpd = NULL;
httpd_handle_t command_httpd = NULL;

volatile bool flagDeploy = false;
volatile bool flagStop = false;

bool deployed = false;

// ============================================================
//  MOTOR / PUMP HELPERS (Direct GPIO Control)
// ============================================================
void stopMotors() {
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void forwardMotors() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
}

void reverseMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
}

void turnLeftMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
}

void turnRightMotors() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
}

void reverseLeftPathMotors() { turnRightMotors(); }
void reverseRightPathMotors() { turnLeftMotors(); }

void pumpOff() {
  digitalWrite(RELAY_PIN, HIGH);
}

void pumpOn() {
  digitalWrite(RELAY_PIN, LOW);
}

// ============================================================
//  ESP-NOW RECEIVE CALLBACK
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  CommandMessage msg;
  memset(&msg, 0, sizeof(msg));

  int copyLen = min(len, (int)sizeof(msg) - 1);
  memcpy(&msg, data, copyLen);
  msg.command[copyLen] = '\0';

  Serial.print("📡 [ESP-NOW] Callback received raw command: ");
  Serial.println(msg.command);

  if (strcmp(msg.command, "DEPLOY") == 0) {
    flagDeploy = true;
  } else if (strcmp(msg.command, "STOP") == 0) {
    flagStop = true;
  } else if (strncmp(msg.command, "GOTO ", 5) == 0) {
    flagDeploy = true;
    Serial.println(msg.command); 
  }
}

// ============================================================
//  MJPEG STREAM HANDLER — /stream on port 80
// ============================================================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[128];

  Serial.println("🎥 [STREAM] Client connected! Starting video stream...");

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

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

  Serial.println("🔌 [STREAM] Client disconnected. Stopped video stream.");
  return res;
}

// ============================================================
//  HTTP COMMAND HANDLERS — port 81
// ============================================================
esp_err_t status_handler(httpd_req_t *req) {
  char text[220];
  snprintf(text, sizeof(text),
           "CAR_OK\nIP=%s\nMAC=%s\nDEPLOYED=%s\nSTREAM=http://%s/stream\n",
           WiFi.localIP().toString().c_str(),
           WiFi.macAddress().c_str(),
           deployed ? "YES" : "NO",
           WiFi.localIP().toString().c_str());

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t pump_on_handler(httpd_req_t *req) {
  pumpOn();
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "PUMP_ON", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t pump_off_handler(httpd_req_t *req) {
  pumpOff();
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "PUMP_OFF", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t deploy_handler(httpd_req_t *req) {
  flagDeploy = true;
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "DEPLOY_BY_HTTP", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t stop_handler(httpd_req_t *req) {
  flagStop = true;
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "STOP_BY_HTTP", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t goto_handler(httpd_req_t *req) {
  char buf[100];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char paramX[16];
    char paramY[16];
    if (httpd_query_key_value(buf, "x", paramX, sizeof(paramX)) == ESP_OK &&
        httpd_query_key_value(buf, "y", paramY, sizeof(paramY)) == ESP_OK) {
        Serial.print("GOTO ");
        Serial.print(paramX);
        Serial.print(" ");
        Serial.println(paramY);
    }
  }
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "GOTO_SENT", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t home_handler(httpd_req_t *req) {
  Serial.println("HOME");
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "HOME_SENT", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// ============================================================
//  HTTP MOTOR CONTROL HANDLERS — Direct control from dashboard
// ============================================================
esp_err_t forward_handler(httpd_req_t *req) {
  Serial.println("🚗 [HTTP] /forward → Motors forward");
  forwardMotors();
  digitalWrite(FLASH_LED, HIGH);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "FORWARD", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t turn_left_handler(httpd_req_t *req) {
  Serial.println("⬅️ [HTTP] /turn_left → Turning left");
  turnLeftMotors();
  digitalWrite(FLASH_LED, HIGH);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "TURN_LEFT", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t turn_right_handler(httpd_req_t *req) {
  Serial.println("➡️ [HTTP] /turn_right → Turning right");
  turnRightMotors();
  digitalWrite(FLASH_LED, HIGH);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "TURN_RIGHT", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t reverse_handler(httpd_req_t *req) {
  Serial.println("⬇️ [HTTP] /reverse → Both motors backward");
  reverseMotors();
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "REVERSE", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t reverse_left_handler(httpd_req_t *req) {
  Serial.println("↩️ [HTTP] /reverse_left → Retracing previous left arc");
  reverseLeftPathMotors();
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "REVERSE_LEFT_PATH", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t reverse_right_handler(httpd_req_t *req) {
  Serial.println("↪️ [HTTP] /reverse_right → Retracing previous right arc");
  reverseRightPathMotors();
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "REVERSE_RIGHT_PATH", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t stop_motors_handler(httpd_req_t *req) {
  Serial.println("⏹️ [HTTP] /stop_motors → Motors stopped");
  stopMotors();
  digitalWrite(FLASH_LED, LOW);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "MOTORS_STOPPED", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// ============================================================
//  START HTTP SERVERS
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
  config.max_uri_handlers = 16;  // Includes forward + reverse return endpoints

  static httpd_uri_t status_uri;
  status_uri.uri = "/status";
  status_uri.method = HTTP_GET;
  status_uri.handler = status_handler;
  status_uri.user_ctx = NULL;

  static httpd_uri_t pump_on_uri;
  pump_on_uri.uri = "/pump_on";
  pump_on_uri.method = HTTP_GET;
  pump_on_uri.handler = pump_on_handler;
  pump_on_uri.user_ctx = NULL;

  static httpd_uri_t pump_off_uri;
  pump_off_uri.uri = "/pump_off";
  pump_off_uri.method = HTTP_GET;
  pump_off_uri.handler = pump_off_handler;
  pump_off_uri.user_ctx = NULL;

  static httpd_uri_t deploy_uri;
  deploy_uri.uri = "/deploy";
  deploy_uri.method = HTTP_GET;
  deploy_uri.handler = deploy_handler;
  deploy_uri.user_ctx = NULL;

  static httpd_uri_t stop_uri;
  stop_uri.uri = "/stop";
  stop_uri.method = HTTP_GET;
  stop_uri.handler = stop_handler;
  stop_uri.user_ctx = NULL;

  static httpd_uri_t goto_uri;
  goto_uri.uri = "/goto";
  goto_uri.method = HTTP_GET;
  goto_uri.handler = goto_handler;
  goto_uri.user_ctx = NULL;

  static httpd_uri_t home_uri;
  home_uri.uri = "/home";
  home_uri.method = HTTP_GET;
  home_uri.handler = home_handler;
  home_uri.user_ctx = NULL;

  if (httpd_start(&command_httpd, &config) != ESP_OK) {
    Serial.println("❌ Failed to start command server.");
    return false;
  }

  httpd_register_uri_handler(command_httpd, &status_uri);
  httpd_register_uri_handler(command_httpd, &pump_on_uri);
  httpd_register_uri_handler(command_httpd, &pump_off_uri);
  httpd_register_uri_handler(command_httpd, &deploy_uri);
  httpd_register_uri_handler(command_httpd, &stop_uri);
  httpd_register_uri_handler(command_httpd, &goto_uri);
  httpd_register_uri_handler(command_httpd, &home_uri);

  // Motor control endpoints (called by Python dashboard)
  static httpd_uri_t forward_uri;
  forward_uri.uri = "/forward";
  forward_uri.method = HTTP_GET;
  forward_uri.handler = forward_handler;
  forward_uri.user_ctx = NULL;

  static httpd_uri_t turn_left_uri;
  turn_left_uri.uri = "/turn_left";
  turn_left_uri.method = HTTP_GET;
  turn_left_uri.handler = turn_left_handler;
  turn_left_uri.user_ctx = NULL;

  static httpd_uri_t turn_right_uri;
  turn_right_uri.uri = "/turn_right";
  turn_right_uri.method = HTTP_GET;
  turn_right_uri.handler = turn_right_handler;
  turn_right_uri.user_ctx = NULL;

  static httpd_uri_t reverse_uri;
  reverse_uri.uri = "/reverse";
  reverse_uri.method = HTTP_GET;
  reverse_uri.handler = reverse_handler;
  reverse_uri.user_ctx = NULL;

  static httpd_uri_t reverse_left_uri;
  reverse_left_uri.uri = "/reverse_left";
  reverse_left_uri.method = HTTP_GET;
  reverse_left_uri.handler = reverse_left_handler;
  reverse_left_uri.user_ctx = NULL;

  static httpd_uri_t reverse_right_uri;
  reverse_right_uri.uri = "/reverse_right";
  reverse_right_uri.method = HTTP_GET;
  reverse_right_uri.handler = reverse_right_handler;
  reverse_right_uri.user_ctx = NULL;

  static httpd_uri_t stop_motors_uri;
  stop_motors_uri.uri = "/stop_motors";
  stop_motors_uri.method = HTTP_GET;
  stop_motors_uri.handler = stop_motors_handler;
  stop_motors_uri.user_ctx = NULL;

  httpd_register_uri_handler(command_httpd, &forward_uri);
  httpd_register_uri_handler(command_httpd, &turn_left_uri);
  httpd_register_uri_handler(command_httpd, &turn_right_uri);
  httpd_register_uri_handler(command_httpd, &reverse_uri);
  httpd_register_uri_handler(command_httpd, &reverse_left_uri);
  httpd_register_uri_handler(command_httpd, &reverse_right_uri);
  httpd_register_uri_handler(command_httpd, &stop_motors_uri);

  Serial.println("✅ Command server started on port 81 (12 endpoints).");
  return true;
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

  // Optimized for smooth streaming over WiFi
  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;   // 320x240, stable and fast
    config.jpeg_quality = 15;              // more compression = smoother stream
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST; // always return freshest frame
  } else {
    config.frame_size = FRAMESIZE_QQVGA;  // fallback if no PSRAM
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_quality(s, 15);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);      // slight contrast boost for fire visibility
    s->set_saturation(s, 1);    // slight saturation boost for fire colors
  }

  Serial.println("✅ Camera initialized.");
  return true;
}

// ============================================================
//  WIFI INIT
// ============================================================
bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE); // Disable WiFi power save entirely for maximum stream stability
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
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n❌ Wi-Fi connection failed.");
    return false;
  }

  Serial.println("\n✅ Connected to hotspot.");
  Serial.print("   IP: "); Serial.println(WiFi.localIP());
  Serial.print("   MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("   Channel: "); Serial.println(WiFi.channel());
  Serial.print("   Stream URL: http://"); Serial.print(WiFi.localIP()); Serial.println("/stream");
  Serial.print("   Status URL: http://"); Serial.print(WiFi.localIP()); Serial.println(":81/status");
  return true;
}

// ============================================================
//  ESP-NOW INIT
// ============================================================
bool initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed.");
    return false;
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("✅ ESP-NOW receiver ready.");
  return true;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println(" CAR ESP-CAM — Stream + Return-to-Base");
  Serial.println("========================================");

  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);  // ON while booting

  // Initialize motor and relay pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Active low relay default OFF

  stopMotors();
  pumpOff();

  if (!initCamera()) {
    digitalWrite(STATUS_LED, LOW); // solid ON = error
    return;
  }

  if (!connectWiFi()) {
    digitalWrite(STATUS_LED, LOW); // solid ON = error
    return;
  }

  if (!initEspNow()) {
    digitalWrite(STATUS_LED, LOW); // solid ON = error
    return;
  }

  // MOST IMPORTANT CHANGE:
  // Start stream immediately at boot, not after DEPLOY.
  bool streamOK = startStreamServer();
  bool cmdOK = startCommandServer();

  if (!streamOK || !cmdOK) {
    digitalWrite(STATUS_LED, LOW); // solid ON = error
    return;
  }

  digitalWrite(STATUS_LED, HIGH); // OFF = ready
  Serial.println("\n🟢 CAR READY.");
  Serial.println("Open /stream in browser before testing ESP-NOW.");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  if (flagDeploy) {
    flagDeploy = false;
    deployed = true;

    Serial.println("🔥 [FIRE DETECTED] DEPLOY command received via ESP-NOW!");
    Serial.println("   [STATUS] Flash LED (GPIO 4) blinked ONCE (Simulating water pump/motors).");

    // [DEV MODE] Commented out since ESP32-CAM is not plugged into the car chassis yet.
    // Uncomment these when deploying to the chassis:
    // forwardMotors();

    // Blink the onboard flash LED once to visually show deployment
    digitalWrite(FLASH_LED, HIGH);
    delay(150);
    digitalWrite(FLASH_LED, LOW);

    // Blink status LED quickly
    for (int i = 0; i < 2; i++) {
      digitalWrite(STATUS_LED, LOW); delay(100);
      digitalWrite(STATUS_LED, HIGH); delay(100);
    }
  }

  if (flagStop) {
    flagStop = false;
    deployed = false;

    Serial.println("🛑 [SAFE] STOP command received via ESP-NOW.");
    Serial.println("   [STATUS] Flash LED (GPIO 4) turned OFF.");

    // Safety action must always physically stop the chassis.
    stopMotors();
    pumpOff();

    digitalWrite(FLASH_LED, LOW);
    digitalWrite(STATUS_LED, HIGH); // OFF (active-low status LED)
  }

  delay(1);
}
