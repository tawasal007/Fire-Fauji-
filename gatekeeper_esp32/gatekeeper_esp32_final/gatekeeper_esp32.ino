/*
 * ============================================================
 *  GATEKEEPER ESP32 — Deploy/Stop ESP-NOW Bridge
 * ============================================================
 *  Laptop sends over USB Serial:
 *    'F' -> send ESP-NOW "DEPLOY" to car(s)
 *    'S' -> send ESP-NOW "STOP" to car(s)
 *
 *  Important:
 *    - Gatekeeper connects to the SAME laptop hotspot as cars.
 *    - This keeps ESP-NOW on the same Wi-Fi channel.
 *    - Video is NOT handled here. Video goes directly ESP-CAM -> laptop.
 *    - Return-to-base motor commands are sent directly by the dashboard
 *      to the ESP-CAM HTTP server in strict timed order.
 *    - This gatekeeper still provides DEPLOY and final safety STOP.
 *
 *  Board: Normal ESP32 DevKit, not ESP32-CAM.
 * ============================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <WebServer.h>

// ============================================================
//  CONFIGURATION
// ============================================================
// Wi-Fi Credentials
const char* WIFI_SSID = "aliswork";
const char* WIFI_PASS = "12345678";

// Gatekeeper static IP
IPAddress staticIP(192, 168, 137, 110);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);

// New Servo ESP32 MAC
uint8_t SERVO_MAC[] = {0xF8, 0xB3, 0xB7, 0x20, 0xE5, 0xC0};

WebServer server(81);

// Paste the car ESP-CAM MAC printed in the car Serial Monitor.
// Example format: {0x80, 0xF3, 0xDA, 0x5F, 0x25, 0xBC}
uint8_t CAR1_MAC[] = {0x80, 0xF3, 0xDA, 0x5F, 0x25, 0xBC};

// Optional second car. Keep zeros if not using.
uint8_t CAR2_MAC[] = {0xEC, 0xE3, 0x34, 0x46, 0xD4, 0x30};

#define STATUS_LED 2

// ============================================================
//  MESSAGE STRUCT — must match car_espcam_final.ino
// ============================================================
typedef struct {
  char command[32];
} CommandMessage;

bool isZeroMac(const uint8_t mac[6]) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] != 0x00) return false;
  }
  return true;
}

void printMac(const uint8_t mac[6]) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
}

// ============================================================
//  ESP-NOW SEND CALLBACK
//  Supports both ESP32 Arduino core 2.x and 3.x where possible.
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  Serial.print("   ESP-NOW status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
}

// ============================================================
//  ADD PEER
// ============================================================
bool addPeer(const uint8_t mac[6], const char* name) {
  if (isZeroMac(mac)) {
    Serial.print("⚠️  "); Serial.print(name); Serial.println(" MAC is empty. Skipping.");
    return false;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;      // 0 = current Wi-Fi channel
  peerInfo.encrypt = false;

  if (esp_now_is_peer_exist(mac)) {
    Serial.print("✅ "); Serial.print(name); Serial.println(" peer already exists.");
    return true;
  }

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result == ESP_OK) {
    Serial.print("✅ Added "); Serial.print(name); Serial.print(" peer: ");
    printMac(mac); Serial.println();
    return true;
  }

  Serial.print("❌ Failed to add "); Serial.print(name); Serial.print(" peer. Error: ");
  Serial.println(result);
  return false;
}

// ============================================================
//  SEND COMMAND
// ============================================================
void sendCommandToMac(const uint8_t mac[6], const char* name, const char* cmd) {
  if (isZeroMac(mac)) return;

  CommandMessage msg;
  memset(&msg, 0, sizeof(msg));
  strncpy(msg.command, cmd, sizeof(msg.command) - 1);

  Serial.print("📡 Sending '"); Serial.print(cmd); Serial.print("' to "); Serial.println(name);

  // Send 3 times for demo reliability.
  for (int i = 0; i < 3; i++) {
    esp_err_t result = esp_now_send(mac, (uint8_t*)&msg, sizeof(msg));
    if (result != ESP_OK) {
      Serial.print("   esp_now_send error: ");
      Serial.println(result);
    }
    delay(80);
  }
}

void sendCommandToAllCars(const char* cmd) {
  sendCommandToMac(CAR1_MAC, "Car 1", cmd);
  sendCommandToMac(CAR2_MAC, "Car 2", cmd);
}

// ============================================================
//  HTTP HANDLERS
// ============================================================
void handleOpenDoors() {
  Serial.println("🚪 Sending ESP-NOW 'e' to Servo ESP32...");
  sendCommandToMac(SERVO_MAC, "Servo ESP32", "e");
  server.send(200, "text/plain", "DOORS_OPEN");
}

void handleCloseDoors() {
  Serial.println("🚪 Sending ESP-NOW 'f' to Servo ESP32...");
  sendCommandToMac(SERVO_MAC, "Servo ESP32", "f");
  server.send(200, "text/plain", "DOORS_CLOSED");
}

void handleIndex() {
  String html = "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                "<style>body{font-family:sans-serif;text-align:center;padding:20px;background:#121212;color:white;} "
                "button{width:90%;max-width:400px;padding:20px;font-size:24px;margin:10px;background:#448aff;color:white;border:none;border-radius:10px;cursor:pointer;}</style>"
                "</head><body><h1>Gatekeeper Servos</h1>"
                "<button onclick=\"fetch('/open_doors')\">🚪 Open Doors</button><br>"
                "<button onclick=\"fetch('/close_doors')\">🚪 Close Doors</button><br>"
                "</body></html>";
  server.send(200, "text/html", html);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Serial.println("\n========================================");
  Serial.println(" GATEKEEPER ESP32 — Deploy / Safety Stop");
  Serial.println("========================================");

  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to hotspot");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to hotspot.");
    Serial.print("   IP: "); Serial.println(WiFi.localIP());
    Serial.print("   Channel: "); Serial.println(WiFi.channel());
  } else {
    Serial.println("\n⚠️  Hotspot connection failed.");
    Serial.println("   ESP-NOW may fail if channel does not match cars.");
  }

  Serial.print("Gatekeeper MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed. Restart board.");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);

  addPeer(CAR1_MAC, "Car 1");
  addPeer(CAR2_MAC, "Car 2");
  addPeer(SERVO_MAC, "Servo ESP32");

  digitalWrite(STATUS_LED, HIGH);

  server.on("/", handleIndex);
  server.on("/open_doors", handleOpenDoors);
  server.on("/close_doors", handleCloseDoors);
  server.begin();
  Serial.println("✅ Command server started on port 81.");

  Serial.println("\n🟢 READY.");
  Serial.println("Send from laptop/Python:");
  Serial.println("  F = DEPLOY");
  Serial.println("  S = FINAL SAFETY STOP");
  Serial.println("Return path is controlled by dashboard HTTP commands.");
  Serial.println();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  server.handleClient();

  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line.startsWith("F") || line.startsWith("f")) {
      Serial.println("\n🔥 Serial F received -> Opening doors, waiting 5 seconds before DEPLOY...");
      digitalWrite(STATUS_LED, HIGH);
      // Immediately tell Servo ESP32 to open doors
      sendCommandToMac(SERVO_MAC, "Servo ESP32", "OPEN_DOORS");
      
      delay(5000);
      sendCommandToAllCars("DEPLOY");
      Serial.println("🚀 DEPLOY sent!");
    }
    else if (line.startsWith("S") || line.startsWith("s")) {
      Serial.println("\n🛑 Serial S received -> STOP");
      digitalWrite(STATUS_LED, LOW);
      sendCommandToAllCars("STOP");
    }
  }

  delay(10);
}
