#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <ESP32Servo.h>

Servo servo1;
Servo servo2;

const int servo1Pin = 13;
const int servo2Pin = 12;

int currentPos = 0;

// Command message struct needs to match the sender
typedef struct {
  char command[32];
} CommandMessage;

CommandMessage incomingReadings;

// Callback function that will be executed when data is received
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  String cmd = String(incomingReadings.command);
  
  Serial.println("\n[ESP-NOW] Received: " + cmd);
  
  if (cmd == "e" || cmd == "OPEN_DOORS") {
    // INVERTED: "Open" means moving to 0
    if (currentPos == 90) {
      Serial.println("🚪 Opening garage doors (Inverted)...");
      for (int pos = currentPos; pos >= 0; pos--) {
        servo1.write(pos);
        servo2.write(180 - pos);
        delay(20);
      }
      currentPos = 0;
      Serial.println("✅ Garage doors opened");
    } else {
      Serial.println("⚠️ Doors already open.");
    }
  } 
  else if (cmd == "f" || cmd == "CLOSE_DOORS") {
    // INVERTED: "Close" means moving to 90
    if (currentPos == 0) {
      Serial.println("🚪 Closing garage doors (Inverted)...");
      for (int pos = currentPos; pos <= 90; pos++) {
        servo1.write(pos);
        servo2.write(180 - pos);
        delay(20);
      }
      currentPos = 90;
      Serial.println("✅ Garage doors closed");
    } else {
      Serial.println("⚠️ Doors already closed.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect

  // 1. Connect to WiFi to sync channel with Gatekeeper for ESP-NOW
  const char* WIFI_SSID = "aliswork";
  const char* WIFI_PASS = "12345678";
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi to sync channel ");
  // Wait up to 10 seconds
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println("\n✅ WiFi Ready! Channel synced.");

  // 2. Read and Print Hardware MAC Address
  Serial.println("\n=============================");
  Serial.println("   SERVO ESP-NOW RECEIVER");
  Serial.println("=============================");
  
  uint8_t mac[6];
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
    Serial.printf("ESP32 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }

  // 3. Initialize Servos
  Serial.println("\n🤖 Initializing Servos...");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Set to Closed/rest position initially
  servo1.write(90);
  servo2.write(90);
  currentPos = 90;
  Serial.println("✅ Servos set to closed position (90 degrees).");

  // 4. Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }
  
  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("✅ ESP-NOW Initialized.");
  Serial.println("📻 Listening for OPEN_DOORS / CLOSE_DOORS commands...");
  Serial.println("\n[Manual Testing]");
  Serial.println("Press 'f' to open both servos");
  Serial.println("Press 'e' to close both servos");
}

void loop() {
  // Manual fallback testing via USB serial
  if (Serial.available() > 0) {
    char key = Serial.read();

    if (key == 'f' || key == 'F') {
      if (currentPos == 0) {
        Serial.println("🚪 [Manual] Opening doors...");
        for (int pos = currentPos; pos <= 90; pos++) {
          servo1.write(pos);
          servo2.write(180 - pos);
          delay(20);
        }
        currentPos = 90;
        Serial.println("✅ Doors opened.");
      }
    }
    else if (key == 'e' || key == 'E') {
      if (currentPos == 90) {
        Serial.println("🚪 [Manual] Closing doors...");
        for (int pos = currentPos; pos >= 0; pos--) {
          servo1.write(pos);
          servo2.write(180 - pos);
          delay(20);
        }
        currentPos = 0;
        Serial.println("✅ Doors closed.");
      }
    }
  }
  delay(10); // Small delay to prevent watchdog reset
}