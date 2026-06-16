/*
 * ============================================================
 *  DEMO SERVO — Container Gate Controller
 * ============================================================
 *  Servo 1 (Left Gate)  → GPIO 12 — moves 100° clockwise
 *  Servo 2 (Right Gate) → GPIO 13 — moves 100° anti-clockwise
 *
 *  Both start at 90° (center).
 *  "Move Servos"    → Servo1 goes to 190°, Servo2 goes to -10° (clamped 0-180)
 *                      i.e. Servo1: 90→190 (capped 180), Servo2: 90→0 (approx)
 *  "Return to Place" → Both return to 90°
 *
 *  Board: ESP32 DevKit / any ESP32
 *  Library: ESP32Servo
 * ============================================================
 */

#include <WiFi.h>
#include <ESP32Servo.h>
#include <WebServer.h>

// ============================================================
//  CONFIGURATION
// ============================================================
const char* WIFI_SSID = "aliswork";
const char* WIFI_PASS = "12345678";

#define SERVO1_PIN 12   // Left gate
#define SERVO2_PIN 13   // Right gate

// Starting position (center is 90 degrees for servos)
// This prevents the startup jerk, as servos default to 90!
#define HOME_POS   90

// Both servos move 90 degrees, but in opposite directions
#define SERVO1_MOVE  180   // 90 + 90
#define SERVO2_MOVE  0     // 90 - 90

#define STATUS_LED 2

// ============================================================
//  GLOBALS
// ============================================================
Servo servo1;
Servo servo2;
WebServer server(80);

bool moved = false;

// ============================================================
//  WEB PAGE
// ============================================================
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Gate Controller</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: 'Segoe UI', Arial, sans-serif;
      background: linear-gradient(135deg, #0a0a1a 0%, #1a1a3e 50%, #0d1b2a 100%);
      color: #f0f0ff;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
    }
    .container {
      background: rgba(18, 18, 42, 0.95);
      border: 2px solid #6c63ff;
      border-radius: 20px;
      padding: 40px 50px;
      text-align: center;
      box-shadow: 0 0 40px rgba(108, 99, 255, 0.3);
      max-width: 420px;
      width: 90%;
    }
    h1 { font-size: 28px; margin-bottom: 8px; color: #ff2e63; }
    .subtitle { font-size: 14px; color: #6a6a8a; margin-bottom: 30px; }
    .status {
      font-size: 22px; font-weight: bold;
      margin-bottom: 30px; padding: 12px 24px;
      border-radius: 12px; transition: all 0.3s ease;
    }
    .status.moved {
      background: rgba(0, 230, 118, 0.15);
      color: #00e676; border: 1px solid #00e676;
    }
    .status.home {
      background: rgba(68, 138, 255, 0.15);
      color: #448aff; border: 1px solid #448aff;
    }
    .btn-group { display: flex; gap: 12px; justify-content: center; flex-wrap: wrap; }
    .btn {
      font-size: 16px; font-weight: bold;
      padding: 14px 24px; border: none; border-radius: 12px;
      cursor: pointer; transition: all 0.2s ease; min-width: 150px;
    }
    .btn:active { transform: scale(0.95); }
    .btn-move {
      background: linear-gradient(135deg, #00e676, #00c853);
      color: #000; box-shadow: 0 4px 20px rgba(0, 230, 118, 0.4);
    }
    .btn-move:hover { box-shadow: 0 6px 30px rgba(0, 230, 118, 0.6); }
    .btn-return {
      background: linear-gradient(135deg, #448aff, #2962ff);
      color: #fff; box-shadow: 0 4px 20px rgba(68, 138, 255, 0.4);
    }
    .btn-return:hover { box-shadow: 0 6px 30px rgba(68, 138, 255, 0.6); }
    .btn-s1 {
      background: linear-gradient(135deg, #ff9100, #e65100);
      color: #fff; box-shadow: 0 4px 15px rgba(255, 145, 0, 0.4);
    }
    .btn-s1:hover { box-shadow: 0 6px 25px rgba(255, 145, 0, 0.6); }
    .btn-s2 {
      background: linear-gradient(135deg, #b388ff, #7c4dff);
      color: #fff; box-shadow: 0 4px 15px rgba(179, 136, 255, 0.4);
    }
    .btn-s2:hover { box-shadow: 0 6px 25px rgba(179, 136, 255, 0.6); }
    .section-label {
      font-size: 11px; color: #6a6a8a; margin: 20px 0 8px;
      text-transform: uppercase; letter-spacing: 2px;
    }
    .info { margin-top: 24px; font-size: 12px; color: #6a6a8a; }
  </style>
</head>
<body>
  <div class="container">
    <h1>&#x1F6AA; Gate Controller</h1>
    <p class="subtitle">Fire Fauji &mdash; Container Gate Demo</p>
    <div id="status" class="status home">&#x1F512; HOME POSITION</div>

    <p class="section-label">Both Gates</p>
    <div class="btn-group">
      <button class="btn btn-move" onclick="sendCmd('move')">&#x1F513; Move Servos</button>
      <button class="btn btn-return" onclick="sendCmd('return')">&#x1F512; Return to Place</button>
    </div>

    <p class="section-label">Individual Control</p>
    <div class="btn-group">
      <button class="btn btn-s1" onclick="sendCmd('s1_move')">S1 Move</button>
      <button class="btn btn-s1" onclick="sendCmd('s1_return')">S1 Return</button>
    </div>
    <div class="btn-group" style="margin-top:10px">
      <button class="btn btn-s2" onclick="sendCmd('s2_move')">S2 Move</button>
      <button class="btn btn-s2" onclick="sendCmd('s2_return')">S2 Return</button>
    </div>

    <p class="info">Servo 1: 0&rarr;90&deg; &middot; Servo 2: 0&rarr;-90&deg; (from center)</p>
  </div>
  <script>
    function sendCmd(action) {
      fetch('/' + action).then(r => r.text()).then(t => {
        var s = document.getElementById('status');
        if (t.trim() === 'MOVED' || t.trim() === 'S1_MOVED' || t.trim() === 'S2_MOVED') {
          s.className = 'status moved';
          s.innerHTML = '&#x1F513; ' + t.trim();
        } else {
          s.className = 'status home';
          s.innerHTML = '&#x1F512; ' + t.trim();
        }
      }).catch(e => console.error(e));
    }
    fetch('/state').then(r => r.text()).then(t => {
      if (t.trim() === 'MOVED') {
        var s = document.getElementById('status');
        s.className = 'status moved';
        s.innerHTML = '&#x1F513; GATES OPEN';
      }
    });
  </script>
</body>
</html>
)rawliteral";

// ============================================================
//  HTTP HANDLERS
// ============================================================
void handleRoot() {
  server.send(200, "text/html", HTML_PAGE);
}

void handleMove() {
  Serial.println("🔓 Moving servos...");

  // Medium sweep: 90->180 (S1) and 90->0 (S2)
  for (int i = 0; i <= 90; i += 2) {
    servo1.write(HOME_POS + i);        // 90 → 180
    servo2.write(HOME_POS - i);        // 90 → 0
    delay(10); // Medium speed
  }
  servo1.write(SERVO1_MOVE);
  servo2.write(SERVO2_MOVE);
  moved = true;
  digitalWrite(STATUS_LED, HIGH);

  Serial.printf("✅ S1 → %d°, S2 → %d°\n", SERVO1_MOVE, SERVO2_MOVE);
  server.send(200, "text/plain", "MOVED");
}

void handleReturn() {
  Serial.println("🔒 Returning servos to home...");

  // Medium return: 180->90 (S1) and 0->90 (S2)
  for (int i = 0; i <= 90; i += 2) {
    servo1.write(SERVO1_MOVE - i);     // 180 → 90
    servo2.write(SERVO2_MOVE + i);     // 0 → 90
    delay(10); // Medium speed
  }
  servo1.write(HOME_POS);
  servo2.write(HOME_POS);
  moved = false;
  digitalWrite(STATUS_LED, LOW);

  Serial.printf("✅ Both servos → home\n");
  server.send(200, "text/plain", "HOME");
}

void handleState() {
  server.send(200, "text/plain", moved ? "MOVED" : "HOME");
}

// Individual servo handlers
void handleS1Move() {
  Serial.println("\xF0\x9F\x94\x93 Servo 1 moving...");
  for (int i = 0; i <= 90; i += 2) {
    servo1.write(HOME_POS + i);
    delay(10);
  }
  servo1.write(SERVO1_MOVE);
  Serial.printf("\xE2\x9C\x85 Servo1 \xE2\x86\x92 %d\xC2\xB0\n", SERVO1_MOVE);
  server.send(200, "text/plain", "S1_MOVED");
}

void handleS1Return() {
  Serial.println("\xF0\x9F\x94\x92 Servo 1 returning...");
  for (int i = 0; i <= 90; i += 2) {
    servo1.write(SERVO1_MOVE - i);
    delay(10);
  }
  servo1.write(HOME_POS);
  Serial.printf("\xE2\x9C\x85 Servo1 \xE2\x86\x92 %d\xC2\xB0\n", HOME_POS);
  server.send(200, "text/plain", "S1_HOME");
}

void handleS2Move() {
  Serial.println("\xF0\x9F\x94\x93 Servo 2 moving...");
  for (int i = 0; i <= 90; i += 2) {
    servo2.write(HOME_POS - i);
    delay(10);
  }
  servo2.write(SERVO2_MOVE);
  Serial.printf("\xE2\x9C\x85 Servo2 \xE2\x86\x92 %d\xC2\xB0\n", SERVO2_MOVE);
  server.send(200, "text/plain", "S2_MOVED");
}

void handleS2Return() {
  Serial.println("\xF0\x9F\x94\x92 Servo 2 returning...");
  for (int i = 0; i <= 90; i += 2) {
    servo2.write(SERVO2_MOVE + i);
    delay(10);
  }
  servo2.write(HOME_POS);
  Serial.printf("\xE2\x9C\x85 Servo2 \xE2\x86\x92 %d\xC2\xB0\n", HOME_POS);
  server.send(200, "text/plain", "S2_HOME");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println(" DEMO SERVO — Container Gate Controller");
  Serial.println("========================================");

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Attach servos and go to home (center)
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(HOME_POS);
  servo2.write(HOME_POS);
  Serial.printf("✅ Servos at home position (%d°)\n", HOME_POS);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected!");
    Serial.print("   IP: "); Serial.println(WiFi.localIP());
    Serial.print("   Open: http://"); Serial.print(WiFi.localIP()); Serial.println("/");
  } else {
    Serial.println("\n❌ WiFi failed. Restarting...");
    ESP.restart();
  }

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/return", handleReturn);
  server.on("/state", handleState);
  server.on("/s1_move", handleS1Move);
  server.on("/s1_return", handleS1Return);
  server.on("/s2_move", handleS2Move);
  server.on("/s2_return", handleS2Return);
  server.begin();

  Serial.println("🟢 READY — open URL in browser.");

  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(100);
    digitalWrite(STATUS_LED, LOW);  delay(100);
  }
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  server.handleClient();
  delay(1);
}
