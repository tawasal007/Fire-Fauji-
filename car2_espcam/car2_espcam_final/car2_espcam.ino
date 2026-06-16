/*
 * ============================================================
 *  CAR 2 — Manual Control (No Camera)
 * ============================================================
 *  Camera is dead, so this runs as a manual-only unit.
 *  - Connects to WiFi (same network as dashboard)
 *  - Serves a premium joystick web UI at http://192.168.137.159/
 *  - WebSocket on port 81 for real-time joystick control
 *  - HTTP endpoints on port 80 for dashboard compatibility
 *
 *  Board: AI Thinker ESP32-CAM
 *  Partition: Huge APP (3MB No OTA / 1MB SPIFFS)
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ============================================================
//  WIFI — connects to laptop hotspot (same network as dashboard)
// ============================================================
const char* WIFI_SSID = "aliswork";
const char* WIFI_PASS = "12345678";

IPAddress staticIP(192, 168, 137, 159);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);

// ============================================================
//  HARDWARE PINS (same as Car 1)
// ============================================================
#define IN1 14
#define IN2 15
#define IN3 13
#define IN4 12
#define ENA 1    // U0T — Serial TX hijacked for motor enable
#define ENB 3    // U0R — Serial RX hijacked for motor enable
#define RELAY 2  // Water pump relay (ACTIVE-LOW)
#define FLASH_LED 4
#define STATUS_LED 33

// ============================================================
//  SERVERS
// ============================================================
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

bool pumpState = false;
String lastCmd = "S";

// ============================================================
//  MOTOR FUNCTIONS
// ============================================================
void Forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void Backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void Left() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void Right() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}
void Stop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ============================================================
//  WEB PAGE — Premium Joystick UI
// ============================================================
const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Car 2 — Manual Control</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{
  font-family:'Segoe UI',system-ui,-apple-system,sans-serif;
  background:linear-gradient(135deg,#0a0a1a 0%,#1a1a3e 50%,#0d0d2b 100%);
  color:#fff;height:100vh;overflow:hidden;
  display:flex;flex-direction:column;align-items:center;
  user-select:none;-webkit-user-select:none;
  touch-action:none;
}

/* ---- Header ---- */
.header{
  padding:12px 20px;width:100%;
  background:rgba(255,255,255,0.03);
  border-bottom:1px solid rgba(255,255,255,0.06);
  display:flex;justify-content:space-between;align-items:center;
}
.title{font-size:18px;font-weight:700;letter-spacing:1px}
.title span{color:#ff6b6b;font-weight:800}
.status{display:flex;align-items:center;gap:8px;font-size:13px}
.dot{width:10px;height:10px;border-radius:50%;transition:all .3s}
.dot.on{background:#4ade80;box-shadow:0 0 8px #4ade80}
.dot.off{background:#f87171;box-shadow:0 0 8px #f87171}

/* ---- Info Bar ---- */
.info{
  display:flex;gap:16px;padding:10px 20px;
  font-size:12px;color:rgba(255,255,255,0.5);
  width:100%;justify-content:center;
}
.info-item{
  background:rgba(255,255,255,0.04);
  padding:6px 14px;border-radius:20px;
  border:1px solid rgba(255,255,255,0.06);
}
.info-val{color:#fff;font-weight:600;margin-left:4px}

/* ---- Main Area ---- */
.main{
  flex:1;display:flex;flex-direction:column;
  align-items:center;justify-content:center;
  width:100%;gap:24px;padding:10px;
}

/* ---- Joystick ---- */
.joy-container{
  position:relative;width:220px;height:220px;
  background:radial-gradient(circle,rgba(255,255,255,0.04) 0%,rgba(255,255,255,0.01) 70%);
  border-radius:50%;
  border:2px solid rgba(255,255,255,0.08);
  box-shadow:0 0 40px rgba(100,100,255,0.05),inset 0 0 30px rgba(0,0,0,0.3);
}
.joy-ring{
  position:absolute;top:50%;left:50%;
  width:180px;height:180px;
  transform:translate(-50%,-50%);
  border-radius:50%;
  border:1px dashed rgba(255,255,255,0.08);
}
.joy-crosshair-h,.joy-crosshair-v{
  position:absolute;background:rgba(255,255,255,0.04);
}
.joy-crosshair-h{top:50%;left:10%;width:80%;height:1px}
.joy-crosshair-v{left:50%;top:10%;width:1px;height:80%}
.joy-knob{
  position:absolute;width:64px;height:64px;
  border-radius:50%;
  background:radial-gradient(circle at 40% 35%,rgba(130,130,255,0.5),rgba(80,80,200,0.3));
  border:2px solid rgba(150,150,255,0.3);
  box-shadow:0 0 20px rgba(100,100,255,0.2),0 4px 12px rgba(0,0,0,0.4);
  top:50%;left:50%;
  transform:translate(-50%,-50%);
  transition:box-shadow .2s;
  z-index:2;
}
.joy-knob.active{
  box-shadow:0 0 30px rgba(130,130,255,0.5),0 4px 16px rgba(0,0,0,0.5);
  border-color:rgba(180,180,255,0.5);
}
.joy-label{
  position:absolute;font-size:11px;color:rgba(255,255,255,0.15);
  font-weight:700;letter-spacing:2px;
}
.joy-label.up{top:8px;left:50%;transform:translateX(-50%)}
.joy-label.down{bottom:8px;left:50%;transform:translateX(-50%)}
.joy-label.left{left:8px;top:50%;transform:translateY(-50%)}
.joy-label.right{right:8px;top:50%;transform:translateY(-50%)}

/* ---- Direction Display ---- */
.dir-display{
  font-size:14px;font-weight:600;
  padding:8px 24px;border-radius:20px;
  background:rgba(255,255,255,0.04);
  border:1px solid rgba(255,255,255,0.08);
  min-width:160px;text-align:center;
  transition:all .2s;
}
.dir-display.moving{
  background:rgba(100,200,255,0.08);
  border-color:rgba(100,200,255,0.2);
  color:#7dd3fc;
}

/* ---- Pump Button ---- */
.pump-area{
  display:flex;gap:12px;align-items:center;
  padding:8px;
}
.pump-btn{
  padding:14px 32px;font-size:15px;font-weight:700;
  border:none;border-radius:12px;cursor:pointer;
  letter-spacing:1px;transition:all .2s;
  text-transform:uppercase;
}
.pump-on{
  background:linear-gradient(135deg,#0ea5e9,#3b82f6);
  color:#fff;
  box-shadow:0 4px 15px rgba(59,130,246,0.3);
}
.pump-on:active{transform:scale(0.96);box-shadow:0 2px 8px rgba(59,130,246,0.2)}
.pump-off{
  background:rgba(255,255,255,0.06);
  color:rgba(255,255,255,0.5);
  border:1px solid rgba(255,255,255,0.1);
}
.pump-off:active{transform:scale(0.96)}
.pump-status{
  font-size:13px;padding:8px 16px;border-radius:8px;
  background:rgba(255,255,255,0.03);min-width:90px;text-align:center;
}
.pump-status.on{color:#4ade80}
.pump-status.off{color:rgba(255,255,255,0.3)}

/* ---- E-Stop ---- */
.estop{
  padding:12px 40px;font-size:14px;font-weight:800;
  border:2px solid rgba(239,68,68,0.3);
  background:rgba(239,68,68,0.08);
  color:#f87171;border-radius:12px;
  cursor:pointer;letter-spacing:2px;
  transition:all .2s;text-transform:uppercase;
}
.estop:active{
  background:rgba(239,68,68,0.2);
  transform:scale(0.96);
}
</style>
</head>
<body>

<div class="header">
  <div class="title">CAR <span>2</span> — MANUAL</div>
  <div class="status">
    <div class="dot off" id="dot"></div>
    <span id="connText">Disconnected</span>
  </div>
</div>

<div class="info">
  <div class="info-item">IP <span class="info-val">192.168.137.159</span></div>
  <div class="info-item">MODE <span class="info-val">MANUAL</span></div>
  <div class="info-item">CAM <span class="info-val" style="color:#f87171">OFFLINE</span></div>
</div>

<div class="main">
  <div class="joy-container" id="joyBase">
    <div class="joy-ring"></div>
    <div class="joy-crosshair-h"></div>
    <div class="joy-crosshair-v"></div>
    <div class="joy-label up">FWD</div>
    <div class="joy-label down">REV</div>
    <div class="joy-label left">L</div>
    <div class="joy-label right">R</div>
    <div class="joy-knob" id="knob"></div>
  </div>

  <div class="dir-display" id="dirDisplay">STOPPED</div>

  <div class="pump-area">
    <button class="pump-btn pump-on" onclick="sendCmd('1')">PUMP ON</button>
    <div class="pump-status off" id="pumpSt">OFF</div>
    <button class="pump-btn pump-off" onclick="sendCmd('0')">PUMP OFF</button>
  </div>

  <button class="estop" onclick="sendCmd('S')">EMERGENCY STOP</button>
</div>

<script>
// ---- WebSocket ----
var ws;
var connected = false;

function connectWS() {
  ws = new WebSocket('ws://' + location.hostname + ':81/', ['arduino']);

  ws.onopen = function() {
    connected = true;
    document.getElementById('dot').className = 'dot on';
    document.getElementById('connText').textContent = 'Connected';
  };

  ws.onclose = function() {
    connected = false;
    document.getElementById('dot').className = 'dot off';
    document.getElementById('connText').textContent = 'Disconnected';
    setTimeout(connectWS, 2000);
  };

  ws.onerror = function() { ws.close(); };
}

connectWS();

function sendCmd(c) {
  if (connected && ws.readyState === 1) ws.send(c);

  // Update pump status
  var ps = document.getElementById('pumpSt');
  if (c === '1') { ps.textContent = 'ON'; ps.className = 'pump-status on'; }
  if (c === '0') { ps.textContent = 'OFF'; ps.className = 'pump-status off'; }
}

// ---- Joystick ----
var base = document.getElementById('joyBase');
var knob = document.getElementById('knob');
var dir = document.getElementById('dirDisplay');
var baseRect, centerX, centerY, maxR;
var dragging = false;
var lastSent = 'S';

function recalc() {
  baseRect = base.getBoundingClientRect();
  centerX = baseRect.left + baseRect.width / 2;
  centerY = baseRect.top + baseRect.height / 2;
  maxR = baseRect.width / 2 - 32;
}
recalc();
window.addEventListener('resize', recalc);

function getDirection(dx, dy) {
  var dist = Math.sqrt(dx * dx + dy * dy);
  if (dist < maxR * 0.25) return 'S';
  var angle = Math.atan2(-dy, dx) * 180 / Math.PI;
  if (angle < 0) angle += 360;
  if (angle >= 45 && angle < 135) return 'F';
  if (angle >= 135 && angle < 225) return 'L';
  if (angle >= 225 && angle < 315) return 'B';
  return 'R';
}

var dirLabels = {
  'F': 'FORWARD', 'B': 'REVERSE',
  'L': 'TURN LEFT', 'R': 'TURN RIGHT',
  'S': 'STOPPED'
};

function moveKnob(cx, cy) {
  var dx = cx - centerX;
  var dy = cy - centerY;
  var dist = Math.sqrt(dx * dx + dy * dy);

  if (dist > maxR) {
    dx = dx / dist * maxR;
    dy = dy / dist * maxR;
  }

  knob.style.left = (baseRect.width / 2 + dx) + 'px';
  knob.style.top = (baseRect.height / 2 + dy) + 'px';

  var d = getDirection(dx, dy);
  dir.textContent = dirLabels[d];
  dir.className = d === 'S' ? 'dir-display' : 'dir-display moving';

  if (d !== lastSent) {
    sendCmd(d);
    lastSent = d;
  }
}

function resetKnob() {
  knob.style.left = '50%';
  knob.style.top = '50%';
  knob.classList.remove('active');
  dragging = false;
  dir.textContent = 'STOPPED';
  dir.className = 'dir-display';
  if (lastSent !== 'S') { sendCmd('S'); lastSent = 'S'; }
}

// Touch events
base.addEventListener('touchstart', function(e) {
  e.preventDefault(); recalc(); dragging = true;
  knob.classList.add('active');
  var t = e.touches[0]; moveKnob(t.clientX, t.clientY);
}, {passive: false});

document.addEventListener('touchmove', function(e) {
  if (!dragging) return; e.preventDefault();
  var t = e.touches[0]; moveKnob(t.clientX, t.clientY);
}, {passive: false});

document.addEventListener('touchend', function(e) {
  if (dragging) resetKnob();
});

// Mouse events
base.addEventListener('mousedown', function(e) {
  e.preventDefault(); recalc(); dragging = true;
  knob.classList.add('active');
  moveKnob(e.clientX, e.clientY);
});

document.addEventListener('mousemove', function(e) {
  if (!dragging) return;
  moveKnob(e.clientX, e.clientY);
});

document.addEventListener('mouseup', function() {
  if (dragging) resetKnob();
});
</script>
</body>
</html>
)=====";

// ============================================================
//  HTTP HANDLERS — Dashboard compatibility endpoints
// ============================================================
void handleRoot() {
  server.send_P(200, "text/html", webpage);
}

void handleForward() {
  Forward(); lastCmd = "F";
  server.send(200, "text/plain", "FORWARD");
}
void handleTurnLeft() {
  Left(); lastCmd = "L";
  server.send(200, "text/plain", "TURN_LEFT");
}
void handleTurnRight() {
  Right(); lastCmd = "R";
  server.send(200, "text/plain", "TURN_RIGHT");
}
void handleReverse() {
  Backward(); lastCmd = "B";
  server.send(200, "text/plain", "REVERSE");
}
void handleStopMotors() {
  Stop(); lastCmd = "S";
  server.send(200, "text/plain", "MOTORS_STOPPED");
}
void handlePumpOn() {
  digitalWrite(RELAY, LOW); pumpState = true;
  server.send(200, "text/plain", "PUMP_ON");
}
void handlePumpOff() {
  digitalWrite(RELAY, HIGH); pumpState = false;
  server.send(200, "text/plain", "PUMP_OFF");
}
void handleStatus() {
  String text = "CAR2_OK\nROLE=CAR_2_MANUAL\nIP=";
  text += WiFi.localIP().toString();
  text += "\nCAMERA=OFFLINE\nPUMP=";
  text += pumpState ? "ON" : "OFF";
  text += "\nLAST_CMD=" + lastCmd + "\n";
  server.send(200, "text/plain", text);
}

// ============================================================
//  WEBSOCKET HANDLER
// ============================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT && length > 0) {
    char c = payload[0];

    if (c == 'F')      Forward();
    else if (c == 'B') Backward();
    else if (c == 'L') Left();
    else if (c == 'R') Right();
    else if (c == 'S') Stop();
    else if (c == '1') { digitalWrite(RELAY, LOW);  pumpState = true; }
    else if (c == '0') { digitalWrite(RELAY, HIGH); pumpState = false; }
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  CAR 2 — Manual Control (No Camera)");
  Serial.println("========================================");

  // Flash LED off
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);  // ON = booting

  // Motor direction pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Stop();

  // Relay (active-low: HIGH = OFF)
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);

  // Connect to WiFi (STA mode — same network as dashboard)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.println("WARNING: Static IP config failed.");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed! Falling back to AP mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("CAR2-MANUAL", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("\nWiFi connected!");
    Serial.print("  IP: ");
    Serial.println(WiFi.localIP());
  }

  Serial.print("  Control URL: http://");
  Serial.print(WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP());
  Serial.println("/");

  // IMPORTANT: Hijack Serial TX/RX pins for motor enables
  // Must be done AFTER all Serial prints!
  Serial.println("  Hijacking GPIO 1/3 for motor enables...");
  Serial.println("\n*** CAR 2 READY — Open URL in browser ***");
  Serial.println("  (No more Serial output after this)");
  delay(200);

  Serial.end();  // Release TX/RX
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
  pinMode(ENB, OUTPUT);
  digitalWrite(ENB, HIGH);

  // HTTP endpoints (dashboard compatible)
  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/turn_left", handleTurnLeft);
  server.on("/turn_right", handleTurnRight);
  server.on("/reverse", handleReverse);
  server.on("/stop_motors", handleStopMotors);
  server.on("/pump_on", handlePumpOn);
  server.on("/pump_off", handlePumpOff);
  server.on("/status", handleStatus);
  server.begin();

  // WebSocket for joystick
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Status LED off = ready
  digitalWrite(STATUS_LED, HIGH);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  webSocket.loop();
  server.handleClient();
}
