# 🔥 Fire Fauji Command Center v3 (Full Project Context)

This document provides a comprehensive overview of the **Fire Fauji** autonomous firefighting system. It explains the entire architecture, the hardware components, the software ecosystem, and the step-by-step mission flow.

---

## 🏗️ System Architecture

The project is distributed across four main components: three microcontrollers (two ESP32-CAMs and one standard ESP32) and one central Python Dashboard running on a PC.

### 1. Python Dashboard (`fire_dashboard.py`)
The "Brain" of the operation. It runs on the laptop, processing video feeds and issuing commands.
- **AI Processing**: Uses a custom-trained YOLOv8 model (`best.onnx`) running via ONNX Runtime with DirectML for AMD GPU acceleration.
- **Dual Camera Handling**: Seamlessly switches between the Box camera (for scanning) and the Car camera (for driving).
- **Dynamic Steering**: Tracks the bounding box of the fire and sends HTTP commands (`/forward`, `/turn_left`, etc.) to steer the car dynamically.
- **Route Recording & Backtracking**: Quietly records exactly how long the car spent driving forward or turning. When the mission is over, it reverses this history to back the car exactly into the garage.
- **State Machine**: Manages the mission flow (Scanning → Deploying → Exiting Garage → 360° Scan → Approaching → Pumping → Returning).

### 2. Box ESP-CAM (`box_espcam_final.ino`)
The "Watchtower". This sits on top of the fire station box.
- **Network**: Static IP `192.168.137.100`.
- **Stream**: Hosts a live MJPEG stream on port 80.
- **HTTP Server**: Listens on port 81 for `/flash_on` and `/flash_off`.

### 3. Gatekeeper ESP32 (`gatekeeper_esp32.ino`)
The "Bridge" and "Garage Controller". It sits plugged into the PC via USB.
- **Role**: Wakes the car up instantly via ESP-NOW, and controls the physical garage doors.
- **Network**: Static IP `192.168.137.110`.
- **Serial Comms**: Listens for the letter `F` from the Python Dashboard via the USB Serial port.
- **ESP-NOW**: Instantly beams a lightweight, ultra-fast `DEPLOY` wireless signal directly to the Car ESP-CAMs.
- **Servos**: Controls two servo motors on **GPIO 26** and **GPIO 27** that act as the garage doors.
- **HTTP Server**: Listens on port 81 for `/open_doors` and `/close_doors` commands from the Python Dashboard.

### 4. Car 1 ESP-CAM (`car_espcam.ino`)
The "Autonomous Vehicle". It sits inside the garage box until deployed.
- **Network**: Static IP `192.168.137.158`.
- **ESP-NOW Listener**: Stays asleep/offline to save power until it receives the `DEPLOY` signal from the Gatekeeper.
- **Direct Motor Control**: Connected directly to an L298N motor driver using **GPIO 14, 15, 13, 12** (with ENA/ENB permanently jumpered to 5V). 
- **Pump**: Connected to a relay on **GPIO 2**.
- **HTTP Server**: Once deployed, it connects to Wi-Fi and hosts a command server on port 81 (`/forward`, `/turn_left`, `/pump_on`, etc.) and a video stream on port 80.

### 5. Car 2 ESP-CAM (`car2_espcam.ino`)
Identical to Car 1, deployed as a second unit.
- **Network**: Static IP `192.168.137.159`.
- All other features are the same as Car 1.

---

## 🌐 Static IP Quick Reference

| Device | IP Address | Stream URL | Status URL | Control URL |
|---|---|---|---|---|
| **Box ESP-CAM** | `192.168.137.100` | `http://192.168.137.100/stream` | `http://192.168.137.100:81/status` | `http://192.168.137.100:81/` |
| **Gatekeeper ESP32** | `192.168.137.110` | — | — | `http://192.168.137.110:81/` |
| **Car 1 ESP-CAM** | `192.168.137.158` | `http://192.168.137.158/stream` | `http://192.168.137.158:81/status` | — |
| **Car 2 ESP-CAM** | `192.168.137.159` | `http://192.168.137.159/stream` | `http://192.168.137.159:81/status` | — |

> **Tip**: Open any stream URL in your browser to watch the live camera feed directly!

---

## 🚦 Complete Mission Flow (Step-by-Step)

Here is exactly what happens when the system is armed:

1. **Scanning**: The Python Dashboard connects to the Box ESP-CAM stream. It continuously runs YOLO on the frame, looking for fire.
2. **Fire Confirmed**: If a fire is detected continuously for 0.5 seconds, the Dashboard locks on and enters `FIRE_CONFIRMED`.
3. **Deployment**:
   - The Dashboard sends `F` to the Gatekeeper over USB.
   - The Gatekeeper instantly beams `DEPLOY` to the Car(s) via ESP-NOW.
   - The Dashboard sends an HTTP request (`/open_doors`) to the Gatekeeper ESP32 to open the garage doors.
   - The Dashboard closes the Box camera stream and waits **13 seconds** for the shutters to fully open and the Car to boot.
4. **Exiting Garage**:
   - After 13 seconds, the Dashboard switches to `EXITING_GARAGE`.
   - The Dashboard sends a blind `/forward` command to the car for exactly **5.0 seconds** to drive the car out into the open.
5. **360° Fire Scan**:
   - The car stops and begins spinning in place (turning right).
   - YOLO runs on the car's live camera feed, scanning for fire in all directions.
   - As soon as fire is detected continuously for **0.5 seconds**, the car locks on and stops spinning.
6. **Car Approaching (Dynamic Steering)**:
   - Using YOLO on the car's video feed, the Dashboard calculates where the fire is in the frame.
   - If the fire is on the left, it sends `/turn_left`. If right, `/turn_right`. If centered, `/forward`.
   - *Crucially*, the dashboard records exactly how long each of these commands lasts.
7. **Distance Estimation**: 
   - The Dashboard calculates the size of the fire's bounding box relative to the video frame.
   - When the fire bounding box takes up > 3% of the frame (roughly 5 meters away), the car stops.
8. **Pumping**:
   - The Dashboard sends `/pump_on` to shoot water at the fire.
   - The car must see the fire in pump range for **1.0 seconds** before activating.
9. **Extinguished**:
   - The Dashboard watches the fire. If no fire is detected for 3 continuous seconds, it assumes the fire is dead.
   - It sends `/pump_off`.
10. **Return to Base (Backtracking)**:
    - The Dashboard takes the recorded history of the car's movements and reverses them.
    - It sends the inverted commands (e.g., if it drove forward for 4 seconds, it sends `/reverse` for 4 seconds).
    - The car seamlessly backs itself up into the garage.
11. **Mission Complete**:
    - The Dashboard hits `/close_doors` on the Gatekeeper to shut the garage. The system resets and awaits the next fire.