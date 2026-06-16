from ultralytics import YOLO
import cv2
import serial
import time

# --- 1. CONNECT TO ESP32 ---
try:
    # Port is set exactly to COM4
    esp = serial.Serial('COM4', baudrate=115200, timeout=0.1)
    time.sleep(2)  # Give ESP32 time to wake up after connection
    print("✅ SUCCESS: Connected to ESP32 on COM4!")
except Exception as e:
    print(f"❌ ERROR: COULD NOT CONNECT TO ESP32 on COM4: {e}")
    print("Make sure the Arduino Serial Monitor is CLOSED!")
    esp = None

# --- 2. LOAD AI MODEL ---
model = YOLO("best.pt")
cap = cv2.VideoCapture(0)

# --- 3. THE STATE TRACKER ---
# We assume the car starts out stopped
last_command = 'S' 

print("\n🎥 Starting camera... Waiting for fire.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame from camera.")
        break

    # verbose=False stops YOLO from printing "Speed: 3.0ms..." every single frame
    results = model(frame, conf=0.5, verbose=False)
    annotated_frame = results[0].plot()

    # If the array of boxes is greater than 0, it saw a fire
    fire_detected = len(results[0].boxes) > 0

    # --- 4. SENDING COMMANDS (ONLY WHEN STATE CHANGES) ---
    if fire_detected and last_command != 'F':
        print("\n🔥 YOLO SEES FIRE!")
        if esp is not None:
            esp.write(b'F')
            print("   -> ⚡ SIGNAL 'F' SENT TO USB (COM4)")
        else:
            print("   -> ❌ BUT ESP32 IS NOT CONNECTED!")
        last_command = 'F'
        
    elif not fire_detected and last_command != 'S':
        print("\n🛑 YOLO SEES NO FIRE.")
        if esp is not None:
            esp.write(b'S')
            print("   -> ⚡ SIGNAL 'S' SENT TO USB (COM4)")
        last_command = 'S'

    # --- 5. LISTEN FOR ESP32 CONFIRMATION ---
    if esp is not None:
        while esp.in_waiting > 0:
            # Read whatever the Sender ESP32 sends back over the USB cable
            esp_reply = esp.readline().decode('utf-8', errors='ignore').strip()
            if esp_reply:
                print(f"   📡 SENDER ESP32 SAYS: {esp_reply}")

    # Show the live camera feed
    cv2.imshow("Fire Detection", annotated_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- 6. CLEAN UP ---
cap.release()
cv2.destroyAllWindows()
if esp is not None:
    esp.close()
    print("Closed connection to COM4.")