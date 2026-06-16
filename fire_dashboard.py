"""
============================================================
  FIRE FAUJI COMMAND CENTER v3 — Return-to-Base Dashboard
============================================================
  Mission flow:
    1. Station camera confirms fire and deploys the car.
    2. Car camera + YOLO steer toward the fire.
    3. Every movement command and its duration are recorded.
    4. END MISSION stops the pump and replays the saved path backward.
    5. Mission completes only after the timed return sequence finishes.

  Return navigation is dead-reckoning based on motor-command timing.
  It does not use GPS or wheel encoders, so calibrate RETURN_TIME_SCALE.
============================================================
"""

import cv2
import numpy as np
import os
import sys
import time
import math
import serial
import threading
import queue
import requests
import tkinter as tk
from PIL import Image, ImageTk
from datetime import datetime

# Prevent ultralytics from auto-installing onnxruntime (which overrides directml)
os.environ['YOLO_AUTOINSTALL'] = 'false'

# Force AMD GPU (DirectML) for ONNX Runtime — must be done BEFORE ultralytics loads
try:
    import onnxruntime as ort
    _OriginalSession = ort.InferenceSession

    class _DMLSession(_OriginalSession):
        """Wraps InferenceSession to always use DirectML (AMD GPU) first."""
        def __init__(self, *args, **kwargs):
            kwargs['providers'] = ['DmlExecutionProvider', 'CPUExecutionProvider']
            super().__init__(*args, **kwargs)
            actual = self.get_providers()
            print(f"   🎮 ONNX session using: {actual}")

    ort.InferenceSession = _DMLSession
    print("🎮 DirectML (AMD GPU) patched into ONNX Runtime.")
except ImportError:
    print("⚠️  onnxruntime not installed — will use PyTorch CPU.")

from ultralytics import YOLO

# ================================================================
#  CONFIGURATION
# ================================================================

# Gatekeeper ESP32 (USB serial)
COM_PORT = "COM3"
BAUD_RATE = 115200

# YOLO model — ONNX + DirectML uses AMD GPU (RX 5600M)
# Falls back to best.pt (CPU) if ONNX not available
YOLO_MODEL_ONNX = "best.onnx"
YOLO_MODEL_PT   = "best.pt"
YOLO_CONF = 0.55                 # Box camera confidence (stationary, clear view)
YOLO_IMGSZ = 320                 # Smaller input = faster inference for box
CAR_YOLO_CONF = 0.45             # Lowered confidence so it detects fire even in the dark!
CAR_YOLO_IMGSZ = 320             # Match ONNX static input size
CAR_PUMP_CONFIRM_TIME = 0.1      # Seconds to confirm fire in pump range before pumping

# Deployment timing
DEPLOY_WAIT_SEC = 13.0           # Wait for garage shutters to open
GARAGE_EXIT_SEC = 2.0            # Drive forward to exit garage
SCAN_360_CONFIRM_SEC = 0.1       # Fire detection time during 360 scan

# Fire confirmation (station camera)
FIRE_CONFIRM_TIME = 0.5          # Lowered to 0.5s for quick testing

# Box ESP-CAM (station camera on top of main box body)
BOX_IP = "192.168.137.100"
BOX_CMD_PORT = 81
BOX_STREAM_URL = f"http://{BOX_IP}/stream"

# Gatekeeper ESP32 (Servos)
GATEKEEPER_IP = "192.168.137.110"
GATEKEEPER_CMD_PORT = 81

# Car 1 ESP-CAM network
CAR_IP = "192.168.137.158"
CAR_CMD_PORT = 81
CAR_STREAM_URL = f"http://{CAR_IP}/stream"

# Car 2 is manual-only (camera dead) — controlled via web joystick at http://192.168.137.159/
# Servo ESP32 — receives OPEN_DOORS/CLOSE_DOORS via Gatekeeper ESP-NOW
# (No direct HTTP connection needed — Gatekeeper forwards commands)

# Distance estimation thresholds (bbox area as fraction of frame)
PUMP_BBOX_THRESHOLD = 0.03       # Lowered to 3% to trigger pump further away (~5 meters)
CLOSE_THRESHOLD = 0.05           # > 5% → close
MEDIUM_THRESHOLD = 0.02          # > 2% → medium

# Pump / extinguish timing
EXTINGUISH_TIMEOUT = 3.0         # Seconds of no fire → declare extinguished
CAR_CONNECT_TIMEOUT = 60.0       # Max seconds to wait for car camera

# Steering (fire center_x as fraction of frame width)
STEER_LEFT_ZONE = 0.35           # fire center_x < 0.35 → turn left
STEER_RIGHT_ZONE = 0.65          # fire center_x > 0.65 → turn right

# Command / return-path timing
COMMAND_THROTTLE_SEC = 0.30      # Minimum gap between changed movement commands
MIN_ROUTE_SEGMENT_SEC = 0.12     # Ignore tiny steering jitter shorter than this
RETURN_TIME_SCALE = 1.00         # Increase/decrease after a straight-line calibration
RETURN_SETTLE_SEC = 0.15         # Small stop between reversed path segments
RETURN_HTTP_TIMEOUT = 1.0
RETURN_HTTP_RETRIES = 3

# Station camera
STATION_WEBCAM_INDEX = BOX_STREAM_URL   # Use box ESP-CAM stream instead of laptop webcam

# Display
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480


# ================================================================
#  COLORS — Premium dark theme
# ================================================================
class C:
    BG_DARK     = "#0a0a12"
    BG_PANEL    = "#12122a"
    BG_HEADER   = "#0d1b2a"
    BG_LOG      = "#08081a"
    VIDEO_BG    = "#06060e"
    RED         = "#ff2e63"
    GREEN       = "#00e676"
    ORANGE      = "#ff9100"
    BLUE        = "#448aff"
    PURPLE      = "#b388ff"
    CYAN        = "#18ffff"
    GOLD        = "#ffd740"
    PINK        = "#ff80ab"
    TEXT        = "#f0f0ff"
    TEXT_DIM    = "#6a6a8a"
    BORDER      = "#1e1e44"
    ACCENT      = "#6c63ff"
    SUCCESS_BG  = "#003d00"


# ================================================================
#  MISSION STATES
# ================================================================
class State:
    SCANNING         = "SCANNING"
    FIRE_CONFIRMED   = "FIRE_CONFIRMED"
    DEPLOYING        = "DEPLOYING"
    EXITING_GARAGE   = "EXITING_GARAGE"
    SCANNING_360     = "SCANNING_360"
    CAR_APPROACHING  = "CAR_APPROACHING"
    PUMPING          = "PUMPING"
    EXTINGUISHED     = "EXTINGUISHED"
    RETURN_READY     = "RETURN_READY"
    RETURNING        = "RETURNING"
    RETURN_FAILED    = "RETURN_FAILED"
    MISSION_COMPLETE = "MISSION_COMPLETE"

STATE_DISPLAY = {
    State.SCANNING:         ("🔍  SCANNING",              C.BLUE),
    State.FIRE_CONFIRMED:   ("🔥  FIRE CONFIRMED",        C.RED),
    State.DEPLOYING:        ("🚀  DEPLOYING CAR",         C.ORANGE),
    State.EXITING_GARAGE:   ("🚗  EXITING GARAGE",        C.CYAN),
    State.SCANNING_360:     ("🔄  360° SCANNING",         C.PURPLE),
    State.CAR_APPROACHING:  ("🚗  CAR APPROACHING",       C.ORANGE),
    State.PUMPING:          ("💧  PUMPING WATER",          C.CYAN),
    State.EXTINGUISHED:     ("✅  FIRE EXTINGUISHED",      C.GREEN),
    State.RETURN_READY:     ("↩️  READY TO RETURN",        C.ORANGE),
    State.RETURNING:        ("🏠  RETURNING TO BASE",      C.CYAN),
    State.RETURN_FAILED:    ("❌  RETURN FAILED",          C.RED),
    State.MISSION_COMPLETE: ("🏁  MISSION COMPLETE",       C.GREEN),
}


# ================================================================
#  THREAD-SAFE CAMERA READER
# ================================================================
class CameraReader:
    """Reads frames from a camera source in a background thread.
    Optimized: flushes stale MJPEG frames to minimize latency."""

    def __init__(self, name, source):
        self.name = name
        self.source = source
        self.frame = None
        self.connected = False
        self.running = True
        self._lock = threading.Lock()

    def start(self):
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()
        return self

    def _loop(self):
        while self.running:
            cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            if not cap.isOpened():
                self.connected = False
                cap.release()
                time.sleep(2)
                continue

            self.connected = True
            while self.running:
                ret, frame = cap.read()
                if not ret or frame is None:
                    self.connected = False
                    break

                # Flush stale frames — grab without decoding
                # to always get the LATEST frame, reducing delay
                for _ in range(2):
                    if cap.grab():
                        ret2, frame2 = cap.retrieve()
                        if ret2 and frame2 is not None:
                            frame = frame2
                    else:
                        break

                with self._lock:
                    self.frame = frame

            cap.release()
            time.sleep(1)

    def get_frame(self):
        with self._lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False


# ================================================================
#  PER-CAR STATE CONTAINER
# ================================================================
class CarState:
    """Holds all state for a single car — camera, HTTP, route, mission."""
    def __init__(self, name, ip, cmd_port, stream_url):
        self.name = name
        self.ip = ip
        self.cmd_port = cmd_port
        self.stream_url = stream_url
        self.cam = None
        self.session = requests.Session()
        self.http_queue = queue.Queue()
        self.command_epoch = 0
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.pump_active = False
        self.pump_start_time = 0.0
        self.last_fire_time = 0.0
        self.fire_seen = False
        self.extinguished_handled = False
        # Per-car sub-state after exiting garage
        self.sub_state = "SCANNING_360"  # SCANNING_360 / APPROACHING / PUMPING / EXTINGUISHED / RETURN_READY
        self.scan_360_start = 0.0
        self.scan_360_fire_first = 0.0
        self.scan_step_turn_start = 0.0
        self.scan_is_turning = False
        self.lunge_start = 0.0
        # Route
        self.route_segments = []
        self.active_motion = None
        self.active_motion_started = None
        self.route_total_time = 0.0
        self.return_plan_remaining = []
        self.return_total_steps = 0
        self.return_completed_steps = 0
        self.return_current_cmd = "\u2014"
        self.return_step_remaining = 0.0
        self.return_abort_event = threading.Event()
        self.return_thread = None
        self.return_error = None
        self.garage_exit_started = False

    def start_cam(self):
        self.cam = CameraReader(self.name, self.stream_url).start()

    def stop_cam(self):
        if self.cam:
            self.cam.stop()
            self.cam = None

    def get_frame(self):
        if self.cam:
            return self.cam.get_frame()
        return None

    def reset(self):
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.pump_active = False
        self.pump_start_time = 0.0
        self.last_fire_time = 0.0
        self.fire_seen = False
        self.extinguished_handled = False
        self.sub_state = "SCANNING_360"
        self.scan_360_start = 0.0
        self.scan_360_fire_first = 0.0
        self.route_segments = []
        self.active_motion = None
        self.active_motion_started = None
        self.route_total_time = 0.0
        self.return_plan_remaining = []
        self.return_total_steps = 0
        self.return_completed_steps = 0
        self.return_current_cmd = "\u2014"
        self.return_step_remaining = 0.0
        self.return_error = None
        self.garage_exit_started = False


# ================================================================
#  FIRE FAUJI DASHBOARD
# ================================================================
class FireDashboard:

    def __init__(self):
        # ---- Tkinter root ----
        self.root = tk.Tk()
        self.root.title("FIRE FAUJI COMMAND CENTER")
        self.root.geometry("1350x720")
        self.root.resizable(True, True)
        self.root.configure(bg=C.BG_DARK)
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        # ---- Mission state ----
        self.state = State.SCANNING
        self.fire_first_seen_time = 0.0
        self.last_fire_time = 0.0
        self.deploy_time = 0.0
        self.running = True

        # ---- Car 1 only (Car 2 is manual, no AI) ----
        self.car1 = CarState("Car 1", CAR_IP, CAR_CMD_PORT, CAR_STREAM_URL)
        self.cars = [self.car1]

        # Legacy aliases for shared code paths
        self.car_cam = None
        self.pump_active = False
        self.fire_seen_on_car = False
        self.extinguished_handled = False
        self.command_epoch = 0

        # ---- Car command + return-path tracking (legacy for shared states) ----
        self.last_car_cmd = None
        self.last_cmd_time = 0.0
        self.car_session = requests.Session()
        self.http_queue = queue.Queue()
        self.target_x = 0.0
        self.target_y = 0.0
        self.car_pump_start_time = 0.0

        # Timed dead-reckoning route
        self.route_segments = []
        self.active_motion = None
        self.active_motion_started = None
        self.route_total_time = 0.0

        self.return_plan_remaining = []
        self.return_total_steps = 0
        self.return_completed_steps = 0
        self.return_current_cmd = "\u2014"
        self.return_step_remaining = 0.0
        self.return_abort_event = threading.Event()
        self.return_thread = None
        self.return_error = None
        self.extinguished_handled = False

        # ---- Serial ----
        self.esp = None
        self.serial_lock = threading.Lock()

        # ---- YOLO — ONNX + DirectML for AMD GPU ----
        if os.path.exists(YOLO_MODEL_ONNX):
            print(f"🤖 Loading ONNX model ({YOLO_MODEL_ONNX}) for AMD GPU...")
            self.model = YOLO(YOLO_MODEL_ONNX, task='detect')
            try:
                import onnxruntime as ort
                providers = ort.get_available_providers()
                if 'DmlExecutionProvider' in providers:
                    print(f"✅ YOLO loaded — AMD GPU (DirectML) active!")
                    print(f"   Available providers: {providers}")
                else:
                    print(f"⚠️  DirectML not found. Providers: {providers}")
            except ImportError:
                print("✅ YOLO ONNX model loaded.")
        elif os.path.exists(YOLO_MODEL_PT):
            print(f"⚠️  ONNX not found, loading {YOLO_MODEL_PT} (CPU mode)...")
            self.model = YOLO(YOLO_MODEL_PT)
            print("✅ YOLO model loaded (CPU). Export to ONNX for GPU boost.")
        else:
            print(f"❌ No YOLO model found in {os.getcwd()}")
            sys.exit(1)

        # ---- Cameras ----
        self.station_cam = None
        self._photo1 = None   # prevent GC of PhotoImage
        self._photo2 = None

        # ---- Build GUI ----
        self._build_gui()

        # One ordered HTTP worker prevents forward/left/right/stop reordering.
        threading.Thread(target=self._http_command_worker, daemon=True).start()
        # Per-car HTTP workers
        for car in self.cars:
            threading.Thread(target=self._http_worker_car, args=(car,), daemon=True).start()

        # ---- Connect gatekeeper ----
        self._connect_gatekeeper()

        # ---- Start station camera ----
        self._start_station_camera()

        # ---- Kick off main loop ----
        self._log("System started. Scanning for fire…", C.BLUE)
        self.root.after(50, self._tick)

    # ============================================================
    #  GUI CONSTRUCTION
    # ============================================================
    def _build_gui(self):
        F = ("Segoe UI", 11)
        FB = ("Segoe UI", 12, "bold")
        FV = ("Consolas", 10, "bold")
        FL = ("Consolas", 10)

        # ---- HEADER ----
        hdr = tk.Frame(self.root, bg=C.BG_HEADER, height=68)
        hdr.grid(row=0, column=0, columnspan=2, sticky="ew")
        hdr.grid_propagate(False)

        # Accent bar at the very top
        accent_bar = tk.Frame(hdr, bg=C.RED, height=3)
        accent_bar.pack(fill="x", side="top")

        hdr_content = tk.Frame(hdr, bg=C.BG_HEADER)
        hdr_content.pack(fill="both", expand=True)

        tk.Label(hdr_content, text="🔥  FIRE FAUJI",
                 font=("Segoe UI", 20, "bold"), fg=C.RED,
                 bg=C.BG_HEADER).pack(side="left", padx=(20, 4), pady=8)

        tk.Label(hdr_content, text="COMMAND CENTER",
                 font=("Segoe UI", 20, "bold"), fg=C.TEXT,
                 bg=C.BG_HEADER).pack(side="left", padx=(0, 12), pady=8)

        # Version badge
        ver_frame = tk.Frame(hdr_content, bg=C.ACCENT, padx=8, pady=2)
        ver_frame.pack(side="left", pady=12)
        tk.Label(ver_frame, text="v3.0", font=("Consolas", 9, "bold"),
                 fg="#ffffff", bg=C.ACCENT).pack()

        # Pulsing status dot
        self._status_dot = tk.Label(hdr_content, text="●", font=("Segoe UI", 16),
                                     fg=C.GREEN, bg=C.BG_HEADER)
        self._status_dot.pack(side="right", padx=20)
        self._dot_visible = True

        self._status_label = tk.Label(hdr_content, text="SYSTEM ONLINE",
                                       font=("Consolas", 9, "bold"),
                                       fg=C.GREEN, bg=C.BG_HEADER)
        self._status_label.pack(side="right")

        # ---- VIDEO PANEL ----
        vf = tk.Frame(self.root, bg=C.VIDEO_BG,
                      highlightbackground=C.ACCENT, highlightthickness=2)
        vf.grid(row=1, column=0, padx=(12, 6), pady=(8, 4), sticky="nsew")

        self._video_source_label = tk.Label(vf, text="📹  STATION CAMERA",
                                             font=("Segoe UI", 9, "bold"),
                                             fg=C.TEXT_DIM, bg=C.VIDEO_BG)
        self._video_source_label.pack(anchor="w", padx=8, pady=(4, 0))

        self.video_label_1 = tk.Label(vf, bg=C.VIDEO_BG)
        self.video_label_1.pack(padx=4, pady=(2, 4))
        self.video_label_2 = self.video_label_1  # Alias — single feed
        self.video_label = self.video_label_1
        self._show_blank("Initializing station camera…")

        # ---- STATUS PANEL ----
        sf = tk.Frame(self.root, bg=C.BG_PANEL, width=540,
                      highlightbackground=C.ACCENT, highlightthickness=2)
        sf.grid(row=1, column=1, padx=(6, 12), pady=(8, 4), sticky="nsew")

        # Status header with icon
        status_hdr = tk.Frame(sf, bg=C.BG_PANEL)
        status_hdr.pack(fill="x", padx=20, pady=(15, 0))
        tk.Label(status_hdr, text="📊  MISSION STATUS",
                 font=("Segoe UI", 14, "bold"), fg=C.TEXT,
                 bg=C.BG_PANEL).pack(side="left")

        # Separator with gradient feel
        sep = tk.Frame(sf, bg=C.ACCENT, height=2)
        sep.pack(fill="x", padx=20, pady=(8, 10))

        # Status rows with alternating backgrounds
        self._sv = {}
        rows = [
            ("🎯 Mission",   "state",       "🔍  SCANNING"),
            ("🔥 Fire",      "fire",        "❌  Not Detected"),
            ("📊 Confidence", "conf",       "—"),
            ("⏱️ Confirm",   "confirm",     f"0.0 / {FIRE_CONFIRM_TIME} sec"),
            ("📏 Distance",  "dist",        "—"),
            ("🧭 Direction", "dir",         "—"),
            ("💧 Pump",      "pump",        "⬜  OFF"),
            ("🌐 Car IP",    "car_ip",      CAR_IP),
            ("📡 Car Feed",  "car_feed",    "⬜  Not Connected"),
            ("🚪 Garage",    "doors",       "🟢 Closed"),
            ("🗺️ Saved Path", "path",       "0 segments"),
            ("⏱️ Path Time", "path_time",   "0.00 s"),
            ("↩️ Return",    "return",      "Idle"),
            ("🚗 Return Cmd", "return_cmd", "—"),
        ]
        status_grid = tk.Frame(sf, bg=C.BG_PANEL)
        status_grid.pack(fill="x", padx=10, pady=2)
        status_grid.grid_columnconfigure(0, weight=1, uniform="col")
        status_grid.grid_columnconfigure(1, weight=1, uniform="col")

        for i, (label, key, default) in enumerate(rows):
            col = i % 2
            row_idx = i // 2
            bg = C.BG_PANEL if row_idx % 2 == 0 else "#161638"
            
            r = tk.Frame(status_grid, bg=bg)
            r.grid(row=row_idx, column=col, sticky="nsew", padx=4, pady=2)
            
            # Shorter labels for dual-column format
            lbl = label.replace(" Confidence", " Conf.").replace(" Direction", " Dir.")
            
            tk.Label(r, text=f" {lbl}:", font=F, fg=C.TEXT_DIM,
                     bg=bg, width=10, anchor="w").pack(side="left", padx=(2, 0))
            v = tk.Label(r, text=default, font=FV, fg=C.TEXT,
                         bg=bg, anchor="w")
            v.pack(side="left", fill="x", expand=True, padx=(0, 2))
            self._sv[key] = v

        # ---- BUTTONS (Compact 2x2 Grid) ----
        btn_frame = tk.Frame(sf, bg=C.BG_PANEL)
        btn_frame.pack(fill="x", padx=15, pady=(8, 4))
        btn_frame.grid_columnconfigure(0, weight=1)
        btn_frame.grid_columnconfigure(1, weight=1)

        self.btn_end = tk.Button(
            btn_frame, text="🛑 END MISSION", font=("Segoe UI", 10, "bold"),
            bg=C.RED, fg="#ffffff", activebackground="#ff5c77",
            activeforeground="#ffffff", relief="flat", cursor="hand2",
            command=self._cmd_end_mission)
        self.btn_end.grid(row=0, column=0, sticky="nsew", padx=2, pady=2, ipady=4)

        self.btn_demo = tk.Button(
            btn_frame, text="🎬 SUCCESS (Demo)",
            font=("Segoe UI", 10, "bold"),
            bg=C.ACCENT, fg="#ffffff", activebackground="#8b80ff",
            activeforeground="#ffffff", relief="flat", cursor="hand2",
            command=self._demo_mission_success)
        self.btn_demo.grid(row=0, column=1, sticky="nsew", padx=2, pady=2, ipady=4)

        self.btn_garage_open = tk.Button(
            btn_frame, text="🚪 OPEN GARAGE",
            font=("Segoe UI", 10, "bold"),
            bg="#f59e0b", fg="#ffffff", activebackground="#fbbf24",
            activeforeground="#ffffff", relief="flat", cursor="hand2",
            command=lambda: [
                self._gatekeeper_http("/open_doors", "🚪 Manual Open Triggered"),
                self._sv_set("doors", "🟢 Open", C.GREEN)
            ])
        self.btn_garage_open.grid(row=1, column=0, sticky="nsew", padx=2, pady=2, ipady=4)

        self.btn_garage_close = tk.Button(
            btn_frame, text="🚪 CLOSE GARAGE",
            font=("Segoe UI", 10, "bold"),
            bg="#3b82f6", fg="#ffffff", activebackground="#60a5fa",
            activeforeground="#ffffff", relief="flat", cursor="hand2",
            command=lambda: [
                self._gatekeeper_http("/close_doors", "🚪 Manual Close Triggered"),
                self._sv_set("doors", "🟢 Closed", C.GREEN)
            ])
        self.btn_garage_close.grid(row=1, column=1, sticky="nsew", padx=2, pady=2, ipady=4)

        # ---- LOG PANEL ----
        lf = tk.Frame(self.root, bg=C.BG_PANEL,
                      highlightbackground=C.ACCENT, highlightthickness=2)
        lf.grid(row=2, column=0, columnspan=2,
                padx=12, pady=(4, 10), sticky="nsew")

        log_hdr = tk.Frame(lf, bg=C.BG_PANEL)
        log_hdr.pack(fill="x", padx=15, pady=(8, 4))
        tk.Label(log_hdr, text="📋  MISSION LOG",
                 font=("Segoe UI", 12, "bold"), fg=C.TEXT,
                 bg=C.BG_PANEL).pack(side="left")

        lc = tk.Frame(lf, bg=C.BG_LOG,
                      highlightbackground=C.BORDER, highlightthickness=1)
        lc.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.log_text = tk.Text(
            lc, bg=C.BG_LOG, fg=C.TEXT, font=FL,
            height=10, wrap="word", state="disabled",
            insertbackground=C.TEXT, selectbackground=C.BLUE,
            borderwidth=0, padx=10, pady=8)

        sb = tk.Scrollbar(lc, command=self.log_text.yview, bg=C.BG_PANEL,
                          troughcolor=C.BG_LOG, activebackground=C.ACCENT)
        self.log_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_text.pack(side="left", fill="both", expand=True)

        # Tag colours for log
        for tag, clr in [("ts", C.TEXT_DIM), ("info", C.TEXT),
                         ("fire", C.RED), ("ok", C.GREEN),
                         ("warn", C.ORANGE), ("sys", C.BLUE),
                         ("gk", C.PURPLE), ("cyan", C.CYAN),
                         ("gold", C.GOLD)]:
            self.log_text.tag_configure(tag, foreground=clr)

        # Grid weights
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Start pulsing status dot animation
        self._pulse_dot()

    # ============================================================
    #  HELPERS — logging & status
    # ============================================================
    def _log(self, msg, color=None):
        tag_map = {C.RED: "fire", C.GREEN: "ok", C.ORANGE: "warn",
                   C.BLUE: "sys", C.PURPLE: "gk", C.CYAN: "cyan"}
        tag = tag_map.get(color, "info")
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{ts}] ", "ts")
        self.log_text.insert("end", f"{msg}\n", tag)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _sv_set(self, key, text, color=None):
        if key in self._sv:
            self._sv[key].configure(text=text)
            if color:
                self._sv[key].configure(fg=color)

    def _set_state(self, s):
        old_label = STATE_DISPLAY.get(self.state, (self.state, C.TEXT))[0]
        self.state = s
        label, clr = STATE_DISPLAY.get(s, (s, C.TEXT))
        self._sv_set("state", label, clr)
        self._log(f"⚡ State: {old_label.strip()} → {label.strip()}", clr)

        # Update header status dot color based on state
        dot_colors = {
            State.SCANNING: C.BLUE,
            State.FIRE_CONFIRMED: C.RED,
            State.DEPLOYING: C.ORANGE,
            State.EXITING_GARAGE: C.CYAN,
            State.CAR_APPROACHING: C.ORANGE,
            State.PUMPING: C.CYAN,
            State.EXTINGUISHED: C.GREEN,
            State.RETURN_READY: C.ORANGE,
            State.RETURNING: C.CYAN,
            State.RETURN_FAILED: C.RED,
            State.MISSION_COMPLETE: C.GREEN,
        }
        dot_clr = dot_colors.get(s, C.GREEN)
        try:
            self._status_dot.configure(fg=dot_clr)
            self._status_label.configure(fg=dot_clr,
                text="MISSION COMPLETE" if s == State.MISSION_COMPLETE else
                     "FIRE DETECTED" if s in (State.FIRE_CONFIRMED, State.CAR_APPROACHING, State.PUMPING) else
                     "DEPLOYING" if s in (State.DEPLOYING, State.EXITING_GARAGE) else
                     "RETURNING" if s == State.RETURNING else
                     "SYSTEM ONLINE")
        except Exception:
            pass

        # Update video source label
        try:
            if s in (State.SCANNING,):
                self._video_source_label.configure(text="📹  STATION CAMERA (Box ESP)")
            elif s in (State.EXITING_GARAGE, State.CAR_APPROACHING, State.PUMPING, State.RETURNING,
                       State.RETURN_READY, State.MISSION_COMPLETE):
                self._video_source_label.configure(text="📹  CAR CAMERA (Live)")
            elif s == State.DEPLOYING:
                self._video_source_label.configure(text="📹  CONNECTING TO CAR...")
        except Exception:
            pass

    def _pulse_dot(self):
        """Animate the status dot to pulse."""
        if not self.running:
            return
        self._dot_visible = not self._dot_visible
        try:
            self._status_dot.configure(
                fg=self._status_dot.cget("fg") if self._dot_visible else C.BG_HEADER)
        except Exception:
            pass
        self.root.after(800, self._pulse_dot)

    def _update_route_status(self):
        active_time = 0.0
        if self.active_motion and self.active_motion_started is not None:
            active_time = max(0.0, time.monotonic() - self.active_motion_started)

        total = self.route_total_time + active_time
        count = len(self.route_segments) + (1 if self.active_motion else 0)
        preview_items = [
            f"{seg['cmd'][0].upper()}:{seg['duration']:.1f}s"
            for seg in self.route_segments[-4:]
        ]
        if self.active_motion:
            preview_items.append(f"{self.active_motion[0].upper()}:{active_time:.1f}s*")

        preview = "  ".join(preview_items) if preview_items else "No movement"
        self._sv_set("path", f"{count} segments · {preview}", C.CYAN if count else C.TEXT_DIM)
        self._sv_set("path_time", f"{total:.2f} s", C.TEXT)

    def _finalize_active_motion(self, now=None):
        """Close the current movement segment and save its elapsed duration."""
        if self.active_motion is None or self.active_motion_started is None:
            return

        now = time.monotonic() if now is None else now
        duration = max(0.0, now - self.active_motion_started)
        cmd = self.active_motion
        self.active_motion = None
        self.active_motion_started = None

        if duration >= MIN_ROUTE_SEGMENT_SEC:
            if self.route_segments and self.route_segments[-1]["cmd"] == cmd:
                self.route_segments[-1]["duration"] += duration
            else:
                self.route_segments.append({"cmd": cmd, "duration": duration})
            self.route_total_time = sum(seg["duration"] for seg in self.route_segments)
            self._log(f"🧭 Saved path segment: {cmd} for {duration:.2f}s", C.CYAN)

        self._update_route_status()

    def _record_motion_transition(self, cmd, now=None):
        """Record only commands that were actually sent to the car."""
        now = time.monotonic() if now is None else now
        if cmd == self.active_motion:
            return

        self._finalize_active_motion(now)
        if cmd in {"forward", "left", "right"}:
            self.active_motion = cmd
            self.active_motion_started = now
        self._update_route_status()

    def _clear_route(self):
        self.route_segments = []
        self.active_motion = None
        self.active_motion_started = None
        self.route_total_time = 0.0
        self.return_plan_remaining = []
        self.return_total_steps = 0
        self.return_completed_steps = 0
        self.return_current_cmd = "—"
        self.return_step_remaining = 0.0
        self.return_error = None
        self._update_route_status()
        self._sv_set("return", "Idle", C.TEXT_DIM)
        self._sv_set("return_cmd", "—", C.TEXT_DIM)

    def _cmd_end_mission(self):
        car = self.cars[0] if self.cars else None
        if (self.state == State.SCANNING and (not car or not car.route_segments)
                and (not car or car.active_motion is None)):
            self._log("ℹ️ No deployed mission or saved route to return.", C.ORANGE)
            return

        if self.state == State.RETURNING:
            self._log("↩️ Return is already in progress.", C.ORANGE)
            return

        if self.state == State.MISSION_COMPLETE:
            self._reset_for_new_mission()
            return

        retry = self.state == State.RETURN_FAILED and bool(self.return_plan_remaining)
        self._begin_return_to_base(retry=retry)

    def _begin_return_to_base(self, retry=False):
        """Stop all mission actions and navigate back using route history."""
        self.pump_active = False
        self._sv_set("pump", "⬜  OFF", C.TEXT_DIM)

        self.command_epoch += 1
        self.last_car_cmd = None
        self.return_abort_event.clear()
        self.return_error = None
        self.btn_end.configure(state="disabled", text="🏠 RETURNING TO BASE…")

        if not retry:
            car = self.cars[0] if self.cars else None
            if car:
                self._finalize_motion_car(car)
                self.return_plan_remaining = list(reversed(car.route_segments))
            else:
                self.return_plan_remaining = []
            self.return_total_steps = len(self.return_plan_remaining)
            self.return_completed_steps = 0

        self._sv_set("return", f"0 / {self.return_total_steps}", C.CYAN)
        self._sv_set("return_cmd", "Starting…", C.ORANGE)
        self._log(f"🏠 END MISSION: Replaying {len(self.return_plan_remaining)} segments in reverse.", C.CYAN)
        
        self._set_state(State.RETURNING)

        self.return_thread = threading.Thread(target=self._return_worker, daemon=True)
        self.return_thread.start()

    def _return_worker(self):
        inverse = {
            "forward": ("reverse", "/reverse"),
            "left":    ("undo-left", "/reverse_left"),
            "right":   ("undo-right", "/reverse_right"),
        }

        car = self.cars[0] if self.cars else None
        if not car:
            self._schedule_return_failure("No car connected.")
            return

        ok, message = self._car_http_sync_to(car, "/pump_off")
        ok2, message2 = self._car_http_sync_to(car, "/stop_motors")
        if not (ok and ok2):
            self._schedule_return_failure(f"Could not safely prepare car: {message}; {message2}")
            return
        time.sleep(RETURN_SETTLE_SEC)

        plan = [dict(seg) for seg in self.return_plan_remaining]
        base_completed = self.return_completed_steps
        total = self.return_total_steps

        for index, segment in enumerate(plan):
            if self.return_abort_event.is_set() or not self.running:
                return

            cmd = segment["cmd"]
            label, endpoint = inverse[cmd]
            duration = max(0.0, segment["duration"] * RETURN_TIME_SCALE)
            
            # Optimization: Reverse turning is physically faster than forward turning.
            if cmd in ["left", "right"]:
                duration = max(0.0, duration * 0.4)  # 0.25s forward becomes 0.100s reverse

            absolute_index = base_completed + index
            self.root.after(0, lambda i=absolute_index, t=total, l=label, d=duration:
                            self._set_return_step_ui(i, t, l, d))

            success, reply = self._car_http_sync_to(car, endpoint)
            if not success:
                self.return_plan_remaining = plan[index:]
                self._car_http_sync_to(car, "/stop_motors")
                self._schedule_return_failure(
                    f"Return command {endpoint} failed after retries: {reply}")
                return

            end_at = time.monotonic() + duration
            while time.monotonic() < end_at:
                if self.return_abort_event.is_set() or not self.running:
                    self._car_http_sync_to(car, "/stop_motors")
                    return
                rem = max(0.0, end_at - time.monotonic())
                self.root.after(0, lambda i=absolute_index, t=total, l=label, r=rem:
                                self._set_return_step_ui(i, t, l, r))
                time.sleep(0.05)

            self._car_http_sync_to(car, "/stop_motors")
            self.return_completed_steps += 1
            time.sleep(RETURN_SETTLE_SEC)

        if self.running:
            self.root.after(0, self._finish_return_success)

    def _set_return_step_ui(self, index, total, label, duration):
        self.return_current_cmd = label
        self.return_step_remaining = duration
        self._sv_set("return", f"{index + 1} / {total}", C.CYAN)
        self._sv_set("return_cmd", f"{label} · {duration:.2f}s", C.ORANGE)
        self._log(f"↩️ Return step {index + 1}/{total}: {label} for {duration:.2f}s", C.CYAN)

    def _schedule_return_failure(self, message):
        if self.running:
            self.root.after(0, lambda m=message: self._finish_return_failure(m))

    def _finish_return_failure(self, message):
        self.return_error = message
        self.return_current_cmd = "STOPPED"
        self._sv_set("return", "Interrupted — retry available", C.RED)
        self._sv_set("return_cmd", "STOPPED", C.RED)
        self._log(f"❌ Return interrupted: {message}", C.RED)
        self.btn_end.configure(state="normal", text="↩️ RETRY RETURN")
        self._set_state(State.RETURN_FAILED)

    def _finish_return_success(self):
        self.return_plan_remaining = []
        self.return_current_cmd = "STOPPED AT BASE"
        self.return_step_remaining = 0.0
        for car in self.cars:
            self._car_http_to(car, "/stop_motors")
        self._gatekeeper_http("/close_doors", "🚪 Mission complete! Closing garage doors.")
        self._sv_set("doors", "🟢 Closed", C.GREEN)
        self._sv_set("doors", "🟢 Closed", C.GREEN)
        self._sv_set("return", "✅ Base reached (timed)", C.GREEN)
        self._sv_set("return_cmd", "STOPPED", C.GREEN)
        self._sv_set("dir", "🏠  AT BASE", C.GREEN)
        self._sv_set("pump", "⬜  OFF", C.TEXT_DIM)
        self._sv_set("fire", "✅  EXTINGUISHED", C.GREEN)
        self._log(
            "🏁 MISSION SUCCESSFUL — Fire extinguished and car returned to base!",
            C.GREEN)
        self._log(
            "🎖️  All objectives completed. System ready for next deployment.",
            C.GREEN)
        self.btn_end.configure(state="normal", text="🔄 NEW MISSION")
        self.mission_complete_time = time.time()
        self._set_state(State.MISSION_COMPLETE)

    def _reset_for_new_mission(self):
        self.return_abort_event.set()
        self.command_epoch += 1
        for car in self.cars:
            self._car_http_to(car, "/pump_off", "💧 Pump OFF.")
            self._car_http_to(car, "/stop_motors", "🛑 Motors stopped.")

        self._gatekeeper_http("/close_doors", "🚪 Closing garage doors for new mission.")
        self._sv_set("doors", "🟢 Closed", C.TEXT_DIM)

        for car in self.cars:
            car.stop_cam()
            car.reset()
            car.route_segments = []
            car.active_motion = None
            car.active_motion_started = None
            car.sub_state = "SCANNING_360"
            car.scan_step_turn_start = 0.0
            car.scan_is_turning = False
            car.lunge_start = 0.0
            car.scan_360_fire_first = 0.0

        self.route_segments = []
        self.active_motion = None
        self.return_plan_remaining = []

        self._sv_set("car_feed", "⬜  Not Connected", C.TEXT_DIM)

        if not self.station_cam:
            self._start_station_camera()

        self.fire_first_seen_time = 0.0
        self.fire_seen_on_car = False
        self.extinguished_handled = False
        self.last_fire_time = 0.0
        self.last_car_cmd = None
        self.last_cmd_time = 0.0
        self._clear_route()
        self.return_abort_event.clear()

        self._sv_set("pump", "⬜  OFF", C.TEXT_DIM)
        self._sv_set("fire", "❌  Not Detected", C.TEXT_DIM)
        self._sv_set("dir", "—", C.TEXT_DIM)
        self._sv_set("dist", "—", C.TEXT_DIM)
        self._sv_set("conf", "—", C.TEXT_DIM)
        self._sv_set("confirm", f"0.0 / {FIRE_CONFIRM_TIME} sec", C.TEXT_DIM)
        self.btn_end.configure(state="normal", text="🛑 END MISSION")
        self._log("🔄 New mission armed. Station camera scanning again.", C.BLUE)
    # ============================================================
    #  SERIAL — Gatekeeper ESP32
    # ============================================================
    def _connect_gatekeeper(self):
        try:
            self.esp = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
            time.sleep(2)
            self._log(f"✅ Gatekeeper connected on {COM_PORT}", C.GREEN)
            threading.Thread(target=self._serial_reader, daemon=True).start()
        except Exception as e:
            self._log(f"⚠️  Gatekeeper not connected: {e}", C.ORANGE)
            self.esp = None

    def _serial_reader(self):
        while self.running and self.esp:
            try:
                with self.serial_lock:
                    if self.esp and self.esp.in_waiting > 0:
                        line = self.esp.readline().decode("utf-8", errors="ignore").strip()
                        if line:
                            self.root.after(0, lambda l=line:
                                self._log(f"📡 GK: {l}", C.PURPLE))
            except Exception:
                pass
            time.sleep(0.05)

    def _send_serial(self, cmd):
        if not self.esp:
            self._log(f"⚠️  No gatekeeper — cannot send '{cmd}'", C.ORANGE)
            return
        try:
            with self.serial_lock:
                self.esp.write(cmd.encode("ascii"))
            self._log(f"➡️  Sent '{cmd}' → Gatekeeper ({COM_PORT})", C.BLUE)
        except Exception as e:
            self._log(f"❌ Serial error: {e}", C.RED)

    # ============================================================
    #  CAR HTTP — non-blocking commands
    # ============================================================
    def _car_http(self, endpoint, log_msg=None):
        """Queue normal mission commands in strict FIFO order."""
        self.http_queue.put((self.command_epoch, endpoint, log_msg))

    def _http_command_worker(self):
        while self.running:
            try:
                epoch, endpoint, log_msg = self.http_queue.get(timeout=0.10)
            except queue.Empty:
                continue

            try:
                # Old commands are discarded when return/new-mission changes epoch.
                if epoch != self.command_epoch:
                    continue

                ok, reply = self._car_http_sync(
                    endpoint, retries=1, respect_return_abort=False)
                if not self.running or epoch != self.command_epoch:
                    continue

                if ok:
                    text = log_msg or f"📤 HTTP → {endpoint}  ✅ Car replied: {reply}"
                    color = C.BLUE if log_msg else C.CYAN
                else:
                    text = f"📤 HTTP → {endpoint}  ❌ Failed: {reply}"
                    color = C.RED
                self.root.after(0, lambda t=text, c=color: self._log(t, c))
            finally:
                self.http_queue.task_done()

    def _car_http_sync(self, endpoint, retries=RETURN_HTTP_RETRIES,
                       respect_return_abort=True):
        """Synchronous ordered HTTP call, used directly by the return worker."""
        url = f"http://{CAR_IP}:{CAR_CMD_PORT}{endpoint}"
        last_error = "unknown error"
        for attempt in range(max(1, retries)):
            if (respect_return_abort and self.return_abort_event.is_set()
                    and endpoint not in {"/stop_motors", "/pump_off"}):
                return False, "return aborted"
            try:
                response = self.car_session.get(url, timeout=RETURN_HTTP_TIMEOUT)
                response.raise_for_status()
                return True, response.text.strip()
            except Exception as exc:
                last_error = str(exc)
                if attempt + 1 < max(1, retries):
                    time.sleep(0.12)
        return False, last_error

    def _gatekeeper_http(self, endpoint, log_msg=None):
        """Send an HTTP GET command to the Gatekeeper ESP in a background thread."""
        def worker():
            try:
                url = f"http://{GATEKEEPER_IP}:{GATEKEEPER_CMD_PORT}{endpoint}"
                res = requests.get(url, timeout=5.0)
                if log_msg:
                    success_msg = log_msg + f" ({res.text.strip()})"
                    self.root.after(0, lambda msg=success_msg: self._log(msg, C.BLUE))
            except Exception as e:
                err_msg = f"❌ Gatekeeper HTTP '{endpoint}' failed: {e}"
                self.root.after(0, lambda msg=err_msg: self._log(msg, C.RED))
        threading.Thread(target=worker, daemon=True).start()

    def _car_move(self, cmd, record=True, force=False):
        """Send one changed movement command and record its real active duration."""
        now = time.monotonic()

        if not force:
            if cmd == self.last_car_cmd:
                self._update_route_status()
                return
            if cmd != "stop" and (now - self.last_cmd_time) < COMMAND_THROTTLE_SEC:
                return

        if record:
            self._record_motion_transition(cmd, now)

        self.last_car_cmd = cmd
        self.last_cmd_time = now
        labels = {
            "forward": ("⬆️ /forward",       "/forward"),
            "left":    ("⬅️ /turn_left",      "/turn_left"),
            "right":   ("➡️ /turn_right",     "/turn_right"),
            "stop":    ("⏹️ /stop_motors",    "/stop_motors"),
        }
        label, endpoint = labels.get(cmd, (cmd, None))
        if endpoint:
            self._car_http(endpoint, f"🚗 Car command: {label}")

    # ---- Per-car HTTP helpers ----
    def _car_http_to(self, car, endpoint, log_msg=None):
        """Queue a command for a specific car."""
        car.http_queue.put((car.command_epoch, endpoint, log_msg))

    def _car_http_sync_to(self, car, endpoint, retries=RETURN_HTTP_RETRIES):
        """Synchronous HTTP GET to a specific car."""
        url = f"http://{car.ip}:{car.cmd_port}{endpoint}"
        last_error = "unknown error"
        for attempt in range(max(1, retries)):
            try:
                response = car.session.get(url, timeout=RETURN_HTTP_TIMEOUT)
                response.raise_for_status()
                return True, response.text.strip()
            except Exception as exc:
                last_error = str(exc)
                if attempt + 1 < max(1, retries):
                    time.sleep(0.12)
        return False, last_error

    def _car_move_to(self, car, cmd, record=True, force=False):
        """Send a movement command to a specific car and record it."""
        now = time.monotonic()
        if not force:
            if cmd == car.last_cmd:
                return
            if cmd != "stop" and (now - car.last_cmd_time) < COMMAND_THROTTLE_SEC:
                return

        if record:
            self._record_motion_transition_car(car, cmd, now)

        car.last_cmd = cmd
        car.last_cmd_time = now
        labels = {
            "forward": ("⬆️ /forward",       "/forward"),
            "left":    ("⬅️ /turn_left",      "/turn_left"),
            "right":   ("➡️ /turn_right",     "/turn_right"),
            "stop":    ("⏹️ /stop_motors",    "/stop_motors"),
        }
        label, endpoint = labels.get(cmd, (cmd, None))
        if endpoint:
            self._car_http_to(car, endpoint, f"🚗 {car.name}: {label}")

    def _record_motion_transition_car(self, car, cmd, now=None):
        now = time.monotonic() if now is None else now
        if cmd == car.active_motion:
            return
        self._finalize_motion_car(car, now)
        if cmd in {"forward", "left", "right"}:
            car.active_motion = cmd
            car.active_motion_started = now

    def _finalize_motion_car(self, car, now=None):
        if car.active_motion is None or car.active_motion_started is None:
            return
        now = time.monotonic() if now is None else now
        duration = max(0.0, now - car.active_motion_started)
        cmd = car.active_motion
        car.active_motion = None
        car.active_motion_started = None
        if duration >= MIN_ROUTE_SEGMENT_SEC:
            if car.route_segments and car.route_segments[-1]["cmd"] == cmd:
                car.route_segments[-1]["duration"] += duration
            else:
                car.route_segments.append({"cmd": cmd, "duration": duration})
            car.route_total_time = sum(seg["duration"] for seg in car.route_segments)

    def _http_worker_car(self, car):
        """HTTP command worker for a specific car."""
        while self.running:
            try:
                epoch, endpoint, log_msg = car.http_queue.get(timeout=0.10)
            except queue.Empty:
                continue
            try:
                if epoch != car.command_epoch:
                    continue
                ok, reply = self._car_http_sync_to(car, endpoint, retries=1)
                if not self.running or epoch != car.command_epoch:
                    continue
                if ok:
                    text = log_msg or f"📤 {car.name} → {endpoint}  ✅ {reply}"
                    color = C.BLUE if log_msg else C.CYAN
                else:
                    text = f"📤 {car.name} → {endpoint}  ❌ {reply}"
                    color = C.RED
                self.root.after(0, lambda t=text, c=color: self._log(t, c))
            finally:
                car.http_queue.task_done()

    # ============================================================
    #  CAMERAS
    # ============================================================
    def _start_station_camera(self):
        self.station_cam = CameraReader("Station", STATION_WEBCAM_INDEX).start()

    def _stop_station_camera(self):
        if self.station_cam:
            self.station_cam.stop()
            self.station_cam = None

    def _start_car_camera(self):
        for car in self.cars:
            car.start_cam()
        self._sv_set("car_feed", "🔄  Connecting…", C.ORANGE)

    # ============================================================
    #  DISPLAY — frames & blank screens
    # ============================================================
    def _show_frame(self, frame, label=None):
        if label is None:
            label = self.video_label_1
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb).resize((VIDEO_WIDTH, VIDEO_HEIGHT), Image.LANCZOS)
        photo = ImageTk.PhotoImage(img)
        self._photo1 = photo
        label.configure(image=photo)

    def _show_frame_car(self, frame, car):
        """Show a frame on the correct video label for a given car."""
        label = self.video_label_1 if car is self.car1 else self.video_label_2
        self._show_frame(frame, label)

    def _show_blank(self, text="", label=None):
        if label is None:
            label = self.video_label_1
        w = VIDEO_WIDTH
        h = VIDEO_HEIGHT
        blank = np.zeros((h, w, 3), dtype=np.uint8)
        blank[:] = (20, 15, 10)
        if text:
            font = cv2.FONT_HERSHEY_SIMPLEX
            sc, th = 0.5, 1
            (tw, _), _ = cv2.getTextSize(text, font, sc, th)
            x = (w - tw) // 2
            cv2.putText(blank, text, (x, h // 2),
                        font, sc, (180, 180, 180), th, cv2.LINE_AA)
        self._show_frame(blank, label)

    def _show_blank_car(self, text, car):
        label = self.video_label_1 if car is self.car1 else self.video_label_2
        self._show_blank(text, label)

    # ============================================================
    #  YOLO HELPERS
    # ============================================================
    def _fire_info(self, results, shape):
        """Returns (detected, center_x_frac, bbox_area_frac, conf, annotated)."""
        annotated = results[0].plot()
        boxes = results[0].boxes
        if len(boxes) == 0:
            return False, 0.5, 0.0, 0.0, annotated

        fh, fw = shape[:2]
        best_area, best_cx, best_conf = 0, 0.5, 0.0
        for b in boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best_area = area
                best_cx = (x1 + x2) / 2.0 / fw
                best_conf = float(b.conf[0])

        return True, best_cx, best_area / (fw * fh), best_conf, annotated

    def _dist_label(self, frac):
        if frac >= PUMP_BBOX_THRESHOLD:
            return "🎯 PUMP RANGE!", C.RED
        if frac >= CLOSE_THRESHOLD:
            return "📏 Close (~1 m)", C.ORANGE
        if frac >= MEDIUM_THRESHOLD:
            return "📏 Medium (~2 m)", C.ORANGE
        return "📏 Far (>3 m)", C.TEXT_DIM

    def _steer(self, cx):
        if cx < STEER_LEFT_ZONE:
            return "⬅️  TURN LEFT", "left"
        if cx > STEER_RIGHT_ZONE:
            return "➡️  TURN RIGHT", "right"
        return "⬆️  FORWARD", "forward"

    # ============================================================
    #  OVERLAY — draw HUD on annotated frames
    # ============================================================
    def _draw_hud(self, frame, lines):
        """Draw semi-transparent HUD text lines at the bottom of frame."""
        h, w = frame.shape[:2]
        y0 = h - 12 - 25 * len(lines)
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, y0 - 8), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        for i, (txt, clr) in enumerate(lines):
            y = y0 + i * 25
            cv2.putText(frame, txt, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, clr, 2, cv2.LINE_AA)
        return frame

    def _draw_progress_bar(self, frame, frac):
        """Draw fire-confirmation progress bar at frame bottom."""
        h, w = frame.shape[:2]
        bw = int(w * frac)
        cv2.rectangle(frame, (0, h - 6), (bw, h), (0, 100, 255), -1)
        cv2.rectangle(frame, (0, h - 6), (w, h), (80, 80, 80), 1)
        return frame

    # ============================================================
    #  MAIN TICK — dispatches to current state handler
    # ============================================================
    def _tick(self):
        if not self.running:
            return
        try:
            handler = {
                State.SCANNING:         self._do_scanning,
                State.FIRE_CONFIRMED:   self._do_fire_confirmed,
                State.DEPLOYING:        self._do_deploying,
                State.EXITING_GARAGE:   self._do_exiting_garage,
                State.SCANNING_360:     self._do_scanning_360,
                State.CAR_APPROACHING:  self._do_car_approaching,
                State.PUMPING:          self._do_pumping,
                State.EXTINGUISHED:     self._do_extinguished,
                State.RETURN_READY:     self._do_return_ready,
                State.RETURNING:        self._do_returning,
                State.RETURN_FAILED:    self._do_return_failed,
                State.MISSION_COMPLETE: self._do_mission_complete,
            }.get(self.state, lambda: None)
            handler()
        except Exception as e:
            self._log(f"❌ Error: {e}", C.RED)
        if self.running:
            self.root.after(66, self._tick)   # ~15 fps (sufficient for fire detection)

    # ============================================================
    #  STATE: SCANNING
    # ============================================================
    def _do_scanning(self):
        if not self.station_cam:
            return
        frame = self.station_cam.get_frame()
        if frame is None:
            self._show_blank("Waiting for station camera…")
            return

        res = self.model(frame, conf=YOLO_CONF, imgsz=YOLO_IMGSZ, verbose=False)
        det, cx, area, conf, ann = self._fire_info(res, frame.shape)

        if det:
            if self.fire_first_seen_time == 0.0:
                self.fire_first_seen_time = time.time()
                
            elapsed = time.time() - self.fire_first_seen_time
            self._sv_set("fire", "🔥  DETECTED", C.RED)
            self._sv_set("conf", f"{conf:.0%}", C.ORANGE)
            self._sv_set("confirm",
                         f"{elapsed:.1f} / {FIRE_CONFIRM_TIME} sec", C.ORANGE)

            # Progress bar
            frac = min(1.0, elapsed / FIRE_CONFIRM_TIME)
            ann = self._draw_progress_bar(ann, frac)

            if elapsed >= FIRE_CONFIRM_TIME:
                self._log(
                    f"🔥 Fire CONFIRMED! ({FIRE_CONFIRM_TIME} seconds "
                    f"straight, conf {conf:.0%})", C.RED)
                self._show_frame(ann)
                self._set_state(State.FIRE_CONFIRMED)
                return
        else:
            if self.fire_first_seen_time > 0.0:
                self.fire_first_seen_time = 0.0
                self._sv_set("confirm",
                             f"0.0 / {FIRE_CONFIRM_TIME} sec", C.TEXT_DIM)
            self._sv_set("fire", "❌  Not Detected", C.TEXT_DIM)
            self._sv_set("conf", "—", C.TEXT_DIM)

        self._show_frame(ann)

    # ============================================================
    #  STATE: FIRE_CONFIRMED — send DEPLOY, one-shot
    # ============================================================
    def _do_fire_confirmed(self):
        self._clear_route()
        self.extinguished_handled = False
        
        if not self.esp:
            self._log("❌ ABORTING DEPLOY: Gatekeeper COM port is not connected! Cannot trigger car.", C.RED)
            self._set_state(State.SCANNING)
            self.fire_first_seen_time = 0.0
            return

        self._send_serial("F\n")
        self._log(f"🚀 DEPLOY signal sent to Gatekeeper! Waiting 5s...", C.ORANGE)
        self._gatekeeper_http("/open_doors", "🚪 Opening garage doors (via Servo ESP32)...")
        self._sv_set("doors", "🟡 Opening...", C.ORANGE)

        self._stop_station_camera()
        self._log("📷 Station camera closed.", C.BLUE)

        self._start_car_camera()
        self.deploy_time = time.time()
        self._set_state(State.DEPLOYING)

    # ============================================================
    #  STATE: DEPLOYING — wait for car cameras
    # ============================================================
    def _do_deploying(self):
        elapsed = time.time() - self.deploy_time

        # Check if any car cam has a frame
        any_frame = None
        for car in self.cars:
            f = car.get_frame()
            if f is not None:
                any_frame = f
                break

        if any_frame is not None:
            if elapsed < DEPLOY_WAIT_SEC:
                f = self._draw_hud(any_frame, [
                    (f"DEPLOYING IN {DEPLOY_WAIT_SEC - elapsed:.1f}s", (0, 165, 255)),
                    ("Waiting for shutters to open...", (255, 255, 255)),
                ])
                self._show_frame(f, self.video_label_1)
                return

            self._log(f"📷 Car camera connected! {DEPLOY_WAIT_SEC:.0f}s passed.", C.GREEN)
            self._sv_set("doors", "🟢 Open", C.GREEN)
            self._sv_set("car_feed", "🟢  Connected", C.GREEN)
            self._log(f"🚗 Car driving blind for {GARAGE_EXIT_SEC:.0f}s to exit garage.", C.BLUE)
            self.exit_start_time = time.time()
            self._set_state(State.EXITING_GARAGE)
            return

        if elapsed < DEPLOY_WAIT_SEC:
            self._show_blank(f"Deploying in {DEPLOY_WAIT_SEC - elapsed:.1f}s...")
        else:
            self._show_blank("Connecting to car camera...")

        if time.time() - self.deploy_time > CAR_CONNECT_TIMEOUT:
            self._log("⚠️  Car camera timeout.", C.ORANGE)

    # ============================================================
    #  STATE: EXITING_GARAGE — both cars drive forward
    # ============================================================
    def _do_exiting_garage(self):
        elapsed = time.time() - self.exit_start_time

        for car in self.cars:
            if not car.garage_exit_started:
                self._car_move_to(car, "forward", force=True)
                car.garage_exit_started = True

        if elapsed >= GARAGE_EXIT_SEC:
            for car in self.cars:
                car.garage_exit_started = False
                self._car_move_to(car, "stop", record=True, force=True)
                car.scan_360_start = time.time()
                car.scan_360_fire_first = 0.0
                self._car_move_to(car, "right", record=False, force=True)
                car.sub_state = "SCANNING_360"
            self._log("🚗 Cars clear of garage. Starting 360° scan.", C.CYAN)
            self._log("🚪 Auto-closing garage doors after exit.", C.GREEN)
            self._gatekeeper_http("/close_doors", "🚪 Auto-closing garage doors (via Servo ESP32)...")
            self._sv_set("doors", "🟢 Closed", C.GREEN)
            self._set_state(State.SCANNING_360)
            return

        for car in self.cars:
            f = car.get_frame()
            if f is not None:
                f = self._draw_hud(f, [
                    (f"{car.name} EXITING: {GARAGE_EXIT_SEC - elapsed:.1f}s", (0, 255, 0))
                ])
                self._show_frame_car(f, car)
            else:
                self._show_blank_car(f"{car.name} exiting...", car)

    # ============================================================
    #  STATE: SCANNING_360 - both cars scan independently
    # ============================================================
    def _do_scanning_360(self):
        for car in self.cars:
            if car.sub_state != "SCANNING_360":
                f = car.get_frame()
                if f is not None:
                    self._show_frame_car(f, car)
                continue

            frame = car.get_frame()
            if frame is None:
                self._show_blank_car(f"{car.name} scanning...", car)
                continue

            res = self.model(frame, conf=CAR_YOLO_CONF, imgsz=CAR_YOLO_IMGSZ, verbose=False)
            det, cx, area, conf, ann = self._fire_info(res, frame.shape)

            if det:
                if car.scan_360_fire_first == 0.0:
                    car.scan_360_fire_first = time.time()
                fire_dur = time.time() - car.scan_360_fire_first

                if fire_dur >= SCAN_360_CONFIRM_SEC:
                    self._log(f"🔥 {car.name}: Fire locked! Lunging forward.", C.RED)
                    self._car_move_to(car, "forward", force=True)
                    car.sub_state = "LUNGING"
                    car.lunge_start = time.time()
                else:
                    ann = self._draw_hud(ann, [
                        (f"{car.name} FIRE ({fire_dur:.1f}/{SCAN_360_CONFIRM_SEC:.1f}s)", (0, 100, 255)),
                    ])
            else:
                car.scan_360_fire_first = 0.0
                
                # Step-Turn Logic (rotate 90 degrees every 2 seconds)
                elapsed_step = time.time() - car.scan_step_turn_start
                if car.scan_step_turn_start == 0.0:
                    car.scan_step_turn_start = time.time()
                    self._car_move_to(car, "stop", record=False, force=True)
                    car.scan_is_turning = False
                    
                if car.scan_is_turning:
                    if elapsed_step >= 0.25: # Turn duration
                        self._car_move_to(car, "stop", record=True, force=True)
                        car.scan_is_turning = False
                        car.scan_step_turn_start = time.time()
                    ann = self._draw_hud(ann, [("TURNING 90...", (200, 100, 255))])
                else:
                    if elapsed_step >= 2.0: # Wait duration
                        self._car_move_to(car, "right", record=True, force=True)
                        car.scan_is_turning = True
                        car.scan_step_turn_start = time.time()
                    ann = self._draw_hud(ann, [(f"WAITING... {2.0 - elapsed_step:.1f}s", (180, 100, 255))])

            self._show_frame_car(ann, car)

        # If all cars done scanning, advance
        if all(c.sub_state != "SCANNING_360" for c in self.cars):
            self._set_state(State.CAR_APPROACHING)
    # ============================================================
    #  STATE: CAR_APPROACHING - per-car YOLO steer + distance
    # ============================================================
    def _do_car_approaching(self):
        for car in self.cars:
            if car.sub_state == "LUNGING":
                elapsed = time.time() - car.lunge_start
                f = car.get_frame()
                if f is not None:
                    f = self._draw_hud(f, [(f"LUNGING: {0.5 - elapsed:.1f}s", (0, 0, 255))])
                    self._show_frame_car(f, car)
                
                if elapsed >= 0.5:
                    self._log(f"🎯 {car.name}: Lunge complete. Pumping.", C.RED)
                    self._car_move_to(car, "stop", record=True, force=True)
                    self._car_http_to(car, "/pump_on", "💧 Pump ON!")
                    car.pump_active = True
                    car.pump_start_time = time.time()
                    car.sub_state = "PUMPING"
            elif car.sub_state == "PUMPING":
                pass
            elif car.sub_state != "SCANNING_360":
                f = car.get_frame()
                if f is not None:
                    self._show_frame_car(f, car)
                continue

        pumping = [c for c in self.cars if c.sub_state == "PUMPING"]
        if pumping and self.state != State.PUMPING:
            self._sv_set("pump", f"🟢  {len(pumping)} car(s) pumping", C.GREEN)
            self._set_state(State.PUMPING)

    # ============================================================
    #  STATE: PUMPING - per-car pump, wait for fire out
    # ============================================================
    def _do_pumping(self):
        for car in self.cars:
            if car.sub_state == "PUMPING":
                frame = car.get_frame()
                elapsed = time.time() - car.pump_start_time
                
                if elapsed >= 4.0:
                    self._log(f"\u2705 {car.name}: Pump ran for 4 seconds. Extinguishing complete.", C.GREEN)
                    self._car_http_to(car, "/pump_off")
                    car.sub_state = "EXTINGUISHED"
                    car.extinguished_handled = False
                else:
                    if frame is not None:
                        frame = self._draw_hud(frame, [
                            (f"{car.name} PUMPING: {4.0 - elapsed:.1f}s", (0, 255, 200))
                        ])
                        self._show_frame_car(frame, car)

            elif car.sub_state == "APPROACHING":
                pass # Replaced by lunging

            else:
                f = car.get_frame()
                if f is not None:
                    self._show_frame_car(f, car)

        # Transition to extinguished if all deployed cars are done
        deployed_cars = [c for c in self.cars if c.sub_state not in ("SCANNING_360", "LUNGING", "APPROACHING", "PUMPING")]
        if len(deployed_cars) == len(self.cars):
            if self.state == State.PUMPING:
                self._sv_set("fire", "\u2705  EXTINGUISHED", C.GREEN)
                self._sv_set("pump", "\u2b1c  OFF", C.TEXT_DIM)
                self._set_state(State.EXTINGUISHED)
    # ============================================================
    #  STATE: EXTINGUISHED — stop and wait for END MISSION
    # ============================================================
    def _do_extinguished(self):
        if self.extinguished_handled:
            return
        self.extinguished_handled = True

        for car in self.cars:
            self._car_move_to(car, "stop", record=True, force=True)
            self._car_http_to(car, "/pump_off", f"💧 {car.name} Pump OFF.")
            car.pump_active = False
        self._sv_set("pump", "⬜  OFF", C.TEXT_DIM)
        self._sv_set("fire", "✅  Extinguished", C.GREEN)
        self._sv_set("dir", "⏹️  STOPPED", C.TEXT_DIM)
        self._sv_set("dist", "—", C.TEXT_DIM)
        self.btn_end.configure(text="↩️ RETURN TO BASE / END MISSION")
        self._log("✅ All fires extinguished. Press END MISSION.", C.GREEN)
        self._set_state(State.RETURN_READY)


    # ============================================================
    #  STATE: RETURN_READY — hold position until user ends mission
    # ============================================================
    def _do_return_ready(self):
        for car in self.cars:
            frame = car.get_frame()
            if frame is not None:
                frame = self._draw_hud(frame, [
                    ("FIRE EXTINGUISHED", (0, 255, 0)),
                    ("Press END MISSION to return", (255, 255, 255)),
                ])
                self._show_frame_car(frame, car)

    # ============================================================
    #  STATE: RETURNING — worker controls motors, GUI shows progress
    # ============================================================
    def _do_returning(self):
        total = max(1, self.return_total_steps)
        done = self.return_completed_steps
        current = self.return_current_cmd
        remaining = max(0.0, self.return_step_remaining)
        self._sv_set("return", f"{min(done + 1, total)} / {total}", C.CYAN)
        self._sv_set("return_cmd", f"{current} \u00b7 {remaining:.1f}s", C.ORANGE)

        for car in self.cars:
            frame = car.get_frame()
            if frame is not None:
                frame = self._draw_hud(frame, [
                    ("RETURNING TO BASE", (255, 255, 0)),
                    (f"Step {min(done + 1, total)}/{total}: {current}", (0, 200, 255)),
                ])
                self._show_frame_car(frame, car)
            else:
                self._show_blank_car(f"Returning: {current}", car)

    def _do_return_failed(self):
        if self.car_cam:
            frame = self.car_cam.get_frame()
            if frame is not None:
                frame = self._draw_hud(frame, [
                    ("RETURN INTERRUPTED — CAR STOPPED", (0, 0, 255)),
                    ("Check Wi-Fi, then press RETRY RETURN", (255, 255, 255)),
                ])
                self._show_frame(frame)

    # ============================================================
    #  STATE: MISSION_COMPLETE — stay finished until NEW MISSION
    # ============================================================
    def _do_mission_complete(self):
        frame = None
        if self.car_cam:
            frame = self.car_cam.get_frame()
        if self.station_cam and frame is None:
            frame = self.station_cam.get_frame()

        if frame is not None:
            h, w = frame.shape[:2]

            # Dark overlay
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, h), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.60, frame, 0.40, 0, frame)

            # Green border glow
            cv2.rectangle(frame, (2, 2), (w - 3, h - 3), (0, 230, 118), 3)
            cv2.rectangle(frame, (6, 6), (w - 7, h - 7), (0, 180, 80), 1)

            # Main success text
            main_text = "MISSION SUCCESSFUL"
            font = cv2.FONT_HERSHEY_SIMPLEX
            (tw, th), _ = cv2.getTextSize(main_text, font, 1.2, 3)
            x = (w - tw) // 2
            y = h // 2 - 40
            # Shadow
            cv2.putText(frame, main_text, (x + 2, y + 2), font, 1.2, (0, 60, 0), 3, cv2.LINE_AA)
            # Main
            cv2.putText(frame, main_text, (x, y), font, 1.2, (0, 255, 120), 3, cv2.LINE_AA)

            # Sub text
            sub_text = "Fire Extinguished Successfully"
            (tw2, _), _ = cv2.getTextSize(sub_text, font, 0.7, 2)
            x2 = (w - tw2) // 2
            cv2.putText(frame, sub_text, (x2, y + 45), font, 0.7, (0, 200, 100), 2, cv2.LINE_AA)

            # Bottom info
            ts = datetime.now().strftime("%H:%M:%S")
            info_text = f"Car returned to base  |  {ts}"
            (tw3, _), _ = cv2.getTextSize(info_text, font, 0.5, 1)
            x3 = (w - tw3) // 2
            cv2.putText(frame, info_text, (x3, h - 20), font, 0.5, (180, 180, 180), 1, cv2.LINE_AA)

            # Checkmark icon
            cx, cy = w // 2, y - 55
            cv2.circle(frame, (cx, cy), 28, (0, 230, 118), 3, cv2.LINE_AA)
            pts = np.array([(cx - 14, cy), (cx - 4, cy + 12), (cx + 16, cy - 10)], dtype=np.int32)
            cv2.polylines(frame, [pts], False, (0, 230, 118), 3, cv2.LINE_AA)

            self._show_frame(frame)
        else:
            self._show_blank("MISSION SUCCESSFUL — Fire Extinguished")

    # ---- DEMO: Hardcoded mission success ----
    def _demo_mission_success(self):
        """Simulate a successful mission for demo video recording.
        Does NOT signal gatekeeper or car. Only uses the box ESP-CAM stream."""
        self._log("🎬 DEMO MODE: Simulating full mission success...", C.GOLD)
        self._log("ℹ️  Demo mode — no gatekeeper/car signals sent.", C.GOLD)

        # Keep station cam alive (box ESP-CAM) — do NOT stop it
        if not self.station_cam:
            self._start_station_camera()

        # Stop car cam if somehow running (demo doesn't need it)
        if self.car_cam:
            self.car_cam.stop()
            self.car_cam = None

        # Set all status fields to success state
        self._sv_set("fire", "✅  EXTINGUISHED", C.GREEN)
        self._sv_set("conf", "92%", C.GREEN)
        self._sv_set("confirm", f"{FIRE_CONFIRM_TIME} / {FIRE_CONFIRM_TIME} sec", C.GREEN)
        self._sv_set("dist", "🎯 PUMP RANGE!", C.GREEN)
        self._sv_set("dir", "🏠  AT BASE", C.GREEN)
        self._sv_set("pump", "⬜  OFF (Complete)", C.TEXT_DIM)
        self._sv_set("car_feed", "🟢  Connected", C.GREEN)
        self._sv_set("path", "5 segments · F:3.2s  L:1.1s  F:2.8s  R:0.9s  F:1.5s", C.CYAN)
        self._sv_set("path_time", "9.50 s", C.TEXT)
        self._sv_set("return", "✅ Base reached (timed)", C.GREEN)
        self._sv_set("return_cmd", "STOPPED", C.GREEN)

        self.btn_end.configure(state="normal", text="🔄 NEW MISSION")
        self.btn_demo.configure(state="disabled", text="✅ Demo Active")

        # Simulated mission log — no actual signals sent
        self._log("🔥 Fire CONFIRMED! (0.5 seconds straight, conf 92%)", C.RED)
        self._log("🚀 DEPLOY signal sent to car!", C.ORANGE)
        self._log("📷 Car camera connected!", C.GREEN)
        self._log("🚗 Car command: ⬆️ /forward", C.BLUE)
        self._log("🎯 Fire in PUMP RANGE! Stopping car, activating pump.", C.RED)
        self._log("💧 Pump ON!", C.CYAN)
        self._log("✅ Fire appears EXTINGUISHED!", C.GREEN)
        self._log("💧 Pump OFF.", C.BLUE)
        self._log("🏠 END MISSION: replaying 5 saved segments in reverse (9.50s estimated).", C.CYAN)
        self._log("↩️ Return step 1/5: reverse for 1.50s", C.CYAN)
        self._log("↩️ Return step 2/5: undo-right for 0.90s", C.CYAN)
        self._log("↩️ Return step 3/5: reverse for 2.80s", C.CYAN)
        self._log("↩️ Return step 4/5: undo-left for 1.10s", C.CYAN)
        self._log("↩️ Return step 5/5: reverse for 3.20s", C.CYAN)
        self._log("🏁 MISSION SUCCESSFUL — Fire extinguished and car returned to base!", C.GREEN)
        self._log("🎖️  All objectives completed. System ready for next deployment.", C.GREEN)

        self._set_state(State.MISSION_COMPLETE)

    # ============================================================
    #  CLEANUP
    # ============================================================
    def _on_closing(self):
        self.return_abort_event.set()
        self.command_epoch += 1
        self.running = False
        self._log("🛑 Shutting down…", C.ORANGE)

        # Safety STOP via HTTP & Gatekeeper
        for car in self.cars:
            try:
                requests.get(f"http://{car.ip}:{car.cmd_port}/stop_motors", timeout=0.5)
            except Exception:
                pass

        try:
            if self.esp:
                with self.serial_lock:
                    self.esp.write(b"S")
        except Exception:
            pass

        # Stop cameras
        self._stop_station_camera()
        for car in self.cars:
            car.stop_cam()

        # Close serial
        if self.esp:
            try:
                self.esp.close()
            except Exception:
                pass

        self.root.destroy()

    # ============================================================
    #  RUN
    # ============================================================
    def run(self):
        self.root.mainloop()


# ================================================================
#  ENTRY POINT
# ================================================================
def main():
    print("🔥 Fire Fauji Command Center v3")
    print("=" * 40)
    dashboard = FireDashboard()
    dashboard.run()
    print("✅ Done.")


if __name__ == "__main__":
    main()
