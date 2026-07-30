#!/usr/bin/env python3
"""
===============================================================
  Restaurant Robot – Pan/Tilt Face Tracker  v4  (COMPLETE REWRITE)
  Hardware : Raspberry Pi 4B
             Raspberry Pi Camera Module 3  (picamera2 / libcamera)
             PCA9685 PWM driver  (I2C 0x40)
             2 × servos  –  pan=ch0  tilt=ch1
  Display  : 5-inch screen
===============================================================
  Architecture
  ─────────────
  THREE independent threads:
    1. Capture thread  – grabs frames from camera at full fps,
                         puts them in a 1-slot buffer (always fresh).
    2. Detection thread – reads frames, runs Haar, updates a shared
                          "target" position.  Runs as fast as the Pi
                          allows (~8-15 fps).
    3. Servo thread    – runs at a fixed 25 Hz regardless of camera
                          speed.  Each tick it smoothly steps the
                          servo toward the current target.  This is
                          what makes movement buttery-smooth even
                          when detection is slow.
    Main thread        – reads the latest annotated display frame
                          and calls cv2.imshow at ~30fps.

  Why previous versions were jerky
  ──────────────────────────────────
  All previous versions updated the servo ONLY when a new detection
  arrived (~8fps).  Between detections the servo sat still, then
  lurched to the next position – that IS the jitter.  A dedicated
  25 Hz servo thread that continuously interpolates eliminates this.

  Face quality improvements
  ──────────────────────────
  • minNeighbors=7  (was 4-5) – far fewer false positives
  • minSize=(90,90)            – ignore tiny/far detections
  • CONFIRM_FRAMES=3           – a face must appear in 3 consecutive
                                 frames before we start tracking it
  • Track by proximity         – we always follow the LARGEST (closest)
                                 face, not the first one detected
  • Position EMA on detection  – smooths out single-frame jitter in
                                 the reported face centre

  Mirror fix
  ───────────
  Detection runs on the RAW (unmirrored) frame.
  Pan direction is NEGATED so servo follows real-world direction.
  Display frame is flipped horizontally for natural selfie view.

  Clean shutdown
  ───────────────
  A threading.Event() signals all threads to stop.  Each thread
  checks it and exits cleanly.  Servo goes to home AFTER the servo
  thread stops – no more post-exit jitter.
===============================================================
"""

import cv2
import os
import time
import threading
import queue
import numpy as np
from collections import deque
from picamera2 import Picamera2
from adafruit_servokit import ServoKit
import logging

# ───────────────────────────────────────────────────────────────
# LOGGING
# ───────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════
# TUNABLE SETTINGS  ← change values here, nowhere else
# ═══════════════════════════════════════════════════════════════

# Camera
FRAME_W    = 640
FRAME_H    = 480
FRAME_RATE = 30

# Servo channels on PCA9685
PAN_CHANNEL  = 0
TILT_CHANNEL = 1

# Servo angle limits (degrees)
PAN_MIN  = 30;   PAN_MAX  = 150
TILT_MIN = 60;   TILT_MAX = 120

# Home position (degrees)
PAN_HOME  = 90
TILT_HOME = 85

# Dead-zone – pixels from frame centre where we do NOT move
DEAD_ZONE_X = 30
DEAD_ZONE_Y = 25

# Proportional gain  (degrees per pixel of error)
KP_PAN  = 0.06
KP_TILT = 0.05

# Maximum servo target change per detection frame (degrees)
# Keeps a single noisy detection from causing a big jump in target
MAX_TARGET_STEP_PAN  = 8.0
MAX_TARGET_STEP_TILT = 6.0

# Servo thread smoothing  ← THIS is what makes movement smooth
# The servo thread runs at 25 Hz and blends current→target at this rate.
# 0.0 = never moves,  1.0 = instant snap.
# 0.12 gives buttery-smooth motion even during slow detection.
SERVO_ALPHA = 0.12

# Face position smoothing (on the detected pixel position, before control)
FACE_POS_ALPHA = 0.50    # 0=lag/smooth  1=raw/responsive

# Face must be detected this many consecutive frames before tracking starts
CONFIRM_FRAMES = 3

# Face must disappear this many frames before we stop tracking
LOSE_FRAMES = 8

# Timing
LOST_FACE_TIMEOUT = 2.5   # seconds before search mode starts
SEARCH_STEP_DELAY = 1.0
STABLE_TIME       = 1.5   # seconds centred before locking

# Servo update rate (Hz)
SERVO_HZ = 25

# Display
SHOW_WINDOW    = True
WINDOW_NAME    = "AURA Face Tracker"
WIN_W          = 640
WIN_H          = 480
MIRROR_DISPLAY = True   # flip display so left/right feels natural (selfie view)

# ═══════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════

def pt(x, y):
    """Always-safe int coordinate tuple for OpenCV."""
    return (int(round(x)), int(round(y)))


def clamp(v, lo, hi):
    return max(float(lo), min(float(hi), float(v)))


def _cv2_data(filename):
    """Resolve path to a cascade XML bundled with the cv2 package."""
    try:
        return os.path.join(os.path.dirname(cv2.__file__), "data", filename)
    except Exception:
        return filename


CASCADE_PATHS = [
    _cv2_data("haarcascade_frontalface_default.xml"),
    _cv2_data("haarcascade_frontalface_alt2.xml"),
    _cv2_data("haarcascade_frontalface_alt.xml"),
    _cv2_data("lbpcascade_frontalface_improved.xml"),
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
    "haarcascade_frontalface_default.xml",
]


def load_cascade():
    tried = []
    for path in CASCADE_PATHS:
        if not path:
            continue
        if os.path.exists(path):
            cc = cv2.CascadeClassifier(path)
            if not cc.empty():
                log.info("Cascade loaded  <-  %s", path)
                return cc
            tried.append("%s  (parse failed)" % path)
        else:
            tried.append("%s  (not found)" % path)
    raise FileNotFoundError(
        "No usable cascade found.\n  Tried:\n    " + "\n    ".join(tried) + "\n"
        "  Fix:  pip install opencv-contrib-python"
    )


# ═══════════════════════════════════════════════════════════════
# SERVO CONTROLLER  –  thread-safe angle store + hardware writer
# ═══════════════════════════════════════════════════════════════

class PanTiltController:
    """
    Stores the CURRENT servo angles (the smooth running position)
    and the TARGET angles (where detection wants us to go).
    The servo thread reads both and steps current toward target.
    """

    def __init__(self):
        log.info("Initialising PCA9685 ...")
        self.kit  = ServoKit(channels=16)
        self._lock = threading.Lock()

        # Current smooth position (what the servo is actually at)
        self._cur_pan  = float(PAN_HOME)
        self._cur_tilt = float(TILT_HOME)

        # Target position (where detection wants us to go)
        self._tgt_pan  = float(PAN_HOME)
        self._tgt_tilt = float(TILT_HOME)

        self._apply(self._cur_pan, self._cur_tilt)
        log.info("Servos at home  pan=%.1f  tilt=%.1f", self._cur_pan, self._cur_tilt)

    def _apply(self, pan, tilt):
        """Write angles to hardware – call with lock held OR from init."""
        self.kit.servo[PAN_CHANNEL ].angle = int(round(clamp(pan,  PAN_MIN,  PAN_MAX)))
        self.kit.servo[TILT_CHANNEL].angle = int(round(clamp(tilt, TILT_MIN, TILT_MAX)))

    def set_target(self, pan, tilt):
        """Detection thread calls this to update where we want to go."""
        with self._lock:
            self._tgt_pan  = clamp(pan,  PAN_MIN, PAN_MAX)
            self._tgt_tilt = clamp(tilt, TILT_MIN, TILT_MAX)

    def smooth_step(self):
        """
        Servo thread calls this at 25 Hz.
        Blends current position toward target using SERVO_ALPHA.
        Returns (cur_pan, cur_tilt) after the step.
        """
        with self._lock:
            self._cur_pan  = SERVO_ALPHA * self._tgt_pan  + (1.0 - SERVO_ALPHA) * self._cur_pan
            self._cur_tilt = SERVO_ALPHA * self._tgt_tilt + (1.0 - SERVO_ALPHA) * self._cur_tilt
            self._apply(self._cur_pan, self._cur_tilt)
            return self._cur_pan, self._cur_tilt

    def go_home_immediate(self):
        """Called from cleanup – snap directly to home, no smoothing."""
        with self._lock:
            self._tgt_pan  = float(PAN_HOME)
            self._tgt_tilt = float(TILT_HOME)
            self._cur_pan  = float(PAN_HOME)
            self._cur_tilt = float(TILT_HOME)
            self._apply(PAN_HOME, TILT_HOME)

    @property
    def cur_pan(self):
        with self._lock: return self._cur_pan
    @property
    def cur_tilt(self):
        with self._lock: return self._cur_tilt
    @property
    def tgt_pan(self):
        with self._lock: return self._tgt_pan
    @property
    def tgt_tilt(self):
        with self._lock: return self._tgt_tilt


# ═══════════════════════════════════════════════════════════════
# FACE DETECTOR  –  strict Haar + confirmation buffer
# ═══════════════════════════════════════════════════════════════

class FaceDetector:
    """
    Wraps the Haar cascade with:
      • strict parameters to reduce false positives
      • a confirmation buffer (face must appear N consecutive frames)
      • a lose buffer (face must disappear N consecutive frames)
      • EMA smoothing on the confirmed face position
    """

    def __init__(self, cascade):
        self.cascade = cascade
        # Confirmation state
        self._confirm_count = 0   # consecutive frames with a face
        self._lose_count    = 0   # consecutive frames WITHOUT a face
        self._confirmed     = False

        # Smoothed face position
        self._sx = float(FRAME_W // 2)
        self._sy = float(FRAME_H // 2)
        self._sw = 120.0
        self._sh = 120.0

    def update(self, gray_frame):
        """
        Returns (cx, cy, w, h) as plain ints if a confirmed face exists,
        otherwise None.  cx/cy are EMA-smoothed.
        """
        faces = self.cascade.detectMultiScale(
            gray_frame,
            scaleFactor  = 1.08,    # balanced: not too fast/slow
            minNeighbors = 7,       # high = fewer false positives
            minSize      = (90, 90),# ignore small/far faces
            flags        = cv2.CASCADE_SCALE_IMAGE,
        )

        if len(faces) > 0:
            # Always track the LARGEST (closest) face
            x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
            raw_cx = float(x + w // 2)
            raw_cy = float(y + h // 2)

            # Confirmation counter
            self._confirm_count += 1
            self._lose_count     = 0

            if not self._confirmed:
                if self._confirm_count >= CONFIRM_FRAMES:
                    self._confirmed = True
                    # Snap position on first confirmation
                    self._sx = raw_cx
                    self._sy = raw_cy
                    self._sw = float(w)
                    self._sh = float(h)
                else:
                    return None   # still confirming

            # EMA smoothing on position + size
            self._sx = FACE_POS_ALPHA * raw_cx + (1.0 - FACE_POS_ALPHA) * self._sx
            self._sy = FACE_POS_ALPHA * raw_cy + (1.0 - FACE_POS_ALPHA) * self._sy
            self._sw = FACE_POS_ALPHA * w      + (1.0 - FACE_POS_ALPHA) * self._sw
            self._sh = FACE_POS_ALPHA * h      + (1.0 - FACE_POS_ALPHA) * self._sh

            return (int(self._sx), int(self._sy), int(self._sw), int(self._sh))

        else:
            # No face this frame
            self._confirm_count = 0
            self._lose_count   += 1
            if self._lose_count >= LOSE_FRAMES:
                self._confirmed = False
            # Return last smoothed position while within lose buffer
            if self._confirmed:
                return (int(self._sx), int(self._sy), int(self._sw), int(self._sh))
            return None


# ═══════════════════════════════════════════════════════════════
# MAIN APPLICATION
# ═══════════════════════════════════════════════════════════════

class RestaurantRobot:

    def __init__(self):
        self._stop_event = threading.Event()

        # ── Camera ──────────────────────────────────────────────
        log.info("Starting Picamera2 ...")
        self.cam = Picamera2()
        cfg = self.cam.create_preview_configuration(
            main={"size": (FRAME_W, FRAME_H), "format": "BGR888"},
            controls={"FrameRate": FRAME_RATE},
        )
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(1.0)
        log.info("Camera ready  %dx%d @ %d FPS", FRAME_W, FRAME_H, FRAME_RATE)

        # ── Servos ───────────────────────────────────────────────
        self.servo = PanTiltController()

        # ── Detector ─────────────────────────────────────────────
        cascade       = load_cascade()
        self.detector = FaceDetector(cascade)

        # ── Shared state (written by detection, read by servo/display) ──
        self._lock          = threading.Lock()
        self.state          = "TRACKING"
        self.last_face_time = time.time()
        self.stable_since   = None
        self.frozen         = False
        self._current_face  = None    # latest confirmed face (cx,cy,w,h)

        # Search pattern
        self._search_angles = [
            (PAN_HOME - 35, TILT_HOME),
            (PAN_HOME,      TILT_HOME),
            (PAN_HOME + 35, TILT_HOME),
            (PAN_HOME,      TILT_HOME - 10),
            (PAN_HOME,      TILT_HOME),
        ]
        self._search_idx = 0

        # FPS counter (for display)
        self._fps          = 0.0
        self._fps_t0       = time.time()
        self._fps_count    = 0
        self._det_fps      = 0.0
        self._det_fps_t0   = time.time()
        self._det_fps_cnt  = 0

        # 1-slot frame buffer between capture & detection threads
        self._frame_q   = queue.Queue(maxsize=1)
        # 1-slot display buffer between detection & main threads
        self._display_q = queue.Queue(maxsize=1)

        # ── Window ───────────────────────────────────────────────
        if SHOW_WINDOW:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, WIN_W, WIN_H)
            cv2.moveWindow(WINDOW_NAME, 0, 0)

        log.info("All systems ready.  Press Q / Esc to quit.")

    # ─────────────────────────────────────────────────────────────
    # THREAD 1 – capture: fills _frame_q as fast as camera allows
    # ─────────────────────────────────────────────────────────────
    def _capture_thread(self):
        while not self._stop_event.is_set():
            try:
                frame = self.cam.capture_array()
                if frame is None or frame.size == 0:
                    continue
                # Drop old frame, put new one (always fresh)
                try:
                    self._frame_q.get_nowait()
                except queue.Empty:
                    pass
                self._frame_q.put(frame)
            except Exception as exc:
                if not self._stop_event.is_set():
                    log.warning("Capture error: %s", exc)

    # ─────────────────────────────────────────────────────────────
    # THREAD 2 – detection: reads frames, runs Haar, updates target
    # ─────────────────────────────────────────────────────────────
    def _detection_thread(self):
        last_search_step = time.time()

        while not self._stop_event.is_set():
            try:
                frame = self._frame_q.get(timeout=0.5)
            except queue.Empty:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            face = self.detector.update(gray)
            now  = time.time()

            # FPS counter for detection
            self._det_fps_cnt += 1
            det_elapsed = now - self._det_fps_t0
            if det_elapsed >= 1.0:
                self._det_fps      = self._det_fps_cnt / det_elapsed
                self._det_fps_t0   = now
                self._det_fps_cnt  = 0

            with self._lock:
                current_face = face

            if face is not None:
                cx, cy, fw, fh = face
                with self._lock:
                    self.last_face_time = now
                    self.state          = "TRACKING"
                    self._search_idx    = 0
                    self._current_face  = face

                if self.frozen:
                    if (abs(cx - FRAME_W // 2) > DEAD_ZONE_X * 2 or
                            abs(cy - FRAME_H // 2) > DEAD_ZONE_Y * 2):
                        with self._lock:
                            self.frozen       = False
                            self.stable_since = None
                else:
                    centred = self._compute_target(cx, cy)
                    with self._lock:
                        if centred:
                            if self.stable_since is None:
                                self.stable_since = now
                            elif now - self.stable_since >= STABLE_TIME:
                                self.frozen = True
                        else:
                            self.stable_since = None
            else:
                with self._lock:
                    self.frozen        = False
                    self.stable_since  = None
                    self._current_face = None
                    elapsed = now - self.last_face_time

                if elapsed > LOST_FACE_TIMEOUT:
                    with self._lock:
                        self.state = "SEARCHING"
                    if now - last_search_step > SEARCH_STEP_DELAY:
                        self._do_search_step()
                        last_search_step = now

            # Build annotated display frame (mirrored)
            display = frame.copy()
            if MIRROR_DISPLAY:
                display = cv2.flip(display, 1)
                disp_face = None
                if face is not None:
                    fcx, fcy, bw, bh = face
                    disp_face = (FRAME_W - fcx, fcy, bw, bh)
            else:
                disp_face = face

            self._draw_overlay(display, disp_face)

            # Push to display queue (drop stale frame)
            try:
                self._display_q.get_nowait()
            except queue.Empty:
                pass
            self._display_q.put(display)

    # ─────────────────────────────────────────────────────────────
    # THREAD 3 – servo: runs at fixed 25 Hz, smooth interpolation
    # ─────────────────────────────────────────────────────────────
    def _servo_thread(self):
        interval = 1.0 / SERVO_HZ
        while not self._stop_event.is_set():
            t0 = time.time()
            self.servo.smooth_step()
            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ─────────────────────────────────────────────────────────────
    # CONTROL LOGIC  –  compute new TARGET angle from face position
    # ─────────────────────────────────────────────────────────────
    def _compute_target(self, cx, cy):
        """
        Called from detection thread.
        Returns True if face is inside dead-zone (centred).

        Pan is NEGATED: face to the right in the camera image means
        the person is to the LEFT in the real world, so we decrease
        pan angle (turn left).
        """
        err_x = cx - FRAME_W // 2   # + = face right = person left in reality
        err_y = cy - FRAME_H // 2   # + = face below centre

        if abs(err_x) <= DEAD_ZONE_X and abs(err_y) <= DEAD_ZONE_Y:
            return True

        # Negate pan to correct for mirror
        delta_pan  = clamp(-KP_PAN  * err_x, -MAX_TARGET_STEP_PAN,  MAX_TARGET_STEP_PAN)
        delta_tilt = clamp( KP_TILT * err_y, -MAX_TARGET_STEP_TILT, MAX_TARGET_STEP_TILT)

        new_tgt_pan  = clamp(self.servo.tgt_pan  + delta_pan,  PAN_MIN,  PAN_MAX)
        new_tgt_tilt = clamp(self.servo.tgt_tilt + delta_tilt, TILT_MIN, TILT_MAX)
        self.servo.set_target(new_tgt_pan, new_tgt_tilt)
        return False

    def _do_search_step(self):
        pan, tilt = self._search_angles[self._search_idx]
        self.servo.set_target(pan, tilt)
        self._search_idx = (self._search_idx + 1) % len(self._search_angles)

    # ─────────────────────────────────────────────────────────────
    # DRAWING
    # ─────────────────────────────────────────────────────────────
    def _draw_angle_bar(self, frame, label, value, vmin, vmax, x, y, bw=115, bh=11):
        frac   = max(0.0, min(1.0, (value - vmin) / max(vmax - vmin, 1)))
        filled = int(bw * frac)
        cv2.rectangle(frame, pt(x, y),          pt(x + bw,     y + bh), (40, 40, 40),    -1)
        cv2.rectangle(frame, pt(x, y),          pt(x + filled, y + bh), (0, 200, 255),   -1)
        cv2.rectangle(frame, pt(x, y),          pt(x + bw,     y + bh), (100, 100, 100),  1)
        cv2.putText(frame, "%s %5.1f" % (label, value),
                    pt(x + bw + 6, y + bh - 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)

    def _draw_overlay(self, frame, face_info):
        fh, fw = frame.shape[:2]
        cx_f, cy_f = fw // 2, fh // 2

        cur_pan  = self.servo.cur_pan
        cur_tilt = self.servo.cur_tilt

        # ── Top banner ────────────────────────────────────────
        overlay = frame.copy()
        cv2.rectangle(overlay, pt(0, 0), pt(fw, 32), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        with self._lock:
            state     = self.state
            det_fps   = self._det_fps
            frozen    = self.frozen

        state_col = (0, 220, 80) if state == "TRACKING" else (0, 160, 255)

        cv2.putText(
            frame,
            "AURA  |  %-10s|  %.1f det-fps  |  Pan %.0f  Tilt %.0f" % (
                state, det_fps, cur_pan, cur_tilt),
            pt(8, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.46, state_col, 1, cv2.LINE_AA)

        # ── Angle bars ────────────────────────────────────────
        self._draw_angle_bar(frame, "PAN ", cur_pan,  PAN_MIN,  PAN_MAX,  8, fh - 40)
        self._draw_angle_bar(frame, "TILT", cur_tilt, TILT_MIN, TILT_MAX, 8, fh - 24)

        # ── Crosshair ─────────────────────────────────────────
        arm = 20
        yc  = (255, 255, 0)
        cv2.line(frame, pt(cx_f - arm, cy_f), pt(cx_f + arm, cy_f), yc, 1, cv2.LINE_AA)
        cv2.line(frame, pt(cx_f, cy_f - arm), pt(cx_f, cy_f + arm), yc, 1, cv2.LINE_AA)
        cv2.circle(frame, pt(cx_f, cy_f), 3, yc, -1, cv2.LINE_AA)

        # ── Dead-zone box ─────────────────────────────────────
        cv2.rectangle(frame,
                      pt(cx_f - DEAD_ZONE_X, cy_f - DEAD_ZONE_Y),
                      pt(cx_f + DEAD_ZONE_X, cy_f + DEAD_ZONE_Y),
                      (0, 255, 255), 1)

        # ── Face annotation ───────────────────────────────────
        if face_info is not None:
            fcx, fcy, bw, bh = face_info
            fx = fcx - bw // 2
            fy = fcy - bh // 2

            color = (0, 255, 80)  if frozen else (0, 165, 255)
            label = "LOCKED"      if frozen else "TRACKING"
            t = 18

            # Corner-tick bounding box
            segs = [
                (fx,      fy,      fx+t,    fy     ), (fx,      fy,      fx,      fy+t   ),
                (fx+bw,   fy,      fx+bw-t, fy     ), (fx+bw,   fy,      fx+bw,   fy+t   ),
                (fx,      fy+bh,   fx+t,    fy+bh  ), (fx,      fy+bh,   fx,      fy+bh-t),
                (fx+bw,   fy+bh,   fx+bw-t, fy+bh  ), (fx+bw,   fy+bh,   fx+bw,   fy+bh-t),
            ]
            for ax, ay, bx, by in segs:
                cv2.line(frame, pt(ax, ay), pt(bx, by), color, 2, cv2.LINE_AA)

            cv2.circle(frame, pt(fcx, fcy), 5, color, -1, cv2.LINE_AA)
            cv2.line(frame, pt(cx_f, cy_f), pt(fcx, fcy), (70, 70, 70), 1, cv2.LINE_AA)

            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            by0 = max(fy - 2, lh + 10)
            cv2.rectangle(frame, pt(fx, by0 - lh - 8), pt(fx + lw + 8, by0), color, -1)
            cv2.putText(frame, label, pt(fx + 4, by0 - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)

    # ─────────────────────────────────────────────────────────────
    # RUN – starts all threads, then pumps the display in main thread
    # ─────────────────────────────────────────────────────────────
    def run(self):
        threads = [
            threading.Thread(target=self._capture_thread,   name="capture",   daemon=True),
            threading.Thread(target=self._detection_thread, name="detection",  daemon=True),
            threading.Thread(target=self._servo_thread,     name="servo",      daemon=True),
        ]
        for t in threads:
            t.start()

        try:
            while not self._stop_event.is_set():
                try:
                    frame = self._display_q.get(timeout=0.5)
                except queue.Empty:
                    continue

                if SHOW_WINDOW:
                    # FPS counter for display
                    self._fps_count += 1
                    now = time.time()
                    elapsed = now - self._fps_t0
                    if elapsed >= 1.0:
                        self._fps      = self._fps_count / elapsed
                        self._fps_t0   = now
                        self._fps_count = 0

                    cv2.imshow(WINDOW_NAME, frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), ord('Q'), 27):
                        log.info("Quit key pressed.")
                        break

        except KeyboardInterrupt:
            log.info("Ctrl+C received.")
        finally:
            self.cleanup()

    # ─────────────────────────────────────────────────────────────
    # CLEANUP  –  stop threads FIRST, then home servo, then camera
    # ─────────────────────────────────────────────────────────────
    def cleanup(self):
        log.info("Shutting down ...")
        # Signal all threads to stop
        self._stop_event.set()
        time.sleep(0.3)   # let threads exit gracefully

        # Now move servo to home (servo thread is gone, safe to call directly)
        try:
            self.servo.go_home_immediate()
            log.info("Servo returned to home.")
        except Exception as exc:
            log.warning("Servo home error: %s", exc)

        # Stop camera
        try:
            self.cam.stop()
            log.info("Camera stopped.")
        except Exception as exc:
            log.warning("Camera stop error: %s", exc)

        cv2.destroyAllWindows()
        log.info("Done. Goodbye!")


# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    robot = RestaurantRobot()
    robot.run()
