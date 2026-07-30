#!/usr/bin/env python3
"""
===============================================================
  Restaurant Robot – Pan/Tilt Face Tracker  v5  (fixed)
  Hardware : Raspberry Pi 4B
             Raspberry Pi Camera Module 3  (picamera2 / libcamera)
             PCA9685 PWM driver  (I2C 0x40)
             2 × servos  –  pan=ch0  tilt=ch1
  Display  : 5-inch screen
===============================================================
  Changes from v5
  ────────────────
  • Fixed pan overshoot (left and right)
      - _compute_pan_target now anchors delta to cur_pan (actual
        servo position) instead of tgt_pan.  The old code accumulated
        delta onto tgt_pan every detection frame, so the target raced
        far ahead while the servo was still moving — classic windup.
      - Removed the erroneous sign negation that was justified by
        "mirrored display" — display flip has no bearing on real-world
        servo direction.
      - Reduced KP_PAN, MAX_TARGET_STEP_PAN and widened DEAD_ZONE_X
        for gentler, more stable tracking.
===============================================================
"""

import cv2
import os
import time
import threading
import queue
import numpy as np
from picamera2 import Picamera2
from adafruit_servokit import ServoKit
import logging
import argparse

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
# TUNABLE SETTINGS
# ═══════════════════════════════════════════════════════════════

# Camera
FRAME_W    = 640
FRAME_H    = 480
FRAME_RATE = 30

# Servo channels on PCA9685
PAN_CHANNEL  = 0
TILT_CHANNEL = 1

# Servo angle limits (degrees)
PAN_MIN  = 70;   PAN_MAX  = 120
TILT_MIN = 60;   TILT_MAX = 120

# Home / natural positions (degrees)
PAN_HOME     = 108    # centre pan
TILT_NATURAL = 110    # natural slightly-upward gaze (held during tracking)
TILT_HOME    = TILT_NATURAL   # home = same natural angle

# Nod settings  ← oscillate between up and down (limited by servo range)
TILT_NOD_UP     = TILT_NATURAL - 16   # degrees UP from natural (- = up, clamped by TILT_MIN)
TILT_NOD_DOWN   = TILT_NATURAL + 10   # degrees DOWN from natural (+ = down, clamped by TILT_MAX)
NOD_COUNT       = 2                   # number of nod cycles
NOD_ALPHA       = 0.45                # blend speed for nod movement (higher = faster)
NOD_HOLD_SEC    = 0.03                # seconds to hold at each extreme
NOD_SETTLE_SEC  = 0.00                # seconds to hold at center between nods

# Look-around settings  ← slow left/right glance, done once before nodding
LOOK_PAN_OFFSET = 20                  # degrees left/right from home to look (clamped by PAN_MIN/MAX)
LOOK_ALPHA      = 0.06                # blend speed for look movement (low = slow, deliberate)
LOOK_HOLD_SEC   = 0.35                # seconds to hold gaze at each side before moving on

# Dead-zone – pixels from frame centre where we do NOT move
DEAD_ZONE_X = 55    # widened from 45 — less twitchy near centre
DEAD_ZONE_Y = 999   # vertical   — effectively infinite (tilt not tracking)

# Proportional gain  (degrees per pixel of error)
KP_PAN  = 0.030   # faster response to face displacement
KP_TILT = 0.0     # tilt gain disabled (horizontal tracking only)

# Maximum servo target change per detection frame (degrees)
MAX_TARGET_STEP_PAN  = 4.0   # allow larger per-frame changes for faster tracking
MAX_TARGET_STEP_TILT = 0.0   # disabled

# Servo thread smoothing
# Lower SERVO_ALPHA = smoother / slower blending toward target
SERVO_ALPHA = 0.12   # slightly faster servo motion toward the target

# Alpha used when returning home (even slower = graceful glide)
HOME_ALPHA = 0.05

# Threshold (degrees) at which smooth-home considers itself "arrived"
HOME_THRESHOLD = 0.5

# Face position smoothing
FACE_POS_ALPHA = 0.40

# Face confirmation / loss
CONFIRM_FRAMES = 3
LOSE_FRAMES    = 8

# Timing
LOST_FACE_TIMEOUT = 2.5
SEARCH_STEP_DELAY = 1.0
STABLE_TIME       = 1.5

# Servo update rate (Hz)
SERVO_HZ = 25

# Display
SHOW_WINDOW    = True
WINDOW_NAME    = "AURA Face Tracker"
WIN_W          = 640
WIN_H          = 480
MIRROR_DISPLAY = True


# ═══════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════

def pt(x, y):
    return (int(round(x)), int(round(y)))

def clamp(v, lo, hi):
    return max(float(lo), min(float(hi), float(v)))

def _cv2_data(filename):
    try:
        return os.path.join(os.path.dirname(cv2.__file__), "data", filename)
    except Exception:
        return filename

CASCADE_PATHS = [
    "/home/aura/Desktop/py/cascades/haarcascade_frontalface_default.xml",
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
# SERVO CONTROLLER
# ═══════════════════════════════════════════════════════════════

class PanTiltController:
    """
    Thread-safe pan/tilt angle store.

    current = the actual servo position (updated by smooth_step).
    target  = where we want to be (set by detection or nod).

    Tilt is normally locked at TILT_NATURAL during tracking.
    Only the nod routine and go_home_smooth() change tilt.
    """

    def __init__(self):
        log.info("Initialising PCA9685 ...")
        self.kit   = ServoKit(channels=16)
        self._lock = threading.Lock()

        self._cur_pan  = float(PAN_HOME)
        self._cur_tilt = float(TILT_HOME)
        self._tgt_pan  = float(PAN_HOME)
        self._tgt_tilt = float(TILT_HOME)

        self._apply(self._cur_pan, self._cur_tilt)
        log.info("Servos at home  pan=%.1f  tilt=%.1f", self._cur_pan, self._cur_tilt)

    def _apply(self, pan, tilt):
        self.kit.servo[PAN_CHANNEL ].angle = int(round(clamp(pan,  PAN_MIN,  PAN_MAX)))
        self.kit.servo[TILT_CHANNEL].angle = int(round(clamp(tilt, TILT_MIN, TILT_MAX)))

    def set_target(self, pan, tilt):
        with self._lock:
            self._tgt_pan  = clamp(pan,  PAN_MIN,  PAN_MAX)
            self._tgt_tilt = clamp(tilt, TILT_MIN, TILT_MAX)

    def set_pan_target(self, pan):
        """Set only pan target — tilt is left unchanged."""
        with self._lock:
            self._tgt_pan = clamp(pan, PAN_MIN, PAN_MAX)

    def set_tilt_target(self, tilt):
        """Set only tilt target — used by nod and home routines."""
        with self._lock:
            self._tgt_tilt = clamp(tilt, TILT_MIN, TILT_MAX)

    def smooth_step(self, alpha=None):
        """
        Blend current → target by alpha.
        Returns (cur_pan, cur_tilt).
        """
        a = SERVO_ALPHA if alpha is None else alpha
        with self._lock:
            self._cur_pan  = a * self._tgt_pan  + (1.0 - a) * self._cur_pan
            self._cur_tilt = a * self._tgt_tilt + (1.0 - a) * self._cur_tilt
            self._apply(self._cur_pan, self._cur_tilt)
            return self._cur_pan, self._cur_tilt

    def go_home_smooth(self):
        """
        Blocking call — smoothly moves pan AND tilt back to home.
        Runs its own tick loop at SERVO_HZ until close enough.
        Call this AFTER the servo thread has stopped.
        """
        log.info("Smooth home: pan → %.0f°  tilt → %.0f°", PAN_HOME, TILT_HOME)
        self.set_target(PAN_HOME, TILT_HOME)
        interval = 1.0 / SERVO_HZ
        while True:
            pan, tilt = self.smooth_step(alpha=HOME_ALPHA)
            if abs(pan - PAN_HOME) < HOME_THRESHOLD and abs(tilt - TILT_HOME) < HOME_THRESHOLD:
                # Snap to exact home
                with self._lock:
                    self._cur_pan  = float(PAN_HOME)
                    self._cur_tilt = float(TILT_HOME)
                    self._apply(PAN_HOME, TILT_HOME)
                break
            time.sleep(interval)
        log.info("Smooth home complete.")

    def go_home_immediate(self):
        with self._lock:
            self._tgt_pan  = float(PAN_HOME)
            self._tgt_tilt = float(TILT_HOME)
            self._cur_pan  = float(PAN_HOME)
            self._cur_tilt = float(TILT_HOME)
            self._apply(PAN_HOME, TILT_HOME)

    def look_around(self):
        """
        Slowly glance left then right (blocking), then return to home pan.
        Tilt is left unchanged. Call this before nod() so the robot visibly
        checks both sides first, then acknowledges with a nod.
        """
        # Clamp here too (not just inside set_pan_target) so the convergence
        # check below compares against the angle the servo can actually
        # reach - otherwise an out-of-range target (e.g. PAN_HOME +
        # LOOK_PAN_OFFSET past PAN_MAX) gets silently clamped in
        # set_pan_target while this loop keeps waiting for the unclamped
        # value, which the servo can never reach, and hangs forever.
        left_target  = clamp(PAN_HOME - LOOK_PAN_OFFSET, PAN_MIN, PAN_MAX)
        right_target = clamp(PAN_HOME + LOOK_PAN_OFFSET, PAN_MIN, PAN_MAX)
        log.info("Looking around (pan: %.0f° ↔ %.0f°) ...", left_target, right_target)
        interval  = 1.0 / SERVO_HZ
        threshold = 0.8

        for pan_target in (left_target, right_target, PAN_HOME):
            self.set_pan_target(pan_target)
            while True:
                cur_pan, _ = self.smooth_step(alpha=LOOK_ALPHA)
                if abs(cur_pan - pan_target) < threshold:
                    break
                time.sleep(interval)
            time.sleep(LOOK_HOLD_SEC)

        # Snap to exact home pan
        with self._lock:
            self._cur_pan = float(PAN_HOME)
            self._tgt_pan = float(PAN_HOME)
            self._apply(self._cur_pan, self._cur_tilt)

        log.info("Look-around complete.")

    def nod(self):
        """
        Perform NOD_COUNT expressive nods (blocking).
        Oscillates between up and down; pan stays at current position.
        Call this before the servo thread starts.
        """
        log.info("Nodding %d times (tilt: %.0f° ↔ %.0f°) ...",
                 NOD_COUNT, TILT_NOD_DOWN, TILT_NOD_UP)
        interval = 1.0 / SERVO_HZ

        for i in range(NOD_COUNT):
            # ── Tilt up ───────────────────────────────────────
            self.set_tilt_target(TILT_NOD_UP)
            threshold = 0.8
            while True:
                _, cur_tilt = self.smooth_step(alpha=NOD_ALPHA)
                if abs(cur_tilt - TILT_NOD_UP) < threshold:
                    break
                time.sleep(interval)
            time.sleep(NOD_HOLD_SEC)

            # ── Tilt down ───────────────────────────────────────
            self.set_tilt_target(TILT_NOD_DOWN)
            while True:
                _, cur_tilt = self.smooth_step(alpha=NOD_ALPHA)
                if abs(cur_tilt - TILT_NOD_DOWN) < threshold:
                    break
                time.sleep(interval)
            time.sleep(NOD_HOLD_SEC)

            # ── Return to centre ──────────────────────────────
            self.set_tilt_target(TILT_NATURAL)
            while True:
                _, cur_tilt = self.smooth_step(alpha=NOD_ALPHA)
                if abs(cur_tilt - TILT_NATURAL) < threshold:
                    break
                time.sleep(interval)

            if i < NOD_COUNT - 1:
                time.sleep(NOD_SETTLE_SEC)

        # Snap to exact natural tilt after nodding
        with self._lock:
            self._cur_tilt = float(TILT_NATURAL)
            self._tgt_tilt = float(TILT_NATURAL)
            self._apply(self._cur_pan, self._cur_tilt)

        log.info("Nod complete.")

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
# FACE DETECTOR
# ═══════════════════════════════════════════════════════════════

class FaceDetector:

    def __init__(self, cascade):
        self.cascade        = cascade
        self._confirm_count = 0
        self._lose_count    = 0
        self._confirmed     = False
        self._sx = float(FRAME_W // 2)
        self._sy = float(FRAME_H // 2)
        self._sw = 120.0
        self._sh = 120.0

    def update(self, gray_frame):
        faces = self.cascade.detectMultiScale(
            gray_frame,
            scaleFactor  = 1.08,
            minNeighbors = 7,
            minSize      = (90, 90),
            flags        = cv2.CASCADE_SCALE_IMAGE,
        )

        if len(faces) > 0:
            x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
            raw_cx = float(x + w // 2)
            raw_cy = float(y + h // 2)

            self._confirm_count += 1
            self._lose_count     = 0

            if not self._confirmed:
                if self._confirm_count >= CONFIRM_FRAMES:
                    self._confirmed = True
                    self._sx = raw_cx
                    self._sy = raw_cy
                    self._sw = float(w)
                    self._sh = float(h)
                else:
                    return None

            self._sx = FACE_POS_ALPHA * raw_cx + (1.0 - FACE_POS_ALPHA) * self._sx
            self._sy = FACE_POS_ALPHA * raw_cy + (1.0 - FACE_POS_ALPHA) * self._sy
            self._sw = FACE_POS_ALPHA * w      + (1.0 - FACE_POS_ALPHA) * self._sw
            self._sh = FACE_POS_ALPHA * h      + (1.0 - FACE_POS_ALPHA) * self._sh

            return (int(self._sx), int(self._sy), int(self._sw), int(self._sh))

        else:
            self._confirm_count = 0
            self._lose_count   += 1
            if self._lose_count >= LOSE_FRAMES:
                self._confirmed = False
            if self._confirmed:
                return (int(self._sx), int(self._sy), int(self._sw), int(self._sh))
            return None


# ═══════════════════════════════════════════════════════════════
# MAIN APPLICATION
# ═══════════════════════════════════════════════════════════════

class RestaurantRobot:

    def __init__(self, max_run_seconds=None, auto_stop_on_lock=False,
                 minimize_window=False, home_on_exit=True):
        self._stop_event      = threading.Event()
        self.start_time       = time.time()
        self.max_run_seconds  = None if max_run_seconds is None else float(max_run_seconds)
        self.auto_stop_on_lock = bool(auto_stop_on_lock)
        self.minimize_window  = bool(minimize_window)
        self.home_on_exit     = bool(home_on_exit)

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

        # ── Startup look-around, then nod (before tracking begins) ──
        self.servo.look_around()
        self.servo.nod()

        # ── Lock tilt at natural angle for the rest of runtime ───
        self.servo.set_tilt_target(TILT_NATURAL)

        # ── Detector ─────────────────────────────────────────────
        cascade       = load_cascade()
        self.detector = FaceDetector(cascade)

        # ── Shared state ─────────────────────────────────────────
        self._lock          = threading.Lock()
        self.state          = "TRACKING"
        self.last_face_time = time.time()
        self.stable_since   = None
        self.frozen         = False
        self._current_face  = None

        self._search_angles = [
            (PAN_HOME - 35, TILT_NATURAL),
            (PAN_HOME,      TILT_NATURAL),
            (PAN_HOME + 35, TILT_NATURAL),
            (PAN_HOME,      TILT_NATURAL),
        ]
        self._search_idx = 0

        self._fps         = 0.0
        self._fps_t0      = time.time()
        self._fps_count   = 0
        self._det_fps     = 0.0
        self._det_fps_t0  = time.time()
        self._det_fps_cnt = 0

        self._frame_q   = queue.Queue(maxsize=1)
        self._display_q = queue.Queue(maxsize=1)

        # ── Window ───────────────────────────────────────────────
        if SHOW_WINDOW:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, WIN_W, WIN_H)
            cv2.moveWindow(WINDOW_NAME, 0, 0)
            try:
                if self.minimize_window:
                    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE, 0)
            except Exception:
                pass

        log.info("All systems ready.  Press Q / Esc to quit.")

    # ─────────────────────────────────────────────────────────────
    # THREAD 1 – capture
    # ─────────────────────────────────────────────────────────────
    def _capture_thread(self):
        while not self._stop_event.is_set():
            try:
                frame = self.cam.capture_array()
                if frame is None or frame.size == 0:
                    continue
                try:
                    self._frame_q.get_nowait()
                except queue.Empty:
                    pass
                self._frame_q.put(frame)
            except Exception as exc:
                if not self._stop_event.is_set():
                    log.warning("Capture error: %s", exc)

    # ─────────────────────────────────────────────────────────────
    # THREAD 2 – detection  (pan only — tilt fixed at TILT_NATURAL)
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

            self._det_fps_cnt += 1
            det_elapsed = now - self._det_fps_t0
            if det_elapsed >= 1.0:
                self._det_fps     = self._det_fps_cnt / det_elapsed
                self._det_fps_t0  = now
                self._det_fps_cnt = 0

            if face is not None:
                cx, cy, fw, fh = face
                with self._lock:
                    self.last_face_time = now
                    self.state          = "TRACKING"
                    self._search_idx    = 0
                    self._current_face  = face

                if self.frozen:
                    if abs(cx - FRAME_W // 2) > DEAD_ZONE_X * 2:
                        with self._lock:
                            self.frozen       = False
                            self.stable_since = None
                else:
                    centred = self._compute_pan_target(cx)
                    with self._lock:
                        if centred:
                            if self.stable_since is None:
                                self.stable_since = now
                            elif now - self.stable_since >= STABLE_TIME:
                                self.frozen = True
                                if self.auto_stop_on_lock:
                                    log.info("Auto-stop on lock triggered.")
                                    self._stop_event.set()
                        else:
                            self.stable_since = None
            else:
                with self._lock:
                    self.frozen        = False
                    self.stable_since  = None
                    self._current_face = None
                    elapsed = now - self.last_face_time
                self.servo.set_pan_target(PAN_HOME)
                if elapsed > LOST_FACE_TIMEOUT:
                    with self._lock:
                        self.state = "SEARCHING"
                    if now - last_search_step > SEARCH_STEP_DELAY:
                        self._do_search_step()
                        last_search_step = now

            # Build display frame
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

            try:
                self._display_q.get_nowait()
            except queue.Empty:
                pass
            self._display_q.put(display)

    # ─────────────────────────────────────────────────────────────
    # THREAD 3 – servo
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
    # CONTROL LOGIC  –  pan only, tilt locked to TILT_NATURAL
    # ─────────────────────────────────────────────────────────────
    def _compute_pan_target(self, cx):
        """
        Update pan target from horizontal face error.
        Tilt target is NOT touched — stays at TILT_NATURAL.
        Returns True if face is inside horizontal dead-zone.

        FIX (v5 → v5-fixed):
          Delta is now added to cur_pan (actual servo position) rather
          than tgt_pan.  The old approach accumulated onto the target
          every detection frame while the servo was still catching up,
          causing the target to race far ahead — windup overshoot.

          Sign correction: KP_PAN * err_x (no negation).
            err_x > 0 → face right of centre → pan increases (turns right).
            err_x < 0 → face left of centre  → pan decreases (turns left).
          If your servo mount is reversed, flip KP_PAN to negative.
        """
        err_x = cx - FRAME_W // 2   # + = face right of centre in raw image

        if abs(err_x) <= DEAD_ZONE_X:
            return True

        # Pan: face right → decrease pan (turn right); face left → increase pan (turn left)
        delta_pan   = clamp(-KP_PAN * err_x, -MAX_TARGET_STEP_PAN, MAX_TARGET_STEP_PAN)
        new_tgt_pan = clamp(self.servo.cur_pan + delta_pan, PAN_MIN, PAN_MAX)
        self.servo.set_pan_target(new_tgt_pan)
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
        cv2.rectangle(frame, pt(x, y),          pt(x + bw,     y + bh), (40, 40, 40),   -1)
        cv2.rectangle(frame, pt(x, y),          pt(x + filled, y + bh), (0, 200, 255),  -1)
        cv2.rectangle(frame, pt(x, y),          pt(x + bw,     y + bh), (100, 100, 100), 1)
        cv2.putText(frame, "%s %5.1f" % (label, value),
                    pt(x + bw + 6, y + bh - 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)

    def _draw_overlay(self, frame, face_info):
        fh, fw = frame.shape[:2]
        cx_f, cy_f = fw // 2, fh // 2

        cur_pan  = self.servo.cur_pan
        cur_tilt = self.servo.cur_tilt

        overlay = frame.copy()
        cv2.rectangle(overlay, pt(0, 0), pt(fw, 32), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        with self._lock:
            state   = self.state
            det_fps = self._det_fps
            frozen  = self.frozen

        state_col = (0, 220, 80) if state == "TRACKING" else (0, 160, 255)

        cv2.putText(
            frame,
            "AURA  |  %-10s|  %.1f det-fps  |  Pan %.0f  Tilt %.0f (fixed)" % (
                state, det_fps, cur_pan, cur_tilt),
            pt(8, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.46, state_col, 1, cv2.LINE_AA)

        self._draw_angle_bar(frame, "PAN ", cur_pan,  PAN_MIN,  PAN_MAX,  8, fh - 40)
        self._draw_angle_bar(frame, "TILT", cur_tilt, TILT_MIN, TILT_MAX, 8, fh - 24)

        arm = 20
        yc  = (255, 255, 0)
        cv2.line(frame, pt(cx_f - arm, cy_f), pt(cx_f + arm, cy_f), yc, 1, cv2.LINE_AA)
        cv2.line(frame, pt(cx_f, cy_f - arm), pt(cx_f, cy_f + arm), yc, 1, cv2.LINE_AA)
        cv2.circle(frame, pt(cx_f, cy_f), 3, yc, -1, cv2.LINE_AA)

        cv2.line(frame,
                 pt(cx_f - DEAD_ZONE_X, 0),
                 pt(cx_f - DEAD_ZONE_X, fh),
                 (0, 255, 255), 1)
        cv2.line(frame,
                 pt(cx_f + DEAD_ZONE_X, 0),
                 pt(cx_f + DEAD_ZONE_X, fh),
                 (0, 255, 255), 1)

        if face_info is not None:
            fcx, fcy, bw, bh = face_info
            fx = fcx - bw // 2
            fy = fcy - bh // 2

            color = (0, 255, 80)  if frozen else (0, 165, 255)
            label = "LOCKED"      if frozen else "TRACKING"
            t = 18

            segs = [
                (fx,    fy,    fx+t,  fy   ), (fx,    fy,    fx,    fy+t ),
                (fx+bw, fy,    fx+bw-t, fy ), (fx+bw, fy,    fx+bw, fy+t ),
                (fx,    fy+bh, fx+t,  fy+bh), (fx,    fy+bh, fx,    fy+bh-t),
                (fx+bw, fy+bh, fx+bw-t, fy+bh), (fx+bw, fy+bh, fx+bw, fy+bh-t),
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
    # RUN
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

                if self.max_run_seconds is not None:
                    if (time.time() - self.start_time) >= self.max_run_seconds:
                        log.info("Max run time reached: %.1f s", self.max_run_seconds)
                        break

                if SHOW_WINDOW:
                    self._fps_count += 1
                    now = time.time()
                    elapsed = now - self._fps_t0
                    if elapsed >= 1.0:
                        self._fps       = self._fps_count / elapsed
                        self._fps_t0    = now
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
    # CLEANUP  –  stop threads, smooth home, stop camera
    # ─────────────────────────────────────────────────────────────
    def cleanup(self):
        log.info("Shutting down ...")
        self._stop_event.set()
        time.sleep(0.35)

        if self.home_on_exit:
            self.servo.go_home_smooth()
        else:
            log.info("Leaving servos at current position (home_on_exit=False).")

        try:
            self.cam.stop()
            log.info("Camera stopped.")
        except Exception as exc:
            log.warning("Camera stop error: %s", exc)

        cv2.destroyAllWindows()
        log.info("Done. Goodbye!")


# ═══════════════════════════════════════════════════════════════
def _parse_args():
    p = argparse.ArgumentParser(description="AURA face tracker v5 fixed")
    p.add_argument("--max-run-seconds",   type=float, default=None)
    p.add_argument("--auto-stop-on-lock", action="store_true")
    p.add_argument("--minimize-window",   action="store_true")
    p.add_argument("--no-home-on-exit",   action="store_true",
                   help="skip smooth home on exit (leave at last position)")
    return p.parse_args()


if __name__ == "__main__":
    args  = _parse_args()
    robot = RestaurantRobot(
        max_run_seconds   = args.max_run_seconds,
        auto_stop_on_lock = args.auto_stop_on_lock,
        minimize_window   = args.minimize_window,
        home_on_exit      = not args.no_home_on_exit,
    )
    robot.run()