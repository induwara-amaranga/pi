"""
Pan/tilt servo controller for the PCA9685-driven camera head.

Shared between robot_face_tracker.py (smooth target-following while
tracking a face) and main_controller.py (the look-around + nod startup
gesture, run once before a face tracker cycle starts).
"""

import json
import logging
import os
import threading
import time

from adafruit_servokit import ServoKit

log = logging.getLogger(__name__)

# Servo channels on PCA9685
PAN_CHANNEL  = 0
TILT_CHANNEL = 1

# Servo angle limits (degrees)
PAN_MIN  = 70;   PAN_MAX  = 120
TILT_MIN = 60;   TILT_MAX = 120

# Home / natural positions (degrees)
PAN_HOME     = 100   # centre pan
TILT_NATURAL = 110    # natural slightly-upward gaze (held during tracking)
TILT_HOME    = TILT_NATURAL   # home = same natural angle

# Nod settings  ← oscillate between up and down (limited by servo range)
# TILT_NOD_DOWN is already pinned at TILT_MAX (110+10=120) so it can't grow
# any further without exceeding the servo's safe tilt range - TILT_NOD_UP
# has plenty of headroom to TILT_MIN (60) though, so that side is widened
# to make the nod read as a clear, deliberate gesture instead of a twitch.
TILT_NOD_UP     = TILT_NATURAL - 30   # degrees UP from natural (- = up, clamped by TILT_MIN)
TILT_NOD_DOWN   = TILT_NATURAL + 10   # degrees DOWN from natural (+ = down, clamped by TILT_MAX)
NOD_COUNT       = 2                   # number of nod cycles
NOD_ALPHA       = 0.65                # blend speed for nod movement (higher = faster)
NOD_HOLD_SEC    = 0.03                # seconds to hold at each extreme (was 0.03 - too brief to register)
NOD_SETTLE_SEC  = 0.08                # seconds to hold at center between nods (was 0 - reps blurred together)

# Look-around settings  ← slow left/right glance, done once before nodding.
# PAN_HOME (108) sits close to PAN_MAX (120), so a single symmetric offset
# clamps hard on the right (108+20=128 -> clamped to 120, only a 12°
# glance) while leaving most of the left-side headroom (108-70=38° max)
# unused. Separate offsets use each side's real travel so both glances are
# actually visible instead of the right one looking like it barely moved.
LOOK_PAN_LEFT_OFFSET  = 32            # degrees left from home (clamped by PAN_MIN)
LOOK_PAN_RIGHT_OFFSET = 12            # degrees right from home (clamped by PAN_MAX)
LOOK_ALPHA      = 0.16                # blend speed for look movement (low = slow, deliberate)
LOOK_HOLD_SEC   = 0.1                 # seconds to hold gaze at each side before moving on

# Servo thread smoothing
# Lower SERVO_ALPHA = smoother / slower blending toward target
SERVO_ALPHA = 0.12   # slightly faster servo motion toward the target

# Alpha used when returning home (even slower = graceful glide)
HOME_ALPHA = 0.05

# Threshold (degrees) at which smooth-home considers itself "arrived"
HOME_THRESHOLD = 0.5

# Servo update rate (Hz)
SERVO_HZ = 25

# Where the last-known pan/tilt angle is persisted between process runs.
# The PCA9685 keeps outputting its last PWM signal even after the Python
# process that set it exits (or is killed, or the Pi loses power) - so on
# the next run, this file is usually a far better guess of the servo's true
# physical position than assuming it's already at PAN_HOME/TILT_HOME. That
# lets __init__ glide home smoothly from wherever it really is instead of
# snapping straight to home.
STATE_FILE          = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".pan_tilt_last_position.json")
STATE_SAVE_INTERVAL = 0.5   # seconds - throttles disk writes during continuous tracking


def clamp(v, lo, hi):
    return max(float(lo), min(float(hi), float(v)))


def _load_last_position():
    try:
        with open(STATE_FILE, "r") as f:
            data = json.load(f)
        return clamp(data["pan"], PAN_MIN, PAN_MAX), clamp(data["tilt"], TILT_MIN, TILT_MAX)
    except Exception:
        return None


def _save_last_position(pan, tilt):
    try:
        with open(STATE_FILE, "w") as f:
            json.dump({"pan": pan, "tilt": tilt}, f)
    except Exception:
        pass


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
        self._last_save_t = 0.0

        last = _load_last_position()
        if last is not None:
            self._cur_pan, self._cur_tilt = last
            log.info("Resuming from last known position  pan=%.1f  tilt=%.1f", *last)
        else:
            self._cur_pan  = float(PAN_HOME)
            self._cur_tilt = float(TILT_HOME)

        self._tgt_pan  = float(PAN_HOME)
        self._tgt_tilt = float(TILT_HOME)

        # Glide to home instead of snapping - the servo may physically be
        # wherever a previous process (or a power cycle) left it, and the
        # first PWM write below is what actually moves it, so it needs to
        # start from the smooth-step loop, not a direct _apply().
        self.go_home_smooth()

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
            cur_pan, cur_tilt = self._cur_pan, self._cur_tilt

        # Throttled so continuous tracking doesn't hammer the SD card - a
        # slightly stale position on a hard crash is an acceptable trade for
        # not writing to disk at SERVO_HZ.
        now = time.time()
        if now - self._last_save_t >= STATE_SAVE_INTERVAL:
            self._last_save_t = now
            _save_last_position(cur_pan, cur_tilt)

        return cur_pan, cur_tilt

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
        _save_last_position(PAN_HOME, TILT_HOME)
        log.info("Smooth home complete.")

    def go_home_immediate(self):
        with self._lock:
            self._tgt_pan  = float(PAN_HOME)
            self._tgt_tilt = float(TILT_HOME)
            self._cur_pan  = float(PAN_HOME)
            self._cur_tilt = float(TILT_HOME)
            self._apply(PAN_HOME, TILT_HOME)
        _save_last_position(PAN_HOME, TILT_HOME)

    def look_around(self):
        """
        Slowly glance left then right (blocking), then return to home pan.
        Tilt is left unchanged. Call this before nod() so the robot visibly
        checks both sides first, then acknowledges with a nod.
        """
        # Clamp here too (not just inside set_pan_target) so the convergence
        # check below compares against the angle the servo can actually
        # reach - otherwise an out-of-range target (e.g. PAN_HOME +
        # LOOK_PAN_RIGHT_OFFSET past PAN_MAX) gets silently clamped in
        # set_pan_target while this loop keeps waiting for the unclamped
        # value, which the servo can never reach, and hangs forever.
        left_target  = clamp(PAN_HOME - LOOK_PAN_LEFT_OFFSET,  PAN_MIN, PAN_MAX)
        right_target = clamp(PAN_HOME + LOOK_PAN_RIGHT_OFFSET, PAN_MIN, PAN_MAX)
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
            cur_tilt = self._cur_tilt

        _save_last_position(PAN_HOME, cur_tilt)
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
            cur_pan = self._cur_pan

        _save_last_position(cur_pan, TILT_NATURAL)
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


if __name__ == "__main__":
    # Run standalone (as a subprocess) so importing this module - and the
    # adafruit_servokit/Blinka stack it pulls in - never shares a process
    # with code that calls RPi.GPIO.setmode(GPIO.BOARD) (e.g.
    # main_controller.py's touch/stepper setup). Blinka's board detection
    # forces RPi.GPIO into BCM mode as a side effect of import, and RPi.GPIO
    # allows only one numbering mode per process, so mixing the two in the
    # same process raises "A different mode has already been set!".
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  %(levelname)-8s  %(message)s",
        datefmt="%H:%M:%S",
    )
    controller = PanTiltController()

    # Hand nods alongside the head gesture. Runs on its own thread, sharing
    # controller.kit (the same ServoKit/I2C connection already open for the
    # head) rather than opening a second one - PCA9685 channel writes go
    # through Blinka's I2CDevice lock, so concurrent writes to different
    # channels (hand=15, pan=0, tilt=1) from two threads on one ServoKit
    # serialize safely instead of racing on the bus.
    import hand_nod
    hand_thread = threading.Thread(target=hand_nod.nod, kwargs={"kit": controller.kit}, daemon=True)
    hand_thread.start()

    controller.look_around()
    controller.nod()

    hand_thread.join(timeout=5.0)
