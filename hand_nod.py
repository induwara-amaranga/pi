import time

from adafruit_servokit import ServoKit

HAND_CHANNEL         = 15
HAND_ACTUATION_RANGE = 180
HAND_NOD_MIN         = 90
HAND_NOD_MAX         = 120
HAND_NOD_HOLD_SEC    = 0.5
HAND_NOD_STEP_DELAY  = 0.02


def _get_servo(kit=None):
    """kit is optional so this can share an already-open ServoKit (e.g. the
    one pan_tilt_controller.py already holds for the head) instead of
    opening a second, independent connection to the same PCA9685 board."""
    if kit is None:
        kit = ServoKit(channels=16)
    servo = kit.servo[HAND_CHANNEL]
    servo.actuation_range = HAND_ACTUATION_RANGE
    return servo


def move_slowly(servo, start, end, step=1, delay=HAND_NOD_STEP_DELAY):
    if start < end:
        angles = range(start, end + 1, step)
    else:
        angles = range(start, end - 1, -step)
    for angle in angles:
        servo.angle = angle
        time.sleep(delay)


def nod(kit=None):
    """HAND_NOD_MIN -> HAND_NOD_MAX -> HAND_NOD_MIN"""
    servo = _get_servo(kit)
    move_slowly(servo, HAND_NOD_MIN, HAND_NOD_MAX)
    time.sleep(HAND_NOD_HOLD_SEC)
    move_slowly(servo, HAND_NOD_MAX, HAND_NOD_MIN)


if __name__ == "__main__":
    nod()
