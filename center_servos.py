#!/usr/bin/env python3
from adafruit_servokit import ServoKit
import time

PAN_CHANNEL  = 0
TILT_CHANNEL = 1

PAN_HOME  = 108
TILT_HOME = 110

PAN_MIN  = 30
PAN_MAX  = 170
TILT_MIN = 60
TILT_MAX = 160

TURN_STEP  = 30    # degrees per left/right/up/down move
TURN_DELAY = 0.5   # seconds to hold each position

kit = ServoKit(channels=16)


def set_pan(angle):
    angle = max(PAN_MIN, min(PAN_MAX, angle))
    kit.servo[PAN_CHANNEL].angle = angle
    return angle


def set_tilt(angle):
    angle = max(TILT_MIN, min(TILT_MAX, angle))
    kit.servo[TILT_CHANNEL].angle = angle
    return angle


def go_home():
    print(f"Moving pan  -> {PAN_HOME}°")
    set_pan(PAN_HOME)
    time.sleep(0.5)
    print(f"Moving tilt -> {TILT_HOME}°")
    set_tilt(TILT_HOME)
    time.sleep(0.5)
    print("Servos at home position.")


def turn_left(steps=1):
    target = PAN_HOME + (TURN_STEP * steps)   # higher angle = left
    actual = set_pan(target)
    print(f"Turn LEFT  -> pan={actual}°")
    time.sleep(TURN_DELAY)


def turn_right(steps=1):
    target = PAN_HOME - (TURN_STEP * steps)   # lower angle = right
    actual = set_pan(target)
    print(f"Turn RIGHT -> pan={actual}°")
    time.sleep(TURN_DELAY)


def look_up(steps=1):
    target = TILT_HOME + (TURN_STEP * steps)  # lower angle = up
    actual = set_tilt(target)
    print(f"Look UP    -> tilt={actual}°")
    time.sleep(TURN_DELAY)


def look_down(steps=1):
    target = TILT_HOME - (TURN_STEP * steps)  # higher angle = down
    actual = set_tilt(target)
    print(f"Look DOWN  -> tilt={actual}°")
    time.sleep(TURN_DELAY)


if __name__ == "__main__":
    go_home()

    print("\n--- Turning left ---")
    turn_left()

    print("\n--- Returning home ---")
    go_home()

    print("\n--- Turning right ---")
    turn_right()

    print("\n--- Returning home ---")
    go_home()

    print("\n--- Looking up ---")
    look_up()

    print("\n--- Returning home ---")
    go_home()

    print("\n--- Looking down ---")
    look_down()

    print("\n--- Returning home ---")
    go_home()

    print("\nDone.")