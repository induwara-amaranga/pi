import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
servo = kit.servo[15]
servo.actuation_range = 180

def move_slowly(start, end, step=1, delay=0.02):
    if start < end:
        angles = range(start, end + 1, step)
    else:
        angles = range(start, end - 1, -step)
    for angle in angles:
        servo.angle = angle
        time.sleep(delay)

def nod():
    """90 -> 120 -> 90"""
    move_slowly(90, 120)
    time.sleep(0.5)
    move_slowly(120, 90)

if __name__ == "__main__":
    nod()