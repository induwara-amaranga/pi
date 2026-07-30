import RPi.GPIO as GPIO
import time

# Pin definitions (BOARD numbering)
IN1 = 16
IN2 = 29
IN3 = 18
IN4 = 15

# Gear ratio
ORBITAL_DIAMETER = 160
INNER_DIAMETER   = 46
GEAR_RATIO       = ORBITAL_DIAMETER / INNER_DIAMETER  # 3.478
STEPS_PER_REV    = int(round(4096 * GEAR_RATIO))      # 14243 steps for 360°

# Half-step sequence
STEP_SEQUENCE = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

PINS = [IN1, IN2, IN3, IN4]
step_index = 0

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    for pin in PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

def set_step(sequence):
    for pin, val in zip(PINS, sequence):
        GPIO.output(pin, val)

def stop_motor():
    for pin in PINS:
        GPIO.output(pin, GPIO.LOW)

def rotate(num_steps, cw=True, delay=0.003):
    global step_index
    for _ in range(num_steps):
        set_step(STEP_SEQUENCE[step_index])
        step_index = (step_index + 1) % 8 if cw else (step_index - 1) % 8
        time.sleep(delay)
    stop_motor()

def main():
    setup()
    try:
        while True:
            print(f"CW: 360° ({STEPS_PER_REV} steps)")
            rotate(STEPS_PER_REV, cw=True)
            time.sleep(1)

            print(f"CCW: 360° ({STEPS_PER_REV} steps)")
            rotate(STEPS_PER_REV, cw=False)
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        stop_motor()
        GPIO.cleanup()

if __name__ == "__main__":
    main()