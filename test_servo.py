from adafruit_servokit import ServoKit
from time import sleep

# Initialize the PCA9685 driver with 16 channels
kit = ServoKit(channels=16)

# Define which channels on the board your servos are plugged into
PAN_CHANNEL = 0
TILT_CHANNEL = 1

def center_servos():
    """Moves both the pan and tilt servos to their center (90 degree) position."""
    print("Centering camera...")
    kit.servo[PAN_CHANNEL].angle = 90
    kit.servo[TILT_CHANNEL].angle = 90
    sleep(1)

def sweep_pan():
    """Sweeps the pan (left/right) servo back and forth."""
    print("Sweeping Pan...")
    # Sweep right
    for a in range(0, 180):
        kit.servo[PAN_CHANNEL].angle = a
        sleep(0.008)
    # Sweep left
    for a in range(179, -1, -1):
        kit.servo[PAN_CHANNEL].angle = a
        sleep(0.008)

def sweep_tilt():
    """Sweeps the tilt (up/down) servo back and forth."""
    print("Sweeping Tilt...")
    # Limit tilt range (e.g., 45 to 135) to prevent the mechanism from hitting itself
    for a in range(45, 135):
        kit.servo[TILT_CHANNEL].angle = a
        sleep(0.008)
    for a in range(134, 44, -1):
        kit.servo[TILT_CHANNEL].angle = a
        sleep(0.008)

# --- Main Program Loop ---
try:
    center_servos()
    
    while True:
        sweep_pan()
        sleep(0.5)
        
        sweep_tilt()
        sleep(0.5)

except KeyboardInterrupt:
    # This safely stops the program and centers the servos if you press Ctrl+C
    print("\nProgram stopped by user.")
    center_servos()