import os

# GPIO mode is BOARD by default so physical pin numbers can match wiring docs.
GPIO_MODE = os.getenv("GPIO_MODE", "BOARD").strip().upper()

# OLED buses: one display is on bus 4 and the other is on either bus 20 or 21.
OLED_I2C_PORTS = [
    int(port.strip())
    for port in os.getenv("OLED_I2C_PORTS", "4,1").split(",")
    if port.strip()
]
OLED_I2C_ADDRESS = int(os.getenv("OLED_I2C_ADDRESS", "0x3C"), 16)

# Touch sensor physical pins (BOARD numbering).
# Note: pin 34 is not a GPIO on Raspberry Pi headers; we keep it as requested
# and allow automatic fallback for sensor 1.
TOUCH_SENSOR_PINS = {
    1: int(os.getenv("TOUCH_SENSOR_1_PIN", "32")),
    2: int(os.getenv("TOUCH_SENSOR_2_PIN", "31")),
    3: int(os.getenv("TOUCH_SENSOR_3_PIN", "33")),
    4: int(os.getenv("TOUCH_SENSOR_4_PIN", "37")),
}
TOUCH_SENSOR_1_FALLBACK_PIN = int(os.getenv("TOUCH_SENSOR_1_FALLBACK_PIN", "32"))

# Logical sensor-to-side mapping used for OLED gaze and shortest-path rotation.
TOUCH_SENSOR_DIRECTIONS = {
    1: "right",
    2: "back",
    3: "front",
    4: "left",
}

# Required unlock sequence.
TOUCH_SEQUENCE = [1, 4, 2, 3]

# Stepper physical pins (BOARD numbering) wired to ULN2003 IN1..IN4.
STEPPER_PINS = [
    int(os.getenv("STEPPER_IN1_PIN", "16")),
    int(os.getenv("STEPPER_IN2_PIN", "29")),
    int(os.getenv("STEPPER_IN3_PIN", "18")),
    int(os.getenv("STEPPER_IN4_PIN", "15")),
]

# Pan/tilt channels on PCA9685.
PAN_CHANNEL = int(os.getenv("PAN_CHANNEL", "0"))
TILT_CHANNEL = int(os.getenv("TILT_CHANNEL", "1"))

# In stepper_module.py → _build_sensor_map()
# Default degrees should be:
default_degrees = {1: 0, 2: 90, 3: 180, 4: 270}