import os
import RPi.GPIO as GPIO

from hardware_config import (
    GPIO_MODE,
    TOUCH_SENSOR_1_FALLBACK_PIN,
    TOUCH_SENSOR_DIRECTIONS,
    TOUCH_SENSOR_PINS,
    TOUCH_SEQUENCE,
)

class TouchModule:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM if GPIO_MODE == "BCM" else GPIO.BOARD)

        self.sensor_pins = dict(TOUCH_SENSOR_PINS)
        self.sensor_directions = dict(TOUCH_SENSOR_DIRECTIONS)
        self.sequence_order = list(TOUCH_SEQUENCE)

        # Some touch boards are active-high, others are active-low.
        # Configure with env var TOUCH_ACTIVE_HIGH=1 or 0 (default: 1).
        self.active_high = os.getenv("TOUCH_ACTIVE_HIGH", "1").strip().lower() in {
            "1", "true", "yes", "y", "on"
        }
        pull_mode = GPIO.PUD_DOWN if self.active_high else GPIO.PUD_UP
        self.trigger_state = GPIO.HIGH if self.active_high else GPIO.LOW

        for sensor_id, pin in sorted(self.sensor_pins.items()):
            self.sensor_pins[sensor_id] = self._setup_touch_pin(sensor_id, pin, pull_mode)

    def _setup_touch_pin(self, sensor_id, pin, pull_mode):
        try:
            GPIO.setup(pin, GPIO.IN, pull_up_down=pull_mode)
            return pin
        except ValueError as exc:
            if sensor_id == 1 and pin != TOUCH_SENSOR_1_FALLBACK_PIN:
                fallback_pin = TOUCH_SENSOR_1_FALLBACK_PIN
                try:
                    GPIO.setup(fallback_pin, GPIO.IN, pull_up_down=pull_mode)
                    print(
                        "Touch sensor 1 pin"
                        f" {pin} is not GPIO-capable; using fallback pin {fallback_pin}."
                    )
                    return fallback_pin
                except ValueError:
                    pass
            raise ValueError(
                f"Invalid touch sensor pin for sensor {sensor_id}: {pin}"
            ) from exc

    def get_touched_sensor(self):
        for sensor_id, pin in sorted(self.sensor_pins.items()):
            if GPIO.input(pin) == self.trigger_state:
                return sensor_id
        return None

    def get_touched_direction(self):
        sensor_id = self.get_touched_sensor()
        return self.sensor_directions.get(sensor_id)

    def get_raw_sensor_states(self):
        return {
            sensor_id: GPIO.input(pin) == self.trigger_state
            for sensor_id, pin in sorted(self.sensor_pins.items())
        }

    def get_raw_states(self):
        sensor_states = self.get_raw_sensor_states()
        return {
            self.sensor_directions[sensor_id]: is_active
            for sensor_id, is_active in sensor_states.items()
        }
