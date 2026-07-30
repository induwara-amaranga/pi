import RPi.GPIO as GPIO
import time
import os

from hardware_config import GPIO_MODE, STEPPER_PINS

class StepperModule:
    HALF_STEP_SEQUENCE = [
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1],
        [1, 0, 0, 1],
    ]

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM if GPIO_MODE == "BCM" else GPIO.BOARD)

        # ULN2003 IN1..IN4 mapping.
        self.pins = list(STEPPER_PINS)

        self.step_delay = float(os.getenv("STEPPER_DELAY", "0.001"))
        self.steps_per_revolution = int(os.getenv("STEPPER_STEPS_PER_REV", "2048"))
        self.steps_per_quarter_turn = max(1, self.steps_per_revolution // 4)
        self.steps_per_degree = self.steps_per_revolution / 360.0
        self.zero_offset_deg = float(os.getenv("STEPPER_ZERO_OFFSET_DEG", "0"))
        self.zero_offset_steps = int(round(self.zero_offset_deg * self.steps_per_degree))
        self.hold_enabled = os.getenv("STEPPER_HOLD", "1").strip().lower() in {
            "1",
            "true",
            "yes",
            "y",
            "on",
        }

        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)

        self.current_step = 0
        self._sequence_index = 0

        self.sensor_position_map = self._build_sensor_map()
        self.direction_map = self._build_direction_map()

    def _build_sensor_map(self):
        default_degrees = {1: 0, 2: 90, 3: 180, 4: 270}
        override = self._parse_sensor_degrees(
            os.getenv("STEPPER_SENSOR_DEGREES", "")
        )
        degrees_map = {**default_degrees, **override}

        return {
            sensor_id: self._degrees_to_steps(deg)
            for sensor_id, deg in degrees_map.items()
        }

    def _build_direction_map(self):
        return {
            "front": self._degrees_to_steps(0),
            "right": self._degrees_to_steps(90),
            "back": self._degrees_to_steps(180),
            "left": self._degrees_to_steps(270),
        }

    def _degrees_to_steps(self, degrees):
        steps = int(round(float(degrees) * self.steps_per_degree))
        return (steps + self.zero_offset_steps) % self.steps_per_revolution

    def _parse_sensor_degrees(self, mapping):
        parsed = {}
        for chunk in mapping.split(","):
            part = chunk.strip()
            if not part or ":" not in part:
                continue
            key_str, deg_str = part.split(":", 1)
            try:
                sensor_id = int(key_str.strip())
                degrees = float(deg_str.strip())
            except ValueError:
                continue
            if sensor_id in (1, 2, 3, 4):
                parsed[sensor_id] = degrees
        return parsed

    def _move_relative_steps(self, delta_steps):
        if delta_steps == 0:
            return

        direction = 1 if delta_steps > 0 else -1
        total_steps = abs(int(delta_steps))

        for _ in range(total_steps):
            self._sequence_index = (
                self._sequence_index + direction
            ) % len(self.HALF_STEP_SEQUENCE)
            phase = self.HALF_STEP_SEQUENCE[self._sequence_index]

            for i, pin in enumerate(self.pins):
                GPIO.output(pin, phase[i])

            self.current_step = (self.current_step + direction) % self.steps_per_revolution
            time.sleep(self.step_delay)

        self.hold_position()

    def rotate_quarter_turn(self, clockwise=True):
        steps = self.steps_per_quarter_turn if clockwise else -self.steps_per_quarter_turn
        self._move_relative_steps(steps)

    def rotate_to_direction(self, direction):
        if direction not in self.direction_map:
            return

        target_step = self.direction_map[direction]
        self._rotate_to_step(target_step)

    def rotate_to_sensor(self, sensor_id):
        if sensor_id not in self.sensor_position_map:
            return

        target_step = self.sensor_position_map[sensor_id]
        self._rotate_to_step(target_step)

    


    def _rotate_to_step(self, target_step):
        target = int(round(target_step)) % self.steps_per_revolution
        diff = (target - self.current_step) % self.steps_per_revolution

        if diff > self.steps_per_revolution / 2:
            diff -= self.steps_per_revolution

        self._move_relative_steps(diff)

    def hold_position(self):
        if not self.hold_enabled:
            for pin in self.pins:
                GPIO.output(pin, False)
            return

        phase = self.HALF_STEP_SEQUENCE[self._sequence_index]
        for i, pin in enumerate(self.pins):
            GPIO.output(pin, phase[i])

    def cleanup(self):
        for pin in self.pins:
            GPIO.output(pin, False)