import RPi.GPIO as GPIO
import time
from hardware_config import GPIO_MODE, STEPPER_PINS
from stepper_module import StepperModule

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

stepper = StepperModule()

print("=== Stepper Position Map ===")
for sensor_id, steps in stepper.sensor_position_map.items():
    degrees = steps / stepper.steps_per_degree
    print(f"  Sensor {sensor_id} → {degrees:.1f}° ({steps} steps)")

print("\n--- Test 1: Go to Sensor 1 (0°) ---")
stepper.rotate_to_sensor(1)
time.sleep(1)

print(f"Current step after Sensor 1: {stepper.current_step}")

print("\n--- Test 2: From Sensor 1 (0°) → Sensor 3 (180°) ---")
print("Expected: rotate 180° (either direction)")
stepper.rotate_to_sensor(3)
time.sleep(1)
print(f"Current step: {stepper.current_step}")

print("\n--- Test 3: From Sensor 3 (180°) → Sensor 4 (270°) ---")
print("Expected: rotate 90° clockwise (shortest)")
stepper.rotate_to_sensor(4)
time.sleep(1)
print(f"Current step: {stepper.current_step}")

print("\n--- Test 4: From Sensor 4 (270°) → Sensor 1 (0°) ---")
print("Expected: rotate 90° clockwise (shortest, NOT 270° back)")
stepper.rotate_to_sensor(1)
time.sleep(1)
print(f"Current step: {stepper.current_step}")

print("\n--- Test 5: From Sensor 1 (0°) → Sensor 4 (270°) ---")
print("Expected: rotate 90° COUNTER-clockwise (shortest)")
stepper.rotate_to_sensor(4)
time.sleep(1)
print(f"Current step: {stepper.current_step}")

stepper.cleanup()
GPIO.cleanup()
print("\nDone.")