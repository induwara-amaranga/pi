import RPi.GPIO as GPIO
import time
from hardware_config import GPIO_MODE, TOUCH_SENSOR_PINS
from stepper_module import StepperModule
from touch_module import TouchModule

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

touch = TouchModule()
stepper = StepperModule()

print("Touch any sensor to rotate to its position (no sequence required)")
print("Sensor 1=0°  Sensor 2=90°  Sensor 3=180°  Sensor 4=270°")
print("Press Ctrl+C to stop\n")

last_sensor = None
last_time = 0

try:
    while True:
        sensor_id = touch.get_touched_sensor()
        now = time.time()
        if sensor_id and (sensor_id != last_sensor or now - last_time > 0.5):
            current_deg = stepper.current_step / stepper.steps_per_degree
            target_deg = list({1:0, 2:90, 3:180, 4:270}.values())[sensor_id - 1]
            print(f"Sensor {sensor_id} touched | "
                  f"From {current_deg:.1f}° → {target_deg}° | "
                  f"Rotating...")
            stepper.rotate_to_sensor(sensor_id)
            print(f"Done. Now at step {stepper.current_step} "
                  f"({stepper.current_step/stepper.steps_per_degree:.1f}°)")
            last_sensor = sensor_id
            last_time = now
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped.")
    stepper.cleanup()
    GPIO.cleanup()