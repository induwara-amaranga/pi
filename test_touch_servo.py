import time
import RPi.GPIO as GPIO
from touch_module import TouchModule
from stepper_module import StepperModule


def main():
    touch = TouchModule()
    stepper = StepperModule()

    print("Touch and Stepper Diagnostic Started")
    print("Wiring expected:")
    print(f"  touch pins -> {touch.sensor_pins}")
    print(f"  touch order -> {touch.sequence_order}")
    print(f"  touch directions -> {touch.sensor_directions}")
    print(f"  stepper pins -> {stepper.pins}")
    print(f"  touch active_high -> {touch.active_high}")
    print("Press Ctrl+C to stop.\n")

    print("Step 1/2: Stepper self-check sweep (front -> right -> back -> left -> front)")
    for direction in ["front", "right", "back", "left", "front"]:
        print(f"  moving: {direction}")
        stepper.rotate_to_direction(direction)
        time.sleep(0.4)

    print("\nStep 2/2: Live touch test")
    print("Touch one pad at a time. Script prints raw pin states and moves stepper.")

    try:
        last_direction = None
        last_trigger_time = 0.0
        debounce_seconds = 0.35

        while True:
            raw_states = touch.get_raw_states()
            direction = touch.get_touched_direction()

            if direction:
                now = time.time()
                if direction != last_direction or (now - last_trigger_time) > debounce_seconds:
                    print(f"Touch detected: {direction} | raw={raw_states}")
                    stepper.rotate_to_direction(direction)
                    last_direction = direction
                    last_trigger_time = now

            time.sleep(0.08)
    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        stepper.cleanup()
        GPIO.cleanup()

if __name__ == "__main__":
    main()