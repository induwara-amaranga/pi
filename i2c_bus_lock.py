"""
Tiny cross-process "the PCA9685's I2C bus is busy" flag.

main_controller.py's OLED blink thread and the pan/tilt gesture / face
tracker subprocesses are separate OS processes, but hardware_config's
default OLED_I2C_PORTS=[4, 1] puts one OLED display on bus 1 - the same
physical bus Adafruit Blinka's ServoKit uses for the PCA9685. The OLED
thread's periodic full-frame redraw and the servo's 25Hz angle updates
then contend for the same bus, which is what causes the servo movement
to look smooth when pan_tilt_controller.py is run standalone but jittery
under main_controller.py. This module lets the servo side flag "I'm
actively driving the bus" via a lock file, and the OLED side check it
before redrawing.

Deliberately stdlib-only (no adafruit imports) so importing this from
oled_module.py never pulls adafruit_servokit/Blinka into
main_controller.py's process - that would force RPi.GPIO into BCM mode
and collide with main_controller's own GPIO.setmode(GPIO.BOARD) call.
"""

import os
import time

LOCK_FILE      = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".i2c_bus_busy")
STALE_SECONDS  = 120   # ignore a leftover lock this old - its holder likely crashed


def mark_busy():
    """Call when a process starts actively driving the PCA9685."""
    try:
        with open(LOCK_FILE, "w") as f:
            f.write(str(os.getpid()))
    except OSError:
        pass


def touch():
    """Refresh the lock's timestamp so a long-running holder doesn't go stale."""
    try:
        os.utime(LOCK_FILE, None)
    except OSError:
        mark_busy()


def clear_busy():
    """Call when a process is done driving the PCA9685."""
    try:
        os.remove(LOCK_FILE)
    except OSError:
        pass


def is_busy():
    """True if some process currently claims the bus (and hasn't gone stale)."""
    try:
        age = time.time() - os.path.getmtime(LOCK_FILE)
    except OSError:
        return False
    return age < STALE_SECONDS
