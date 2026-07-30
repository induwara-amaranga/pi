#!/usr/bin/env python3
"""
AURA Body Rotation Controller
Uses TouchModule + StepperModule with hardware_config.py pins.

BOARD pin mapping:
  Stepper: IN1=16, IN2=29, IN3=18, IN4=15
  Touch:   1(FRONT)=32, 2(BACK)=31, 3(LEFT)=33, 4(RIGHT)=37

Gear ratio: 160/46 = 3.478
Steps for 360°: 14243

Direction layout (cardinal):
  front, right, back, left  (clockwise order)

Wire/rotation rules:
  - Robot tracks current facing direction as a cardinal state (not angle).
  - Robot NEVER rotates through/across BACK.
  - Maximum rotation is 270° (3 cardinal steps).
  - Shortest path is chosen, but if shortest path crosses BACK, the
    opposite (longer, non-back-crossing) path is taken instead.
  - Touching the currently-faced direction = no movement.
  - When at BACK, the robot always exits in the direction OPPOSITE to
    how it arrived (back_entry_side state). Arrived CW via right → exit
    CCW toward left side. Arrived CCW via left → exit CW toward right side.
"""

import os
import time
import logging


# ── Gear ratio + steps for 360° ─────────────────────────────────
ORBITAL_DIAMETER   = 160
INNER_DIAMETER     = 46
GEAR_RATIO         = ORBITAL_DIAMETER / INNER_DIAMETER   # 3.478
STEPS_PER_360      = int(round(4096 * GEAR_RATIO))       # 14243

os.environ.setdefault("STEPPER_STEPS_PER_REV", str(STEPS_PER_360))
os.environ.setdefault("STEPPER_DELAY",         "0.001")
os.environ.setdefault("STEPPER_HOLD",          "1")
os.environ.setdefault("TOUCH_ACTIVE_HIGH",     "1")

from touch_module   import TouchModule
from stepper_module import StepperModule
from hardware_config import TOUCH_SENSOR_PINS, TOUCH_SENSOR_DIRECTIONS, STEPPER_PINS


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════
# DIRECTION CONSTANTS
# ═══════════════════════════════════════════════════════════════

# Cardinals in clockwise order. Index position is used for
# shortest-path arithmetic.
CARDINALS = ["front", "right", "back", "left"]   # CW order
#             0        1        2       3

def _idx(direction: str) -> int:
    """Return 0-3 index of a direction string."""
    return CARDINALS.index(direction.lower().strip())


def _degrees_to_steps(degrees: float) -> int:
    """Convert a signed degree value to signed stepper steps."""
    return int(round(degrees * STEPS_PER_360 / 360.0))


def compute_rotation(current_dir: str, target_dir: str, force_clockwise: bool = None):
    """
    Compute the rotation needed to go from current_dir to target_dir
    while NEVER crossing through BACK.

    Args:
        current_dir:     Direction robot is currently facing.
        target_dir:      Direction robot should face.
        force_clockwise: If True, force CW rotation.
                         If False, force CCW rotation.
                         If None (default), choose shortest non-back path.
                         Used when leaving BACK to ensure exit is opposite
                         to entry direction.

    Returns:
        (delta_degrees: float, path_description: str)
        Positive = CW, Negative = CCW.
    """
    ci = _idx(current_dir)
    ti = _idx(target_dir)

    if ci == ti:
        return 0.0, "no movement"

    # Clockwise steps (positive) and CCW steps (negative) to reach target
    cw_steps  = (ti - ci) % 4          # 1, 2, or 3
    ccw_steps = (ci - ti) % 4          # 1, 2, or 3

    cw_degrees  =  cw_steps  * 90.0
    ccw_degrees = -ccw_steps * 90.0

    # ── Forced direction (used when exiting BACK) ────────────────
    if force_clockwise is True:
        return cw_degrees,  f"CW {cw_degrees:.0f}° (back-exit forced)"
    if force_clockwise is False:
        return ccw_degrees, f"CCW {abs(ccw_degrees):.0f}° (back-exit forced)"

    # ── Normal: shortest path that doesn't cross BACK ───────────
    def _path_crosses_back(start_idx: int, steps: int, clockwise: bool) -> bool:
        """Check whether arc passes through index 2 (back)."""
        step = 1 if clockwise else -1
        pos  = start_idx
        for _ in range(steps):
            pos = (pos + step) % 4
            if pos == 2:                # back index
                return True
        return False

    cw_crosses_back  = _path_crosses_back(ci, cw_steps,  clockwise=True)
    ccw_crosses_back = _path_crosses_back(ci, ccw_steps, clockwise=False)

    # Choose shortest path first; override if it crosses back
    if cw_steps <= ccw_steps:
        # Prefer CW (shorter or equal)
        if not cw_crosses_back:
            return cw_degrees,  f"CW {cw_degrees:.0f}°"
        elif not ccw_crosses_back:
            return ccw_degrees, f"CCW {abs(ccw_degrees):.0f}° (avoiding back)"
        else:
            # Both cross back — should never happen for 4-cardinal system,
            # but guard anyway: pick the one that doesn't land on back
            log.warning("Both paths cross BACK — choosing CW anyway.")
            return cw_degrees, f"CW {cw_degrees:.0f}° (forced)"
    else:
        # Prefer CCW (shorter)
        if not ccw_crosses_back:
            return ccw_degrees, f"CCW {abs(ccw_degrees):.0f}°"
        elif not cw_crosses_back:
            return cw_degrees,  f"CW {cw_degrees:.0f}° (avoiding back)"
        else:
            log.warning("Both paths cross BACK — choosing CCW anyway.")
            return ccw_degrees, f"CCW {abs(ccw_degrees):.0f}° (forced)"


# ═══════════════════════════════════════════════════════════════
# BODY ROTATION CONTROLLER
# ═══════════════════════════════════════════════════════════════

class BodyRotationController:
    """
    Direction-state-based rotation controller.

    Tracks the robot's current facing direction as one of:
      front | right | back | left

    Never rotates through BACK. Max rotation = 270°.

    back_entry_side  — records which side the robot used to reach BACK:
      "right"  → robot arrived at back by rotating CW (through right)
      "left"   → robot arrived at back by rotating CCW (through left)
    When a sensor fires while at BACK, the robot exits in the direction
    OPPOSITE to back_entry_side, unwinding the wire correctly.
    """

    def __init__(self, stepper: StepperModule, starting_direction: str):
        if starting_direction.lower().strip() not in CARDINALS:
            raise ValueError(
                f"Invalid starting direction '{starting_direction}'. "
                f"Must be one of: {CARDINALS}"
            )
        self.stepper           = stepper
        self.current_direction = starting_direction.lower().strip()

        # Tracks which side the robot crossed to reach BACK.
        # None when not at back, or when starting_direction is back
        # (entry side unknown — defaults to safe CCW exit on first move).
        self.back_entry_side: str | None = (
            None if starting_direction != "back" else "unknown"
        )

        log.info(
            "BodyRotationController ready. Starting direction = %s",
            self.current_direction.upper(),
        )
        log.info(
            "Stepper BOARD pins: IN1=%s IN2=%s IN3=%s IN4=%s",
            *stepper.pins,
        )
        log.info("Steps for 360° = %d", STEPS_PER_360)

    # ── Core rotation ───────────────────────────────────────────

    def face(self, target_direction: str):
        """
        Rotate to face target_direction from the current direction.
        Path never crosses BACK. Max arc = 270°.

        Special rule when currently at BACK:
          The robot must exit in the direction OPPOSITE to how it arrived,
          so it unwinds the wire rather than twisting it further.
            arrived via right (CW)  → exit must be CCW  (force_clockwise=False)
            arrived via left  (CCW) → exit must be CW   (force_clockwise=True)
            arrived via unknown     → default to CCW exit (safe assumption)
        """
        target_direction = target_direction.lower().strip()

        if target_direction not in CARDINALS:
            log.warning("Unknown direction: '%s'", target_direction)
            return

        # ── Determine force_clockwise override when leaving BACK ─
        force_cw = None
        if self.current_direction == "back" and target_direction != "back":
            if self.back_entry_side == "right":
                # Arrived CW through right → must exit CCW
                force_cw = False
                log.info(
                    "At BACK (entered via RIGHT/CW) → forcing CCW exit to unwind."
                )
            elif self.back_entry_side == "left":
                # Arrived CCW through left → must exit CW
                force_cw = True
                log.info(
                    "At BACK (entered via LEFT/CCW) → forcing CW exit to unwind."
                )
            else:
                # Unknown entry (started at back) → default CCW exit
                force_cw = False
                log.info(
                    "At BACK (entry side unknown) → defaulting to CCW exit."
                )

        delta_degrees, path_desc = compute_rotation(
            self.current_direction, target_direction, force_clockwise=force_cw
        )

        if delta_degrees == 0.0:
            log.info(
                "Already facing %s — no movement.", self.current_direction.upper()
            )
            return

        delta_steps = _degrees_to_steps(delta_degrees)

        log.info(
            "Rotating %s | %s → %s | %s | steps=%d",
            path_desc,
            self.current_direction.upper(),
            target_direction.upper(),
            f"{delta_degrees:+.1f}°",
            abs(delta_steps),
        )

        self.stepper._move_relative_steps(delta_steps)

        # ── Update back_entry_side when arriving at or leaving BACK ─
        if target_direction == "back":
            # Record which side we crossed to get here
            self.back_entry_side = "right" if delta_degrees > 0 else "left"
            log.info(
                "Arrived at BACK via %s side (%s).",
                self.back_entry_side.upper(),
                "CW" if delta_degrees > 0 else "CCW",
            )
        else:
            self.back_entry_side = None   # no longer at back

        self.current_direction = target_direction
        log.info("Now facing %s.", self.current_direction.upper())

    def go_home(self):
        """Return to FRONT."""
        log.info("Returning HOME (FRONT) ...")
        self.face("front")


# ═══════════════════════════════════════════════════════════════
# STARTUP: ask user for starting direction
# ═══════════════════════════════════════════════════════════════

def ask_starting_direction() -> str:
    """
    Prompt the operator to confirm which physical direction
    the robot is currently facing before the program runs.
    """
    print("\n" + "=" * 50)
    print("  AURA Body Rotation Controller — Startup")
    print("=" * 50)
    print("  Which direction is the robot currently facing?")
    print()
    for i, d in enumerate(CARDINALS, 1):
        print(f"    [{i}] {d.upper()}")
    print()

    while True:
        raw = input("  Enter number (1-4) or direction name: ").strip().lower()

        # Accept numeric shortcut
        if raw in {"1", "2", "3", "4"}:
            direction = CARDINALS[int(raw) - 1]
            print(f"  Starting direction set to: {direction.upper()}\n")
            return direction

        # Accept direction name
        if raw in CARDINALS:
            print(f"  Starting direction set to: {raw.upper()}\n")
            return raw

        print("  Invalid input. Please enter 1-4 or front/right/back/left.")


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    # Ask starting direction BEFORE initialising hardware
    starting_direction = ask_starting_direction()

    touch   = TouchModule()
    stepper = StepperModule()
    robot   = BodyRotationController(stepper, starting_direction)

    log.info(
        "Touch BOARD pins → FRONT:%d  BACK:%d  LEFT:%d  RIGHT:%d",
        TOUCH_SENSOR_PINS[1],   # front
        TOUCH_SENSOR_PINS[2],   # back
        TOUCH_SENSOR_PINS[3],   # left
        TOUCH_SENSOR_PINS[4],   # right
    )
    log.info("Watching touch sensors ...")

    last_direction = None

    try:
        while True:
            direction = touch.get_touched_direction()

            if direction and direction != last_direction:
                log.info("Touch detected: %s", direction.upper())
                robot.face(direction)
                last_direction = direction
                time.sleep(0.5)   # debounce

            elif not direction:
                last_direction = None

            time.sleep(0.05)

    except KeyboardInterrupt:
        log.info("Ctrl+C received.")
    finally:
        robot.go_home()
        stepper.cleanup()
        log.info("Shutdown complete.")


if __name__ == "__main__":
    main()