from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from luma.core.render import canvas
import time

# ── init ──────────────────────────────────────────────────────────────────────

def init_displays():
    displays = []

    try:
        serial1 = i2c(port=4, address=0x3C)
        oled1 = ssd1306(serial1, width=128, height=64)
        oled1.contrast(255)
        oled1.show()
        displays.append(("OLED1 (bus 4)", oled1))
        print("OLED 1 initialized OK on bus 4")
    except Exception as e:
        print(f"OLED 1 FAILED: {e}")

    try:
        serial2 = i2c(port=1, address=0x3C)
        oled2 = ssd1306(serial2, width=128, height=64)
        oled2.contrast(255)
        oled2.show()
        displays.append(("OLED2 (bus 1)", oled2))
        print("OLED 2 initialized OK on bus 1")
    except Exception as e:
        print(f"OLED 2 FAILED: {e}")

    return displays


# ── drawing helpers (copied exactly from OLEDModule) ──────────────────────────

def draw_eyes(draw, state="open", direction="center"):
    x_shift = {"center": 0, "left": -15, "right": 15, "front": 0, "back": 0}.get(direction, 0)
    eye_box = (44 + x_shift, 15, 84 + x_shift, 55)

    if state == "open":
        draw.ellipse(eye_box, fill="white")
    else:
        draw.line((44 + x_shift, 35, 84 + x_shift, 35), fill="white", width=4)


def update_all(displays, state="open", direction="center"):
    for name, device in displays:
        with canvas(device) as draw:
            draw_eyes(draw, state=state, direction=direction)


# ── tests ─────────────────────────────────────────────────────────────────────

def test_displays(displays):
    if not displays:
        print("No displays found!")
        return

    print(f"\nTesting {len(displays)} display(s)...\n")

    # Test 1 - open eyes centered
    print("Test 1: Eyes open - center")
    update_all(displays, state="open", direction="center")
    time.sleep(2)

    # Test 2 - look left
    print("Test 2: Look left")
    update_all(displays, state="open", direction="left")
    time.sleep(2)

    # Test 3 - look right
    print("Test 3: Look right")
    update_all(displays, state="open", direction="right")
    time.sleep(2)

    # Test 4 - look front (no shift, same as center)
    print("Test 4: Look front")
    update_all(displays, state="open", direction="front")
    time.sleep(2)

    # Test 5 - back
    print("Test 5: Look back")
    update_all(displays, state="open", direction="back")
    time.sleep(2)

    # Test 6 - blink (closed eyes)
    print("Test 6: Blink")
    update_all(displays, state="closed", direction="center")
    time.sleep(0.3)
    update_all(displays, state="open", direction="center")
    time.sleep(1)

    # Test 7 - blink 5 times
    print("Test 7: Blink 5 times")
    for _ in range(5):
        update_all(displays, state="closed", direction="center")
        time.sleep(0.2)
        update_all(displays, state="open", direction="center")
        time.sleep(0.5)

    # Test 8 - sweep left → center → right → center
    print("Test 8: Sweep directions")
    for direction in ["left", "center", "right", "center", "front", "center"]:
        update_all(displays, state="open", direction=direction)
        time.sleep(1)

    # Final - clear
    for name, device in displays:
        device.clear()

    print("\nAll tests done!")


# ── main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    displays = init_displays()
    test_displays(displays)