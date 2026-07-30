import os
import time
import threading
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from luma.core.render import canvas

from hardware_config import OLED_I2C_ADDRESS, OLED_I2C_PORTS

class OLEDModule:
    _FONT_5X7 = {
        "A": [
            "01110",
            "10001",
            "10001",
            "11111",
            "10001",
            "10001",
            "10001",
        ],
        "U": [
            "10001",
            "10001",
            "10001",
            "10001",
            "10001",
            "10001",
            "01110",
        ],
        "R": [
            "11110",
            "10001",
            "10001",
            "11110",
            "10100",
            "10010",
            "10001",
        ],
    }

    def __init__(self):
        self.devices = []

        for bus in OLED_I2C_PORTS:
            bus_path = f"/dev/i2c-{bus}"
            if not os.path.exists(bus_path):
                print(f"OLED bus {bus} not available at {bus_path}; skipping")
                continue

            try:
                serial = i2c(port=bus, address=OLED_I2C_ADDRESS)
                device = ssd1306(serial)
                self.devices.append(device)
                print(f"OLED initialized on I2C bus {bus} at address {OLED_I2C_ADDRESS:#x}")
            except Exception as e:
                print(f"OLED Init Error on I2C bus {bus}: {e}")

        self.current_direction = "center"
        self.display_mode = "aura"
        self.is_blinking = False
        self._aura_phase = 0
        self._reset_timer = None  # To keep track of the auto-center timer
        
        if self.devices:
            threading.Thread(target=self._blink_scheduler, daemon=True).start()

    def _measure_text(self, text, scale, spacing):
        width = 0
        for index, _ in enumerate(text):
            width += 5 * scale
            if index < len(text) - 1:
                width += spacing * scale
        height = 7 * scale
        return width, height

    def _draw_big_text(self, draw, text, x, y, scale, spacing):
        cursor_x = x
        for index, ch in enumerate(text):
            glyph = self._FONT_5X7.get(ch)
            if glyph:
                for row_idx, row in enumerate(glyph):
                    for col_idx, bit in enumerate(row):
                        if bit == "1":
                            x0 = cursor_x + col_idx * scale
                            y0 = y + row_idx * scale
                            draw.rectangle(
                                (x0, y0, x0 + scale - 1, y0 + scale - 1),
                                fill="white",
                            )
            cursor_x += (5 + spacing) * scale

    def _draw_aura(self, draw, label):
        phase = self._aura_phase % 4
        offset_x = [-2, 0, 2, 0][phase]
        offset_y = [0, 1, 0, -1][phase]

        scale = 6 if len(label) == 2 else 5
        spacing = 1
        text_w, text_h = self._measure_text(label, scale, spacing)
        start_x = max(0, (128 - text_w) // 2 + offset_x)
        start_y = max(0, (64 - text_h) // 2 + offset_y)

        self._draw_big_text(draw, label, start_x, start_y, scale, spacing)

    def _draw_eyes(self, draw, state="open", direction="center"):
        x_shift = {"center": 0, "left": -15, "right": 15, "front": 0, "back": 0}.get(direction, 0)

        # ── Eye size — adjust these ──────────────────────────
        EYE_W = 26    # half-width  (was 20 → gives 40px wide)
        EYE_H = 26    # half-height (was 20 → gives 40px tall)
        CX    = 64    # center x of screen
        CY    = 35    # center y of screen

        eye_box = (
            CX - EYE_W + x_shift,   # left
            CY - EYE_H,             # top
            CX + EYE_W + x_shift,   # right
            CY + EYE_H,             # bottom
        )

        if state == "open":
            draw.ellipse(eye_box, fill="white")
        else:
            draw.line(
                (CX - EYE_W + x_shift, CY, CX + EYE_W + x_shift, CY),
                fill="white", width=4
            )

    def _blink_scheduler(self):
        eye_last_blink = time.time()
        while True:
            if self.display_mode == "aura":
                self._aura_phase = (self._aura_phase + 1) % 4
                self._update_display()
                time.sleep(0.35)
                continue

            now = time.time()
            if now - eye_last_blink >= 5:
                self.is_blinking = True
                self._update_display()
                time.sleep(0.3)
                self.is_blinking = False
                self._update_display()
                eye_last_blink = now

            time.sleep(0.05)

    def _update_display(self):
        total = len(self.devices)
        for index, device in enumerate(self.devices):
            with canvas(device) as draw:
                if self.display_mode == "aura":
                    if total >= 2:
                        label = "AU" if index == 0 else "RA"
                    else:
                        label = "AURA"
                    self._draw_aura(draw, label)
                else:
                    state = "closed" if self.is_blinking else "open"
                    self._draw_eyes(draw, state=state, direction=self.current_direction)

    def show_aura(self):
        self.display_mode = "aura"
        self.is_blinking = False
        if self._reset_timer is not None:
            self._reset_timer.cancel()
            self._reset_timer = None
        self._update_display()

    def look_at(self, direction):
        """Sets the direction and starts a timer to return to center."""
        self.display_mode = "eyes"
        self.current_direction = direction
        self._update_display()

        # If we are moving to a position other than center, start a timer to go back
        if direction != "center":
            # Cancel existing timer if a new touch happens before the old one finished
            if self._reset_timer is not None:
                self._reset_timer.cancel()

            # Set a 3-second delay before returning to center
            self._reset_timer = threading.Timer(3.0, self.look_at, args=["center"])
            self._reset_timer.start()