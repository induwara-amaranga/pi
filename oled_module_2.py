# import time
# import threading
# from luma.core.interface.serial import i2c
# from luma.oled.device import ssd1306
# from luma.core.render import canvas

# class OLEDModule:
#     def __init__(self):
#         self.devices = []
#         try:
#             # Main I2C Bus (Port 1)
#             serial1 = i2c(port=1, address=0x3C)
#             self.devices.append(ssd1306(serial1))
            
#             # Software I2C Bus (Port 3)
#             serial2 = i2c(port=3, address=0x3C)
#             self.devices.append(ssd1306(serial2))
#         except Exception as e:
#             print(f"OLED Init Error: {e}")

#         self.current_direction = "center"
#         self.is_blinking = False
        
#         if self.devices:
#             threading.Thread(target=self._blink_scheduler, daemon=True).start()

#     def _draw_eyes(self, draw, state="open", direction="center"):
#         x_shift = {"center": 0, "left": -20, "right": 20, "front": 0, "back": 0}.get(direction, 0)
#         if state == "open":
#             draw.ellipse((20 + x_shift, 20, 50 + x_shift, 50), fill="white")
#             draw.ellipse((78 + x_shift, 20, 108 + x_shift, 50), fill="white")
#         else:
#             draw.line((20 + x_shift, 35, 50 + x_shift, 35), fill="white", width=4)
#             draw.line((78 + x_shift, 35, 108 + x_shift, 35), fill="white", width=4)

#     def _blink_scheduler(self):
#         while True:
#             time.sleep(10)
#             self.is_blinking = True
#             self._update_display()
#             time.sleep(0.3) 
#             self.is_blinking = False
#             self._update_display()

#     def _update_display(self):
#         for device in self.devices:
#             with canvas(device) as draw:
#                 state = "closed" if self.is_blinking else "open"
#                 self._draw_eyes(draw, state=state, direction=self.current_direction)

#     def look_at(self, direction):
#         self.current_direction = direction
#         self._update_display()