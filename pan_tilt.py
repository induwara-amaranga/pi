from hardware_config import PAN_CHANNEL, TILT_CHANNEL

try:
    from adafruit_servokit import ServoKit
except Exception:  # pragma: no cover - hardware import guard
    ServoKit = None


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class PanTiltModule:
    def __init__(
        self,
        pan_home=90,
        tilt_home=85,
        pan_limits=(30, 150),
        tilt_limits=(60, 120),
    ):
        if ServoKit is None:
            raise RuntimeError("adafruit_servokit is required for PanTiltModule")

        self.kit = ServoKit(channels=16)
        self.pan_channel = PAN_CHANNEL
        self.tilt_channel = TILT_CHANNEL

        self.pan_min, self.pan_max = pan_limits
        self.tilt_min, self.tilt_max = tilt_limits

        self.pan_angle = _clamp(float(pan_home), self.pan_min, self.pan_max)
        self.tilt_angle = _clamp(float(tilt_home), self.tilt_min, self.tilt_max)
        self._apply_angles()

    def _apply_angles(self):
        self.kit.servo[self.pan_channel].angle = int(round(self.pan_angle))
        self.kit.servo[self.tilt_channel].angle = int(round(self.tilt_angle))

    def set_angles(self, pan=None, tilt=None):
        if pan is not None:
            self.pan_angle = _clamp(float(pan), self.pan_min, self.pan_max)
        if tilt is not None:
            self.tilt_angle = _clamp(float(tilt), self.tilt_min, self.tilt_max)
        self._apply_angles()

    def center(self):
        self.set_angles(
            pan=(self.pan_min + self.pan_max) / 2,
            tilt=(self.tilt_min + self.tilt_max) / 2,
        )

    def track_face(self, error_x, error_y, gain_pan=15.0, gain_tilt=12.0):
        # error_x/error_y are expected in range [-1.0, 1.0].
        target_pan = self.pan_angle - (float(error_x) * gain_pan)
        target_tilt = self.tilt_angle + (float(error_y) * gain_tilt)
        self.set_angles(target_pan, target_tilt)
