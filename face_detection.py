#!/usr/bin/env python3
"""
AURA Face Detector – Haar Cascade + Picamera2
No servo tracking, display only.
"""

import cv2
import os
import time
import logging
from picamera2 import Picamera2

# ───────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(levelname)-8s  %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════
# SETTINGS
# ═══════════════════════════════════════════════════════════════
FRAME_W     = 640
FRAME_H     = 480
FRAME_RATE  = 30
MIRROR      = True
WINDOW_NAME = "AURA Face Detector"
CONF_FRAMES = 3    # consecutive frames before confirming face
LOSE_FRAMES = 8    # consecutive frames without face before dropping

# ═══════════════════════════════════════════════════════════════
# LOAD CASCADE
# ═══════════════════════════════════════════════════════════════
def load_cascade():
    paths = [
        os.path.join(os.path.dirname(cv2.__file__), "data", "haarcascade_frontalface_default.xml"),
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        "haarcascade_frontalface_default.xml",
    ]
    for path in paths:
        if os.path.exists(path):
            cc = cv2.CascadeClassifier(path)
            if not cc.empty():
                log.info("Cascade loaded: %s", path)
                return cc
    raise FileNotFoundError("No cascade found. Try: pip install opencv-contrib-python")


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════
def main():
    cascade = load_cascade()

    # ── Camera ──────────────────────────────────────────────────
    log.info("Starting camera ...")
    cam = Picamera2()
    cfg = cam.create_preview_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "BGR888"},
        controls={"FrameRate": FRAME_RATE},
    )
    cam.configure(cfg)
    cam.start()
    time.sleep(1.0)
    log.info("Camera ready.")

    # ── Window ──────────────────────────────────────────────────
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, FRAME_W, FRAME_H)
    cv2.moveWindow(WINDOW_NAME, 0, 0)

    # ── State ───────────────────────────────────────────────────
    confirm_count = 0
    lose_count    = 0
    confirmed     = False
    sx = float(FRAME_W // 2)   # smoothed face x
    sy = float(FRAME_H // 2)   # smoothed face y
    sw = 120.0                  # smoothed face w
    sh = 120.0                  # smoothed face h
    ALPHA = 0.5                 # EMA smoothing factor

    # ── FPS ─────────────────────────────────────────────────────
    fps       = 0.0
    fps_count = 0
    fps_t0    = time.time()

    log.info("Running. Press Q / Esc to quit.")

    try:
        while True:
            frame = cam.capture_array()
            if frame is None:
                continue

            # ── Detection ────────────────────────────────────────
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            faces = cascade.detectMultiScale(
                gray,
                scaleFactor  = 1.08,
                minNeighbors = 7,
                minSize      = (90, 90),
                flags        = cv2.CASCADE_SCALE_IMAGE,
            )

            face_info = None

            if len(faces) > 0:
                # Track largest (closest) face
                x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
                raw_cx = float(x + w // 2)
                raw_cy = float(y + h // 2)

                confirm_count += 1
                lose_count     = 0

                if not confirmed:
                    if confirm_count >= CONF_FRAMES:
                        confirmed = True
                        sx, sy, sw, sh = raw_cx, raw_cy, float(w), float(h)
                else:
                    # EMA smoothing
                    sx = ALPHA * raw_cx + (1 - ALPHA) * sx
                    sy = ALPHA * raw_cy + (1 - ALPHA) * sy
                    sw = ALPHA * w      + (1 - ALPHA) * sw
                    sh = ALPHA * h      + (1 - ALPHA) * sh
                    face_info = (int(sx), int(sy), int(sw), int(sh))

            else:
                confirm_count  = 0
                lose_count    += 1
                if lose_count >= LOSE_FRAMES:
                    confirmed = False
                if confirmed:
                    face_info = (int(sx), int(sy), int(sw), int(sh))

            # ── Mirror display ───────────────────────────────────
            display = cv2.flip(frame, 1) if MIRROR else frame.copy()

            # ── Draw face ────────────────────────────────────────
            if face_info is not None:
                fcx, fcy, fw2, fh2 = face_info
                x1 = fcx - fw2 // 2
                y1 = fcy - fh2 // 2
                x2 = fcx + fw2 // 2
                y2 = fcy + fh2 // 2

                # Mirror box
                if MIRROR:
                    x1m = FRAME_W - x2
                    x2m = FRAME_W - x1
                    fcx = FRAME_W - fcx
                    x1, x2 = x1m, x2m

                color = (0, 165, 255)
                t = 18

                # Corner tick bounding box
                segs = [
                    (x1,    y1,    x1+t,  y1  ), (x1,    y1,    x1,    y1+t),
                    (x2,    y1,    x2-t,  y1  ), (x2,    y1,    x2,    y1+t),
                    (x1,    y2,    x1+t,  y2  ), (x1,    y2,    x1,    y2-t),
                    (x2,    y2,    x2-t,  y2  ), (x2,    y2,    x2,    y2-t),
                ]
                for ax, ay, bx, by in segs:
                    cv2.line(display, (ax, ay), (bx, by), color, 2, cv2.LINE_AA)

                cv2.circle(display, (fcx, fcy), 5, color, -1, cv2.LINE_AA)

                # Label
                label = "FACE"
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
                by0 = max(y1 - 2, lh + 10)
                cv2.rectangle(display, (x1, by0 - lh - 8), (x1 + lw + 8, by0), color, -1)
                cv2.putText(display, label, (x1 + 4, by0 - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)

            # ── Crosshair ────────────────────────────────────────
            cx_f, cy_f = FRAME_W // 2, FRAME_H // 2
            cv2.line(display, (cx_f - 20, cy_f), (cx_f + 20, cy_f), (255, 255, 0), 1)
            cv2.line(display, (cx_f, cy_f - 20), (cx_f, cy_f + 20), (255, 255, 0), 1)

            # ── FPS counter ──────────────────────────────────────
            fps_count += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                fps       = fps_count / (now - fps_t0)
                fps_t0    = now
                fps_count = 0

            # ── Top banner ───────────────────────────────────────
            overlay = display.copy()
            cv2.rectangle(overlay, (0, 0), (FRAME_W, 32), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.55, display, 0.45, 0, display)
            status = "TRACKING" if face_info else "SEARCHING"
            color  = (0, 220, 80) if face_info else (0, 160, 255)
            cv2.putText(display,
                        f"AURA  |  {status}  |  {fps:.1f} fps",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)

            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                log.info("Quit.")
                break

    except KeyboardInterrupt:
        log.info("Ctrl+C received.")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        log.info("Done.")


if __name__ == "__main__":
    main()