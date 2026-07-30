import os
from dotenv import load_dotenv

load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# if not GEMINI_API_KEY:
#     raise ValueError("GEMINI_API_KEY is not set in environment variables")

MQTT_BROKER = os.getenv("MQTT_BROKER", "10.169.209.167")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))

# ALSA default/dmix routing can be unreliable on the Pi, so the mic device is pinned
# explicitly. Run: python -c "import speech_recognition as sr; print(sr.Microphone.list_microphone_names())"
# to find the correct index if the USB mic changes.
MIC_DEVICE_INDEX = int(os.getenv("MIC_DEVICE_INDEX", 1))

USE_WAKE_WORD = True
WAKE_WORD = "hi aura"

POSSIBLE_WAKE_PHRASES = [
    "hi aura",
    "hey aura",
    "hello aura",
    "hiora",
    "heura",
    "hi ora",
    "high aura",
    "hiya aura",
    "hi awra",
    "hey ora",
    "hi ara",
    "hiara",
    "hey ara",
    "i aura",
    "aura",
    "hey aura",
]