import os
from dotenv import load_dotenv

load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# if not GEMINI_API_KEY:
#     raise ValueError("GEMINI_API_KEY is not set in environment variables")

MQTT_BROKER = os.getenv("MQTT_BROKER", "10.169.209.167")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))

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