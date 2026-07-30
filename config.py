import os
from dotenv import load_dotenv

load_dotenv()

# --- AI provider selection ---
# Set AI_PROVIDER=claude|anthropic|groq in .env to switch VoiceModule to that API.
AI_PROVIDER = os.getenv("AI_PROVIDER", "gemini").strip().lower()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
CLAUDE_API_KEY = os.getenv("CLAUDE_API_KEY") or os.getenv("ANTHROPIC_API_KEY")
GROQ_API_KEY = os.getenv("GROQ_API_KEY")

_PROVIDER_KEYS = {
    "gemini": GEMINI_API_KEY,
    "claude": CLAUDE_API_KEY,
    "anthropic": CLAUDE_API_KEY,
    "groq": GROQ_API_KEY,
}


def get_ai_api_key() -> str | None:
    """Return the API key for whichever provider AI_PROVIDER selects."""
    return _PROVIDER_KEYS.get(AI_PROVIDER)

MQTT_BROKER = os.getenv("MQTT_BROKER", "10.169.209.167")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))

# ALSA default/dmix routing can be unreliable on the Pi, so the mic device is pinned
# explicitly. Run: python -c "import speech_recognition as sr; print(sr.Microphone.list_microphone_names())"
# to find the correct index if the USB mic changes.
MIC_DEVICE_INDEX = int(os.getenv("MIC_DEVICE_INDEX", 1))

USE_WAKE_WORD = False
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