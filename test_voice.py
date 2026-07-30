import argparse
import os

import speech_recognition as sr
from dotenv import load_dotenv

from audio_module import AudioModule
from config import GEMINI_API_KEY, MIC_DEVICE_INDEX, WAKE_WORD
from voice_module import VoiceModule

EXIT_PHRASES = {"exit", "quit", "stop", "goodbye", "bye"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Full pipeline test for AURA: mic -> STT -> cleanup -> Gemini -> speaker."
    )
    parser.add_argument(
        "--wake-word",
        action="store_true",
        help="Wait for wake word before listening.",
    )
    parser.add_argument(
        "--no-wake-word",
        action="store_true",
        help="Skip wake word and listen immediately.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Run a single turn instead of looping until an exit phrase.",
    )
    return parser.parse_args()


def resolve_wake_word(args: argparse.Namespace) -> bool:
    if args.wake_word and args.no_wake_word:
        return False
    if args.wake_word:
        return True
    if args.no_wake_word:
        return False
    return False


def print_mic_check() -> None:
    print("\n" + "=" * 50)
    print("🎤 Mic check")
    try:
        names = sr.Microphone.list_microphone_names()
        for i, name in enumerate(names):
            marker = " <-- configured (MIC_DEVICE_INDEX)" if i == MIC_DEVICE_INDEX else ""
            print(f"  [{i}] {name}{marker}")
        if MIC_DEVICE_INDEX >= len(names):
            print(f"⚠️  MIC_DEVICE_INDEX={MIC_DEVICE_INDEX} is out of range for the devices above.")
    except Exception as e:
        print(f"⚠️  Could not list microphones: {e}")
    print("=" * 50)


def run_turn(voice: VoiceModule, audio: AudioModule, use_wake_word: bool) -> bool:
    """Run one full mic -> reply turn. Returns False if the conversation should end."""
    if use_wake_word:
        print("\n" + "-" * 50)
        print(f"💤 Waiting for wake word: '{WAKE_WORD}'...")
        voice.listen_for_wake_word(WAKE_WORD)
        print("✅ Wake word detected!")
        audio.speak_text("Yes, how can I help you?")

    print("👂 Listening for your command...")
    user_text = voice.listen_and_convert_to_text(timeout=8, phrase_time_limit=8)

    if not user_text:
        print("⚠️  No speech recognized. Try again.")
        return True

    print(f"👤 You said: {user_text}")

    if user_text.lower().strip() in EXIT_PHRASES:
        print("👋 Exit phrase detected — ending test.")
        audio.speak_text("Goodbye. Have a nice day.")
        return False

    cleaned_text = voice.clean_up_text(user_text)
    if cleaned_text != user_text:
        print(f"📝 Cleaned up: {cleaned_text}")

    print("🤔 Thinking...")
    reply = voice.get_gemini_response(cleaned_text)

    print("🔊 Responding...")
    audio.speak_text(reply)
    print("-" * 50)
    return True


def main() -> None:
    load_dotenv()
    api_key = os.getenv("GEMINI_API_KEY") or GEMINI_API_KEY
    if not api_key:
        print("ERROR: GEMINI_API_KEY is not set. Add it to .env or config.py.")
        return

    args = parse_args()
    use_wake_word = resolve_wake_word(args)

    #print_mic_check()

    audio = AudioModule()
    voice = VoiceModule(gemini_api_key=api_key)

    print_mic_check()
    print("\nFull pipeline test started.")
    print("Say 'exit', 'stop', or 'goodbye' to end, or press Ctrl+C.")
    if use_wake_word:
        print(f"Wake word mode enabled: '{WAKE_WORD}'.")
    else:
        print("Wake word mode disabled — calibrating and listening directly for your command.")

    try:
        while True:
            keep_going = run_turn(voice, audio, use_wake_word)
            if args.once or not keep_going:
                break
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user.")

    print("Full pipeline test completed.")


if __name__ == "__main__":
    main()
