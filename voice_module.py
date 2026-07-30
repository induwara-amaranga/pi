import threading
import time

import speech_recognition as sr
from config import POSSIBLE_WAKE_PHRASES, MIC_DEVICE_INDEX
from audio_preprocessing import preprocess_audio
from ai_client import create_ai_client, AIClient


class VoiceModule:
    def __init__(self, api_key: str, provider: str = "gemini", ai_client: AIClient | None = None):
        self.recognizer = sr.Recognizer()

        # Mic sensitivity settings
        self.recognizer.energy_threshold = 300
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.8

        self.ai_client = ai_client or create_ai_client(provider, api_key)

        # Only one thread can hold the mic device at a time (the Pi's ALSA
        # route for it has no dmix, so two concurrent streams fight over the
        # same hardware and PortAudio kills one out from under the other).
        # _mic_lock serializes access; _yield_mic lets an on-demand listener
        # (e.g. the UI mic button, running on the MQTT thread) signal the
        # wake-word loop to release the mic between attempts instead of
        # holding it indefinitely.
        self._mic_lock = threading.Lock()
        self._yield_mic = threading.Event()

    def listen_for_wake_word(self, wake_word: str = "hi aura") -> bool:
        """
        Continuously listens until the wake word is detected. Releases the
        mic between attempts so an on-demand listen elsewhere can take over.
        """
        print(f"Waiting for wake word: '{wake_word}'")

        calibrated = False

        while True:
            if self._yield_mic.is_set():
                time.sleep(0.05)
                continue

            try:
                with self._mic_lock:
                    with sr.Microphone(device_index=MIC_DEVICE_INDEX) as source:
                        if not calibrated:
                            self.recognizer.adjust_for_ambient_noise(source, duration=1)
                            calibrated = True

                        audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=4)

                try:
                    audio = preprocess_audio(audio)
                except Exception as e:
                    print(f"Audio preprocessing error (using raw audio): {e}")

                heard_text = self.recognizer.recognize_google(audio).lower().strip()

                print(f"Heard: {heard_text}")

                if any(phrase in heard_text for phrase in POSSIBLE_WAKE_PHRASES):
                    print("Wake word detected.")
                    return True

            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                continue
            except sr.RequestError as e:
                print(f"Speech service error while waiting for wake word: {e}")
                continue
            except Exception as e:
                print(f"Unexpected wake word error: {e}")
                continue

    def listen_and_convert_to_text(self, timeout: int = 5, phrase_time_limit: int = 8) -> str | None:
        """
        Listen once and convert speech to text. Signals the wake-word loop
        (if running) to yield the mic first, then takes the lock so the two
        can't capture from the same device at once.
        """
        self._yield_mic.set()
        try:
            with self._mic_lock:
                try:
                    with sr.Microphone(device_index=MIC_DEVICE_INDEX) as source:
                        print("Listening for user command...")
                        self.recognizer.adjust_for_ambient_noise(source, duration=0.5)

                        audio = self.recognizer.listen(
                            source,
                            timeout=timeout,
                            phrase_time_limit=phrase_time_limit
                        )

                    try:
                        audio = preprocess_audio(audio)
                    except Exception as e:
                        print(f"Audio preprocessing error (using raw audio): {e}")

                    print("Converting speech to text...")
                    text = self.recognizer.recognize_google(audio).strip()
                    print(f"User said: {text}")
                    return text

                except sr.WaitTimeoutError:
                    print("No speech detected before timeout.")
                    return None
                except sr.UnknownValueError:
                    print("Could not understand the audio.")
                    return None
                except sr.RequestError as e:
                    print(f"Speech recognition service error: {e}")
                    return None
                except Exception as e:
                    print(f"Unexpected speech-to-text error: {e}")
                    return None
        finally:
            self._yield_mic.clear()

    def clean_up_text(self, raw_text: str) -> str:
        """
        Correct likely speech-to-text errors and return a concise, coherent
        version of what the user probably said, without answering it.
        """
        try:
            prompt = f"""
You are cleaning up a speech-to-text transcript from a customer talking to a restaurant
robot assistant. The transcript may contain misheard words, missing words, or awkward
phrasing caused by transcription errors.

Rules:
- Fix likely transcription errors and produce the most probable intended sentence.
- Keep it concise and natural. Do not add new requests or information that wasn't implied.
- Do not answer the question or respond to it in any way - only correct the transcript.
- If the transcript is already clear, return it unchanged.
- Return only the corrected sentence, with no quotes, labels, or explanation.



Transcript: "{raw_text}"
"""
            cleaned = self.ai_client.generate(prompt).strip().strip('"')
            return cleaned or raw_text

        except Exception as e:
            print(f"Text cleanup error: {e}")
            return raw_text

    def get_ai_response(self, user_text: str, menu_context: str | None = None) -> str:
        """
        Send user text to the configured AI provider and return AURA's reply,
        using current menu context when available.
        """
        try:
            menu_section = ""
            if menu_context:
                menu_section = f"Current menu and availability:\n{menu_context}\n\n"

            prompt = f"""
You are AURA, a smart and friendly restaurant robot assistant.

Rules:
- Reply as AURA.
- Keep responses short, clear, and polite.
- Answer only using the current menu information when the user asks about food availability or prices.
- If the user asks about an item not on the current menu, say it is unavailable.
- Help with greetings, menu questions, ordering, and table assistance.
- If unclear, ask a short follow-up question.
- Do not mention that you are an AI model unless directly asked.

{menu_section}User said: {user_text}
"""

            reply = self.ai_client.generate(prompt).strip()

            if reply:
                print(f"AURA reply: {reply}")
                return reply

            return "Sorry, I could not generate a response."

        except Exception as e:
            print(f"AI response error: {e}")
            return "Sorry, I had trouble processing that request."