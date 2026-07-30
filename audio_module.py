import hashlib
import os
import tempfile
import threading
from gtts import gTTS
import pygame

CACHE_DIR = os.path.join(os.path.dirname(__file__), "tts_cache")


class AudioModule:
    def __init__(self):
        self.enabled = False
        self._lock = threading.Lock()
        self._active_threads = []
        try:
            pygame.mixer.init()
            self.enabled = True
        except Exception as e:
            print(f"Audio disabled: {e}")

    def _play_audio_file(self, audio_path: str, delete_after: bool) -> None:
        try:
            pygame.mixer.music.load(audio_path)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

            pygame.mixer.music.unload()
        finally:
            if delete_after and os.path.exists(audio_path):
                os.remove(audio_path)

    def speak_text(self, text: str, lang: str = "en", cache: bool = False):
        """
        Convert text to speech and play through speaker.

        cache=True reuses a saved mp3 for repeated fixed phrases (greetings,
        acknowledgements, goodbyes) instead of hitting the gTTS network API
        again. Leave this False for AI-generated replies, which are usually
        unique and would just fill the cache with one-off files.
        """
        if not self.enabled:
            print(f"AURA speaking (audio disabled): {text}")
            return

        try:
            print(f"AURA speaking: {text}")

            if cache:
                os.makedirs(CACHE_DIR, exist_ok=True)
                cache_key = hashlib.sha1(f"{lang}:{text}".encode("utf-8")).hexdigest()
                audio_path = os.path.join(CACHE_DIR, f"{cache_key}.mp3")
                delete_after = False

                if not os.path.exists(audio_path):
                    gTTS(text=text, lang=lang).save(audio_path)
            else:
                with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as temp_file:
                    audio_path = temp_file.name
                delete_after = True

                gTTS(text=text, lang=lang).save(audio_path)

            with self._lock:
                thread = threading.Thread(
                    target=self._play_audio_file,
                    args=(audio_path, delete_after),
                    daemon=True,
                )
                thread.start()
                self._active_threads.append(thread)

        except Exception as e:
            print(f"Text-to-speech error: {e}")