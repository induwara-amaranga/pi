import os
import tempfile
import threading
from gtts import gTTS
import pygame


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

    def _play_audio_file(self, temp_audio_path: str) -> None:
        try:
            pygame.mixer.music.load(temp_audio_path)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

            pygame.mixer.music.unload()
        finally:
            if os.path.exists(temp_audio_path):
                os.remove(temp_audio_path)

    def speak_text(self, text: str, lang: str = "en"):
        """
        Convert text to speech and play through speaker.
        """
        if not self.enabled:
            print(f"AURA speaking (audio disabled): {text}")
            return

        try:
            print(f"AURA speaking: {text}")

            tts = gTTS(text=text, lang=lang)

            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as temp_file:
                temp_audio_path = temp_file.name

            tts.save(temp_audio_path)

            with self._lock:
                thread = threading.Thread(
                    target=self._play_audio_file,
                    args=(temp_audio_path,),
                    daemon=True,
                )
                thread.start()
                self._active_threads.append(thread)

        except Exception as e:
            print(f"Text-to-speech error: {e}")