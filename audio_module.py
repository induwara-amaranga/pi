import os
import tempfile
from gtts import gTTS
import pygame


class AudioModule:
    def __init__(self):
        self.enabled = False
        try:
            pygame.mixer.init()
            self.enabled = True
        except Exception as e:
            print(f"Audio disabled: {e}")

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

            pygame.mixer.music.load(temp_audio_path)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

            pygame.mixer.music.unload()

            if os.path.exists(temp_audio_path):
                os.remove(temp_audio_path)

        except Exception as e:
            print(f"Text-to-speech error: {e}")