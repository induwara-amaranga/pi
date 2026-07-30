import speech_recognition as sr
import numpy as np
import noisereduce as nr
from scipy.signal import butter, filtfilt

def bandpass_filter(audio_data, sample_rate, low=200, high=3800):
    """Keep only speech frequencies"""
    nyq = sample_rate / 2
    b, a = butter(4, [low/nyq, high/nyq], btype='band')
    return filtfilt(b, a, audio_data)

def preprocess_audio(audio: sr.AudioData) -> sr.AudioData:
    sample_rate = audio.sample_rate
    raw = np.frombuffer(audio.get_raw_data(convert_rate=sample_rate, convert_width=2), dtype=np.int16).astype(np.float32)

    # 1. Noise reduction (stationary=True works better for USB mics)
    reduced = nr.reduce_noise(y=raw, sr=sample_rate, stationary=True, prop_decrease=0.8)

    # 2. Bandpass filter
    filtered = bandpass_filter(reduced, sample_rate)

    # 3. Normalize
    max_val = np.max(np.abs(filtered))
    if max_val > 0:
        filtered = filtered / max_val * 32767 * 0.9

    return sr.AudioData(filtered.astype(np.int16).tobytes(), sample_rate, 2)


# --- Find your USB mic index ---
# Run once to check: python3 -c "import speech_recognition as sr; print(sr.Microphone.list_microphone_names())"
USB_MIC_INDEX = 1  # replace with your device index e.g. 2

recognizer = sr.Recognizer()
recognizer.dynamic_energy_threshold = True
recognizer.pause_threshold = 0.8
recognizer.phrase_time_limit = 10

with sr.Microphone(device_index=USB_MIC_INDEX) as source:
    print("Calibrating...")
    recognizer.adjust_for_ambient_noise(source, duration=3)
    print(f"Energy threshold set to: {recognizer.energy_threshold:.1f}")
    print("Listening...")
    audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)

audio = preprocess_audio(audio)

try:
    text = recognizer.recognize_google(audio, language="en-US")
    print("You said:", text)
except sr.UnknownValueError:
    print("Could not understand.")
except sr.RequestError as e:
    print(f"API error: {e}")