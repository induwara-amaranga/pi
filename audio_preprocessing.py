import numpy as np
import noisereduce as nr
import speech_recognition as sr
from scipy.signal import butter, filtfilt


def bandpass_filter(audio_data, sample_rate, low=200, high=3800):
    """Keep only speech frequencies."""
    nyq = sample_rate / 2
    b, a = butter(4, [low / nyq, high / nyq], btype='band')
    return filtfilt(b, a, audio_data)


def preprocess_audio(audio: sr.AudioData) -> sr.AudioData:
    """Noise-reduce, bandpass-filter, and normalize captured audio before recognition."""
    sample_rate = audio.sample_rate
    raw = np.frombuffer(
        audio.get_raw_data(convert_rate=sample_rate, convert_width=2), dtype=np.int16
    ).astype(np.float32)

    # Noise reduction (stationary=True works better for USB mics)
    reduced = nr.reduce_noise(y=raw, sr=sample_rate, stationary=True, prop_decrease=0.8)

    # Bandpass filter to keep only speech frequencies
    filtered = bandpass_filter(reduced, sample_rate)

    # Normalize
    max_val = np.max(np.abs(filtered))
    if max_val > 0:
        filtered = filtered / max_val * 32767 * 0.9

    return sr.AudioData(filtered.astype(np.int16).tobytes(), sample_rate, 2)
