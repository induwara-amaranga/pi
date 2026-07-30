import speech_recognition as sr

from audio_preprocessing import preprocess_audio

USB_MIC_INDEX = 1  # replace with your device index

recognizer = sr.Recognizer()
recognizer.dynamic_energy_threshold = True
recognizer.pause_threshold = 0.8
recognizer.phrase_time_limit = 10

try:
    with sr.Microphone(device_index=USB_MIC_INDEX) as source:
        print("Calibrating...")
        recognizer.adjust_for_ambient_noise(source, duration=3)
        print(f"Energy threshold set to: {recognizer.energy_threshold:.1f}")
        print("Listening...")
        raw_audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)
except sr.WaitTimeoutError:
    print("No voice detected.")
    exit()

# --- Save RAW audio (before preprocessing) ---
with open("debug_raw.wav", "wb") as f:
    f.write(raw_audio.get_wav_data())
print("Saved debug_raw.wav")

# --- Preprocess ---
audio = preprocess_audio(raw_audio)

# --- Save PREPROCESSED audio (after noise reduction/filter/normalize) ---
with open("debug_preprocessed.wav", "wb") as f:
    f.write(audio.get_wav_data())
print("Saved debug_preprocessed.wav")

# --- Recognize ---
try:
    text = recognizer.recognize_google(audio, language="en-US")
    print("You said:", text)
except sr.UnknownValueError:
    print("Could not understand.")
except sr.RequestError as e:
    print(f"API error: {e}")