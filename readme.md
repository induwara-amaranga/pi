# 🤖 AURA Pi Controller

This module handles the **robot-side intelligence** of the AURA (Automated Urban Restaurant Assistant) system.
It enables **voice interaction, AI responses, and hardware control** on the Raspberry Pi.

---

## 🚀 Features

* 🎤 Voice input using microphone (SpeechRecognition)
* 🧠 AI responses using Google Gemini
* 🔊 Text-to-speech output (gTTS + pygame)
* 🗣️ Wake word detection ("Hi AURA")
* 📡 MQTT communication support (for backend integration)
* ⚙️ Modular design for hardware components (servo, OLED, touch, camera)

---

## 📁 Project Structure

```
pi_controller/
│
├── main_controller.py        # Entry point (runs the system)
├── config.py                # Configuration settings
│
├── audio_module.py          # Text-to-speech (speaker)
├── voice_module.py          # Speech recognition + Gemini AI
├── mqtt_client.py           # MQTT communication
├── servo_module.py          # Servo control (pan/tilt)
├── oled_module.py           # OLED display
├── touch_module.py          # Touch input handling
├── face_module.py           # Face tracking (OpenCV / DeepFace)
│
├── requirements.txt         # Python dependencies
├── .env.example             # Environment variable template
├── .gitignore               # Ignore sensitive files
└── README.md                # Documentation
```

---

## ⚙️ Setup Instructions

### 1. Create Virtual Environment (recommended)

```bash
python -m venv venv
venv\Scripts\activate   # Windows
# OR
source venv/bin/activate  # Linux / Raspberry Pi
```

---

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

---

### 3. Setup Environment Variables

Create a `.env` file inside `pi_controller/`:

```env
GEMINI_API_KEY=your_real_gemini_api_key
MQTT_BROKER=localhost
MQTT_PORT=1883
```

⚠️ Do NOT commit `.env` to GitHub.

---

### 4. Run the System

```bash
python main_controller.py
```

---

## 🎤 Usage

1. Start the program
2. Say wake word:
   **"Hi AURA" / "Hey AURA"**
3. Ask a question or give a command
4. AURA responds via speaker

Example:

```
User: Hi AURA
User: What is today's special?
AURA: Today's special is grilled chicken with garlic sauce.
```

---

## 🔐 Environment Files

| File           | Purpose                   |
| -------------- | ------------------------- |
| `.env`         | Real secrets (NOT pushed) |
| `.env.example` | Template for others       |

---

## 📡 MQTT Integration (Future)

This module is designed to connect with:

* Spring Boot Backend
* Mosquitto Broker (Docker)

Example use:

* Send orders
* Receive commands
* Sync robot with server

---

## ⚠️ Notes

* Microphone and speaker must be properly configured
* On Raspberry Pi, install audio drivers if needed
* Some modules (GPIO, servo) require actual hardware

---

## 🧠 Technologies Used

* Python 3.12
* SpeechRecognition
* Google Gemini API
* gTTS
* pygame
* OpenCV / DeepFace (optional)
* MQTT (paho-mqtt)

---

## 👨‍💻 Author

Part of **AURA - Automated Urban Restaurant Assistant**
University of Peradeniya - Computer Engineering

---
