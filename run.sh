#!/bin/bash
cd ~/Desktop/pi || { echo "Directory not found"; exit 1; }
source venv/bin/activate
python main_controller.py