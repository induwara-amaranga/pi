#!/usr/bin/env bash
#
# install_pi.sh - One-shot setup for the AURA Pi controller on Raspberry Pi OS.
#
# Installs system packages, enables the hardware interfaces the code needs
# (I2C for OLED/servo driver, camera for face tracking), sets group
# permissions, creates a Python venv, and installs everything in
# requirements.txt (including Adafruit's CircuitPython servo stack).
#
# Usage:
#   chmod +x install_pi.sh
#   ./install_pi.sh
#
# Re-running is safe; every step is idempotent.

set -euo pipefail

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/venv"
REBOOT_REQUIRED=0

info()  { printf '\033[1;34m[INFO]\033[0m %s\n' "$1"; }
warn()  { printf '\033[1;33m[WARN]\033[0m %s\n' "$1"; }
ok()    { printf '\033[1;32m[ OK ]\033[0m %s\n' "$1"; }
die()   { printf '\033[1;31m[FAIL]\033[0m %s\n' "$1"; exit 1; }

if [[ "$(uname -s)" != "Linux" ]]; then
    die "This script targets Raspberry Pi OS (Linux). Run it on the Pi itself."
fi

if [[ "${EUID}" -eq 0 ]]; then
    die "Don't run this as root/sudo. It will call sudo itself where needed."
fi

command -v sudo >/dev/null 2>&1 || die "sudo is required but not found."

# ---------------------------------------------------------------------------
# OS detection
# ---------------------------------------------------------------------------
# Package names and boot config location differ between Raspberry Pi OS
# Bullseye and Bookworm (and Bookworm itself renamed libcamera-apps ->
# rpicam-apps partway through its life). Detect rather than assume so this
# works on whatever image is actually on the Pi 4B.

OS_CODENAME="unknown"
if [[ -f /etc/os-release ]]; then
    OS_CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME:-unknown}")"
fi

BOOT_CONFIG=""
for candidate in /boot/firmware/config.txt /boot/config.txt; do
    if [[ -f "${candidate}" ]]; then
        BOOT_CONFIG="${candidate}"
        break
    fi
done

info "Detected Raspberry Pi OS codename: ${OS_CODENAME} (boot config: ${BOOT_CONFIG:-not found})"

# ---------------------------------------------------------------------------
# 1. System packages
# ---------------------------------------------------------------------------

info "Updating apt package index..."
sudo apt-get update -y

info "Installing core system dependencies..."
# python3-libgpiod pulls in whichever libgpiod runtime SONAME the OS ships
# (libgpiod2 on Bullseye/Bookworm, libgpiod3 on Trixie) - don't pin it
# explicitly. Likewise libatlas-base-dev was dropped from Debian for Trixie
# (Atlas is obsolete); libopenblas-dev is the BLAS/LAPACK provider used
# instead and is available on all three releases.
sudo apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    i2c-tools \
    python3-libgpiod \
    libopenblas-dev \
    libopenjp2-7 \
    libjpeg-dev \
    zlib1g-dev \
    portaudio19-dev \
    libasound2-dev \
    flac \
    ffmpeg \
    libsdl2-2.0-0 \
    libsdl2-mixer-2.0-0 \
    libsdl2-image-2.0-0 \
    libsdl2-ttf-2.0-0

ok "Core system packages installed."

info "Installing camera stack (picamera2 + libcamera bindings)..."
if ! sudo apt-get install -y --no-install-recommends python3-picamera2 python3-libcamera; then
    warn "python3-picamera2/python3-libcamera not available via apt on this OS image."
    warn "Run 'sudo apt-get update && sudo apt-get full-upgrade' to get a current Raspberry Pi OS image, then re-run this script."
fi

# CLI camera tools are only for diagnostics (rpicam-hello / libcamera-hello);
# the app itself only needs the python3-picamera2 bindings above. Package
# name changed from libcamera-apps to rpicam-apps on newer Bookworm images,
# so try each candidate until one installs.
info "Installing camera diagnostic CLI tools..."
sudo apt-get install -y --no-install-recommends rpicam-apps-lite \
    || sudo apt-get install -y --no-install-recommends libcamera-apps-lite \
    || sudo apt-get install -y --no-install-recommends rpicam-apps \
    || sudo apt-get install -y --no-install-recommends libcamera-apps \
    || warn "No rpicam-apps/libcamera-apps package found - not fatal, this only affects the rpicam-hello/libcamera-hello test commands."

# ---------------------------------------------------------------------------
# 2. Enable I2C and camera interfaces
# ---------------------------------------------------------------------------
# raspi-config's nonint mode lets us flip interface toggles from a script.
# do_i2c/do_camera return 0 (enabled) if a config change was made. On recent
# Bookworm images the camera is always auto-detected and do_camera may not
# exist as an option, so that failure is not treated as fatal.

if command -v raspi-config >/dev/null 2>&1; then
    info "Enabling I2C interface (for OLED displays + PCA9685 servo driver)..."
    if sudo raspi-config nonint do_i2c 0; then
        REBOOT_REQUIRED=1
    fi

    if sudo raspi-config nonint do_camera 0 2>/dev/null; then
        REBOOT_REQUIRED=1
        ok "Camera interface enabled via raspi-config."
    else
        info "raspi-config has no separate camera toggle on this image (camera is auto-detected via config.txt on Bookworm)."
    fi
else
    warn "raspi-config not found - skipping automatic interface enablement."
    warn "Manually enable I2C (and, on Bullseye, Camera) via: sudo raspi-config -> Interface Options"
fi

# Belt-and-braces: make sure camera_auto_detect=1 is set in the boot config.
# This is the default on Bookworm and harmless on Bullseye's libcamera stack.
if [[ -n "${BOOT_CONFIG}" ]]; then
    if grep -q '^camera_auto_detect=' "${BOOT_CONFIG}"; then
        sudo sed -i 's/^camera_auto_detect=.*/camera_auto_detect=1/' "${BOOT_CONFIG}"
    else
        echo 'camera_auto_detect=1' | sudo tee -a "${BOOT_CONFIG}" >/dev/null
    fi
fi

# Make sure the i2c-dev kernel module is loaded now (not just on next boot).
sudo modprobe i2c-dev 2>/dev/null || true

if ! grep -q '^i2c-dev' /etc/modules 2>/dev/null; then
    echo 'i2c-dev' | sudo tee -a /etc/modules >/dev/null
fi

# ---------------------------------------------------------------------------
# 3. Group membership for GPIO / I2C / camera / audio access without sudo
# ---------------------------------------------------------------------------

TARGET_USER="${SUDO_USER:-$USER}"
info "Ensuring user '${TARGET_USER}' can access GPIO/I2C/camera/audio devices..."

for grp in gpio i2c spi video audio dialout render; do
    if getent group "$grp" >/dev/null 2>&1; then
        if ! id -nG "$TARGET_USER" | grep -qw "$grp"; then
            sudo usermod -aG "$grp" "$TARGET_USER"
            info "Added ${TARGET_USER} to group '${grp}' (takes effect after re-login/reboot)."
            REBOOT_REQUIRED=1
        fi
    fi
done

ok "Group membership configured."

# ---------------------------------------------------------------------------
# 4. Python virtual environment
# ---------------------------------------------------------------------------
# --system-site-packages so the apt-installed picamera2/libcamera bindings
# (which are not reliably pip-installable) are visible inside the venv.

if [[ ! -d "${VENV_DIR}" ]]; then
    info "Creating virtual environment at ${VENV_DIR} (with system site packages)..."
    python3 -m venv --system-site-packages "${VENV_DIR}"
else
    info "Virtual environment already exists at ${VENV_DIR}."
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

info "Upgrading pip/setuptools/wheel..."
pip install --upgrade pip setuptools wheel

# ---------------------------------------------------------------------------
# 5. Python dependencies (requirements.txt + Adafruit servo stack)
# ---------------------------------------------------------------------------

info "Installing Python requirements from requirements.txt..."
pip install -r "${SCRIPT_DIR}/requirements.txt"

# adafruit-circuitpython-servokit and adafruit-blinka are already listed in
# requirements.txt, but Blinka ships a one-time platform detection step that
# benefits from being run explicitly right after install.
info "Verifying Adafruit Blinka platform detection (servo/I2C support)..."
python3 -c "import board; print('Blinka board module OK:', board.board_id if hasattr(board, \"board_id\") else 'detected')" \
    || warn "Blinka could not detect the board automatically. If servo control fails, run: pip install --upgrade adafruit-blinka and re-check ${BOOT_CONFIG:-your boot config.txt}."

deactivate

ok "Python dependencies installed into ${VENV_DIR}."

# ---------------------------------------------------------------------------
# 6. Project environment file
# ---------------------------------------------------------------------------

if [[ ! -f "${SCRIPT_DIR}/.env" && -f "${SCRIPT_DIR}/.env.example" ]]; then
    cp "${SCRIPT_DIR}/.env.example" "${SCRIPT_DIR}/.env"
    warn "Created .env from .env.example - fill in your real API keys before running the app."
else
    info ".env already present, leaving it untouched."
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------

echo
ok "Setup complete."
echo "  Activate the environment with:  source ${VENV_DIR}/bin/activate"
echo "  Run the controller with:        python main_controller.py"
echo "  Check I2C devices with:         i2cdetect -y 1"

if [[ "${REBOOT_REQUIRED}" -eq 1 ]]; then
    echo
    warn "I2C/camera were enabled or your user's groups changed - a reboot is required for these to take effect."
    warn "Run: sudo reboot"
fi
