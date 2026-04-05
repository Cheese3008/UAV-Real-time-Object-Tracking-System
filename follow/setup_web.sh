#!/bin/bash

set -e

echo "========== SETUP WEB ENV =========="

cd ~/follow

if [ ! -d "web_env" ]; then
    echo "[INFO] Creating virtual environment..."
    python3 -m venv web_env
else
    echo "[INFO] web_env already exists"
fi

echo "[INFO] Activating virtual environment..."
source web_env/bin/activate

echo "[INFO] Upgrading pip, setuptools, wheel..."
pip install --upgrade pip setuptools wheel

echo "[INFO] Installing Python packages..."
pip install \
    fastapi \
    "uvicorn[standard]" \
    opencv-python \
    "numpy<2" \
    pyyaml \
    aiortc \
    av

echo "[INFO] Verifying installations..."
python - <<EOF
import yaml
import numpy
import websockets
print("[OK] yaml loaded")
print("[OK] numpy version:", numpy.__version__)
print("[OK] websockets loaded")
EOF

echo "========== SETUP DONE =========="