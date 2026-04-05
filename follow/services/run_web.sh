#!/bin/bash
set -e

echo "========== START WEBRTC SERVER =========="

cd ~/follow/web/backend
source ~/follow/web_env/bin/activate

uvicorn app:app --host 0.0.0.0 --port 8000