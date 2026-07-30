#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
python3 tools/mock_devices.py &
MOCK_PID=$!
trap 'kill "$MOCK_PID" 2>/dev/null || true' EXIT
sleep 1
python3 app.py --config config.mock.json
