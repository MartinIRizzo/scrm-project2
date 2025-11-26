#!/bin/bash

echo "🛑 Stopping SCRM project 2 Environment..."
docker compose down

echo "✅ Environment stopped."
echo ""
echo "Your work in ros2_ws/ and shared/ is preserved."
echo "Run ./run.sh to start again."
