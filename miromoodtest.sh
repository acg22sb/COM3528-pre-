#!/bin/bash

cleanup() {
    echo "Shutting down MiRo system..."
    kill $MOOD_PID
    exit
}

trap cleanup SIGINT

echo "=========================================================================="
echo "   STARTING MIRO OBJECT PERMANCE - TESTING HUMAN RECOGNITION WITH MOOD"
echo "=========================================================================="

echo "[1/3] Ensuring YOLO Server is ready..."
./yolo-docker.sh start &> /dev/null

sleep 5 

echo "[2/3] Starting Mood Controller..."
python3 mood_controller.py &
MOOD_PID=$!

sleep 2

echo "[3/3] Starting Master Human Tracker..."
echo "--> System is Live! Press Ctrl+C to quit."
echo "----------------------------------------"
python3 masternode.py

# When masternode closes, run cleanup
cleanup
