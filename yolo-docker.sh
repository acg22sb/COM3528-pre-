#!/usr/bin/env bash
#
# yolo-docker.sh — Helper script for the COM3528-pre YOLO node
#
# Purpose:
#   A simple helper script to manage the YOLO node Docker container.
#   This version *knows* 'docker compose' (v2) is installed.
#
# Usage:
#   ./yolo-docker.sh <command>
#
# Commands:
#   start   - Build and start the container in the background.
#   stop    - Stop and remove the container.
#   term    - Open an interactive shell (bash) inside the running container.
#   logs    - View the live logs from the container.

set -euo pipefail

# This must match the name of the service in your 'docker-compose.yaml' file.
SERVICE_NAME="yolo_node"

# Wrapper for the compose command.
# We hardcode 'docker compose' (v2) because we know it works.
dc() {
    docker compose "$@"
}

# --- Show usage ---
show_usage() {
    awk '/^# Usage:/,/^# ---/' "$0" | grep -vE '(Usage:|---)' | sed 's/^#//'
}

# --- Commands ---

# Start container
start() {
    echo "🚀 Building and starting '$SERVICE_NAME' container..."
    dc up --build -d
    echo "✅ Container is running."
    echo "   Use './yolo-docker.sh logs' to see the output."
}

# Stop container
stop() {
    echo "🛑 Stopping and removing '$SERVICE_NAME' container..."
    dc down --remove-orphans
    echo "✅ Container stopped and removed."
}

# Attach shell
term() {
    echo "🔗 Attaching an interactive shell to '$SERVICE_NAME'..."
    echo "   (Type 'exit' or press Ctrl+D to leave)"
    dc exec "$SERVICE_NAME" /bin/bash
}

# View logs
logs() {
    echo "📋 Tailing logs for '$SERVICE_NAME'..."
    echo "   (Press Ctrl+C to stop)"
    dc logs -f --tail 100
}

# --- Command dispatcher ---
COMMAND=${1:-}

case "$COMMAND" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    term)
        term
        ;;
    logs)
        logs
        ;;
    *)
        show_usage
        exit 1
        ;;
esac
