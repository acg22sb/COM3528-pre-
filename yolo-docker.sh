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
#   build   - (Re)Builds your Docker image. Run this after changing Dockerfile or requirements.txt.
#   start   - Starts the container. Creates it if it doesn't exist. Does NOT rebuild.
#   stop    - Stops the container without removing it. (Fast!)
#   down    - Stops AND removes the container. (Use this to clean up)
#   term    - Open an interactive shell (bash) inside the running container.
#   logs    - View the live logs from the container.

set -euo pipefail

# This must match the 'service' name in your 'docker-compose.yaml'
SERVICE_NAME="yolo"
# This must match the 'container_name' in your 'docker-compose.yaml'
CONTAINER_NAME="yolo_detector"

# Wrapper for the compose command.
# We hardcode 'docker compose' (v2) because we know it works.
dc() {
    docker compose "$@"
}

# --- Show usage ---
show_usage() {
    # Updated to show the new command list
    awk '/^# Commands:/,/^# ---/' "$0" | grep -vE '(Commands:|---)' | sed 's/^#//'
}

# --- Commands ---

# Build container image
build() {
    echo "🚀 Building image for '$SERVICE_NAME' container..."
    dc build
    echo "✅ Image built."
}

# Start container (without rebuilding)
start() {
    echo "🚀 Starting '$SERVICE_NAME' container..."
    # 'up -d' will create the container if it's missing,
    # or start the existing one if it's just stopped.
    # We removed '--build' so it's fast.
    dc up -d
    echo "✅ Container is running."
    echo "   Use './yolo-docker.sh logs' to see the output."
}

# Stop container (without removing)
stop() {
    echo "🛑 Stopping '$SERVICE_NAME' container (it will still exist)..."
    # 'stop' just pauses the container
    dc stop
    echo "✅ Container stopped."
}

# Stop AND remove container
down() {
    echo "🔥 Stopping AND removing '$SERVICE_NAME' container..."
    # 'down' is the old 'stop' command
    dc down --remove-orphans
    echo "✅ Container stopped and removed."
}

# Attach shell
term() {
    echo "🔗 Attaching an interactive shell to '$CONTAINER_NAME'..."
    echo "   (Type 'exit' or press Ctrl+D to leave)"
    # Use 'docker exec' directly with the container_name, as it's more reliable
    docker exec -it "$CONTAINER_NAME" /bin/bash
}

# View logs
logs() {
    echo "📋 Tailing logs for '$SERVICE_NAME'..."
    echo "   (Press Ctrl+C to stop)"
    dc logs -f --tail 100 "$SERVICE_NAME"
}

# --- Command dispatcher ---
COMMAND=${1:-}

case "$COMMAND" in
    build)
        build
        ;;
    start)
        start
        ;;
    stop)
        stop
        ;;
    down)
        down
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
