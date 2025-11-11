# Base image: Ubuntu 20.04 (for ROS Noetic & Python 3.8)
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install prerequisites
RUN apt-get update && apt-get install -y \
    software-properties-common \
    lsb-release \
    gnupg2 \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS Noetic (Minimal)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list > /dev/null

# --- CHANGED ---
# Install *only* the minimal ROS libraries needed.
# We CANNOT install 'ros-noetic-cv-bridge' from apt.
RUN apt-get update && apt-get install -y \
    python3-rospy \
    ros-noetic-vision-msgs \
    && rm -rf /var/lib/apt/lists/*
    
# Source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"] 

WORKDIR /app

# Install CPU-only torch/torchvision *compatible with Python 3.8*
# We pin older, stable versions to avoid dependency conflicts.
RUN pip3 install --no-cache-dir \
    "torch==1.13.1" \
    "torchvision==0.14.1" \
    "typing-extensions<4.5" \
    --index-url https://download.pytorch.org/whl/cpu

# Install ultralytics (from requirements.txt)
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# --- NEW FIX ---
# Install a modern, pip-based cv_bridge replacement.
# This version will use the NEW numpy and opencv installed by ultralytics,
# completely avoiding the apt/pip conflict.
RUN pip3 install --no-cache-dir cv-bridge-py

# --- REMOVED ---
# The old fix for PyYAML and numpy is no longer needed and would break ultralytics.
# We now use the new versions of these libraries installed by pip.

# Setup Application
COPY yolo_node.py .
COPY yolov8n.pt .
RUN chmod +x yolo_node.py

# This CMD is a fallback, your docker-compose.yaml command will be used instead
CMD ["source /opt/ros/noetic/setup.bash && python3 /app/yolo_node.py"]
