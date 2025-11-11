# Matches miro-docker's OS and provides GPU support
FROM ubuntu20.04

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

# Install ROS Noetic
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list > /dev/null

RUN apt-get update && apt-get install -y \
    python3-rospy \
    ros-noetic-cv-bridge \
    ros-noetic-vision-msgs \
    && rm -rf /var/lib/apt/lists/*
    
# Source ROS environment for all bash shells
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"] 

WORKDIR /app

COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Setup Application 
COPY yolo_node.py .
COPY yolov8n.pt .  

RUN chmod +x yolo_node.py

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /opt/ros/noetic/setup.bash && ./yolo_node.py"]
