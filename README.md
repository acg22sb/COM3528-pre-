
# YOLOv8 ROS Detection Node

## Setup

1.  **Clone this Repository:**

    ```bash
    git clone https://github.com/acg22sb/COM3528-pre-.git
    cd COM3528-pre-
    ```

2.  **Download nano YOLOv8 Model:**
    ```bash
    wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
    ```
-----

## Testing


### Terminal 1: Start `miro-docker`

```bash
cd ../miro-docker
./miro-docker.sh start
```

### Terminal 2: Start `roscore`

```bash
cd ../miro-docker
./miro-docker.sh term
# You are now inside the MiRo container
roscore
```

### Terminal 3: Start MiRo Simulation

```bash
cd ../miro-docker
./miro-docker.sh term
# You are now inside the MiRo container
miro sim
```

*A Gazebo window should appear*

### Terminal 4: Start the YOLO Node (This Project)

```bash
# Navigate to this project's folder
cd ../yolo_docker
docker-compose up --build
```
It will print:

```
[YOLO Node] ROS Master found!
--- [YOLO Node] Ready and entering spin loop ---
```

*This proves it is connected and working. Leave it running.*

### Terminal 5: Verify Detections

```bash
cd ../miro-docker
./miro-docker.sh term
# You are now inside the MiRo container
rostopic echo /yolo/detections
```

*If you see `vision_msgs/Detection2DArray` messages streaming, the system is working*
