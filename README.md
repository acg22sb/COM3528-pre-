
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
'''bash
pip3 install numpy requests opencv-python-headless
'''

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
./yolo-docker.sh start
./yolo-docker.sh term
```
## Example Json for a man and potted plant
'''
Response JSON: [
  {
    "box": [
      900.274658203125,
      529.3267822265625,
      2838.866455078125,
      2135.089599609375
    ],
    "class_id": 0,
    "class_name": "person",
    "confidence": 0.9399809241294861
  },
  {
    "box": [
      1.75872802734375,
      12.514892578125,
      1277.793212890625,
      2119.076904296875
    ],
    "class_id": 58,
    "class_name": "potted plant",
    "confidence": 0.5024356842041016
  }
]
'''
