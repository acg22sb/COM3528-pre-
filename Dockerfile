FROM python:3.10-slim

RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# This avoids the massive NVIDIA downloads
RUN pip3 install --no-cache-dir \
    "torch==2.3.1" \
    "torchvision==0.18.1" \
    --index-url https://download.pytorch.org/whl/cpu
    
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

COPY yolo_server.py .
COPY yolov8n.pt .
COPY testscript.py .
COPY testvscript.py .
COPY testvideo.mp4 .

EXPOSE 5000

CMD ["python3", "/app/yolo_server.py"]
