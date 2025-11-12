#!/usr/bin/env python3

from flask import Flask, request, jsonify
from ultralytics import YOLO
from PIL import Image
import io
import cv2
import numpy as np
import sys

# --- Configuration ---
MODEL_PATH = 'yolov8n.pt'
CONFIDENCE_THRESHOLD = 0.5

app = Flask(__name__)

try:
    model = YOLO(MODEL_PATH)
    print(f"Successfully loaded YOLO model from {MODEL_PATH}")
except Exception as e:
    print(f"CRITICAL: Failed to load YOLO model '{MODEL_PATH}'. Error: {e}")
    sys.exit(1)

@app.route('/detect', methods=['POST'])
def detect():
    """
    Handle an image upload, run YOLO detection, and return JSON results.
    Expects a POST request with a file part named 'image'.
    """
    if 'image' not in request.files:
        return jsonify({"error": "No 'image' file part in request"}), 400

    file = request.files['image']

    try:
        img_bytes = file.read()

        # Convert bytes to a PIL Image
        img_pil = Image.open(io.BytesIO(img_bytes))

        # Convert PIL Image to an OpenCV (numpy) array in BGR format
        cv_image = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

    except Exception as e:
        return jsonify({"error": f"Failed to decode image: {e}"}), 400

    # Run YOLO inference
    results = model(cv_image, verbose=False)

    # Format results into a JSON list
    detections = []
    for res in results[0].boxes:
        score = float(res.conf[0])

        # Apply confidence threshold
        if score < CONFIDENCE_THRESHOLD:
            continue

        class_id = int(res.cls[0])
        class_name = model.names[class_id]
        box = res.xyxy[0].cpu().numpy().tolist() # [x1, y1, x2, y2]

        detections.append({
            "class_id": class_id,
            "class_name": class_name,
            "confidence": score,
            "box": box
        })

    # Return the JSON response, no images
    return jsonify(detections)

if __name__ == '__main__':
    print("--- Starting YOLO Flask Server ---")
    app.run(host='0.0.0.0', port=5000)
