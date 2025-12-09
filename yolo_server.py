#!/usr/bin/env python3

from flask import Flask, request, jsonify
from ultralytics import YOLO
from PIL import Image
import io
import cv2
import numpy as np
import sys

# --- Configuration ---
MODEL_PATH = 'yolo11l.pt'
CONFIDENCE_THRESHOLD = 0.1
TARGET_CLASS = 'banana'

app = Flask(__name__)

try:
    model = YOLO(MODEL_PATH)
    print(f"Successfully loaded YOLO model from {MODEL_PATH}")
except Exception as e:
    print(f"CRITICAL: Failed to load YOLO model '{MODEL_PATH}'. Error: {e}")
    sys.exit(1)

@app.route('/detect', methods=['POST'])
def detect():
    if 'image' not in request.files:
        return jsonify({"error": "No 'image' file part in request"}), 400

    file = request.files['image']

    try:
        img_bytes = file.read()
        img_pil = Image.open(io.BytesIO(img_bytes))
        cv_image = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        cv_image = cv2.resize(cv_image, (640, 360))
    except Exception as e:
        return jsonify({"error": f"Failed to decode image: {e}"}), 400

    # Run YOLO inference
    results = model(cv_image, verbose=False)

    target_object = None
    max_conf = 0.0

    for res in results[0].boxes:
        score = float(res.conf[0])

        if score < CONFIDENCE_THRESHOLD:
            continue

        class_id = int(res.cls[0])
        class_name = model.names[class_id]

        if class_name == TARGET_CLASS and score > max_conf:
            max_conf = score
            box = res.xyxy[0].cpu().numpy().tolist()
            
            target_object = {
                "class_id": class_id,
                "class_name": class_name,
                "confidence": score,
                "box": box
            }

    return jsonify([target_object] if target_object else [])

if __name__ == '__main__':
    print("--- Starting YOLO Server ---")
    print(f"--- Filtering for: {TARGET_CLASS} ---")
    app.run(host='0.0.0.0', port=5000)