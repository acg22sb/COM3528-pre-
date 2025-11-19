import requests
import cv2
import numpy as np
import sys
import os
import json # Import json for pretty printing

# --- Configuration ---
SERVER_URL = "http://127.0.0.1:5000/detect"
VIDEO_FILE = "testvideo.mp4"
OUTPUT_IMAGE = "last_test_frame.jpg"
# ---------------------

def send_frame_to_server(frame):
    """
    Encodes a single video frame and sends it to the YOLO server.
    """
    print(f"Attempting to connect to server at {SERVER_URL}...")
    try:
        # Encode the frame as a .jpg in memory
        is_success, buffer = cv2.imencode(".jpg", frame)
        if not is_success:
            print("--- TEST FAILED ---")
            print("Error: Could not encode frame to JPG format.")
            return

        # This function is now correct
        # Format the image as a 'multipart/form-data' file upload.
        # This matches what the server (request.files['image']) expects.
        files_to_send = {
            'image': ('video_frame.jpg', buffer.tobytes(), 'image/jpeg')
        }
        
        # Send the request with the 'files' parameter, not 'data'
        response = requests.post(SERVER_URL, files=files_to_send, timeout=10)
        # -----------------------

        # Check for a successful response
        response.raise_for_status() 

        # --- Test Results ---
        print("\n--- Test Results ---")
        print(f"Server Status Code: {response.status_code}")
        print("Connection SUCCESSFUL.")
        
        detections = response.json()
        print(f"Detections Found: {len(detections)}")
        # Pretty-print the JSON response
        print(f"Response JSON: {json.dumps(detections, indent=2)}")

    except requests.exceptions.ConnectionError:
        print("\n--- TEST FAILED ---")
        print("ConnectionError: Could not connect to the server.")
        print("Please make sure the YOLO container is running:")
        print("  ./yolo-docker.sh start")
        print("And check the logs:")
        print("  ./yolo-docker.sh logs")
    except requests.exceptions.HTTPError as e:
        print(f"\n--- TEST FAILED ---")
        print(f"HTTP Error: {e.response.status_code} {e.response.reason}")
        print(f"Server response: {e.response.text}")
    except Exception as e:
        print(f"\n--- TEST FAILED ---")
        print(f"An unexpected error occurred: {e}")

def main():
    # --- 1. Get Frame Number ---
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} [frame_number]")
        print(f"Example: python3 {sys.argv[0]} 150") # Fixed example
        sys.exit(1)
        
    try:
        frame_number = int(sys.argv[1])
    except ValueError:
        print(f"Error: '{sys.argv[1]}' is not a valid frame number.")
        sys.exit(1)

    # --- 2. Check if video file exists ---
    # --- THIS IS THE FIX ---
    # Look for the video file in the current directory, not in "/app/"
    video_path = VIDEO_FILE 
    if not os.path.exists(video_path):
        print(f"Error: Video file not found: {video_path}")
        print("Please make sure the video is in your project folder.")
        sys.exit(1)
    # -----------------------

    # --- 3. Open Video and Seek to Frame ---
    cap = cv2.VideoCapture(video_path) # Use the corrected path
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        
    # Set the video to the desired frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
    
    ret, frame = cap.read()
    
    if not ret:
        print(f"Error: Could not read frame {frame_number}.")
        print("The video might be shorter than that. Try a smaller number.")
        cap.release()
        sys.exit(1)
    
    cap.release()
    
    # --- 4. Save and Send ---
    # Save a copy so you can see what was tested
    cv2.imwrite(OUTPUT_IMAGE, frame)
    print(f"Saved frame {frame_number} to {OUTPUT_IMAGE}")

    # Send the frame to the server
    send_frame_to_server(frame)

if __name__ == "__main__":
    main()
