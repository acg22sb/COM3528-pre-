import requests
import numpy as np
import cv2
import json

# --- Configuration ---
# This is the address of your running container
# (it's 'localhost' because port 5000 is mapped)
SERVER_URL = "http://127.0.0.1:5000/detect"

def create_dummy_image():
    """Creates a blank 640x480 black image."""
    # Create a black numpy array
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    return img

def test_yolo_server():
    """
    Sends a dummy image to the YOLO server and prints the response.
    """
    print(f"Attempting to connect to server at {SERVER_URL}...")
    
    # 1. Create a dummy image
    image_to_send = create_dummy_image()
    
    # 2. Encode the image as a JPEG in memory
    # We use cv2.imencode to turn the numpy array into bytes
    is_success, img_buffer = cv2.imencode(".jpg", image_to_send)
    if not is_success:
        print("Error: Could not encode image to JPG.")
        return

    # 3. Prepare the file for the POST request
    # This format ('image', (filename, file_bytes, content_type))
    # is what the server's 'request.files['image']' expects.
    files_to_send = {
        'image': ('dummy_image.jpg', img_buffer.tobytes(), 'image/jpeg')
    }

    try:
        # 4. Send the request
        response = requests.post(SERVER_URL, files=files_to_send, timeout=10)
        
        # 5. Print the results
        print(f"\n--- Test Results ---")
        print(f"Server Status Code: {response.status_code}")
        
        if response.status_code == 200:
            print("Connection SUCCESSFUL.")
            detections = response.json()
            print(f"Detections Found: {len(detections)}")
            print(f"Response JSON: {json.dumps(detections, indent=2)}")
        else:
            print(f"Server returned an error: {response.text}")

    except requests.exceptions.ConnectionError:
        print("\n--- TEST FAILED ---")
        print("ConnectionError: Could not connect to the server.")
        print("Please make sure the YOLO container is running:")
        print("  ./yolo-docker.sh start")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

if __name__ == "__main__":
    test_yolo_server()
