#!/usr/bin/env python3

import rospy
import cv2
import sys
import time
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLONode:
    
    def __init__(self):
        # --- Parameters ---
        self.model_path = rospy.get_param('~model_path', 'yolov8n.pt')
        self.image_topic = rospy.get_param('~image_topic', '/miro/sensors/camr/compressed')
        self.detection_topic = rospy.get_param('~detection_topic', '/yolo/detections')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)

        # --- Load YOLO Model ---
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            rospy.logerr(f"CRITICAL: Failed to load YOLO model '{self.model_path}'. Error: {e}")
            sys.exit(1)

        # --- ROS Setup ---
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            self.image_topic, 
            CompressedImage, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        self.detection_pub = rospy.Publisher(self.detection_topic, Detection2DArray, queue_size=10)
        
        rospy.loginfo(f"YOLO node configured. Subscribing to {self.image_topic}.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting compressed image: {e}")
            return

        results = self.model(cv_image, verbose=False)
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header 

        for res in results[0].boxes:
            score = float(res.conf[0])
            if score < self.confidence_threshold:
                continue

            class_id = int(res.cls[0])
            x1, y1, x2, y2 = res.xyxy[0]

            detection_msg = Detection2D()
            detection_msg.header = msg.header
            
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(class_id)
            hypothesis.score = score
            detection_msg.results.append(hypothesis)

            detection_msg.bbox.center.x = (x1 + x2) / 2
            detection_msg.bbox.center.y = (y1 + y2) / 2
            detection_msg.bbox.size_x = x2 - x1
            detection_msg.bbox.size_y = y2 - y1

            detection_array_msg.detections.append(detection_msg)

        if len(detection_array_msg.detections) > 0:
            self.detection_pub.publish(detection_array_msg)

    def run(self):
        rospy.loginfo("--- [YOLO Node] Ready and entering spin loop ---")
        rospy.spin()
        rospy.loginfo("--- [YOLO Node] Exiting spin loop ---")

if __name__ == '__main__':
    try:
        rospy.loginfo("[YOLO Node] Waiting for ROS Master...")
        
        # rospy.init_node() WILL block and wait for roscore to be available.
        # The separate wait_for_master call was incorrect and unnecessary.
        rospy.init_node('yolo_detector', anonymous=True)
        
        rospy.loginfo("[YOLO Node] ROS Master found and node initialized!")

        node = YOLONode()
        node.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("[YOLO Node] Shutting down due to ROS Interrupt.")
    except rospy.ROSException as e:
        # This will catch a failure from init_node (e.g., if roscore is never found)
        rospy.logerr(f"[YOLO Node] CRITICAL: Could not initialize node: {e}")
        sys.exit(1) # Force an error code 1
    except Exception as e:
        rospy.logerr(f"[YOLO Node] An unhandled exception occurred: {e}")
        sys.exit(1)
