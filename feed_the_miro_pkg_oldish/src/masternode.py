#!/usr/bin/env python3
# Run Mood Controller then this
import rospy
import os
import cv2
import math

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Float32

from camera_reader import MiRoCameraReader
from testvscript import send_frame_to_server

TARGET_CLASS = "person" 

class MiroDepthCalculator:
    def __init__(self):
        # Constants from Ling 2019 ROBIO Paper & MiRo Hardware Specs
        self.BASELINE = 0.104       # Distance between eyes in meters
        self.FOCAL_LENGTH = 184.75  # Focal length in pixels
        
        # Camera Resolution (640x360 as per paper)
        self.IMG_WIDTH = 640
        self.IMG_HEIGHT = 360
        
        self.cx = self.IMG_WIDTH / 2
        self.cy = self.IMG_HEIGHT / 2

    def get_location(self, left_pixel, right_pixel_x):
        u_L, v_L = left_pixel
        u_R = right_pixel_x

        disparity = u_L - u_R
        
        # Safety check for infinite distance or negative disparity
        if disparity <= 0:
            return None 

        z = (self.FOCAL_LENGTH * self.BASELINE) / disparity

        x = (u_L - self.cx) * z / self.FOCAL_LENGTH

        y = (v_L - self.cy) * z / self.FOCAL_LENGTH

        total_distance = math.sqrt(x**2 + y**2 + z**2)

        angle_rad = math.atan2(x, z)
        angle_deg = math.degrees(angle_rad)

        return {
            "x": x, "y": y, "z": z,     
            "distance": total_distance, 
            "angle": angle_deg
        }

class MasterNode(MiRoCameraReader):
    def __init__(self):
        # Initialize the parent class
        super().__init__()
        
        # Initialize Depth Calculator
        self.calc = MiroDepthCalculator()
        
        # Publishers
        self.pub_visible = rospy.Publisher('/object_visible', Bool, queue_size=1)
        self.pub_certainty = rospy.Publisher('/object_certainty', Float32, queue_size=1)
        
        # Publishers for spatial data
        self.pub_dist = rospy.Publisher('/object_dist', Float32, queue_size=1)
        self.pub_angle = rospy.Publisher('/object_angle', Float32, queue_size=1)
        
        rospy.loginfo(f"Running master node. Tracking: {TARGET_CLASS}")

    def get_target_center(self, detections):
        if not detections:
            return None, 0.0

        chosen_object = None
        max_conf = 0.0

        for obj in detections:
            # Check if the label matches our target
            label = obj.get('class_name', obj.get('label', ''))
            
            if label == TARGET_CLASS:
                conf = obj.get('confidence', 0.0)
                if conf > max_conf:
                    max_conf = conf
                    chosen_object = obj
        
        if chosen_object:
            # Parse Box: [xmin, ymin, xmax, ymax]
            box = chosen_object.get('box', [])
            if len(box) == 4:
                xmin, ymin, xmax, ymax = box
                
                center_x = (xmin + xmax) / 2.0
                center_y = (ymin + ymax) / 2.0
                
                return (center_x, center_y), max_conf

        return None, 0.0

    def run(self):
        # Check 5 times a second
        rate = rospy.Rate(5) 
        
        while not rospy.is_shutdown():
            # Pair of eyes (Left=0, Right=1)
            if self.new_frame[0] and self.new_frame[1]: 
                left_image = self.frames[0]
                right_image = self.frames[1]
                
                # Look for object in left eye
                dets_left = send_frame_to_server(left_image)
                center_L, conf_L = self.get_target_center(dets_left)
                
                target_detected = False
                dist = 0.0
                angle = 0.0
                
                # If found in left, look in right eye to calculate depth
                if center_L:
                    target_detected = True
                    
                    dets_right = send_frame_to_server(right_image)
                    center_R, _ = self.get_target_center(dets_right)
                    
                    if center_R:
                        res = self.calc.get_location(center_L, center_R[0])
                        
                        if res:
                            dist = res['distance']
                            angle = res['angle']
                            rospy.loginfo(f"Object Found: {dist:.2f}m | {angle:.1f} deg")
                        else:
                            rospy.logwarn("Stereo Mismatch (Negative Disparity)")
                    else:
                        rospy.loginfo("Object in Left eye only (No Depth)")

                # Publish Data
                self.pub_visible.publish(target_detected)
                self.pub_certainty.publish(conf_L * 100)
                self.pub_dist.publish(dist)
                self.pub_angle.publish(angle)

                self.new_frame[0] = False
                self.new_frame[1] = False
            
            rate.sleep()

if __name__ == "__main__":
    master = MasterNode()
    master.run()
