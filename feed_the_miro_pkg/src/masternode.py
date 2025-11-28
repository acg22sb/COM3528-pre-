#!/usr/bin/env python3
# Run Mood Controller then this
import rospy
import os
import cv2

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Float32

from camera_reader import MiRoCameraReader
from testvscript import send_frame_to_server

class MasterNode(MiRoCameraReader):
    def __init__(self):
        # Initialize the parent class 
        super().__init__()
        
        # connection to the Mood Controller
        self.pub_visible = rospy.Publisher('/object_visible', Bool, queue_size=1)
        self.pub_certainty = rospy.Publisher('/object_certainty', Float32, queue_size=1)
        
        rospy.loginfo("Running master node")

    def run(self):
        rate = rospy.Rate(10) # Check 10 times a second
        
        while not rospy.is_shutdown():
            if self.new_frame[0]: 
                left_image = self.frames[0]
                
                detections = send_frame_to_server(left_image)
                
                human_seen = False
                confidence = 0.0
                
                if detections:
                    for obj in detections:
                        label = obj.get('label', obj.get('class_name', ""))
                        if label == 'human' or label == 'person':
                            human_seen = True
                            confidence = obj.get('confidence', 0.0) * 100 
                            rospy.loginfo(f"Human Detected! Certainty: {confidence:.1f}%")
                            break

                # Publish for Mood Controller
                self.pub_visible.publish(human_seen)
                self.pub_certainty.publish(confidence)

                # Reset
                self.new_frame[0] = False
            
            rate.sleep()

if __name__ == "__main__":
    master = MasterNode()
    master.run()
