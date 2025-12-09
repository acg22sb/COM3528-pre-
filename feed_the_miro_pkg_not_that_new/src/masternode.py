#!/usr/bin/env python3
# Run Mood Controller then this
import rospy
import os
import cv2
import math

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry

from camera_reader import MiRoCameraReader
from testvscript import send_frame_to_server
from object_permanence_v2 import ObjectPermanenceManager
from subscriber_odom import OdomSubscriber
from tf.transformations import euler_from_quaternion, quaternion_from_euler

TARGET_CLASS = "banana" 

class MiroDepthCalculator:
    def __init__(self):
        # Constants from Ling 2019 ROBIO Paper & MiRo Hardware Specs
        self.BASELINE = 0.0862     # Distance between eyes in meters
        self.FOCAL_LENGTH = 180.28  # Focal length in pixels
        
        # Camera Resolution (640x360 as per paper)
        self.IMG_WIDTH = 640
        self.IMG_HEIGHT = 360
        self.cx = self.IMG_WIDTH / 2
        self.cy = self.IMG_HEIGHT / 2

        self.EYE_DIVERGENCE = 27.0
        
    def get_location(self, left_pixel, right_pixel_x):
        u_L, v_L = left_pixel
        u_R = right_pixel_x

        disparity = u_L - u_R

        alpha_L =  math.atan((u_L - self.cx) / self.FOCAL_LENGTH)
        alpha_R =  math.atan((u_R - self.cx) / self.FOCAL_LENGTH)
        
        offset_L = math.radians(-self.EYE_DIVERGENCE)
        offset_R = math.radians(self.EYE_DIVERGENCE)

        theta_L = alpha_L + offset_L
        theta_R = alpha_R + offset_R

        denom = math.tan(theta_L) - math.tan(theta_R)
        # Safety check for infinite distance or negative disparity
        if abs(denom) < 0.001:
            return None 

        z = self.BASELINE / denom

        if z <=0:
            return None

        x = z * math.tan(theta_L) - (self.BASELINE / 2.0)

        y = (v_L - self.cy) * z / self.FOCAL_LENGTH

        #testing formula
        total_distance = math.sqrt(x**2 + y**2 + z**2)
        #total_distance = z

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

        self.object_permanence_manager = ObjectPermanenceManager()
        ##self.odom = OdomSubscriber()
        self.x = self.x0 = 0
        self.y = self.y0 = 0
        self.theta = self.theta0 = 0
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(self.topic_base_name + "/sensors/odom", Odometry, self.callback)

    def callback(self, data):
        orientation = data.pose.pose.orientation
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        (_, _, self.theta) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

    def reset(self):
        # Not a true reset, rather a change in frame of reference
        self.x0 = self.x
        self.y0 = self.y
        self.theta0 = self.theta

    def whereAmI(self):
        data = {
            "X": self.x - self.x0,
            "Y": self.y - self.y0,
            "Yaw": self.theta - self.theta0,
        }

        return data


    def get_target_center(self, detections):
        if not detections:
            return None, 0.0

        chosen_object = None
        max_conf = 0.0

        for obj in detections:
            # Check if the label matches our target
            label = obj.get('class_name', obj.get('label', ''))

            print(label)
            
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
        pos_data = self.whereAmI()
        
        while not rospy.is_shutdown():
            # Pair of eyes (Left=0, Right=1)
            if self.new_frame[0] and self.new_frame[1]: 
                left_image = self.frames[0]
                right_image = self.frames[1]
                
                # Look for object in left eye
                dets_left = send_frame_to_server(left_image)
                center_L, conf_L = self.get_target_center(dets_left)
                try:
                    lefteyex, lefteyey = center_L
                    rospy.loginfo("Center x and center y of left eye target: {0} and {1}".format(lefteyex, lefteyey))
                except:
                    rospy.loginfo("No left eye target")
                target_detected = False
                dist = 0.0
                angle = 0.0
                
                successful_observation = False
                
                # If found in left, look in right eye to calculate depth
                if center_L:
                    target_detected = True
                    
                    dets_right = send_frame_to_server(right_image)
                    center_R, _ = self.get_target_center(dets_right)
                    try:
                        righteyex, righteyey = center_R
                        rospy.loginfo("Center x and center y of right eye target: {0} and {1}".format(righteyex, righteyey))
                    except:
                        rospy.loginfo("No right eye target")
                    if center_R:
                        res = self.calc.get_location(center_L, center_R[0])
                        
                        boxes = [None, None]
                        for i, detections in enumerate([dets_left, dets_right]):
                            max_conf = 0.0
                            chosen_object = None
                            for obj in detections:
                                label = obj.get('class_name', obj.get('label', ''))
                                
                                if label == TARGET_CLASS:
                                    conf = obj.get('confidence', 0.0)
                                    if conf > max_conf:
                                        max_conf = conf
                                        chosen_object = obj
                            
                                if chosen_object:
                                    boxes[i] = chosen_object.get('box', [])

                        left_box, right_box = boxes[0], boxes[1]

                        if res:
                            obj_width = ((left_box[2] - left_box[0]) + (right_box[2] - right_box[0])) / 2
                            obj_height = ((left_box[3] - left_box[1]) + (right_box[3] - right_box[1])) / 2
                            obj_len = math.hypot(obj_width, obj_height)
                            exp_len_at_1m = 50
                            dist = exp_len_at_1m / obj_len
                            #dist = res['distance']
                            angle = res['angle']
                            rospy.loginfo(f"Object Found: {dist:.2f}m | {angle:.1f} deg")
                            successful_observation = True
                        else:
                            rospy.logwarn("Stereo Mismatch (Negative Disparity)")
                    else:
                        rospy.loginfo("Object in Left eye only (No Depth)")

                if successful_observation:
                    arena_width = 5
                    converted_dist = dist * (1 / arena_width)


                    miro_angle = pos_data["Yaw"]
                    abs_angle = angle - miro_angle
                    dx, dy = (math.cos(math.radians(abs_angle)) - pos_data["X"]) * converted_dist, (math.sin(math.radians(abs_angle)) - pos_data["Y"]) * converted_dist
                    self.object_permanence_manager.add_observation((dx, dy), 0.5)

                target_pos = self.object_permanence_manager.get_target_pos()

                print(f"Target Position {target_pos}")

                if target_pos:
                    dx, dy = target_pos[0] * arena_width - pos_data["X"], target_pos[1] * arena_width - pos_data["Y"]
                    move_dist = math.hypot(dx, dy)
                    move_dir = math.degrees(math.atan2(dy, dx))
                    print(f"Gotta get moving in direction {move_dir} degrees, distance {move_dist}")
                    self.pub_dist.publish(move_dist)
                    self.pub_angle.publish(move_dir)
                else:
                    self.pub_dist.publish(dist)
                    self.pub_angle.publish(angle)


                # Publish Data
                self.pub_visible.publish(target_detected)
                self.pub_certainty.publish(conf_L * 100)

                self.new_frame[0] = False
                self.new_frame[1] = False
            
            rate.sleep()

if __name__ == "__main__":
    master = MasterNode()
    master.run()
