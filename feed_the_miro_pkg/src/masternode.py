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
from geometry_msgs.msg import TwistStamped

from camera_reader import MiRoCameraReader
from testvscript import send_frame_to_server
from object_permanence_v2 import ObjectPermanenceManager
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading


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
            if denom >= 0:
                denom = 0.001
            else:
                denom = -0.001
            #return None

        z = self.BASELINE / denom

        if z <= 0:
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

        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(self.topic_base_name + "/sensors/odom", Odometry, self.callback)
        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME", "miro") 
        self.pub_cmd = rospy.Publisher(topic_root + "/control/cmd_vel", TwistStamped, queue_size=1)

        self.linear_velocity = 0
        self.angular_velocity = 0

        self.vel_lock = threading.Lock()

        motion_thread = threading.Thread(target=self.motion_loop, args=(self.pub_cmd,))
        motion_thread.start()

    def callback(self, data):
        orientation = data.pose.pose.orientation
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        (_, _, self.theta) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
    def wrap_angle(self, angle):
        return angle % (2 * math.pi)
    
    def shortest_angle_dir_deg(self, a, b):
        return (a - b + 180) % 360 - 180

    def whereAmI(self):
        if self.x is None or self.y is None or self.theta is None or self.x0 is None or self.y0 is None or self.theta0 is None:
            return {"X": 0.0, "Y": 0.0, "Yaw": 0.0}
        
        dx_ros = self.x - self.x0
        dy_ros = self.y - self.y0

        dx_rot = dx_ros * math.cos(self.theta0) + dy_ros * math.sin(self.theta0)
        dy_rot = dx_ros * math.sin(self.theta0) - dy_ros * math.cos(self.theta0)

        yaw = 360 - (180 / math.pi) * self.wrap_angle(self.theta - self.theta0)

        return {
            "X": dy_rot,
            "Y": dx_rot,
            "Yaw": yaw,
        }


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
    
    def wait_for_odom(self, timeout=5.0):
        start = time.time()
        while (self.x is None or self.y is None or self.theta is None) and not rospy.is_shutdown():
            if time.time() - start > timeout:
                rospy.logwarn("Timeout waiting for initial odometry")
                break
            rospy.sleep(0.05)
        # Set the initial frame
        self.x0 = self.x
        self.y0 = self.y
        self.theta0 = self.theta

    def motion_loop(self, pub):
        velocity = TwistStamped()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            with self.vel_lock:
                velocity.twist.linear.x = self.linear_velocity
                velocity.twist.angular.z = self.angular_velocity
            pub.publish(velocity)
            rate.sleep()

    def run(self):
        # Check 5 times a second
        rate = rospy.Rate(5)

        behaviour_mode = "SEARCH"

        self.x = self.y = self.theta = None
        self.x0 = self.y0 = self.theta0 = None

        rospy.loginfo("Waiting for odom")

        self.wait_for_odom()

        rospy.loginfo("Starting")

        last_observation_time = None

        target_pos = None

        
        while not rospy.is_shutdown():

            pos_data = self.whereAmI()

            odom_x, odom_y, odom_angle = pos_data["X"], pos_data["Y"], pos_data["Yaw"]

            rospy.loginfo(f"ODOM: {odom_x:.3f}, {odom_y:.3f}, {odom_angle:.1f}")

            # Pair of eyes (Left=0, Right=1)
            if self.new_frame[0] and self.new_frame[1]: 
                left_image = self.frames[0]
                right_image = self.frames[1]
                
                # Look for object in left eye
                dets_left = send_frame_to_server(left_image)
                center_L, conf_L = self.get_target_center(dets_left)
                dets_right = send_frame_to_server(right_image)
                center_R, _ = self.get_target_center(dets_right)
                
                dist = 0.0
                angle = 0.0
                
                successful_observation = False
                
                # If found in left, look in right eye to calculate depth
                if not behaviour_mode == "SEARCH":
                    pass
                elif (center_L and center_R):
                    self.angular_velocity = 0
                    self.linear_velocity = 0
                    rospy.loginfo(f"Seen it, stop!")
                    lefteyex, lefteyey = center_L
                    rospy.loginfo("Center x and center y of left eye target: {0} and {1}".format(lefteyex, lefteyey))
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

                    last_observation_time = time.perf_counter()



                elif center_R and not center_L:
                    rospy.loginfo("Object in Right eye only (No Depth), Rotating Right...")
                    with self.vel_lock:
                        self.angular_velocity = -0.4
                        self.linear_velocity = 0
                elif center_L and not center_R:
                    rospy.loginfo("Object only in Left Eye, Rotating Left...")                  
                    with self.vel_lock:
                        self.angular_velocity = 0.4
                        self.linear_velocity = 0
                else:
                    rospy.loginfo("No Object, Rotating Right...")
                    with self.vel_lock:
                        self.angular_velocity = -0.4
                        self.linear_velocity = 0
                    
                    if last_observation_time != None and time.perf_counter() - last_observation_time > 3:
                        target_pos = self.object_permanence_manager.get_target_pos()
                        if target_pos != None:
                            behaviour_mode = "FETCH"
                            rospy.loginfo("ENTERING FETCH MODE!!!")
                    

                if successful_observation:
                    global_angle_rad = math.radians(odom_angle + angle)
                    dy = dist * math.cos(global_angle_rad)
                    dx = dist * math.sin(global_angle_rad)

                    ox = odom_x + dx
                    oy = odom_y + dy
                    self.object_permanence_manager.add_observation((ox, oy), 0.5)

                self.object_permanence_manager.tick()

                

                


                if behaviour_mode == "FETCH":
                    print(f"Target Position {target_pos}")
                    
                    target_pos = self.object_permanence_manager.get_target_pos()
                    if target_pos == None:
                        rospy.loginfo("I forgot the target")
                        #behaviour_mode = "SEARCH"
                        self.linear_velocity = 0
                    else:
                        dx, dy = target_pos[0] - odom_x, target_pos[1] - odom_y
                        move_dist = math.hypot(dx, dy)
                        move_dir = math.degrees(self.wrap_angle(math.atan2(-dy, dx)))
                        rospy.loginfo(f"Got a target, moving in direction {move_dir} degrees, distance {move_dist}")
                        vel_cap = 0.5
                        ang_vel_cap = 0.5
                        #self.linear_velocity = min(move_dist * 0.3, vel_cap)
                        #self.angular_velocity = max(min(0.1 * -self.shortest_angle_dir_deg(odom_angle, move_dir), ang_vel_cap), - ang_vel_cap)
                        self.pub_dist.publish(move_dist)
                        angle_to_send_follower  = (move_dir) - odom_angle
                        rospy.loginfo(angle_to_send_follower)
                        while angle_to_send_follower > 180:
                            angle_to_send_follower -= 360
                        while angle_to_send_follower < -180:
                            angle_to_send_follower += 360
                        rospy.loginfo(f"follower being sent: {angle_to_send_follower}")
                        self.pub_angle.publish(angle_to_send_follower)


                # Publish Data
                self.pub_visible.publish(last_observation_time != None)
                self.pub_certainty.publish(conf_L * 100)

                self.new_frame[0] = False
                self.new_frame[1] = False
            
            rate.sleep()

if __name__ == "__main__":
    master = MasterNode()
    master.run()
