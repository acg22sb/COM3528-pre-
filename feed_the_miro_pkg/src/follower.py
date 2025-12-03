#!/usr/bin/env python3
import os
import rospy
import numpy as np

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float32

class MiroFollower:
    def __init__(self):
        rospy.init_node("miro_follower", anonymous=True)
        
        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME", "miro") 

        self.stop_distance = 0.10  # cm
        self.linear_gain = 0.4     # How fast to accelerate
        self.angular_gain = 0.02   # How fast to turn
        self.max_speed = 0.4       # m/s

        self.is_visible = False
        self.target_dist = 0.0
        self.target_angle = 0.0

        self.pub_cmd = rospy.Publisher(topic_root + "/control/cmd_vel", TwistStamped, queue_size=10)

        rospy.Subscriber('/object_visible', Bool, self.cb_visible)
        rospy.Subscriber('/object_dist', Float32, self.cb_dist)
        rospy.Subscriber('/object_angle', Float32, self.cb_angle)

        rospy.loginfo("MiRo Follower Started. Waiting for data...")

    def cb_visible(self, msg):
        self.is_visible = msg.data

    def cb_dist(self, msg):
        self.target_dist = msg.data

    def cb_angle(self, msg):
        self.target_angle = msg.data

    def loop(self):
        rate = rospy.Rate(10) # Hz
        
        while not rospy.is_shutdown():
            velocity = TwistStamped()
            
            if self.is_visible:
                turn_val = -1 * self.target_angle * self.angular_gain
                velocity.twist.angular.z = turn_val

                if self.target_dist > self.stop_distance:
                    # Simple P-controller: Speed proportional to error
                    error_dist = self.target_dist - self.stop_distance
                    fwd_val = error_dist * self.linear_gain
                    
                    # Caping speed
                    fwd_val = min(fwd_val, self.max_speed)
                    
                    velocity.twist.linear.x = fwd_val
                else:
                    # if too close, stop linear movement but still allow turning to face object
                    velocity.twist.linear.x = 0.0
                
                # Debug line
                rospy.loginfo(f"Dist: {self.target_dist:.2f} | Ang: {self.target_angle:.1f} | Cmd: [v={velocity.twist.linear.x:.2f}, w={velocity.twist.angular.z:.2f}]")
                
            else:
                velocity.twist.linear.x = 0.0
                velocity.twist.angular.z = 0.0

            self.pub_cmd.publish(velocity)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = MiroFollower()
        node.loop()
    except rospy.ROSInterruptException:
        pass
