#!/usr/bin/env python

# Message types
from geometry_msgs.msg import TwistStamped

# Other imports
import os
import time
import rospy
import numpy as np
import miro2 as miro

# Set robot name
topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

# Python needs to initialise a ROS node for publishing data
rospy.init_node("py_pub", anonymous=True)

# Define ROS publisher
pub_cmd = rospy.Publisher(topic_root + "/control/cmd_vel", TwistStamped, queue_size=10)

# Create object to hold published data
velocity = TwistStamped()

# Set left/right wheels speeds in m/s
l_val = np.random.uniform(-1, 1, 1)
r_val = np.random.uniform(-1, 1, 1)
wheel_speed = [l_val, r_val]

# Convert to command velocity
(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)

# Set velocity values
velocity.twist.linear.x = dr
velocity.twist.angular.z = dtheta

# Publish command velocity for 3 seconds
t_start = time.clock()

while not rospy.core.is_shutdown():
    pub_cmd.publish(velocity)

    if time.clock() >= t_start + 3:
        break

# Take a look at the client_gui.py code (mdk/bin/shared/client_gui.py) for a more detailed use of ROSPy
