#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Minimal ROS node that only reads MiRo's camera streams
and displays the images. No movement, no processing.
"""

import os
import cv2
import rospy
from testvscript import send_frame_to_server

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class MiRoCameraReader:

    def __init__(self):
        rospy.init_node("miro_camera_reader", anonymous=True)

        self.bridge = CvBridge()
        self.frames = [None, None]
        self.new_frame = [False, False]

        base = "/" + os.getenv("MIRO_ROBOT_NAME")

        rospy.Subscriber(
            base + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_left,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.Subscriber(
            base + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_right,
            queue_size=1,
            tcp_nodelay=True
        )

        print("MiRo camera reader running...")

    def callback_left(self, msg):
        self._process_frame(msg, index=0)

    def callback_right(self, msg):
        self._process_frame(msg, index=1)

    def _process_frame(self, msg, index):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "rgb8")
            self.frames[index] = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.new_frame[index] = True
        except CvBridgeError:
            pass

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            # LEFT CAMERA = index 0
            if self.new_frame[0]:
                left_frame = self.frames[0]

                # Show the image (optional)
                cv2.imshow("camera_0", left_frame)
                cv2.waitKey(1)

                # Call your YOLO sending function
                send_frame_to_server(left_frame)

                # Mark frame as processed
                self.new_frame[0] = False

            # If you still want to display right camera, keep this:
            if self.new_frame[1]:
                cv2.imshow("camera_1", self.frames[1])
                cv2.waitKey(1)
                self.new_frame[1] = False

            rate.sleep()


if __name__ == "__main__":
    MiRoCameraReader().run()
