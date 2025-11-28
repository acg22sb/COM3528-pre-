#!/usr/bin/env python3
import rospy
import threading
from std_msgs.msg import Bool, Float32, UInt16MultiArray, Float32MultiArray
import math

class MoodController:
    def __init__(self):
        rospy.init_node('mood_controller', anonymous=True)

        # parameters
        self.update_hz = rospy.get_param('~update_hz', 10.0)
        self.wag_speed = rospy.get_param('~wag_speed', 5)
        self.happy_threshold = rospy.get_param('~happy_threshold', 0.5)
        self.sad_threshold = rospy.get_param('~sad_threshold', -0.5)

        # state
        self.object_visible = False
        self.object_certainty = 0.0
        self.mood = 0.0
        
        self.is_wagging = False
        self.is_making_sound = False
        self.lock = threading.Lock()

        self.pub_cosmetic = rospy.Publisher(
            "/miro/control/cosmetic_joints",
            Float32MultiArray,
            queue_size=1
        )

        self.pub_tone = rospy.Publisher(
            "/miro/control/tone",
            UInt16MultiArray,
            queue_size=1
        )

        # subscribers
        rospy.Subscriber('/object_visible', Bool, self.cb_visible)
        rospy.Subscriber('/object_certainty', Float32, self.cb_certainty)

        # timer
        rospy.Timer(rospy.Duration(1.0/self.update_hz), self.update_mood)

        rospy.loginfo("Mood Controller started.")

    def cb_visible(self, msg):
        self.object_visible = msg.data

    def cb_certainty(self, msg):
        self.object_certainty = msg.data

    def send_tail(self, wag_value):
        msg = Float32MultiArray()
        msg.data = [
            0.5,           # droop
            wag_value,     # wag (0=left, 1=right)
            0.5, 0.5,      # eyes
            0.5, 0.5       # ears
        ]
        self.pub_cosmetic.publish(msg)

    def send_tone(self, freq, volume, duration):
        msg = UInt16MultiArray()
        msg.data = [freq, volume, int(duration * 50)]
        self.pub_tone.publish(msg)

    def wag_tail(self, wags):
        self.is_wagging = True
        try:
            delay = 1.0 / float(self.wag_speed)

            with self.lock:
                for _ in range(wags):
                    self.send_tail(0.0)  # left
                    rospy.sleep(delay)
                    self.send_tail(1.0)  # right
                    rospy.sleep(delay)

                # center tail
                self.send_tail(0.5)

        finally:
            self.is_wagging = False

    def play_happy_sound(self):
        self.is_making_sound = True
        try:
            with self.lock:
                self.send_tone(800, 200, 0.5)
                rospy.sleep(1.0)
        finally:
            self.is_making_sound = False

    def play_sad_sound(self):
        self.is_making_sound = True
        try:
            with self.lock:
                self.send_tone(200, 150, 1.0)
                rospy.sleep(1.5)
        finally:
            self.is_making_sound = False

    def update_mood(self, event):
        cert_factor = min(max(self.object_certainty, 0.0), 100.0) / 100.0

        if self.object_visible:
            self.mood += 0.05 * cert_factor
        elif cert_factor > 0.0:
            self.mood -= 0.04 * cert_factor
        else:
            self.mood += (0.0 - self.mood) * 0.02

        self.mood = max(-1.0, min(1.0, self.mood))

        if self.mood > self.happy_threshold:
            wags = max(1, int(self.mood * 5))

            if not self.is_wagging:
                threading.Thread(target=self.wag_tail, args=(wags,), daemon=True).start()

            if not self.is_making_sound:
                threading.Thread(target=self.play_happy_sound, daemon=True).start()

        elif self.mood < self.sad_threshold:
            if not self.is_wagging:
                threading.Thread(target=self.wag_tail, args=(1,), daemon=True).start()

            if not self.is_making_sound:
                threading.Thread(target=self.play_sad_sound, daemon=True).start()


if __name__ == "__main__":
    MoodController()
    rospy.spin()
