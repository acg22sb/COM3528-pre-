#!/usr/bin/env python3
import rospy
import threading
import os
from std_msgs.msg import Bool, Float32, Float32MultiArray

try:
    from miro2.lib import miro_ros_client
except ImportError:
    rospy.logfatal("Could not import miro2.lib. Please source ~/mdk/setup.bash")
    exit(1)

class MoodController:
    def __init__(self):
        rospy.init_node('mood_controller', anonymous=True)

        self.miro_client = miro_ros_client.MiroClient(robot='rob01', wait_for_services=False)

        # parameters
        self.update_hz = rospy.get_param('~update_hz', 10.0)
        self.wag_speed = rospy.get_param('~wag_speed', 5)
        self.happy_threshold = rospy.get_param('~happy_threshold', 0.5)
        self.sad_threshold = rospy.get_param('~sad_threshold', -0.5)

        # s01 is typically a happy chirp, s05 is a sad/low squeak
        self.sound_happy = os.path.expanduser("~/miro/share/media/s01.wav")
        self.sound_sad = os.path.expanduser("~/miro/share/media/s05.wav")

        # Fallback check (some robots use ~/mdk instead of ~/miro)
        if not os.path.exists(self.sound_happy):
            self.sound_happy = os.path.expanduser("~/mdk/share/media/s01.wav")
        if not os.path.exists(self.sound_sad):
            self.sound_sad = os.path.expanduser("~/mdk/share/media/s05.wav")

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
            0.0, 0.0,      # eyes
            wag_value, wag_value        # ears
        ]
        self.pub_cosmetic.publish(msg)

    #New Method to Stream Audio
    def drive_voice(self, filename):
        if os.path.exists(filename):
            rospy.loginfo(f"Streaming sound: {filename}")
            # This function from miro2 library streams the .wav file
            self.miro_client.push_audio_stream(filename)
        else:
            rospy.logwarn(f"Audio file not found: {filename}")

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
                # REPLACED: send_tone with drive_voice
                self.drive_voice(self.sound_happy)
                rospy.sleep(1.0) # Wait approx time for sound to finish
        finally:
            self.is_making_sound = False

    def play_sad_sound(self):
        self.is_making_sound = True
        try:
            with self.lock:
                # REPLACED: send_tone with drive_voice
                self.drive_voice(self.sound_sad)
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
