#!/usr/bin/env python3
import rospy
import threading
from std_msgs.msg import Bool, Float32
import math

# Import the interface file you pasted earlier
# Make sure miro_ros_interface.py is in the same folder!
from miro_ros_interface import MiRoPublishers

class MoodController:
    def __init__(self):
        # 1. Init Node
        # In ROS 1, we init the node first. 
        # anonymous=True allows multiple instances if needed.
        rospy.init_node('mood_controller', anonymous=True)

        # 2. Instantiate the Interface (It uses the existing rospy node)
        self.miro_pub = MiRoPublishers()

        # 3. Parameters (using rospy.get_param)
        # '~name' means it looks for private parameters
        self.update_hz = rospy.get_param('~update_hz', 10.0)
        self.wag_speed = rospy.get_param('~wag_speed', 5)
        self.happy_threshold = rospy.get_param('~happy_threshold', 0.5)
        self.sad_threshold = rospy.get_param('~sad_threshold', -0.5)

        # 4. Internal state
        self.object_visible = False
        self.object_certainty = 0.0
        self.mood = 0.0
        
        # Flags to prevent "Thread Bombing" (Crash prevention)
        self.is_wagging = False
        self.is_making_sound = False
        self.lock = threading.Lock()

        # 5. Subscribers
        rospy.Subscriber('/object_visible', Bool, self.cb_visible)
        rospy.Subscriber('/object_certainty', Float32, self.cb_certainty)

        # 6. Timer
        # ROS 1 Timers pass an 'event' object to the callback
        rospy.Timer(rospy.Duration(1.0/self.update_hz), self.update_mood)

        rospy.loginfo("Mood Controller (ROS 1) started.")

    def cb_visible(self, msg):
        self.object_visible = bool(msg.data)

    def cb_certainty(self, msg):
        self.object_certainty = float(msg.data)

    def update_mood(self, event):
        # Logic: Calculate Mood
        cert_factor = min(max(self.object_certainty, 0.0), 100.0) / 100.0

        if self.object_visible:
            self.mood += 0.05 * cert_factor
        elif cert_factor > 0.0:
            self.mood -= 0.04 * cert_factor # Anxiety
        else:
            # Drift to neutral
            self.mood += (0.0 - self.mood) * 0.02

        # Clamp mood between -1 and 1
        self.mood = max(-1.0, min(1.0, self.mood))

        # Logic: Behaviour Triggers
        # We check if an action is ALREADY happening before starting a new one
        if self.mood > self.happy_threshold:
            wags = max(1, int(self.mood * 5))
            
            if not self.is_wagging:
                t = threading.Thread(target=self.wag_tail, args=(wags,))
                t.daemon = True
                t.start()
            
            if not self.is_making_sound:
                t = threading.Thread(target=self.play_happy_sound)
                t.daemon = True
                t.start()

        elif self.mood < self.sad_threshold:
            if not self.is_wagging:
                t = threading.Thread(target=self.wag_tail, args=(1,))
                t.daemon = True
                t.start()
                
            if not self.is_making_sound:
                 t = threading.Thread(target=self.play_sad_sound)
                 t.daemon = True
                 t.start()

    def wag_tail(self, wags):
        self.is_wagging = True
        try:
            speed = self.wag_speed
            delay = 1.0 / float(speed)
            
            with self.lock:
                for _ in range(wags):
                    # ROS 1 Interface uses specific keywords
                    self.miro_pub.pub_cosmetic_joints(wag='left')
                    rospy.sleep(delay)
                    self.miro_pub.pub_cosmetic_joints(wag='right')
                    rospy.sleep(delay)
                # Center tail at end
                self.miro_pub.pub_cosmetic_joints(wag=0.5)
        except Exception as e:
            rospy.logwarn(f"Wag Error: {e}")
        finally:
            self.is_wagging = False

    def play_happy_sound(self):
        self.is_making_sound = True
        try:
            with self.lock:
                # Using pub_tone from your interface file
                # duration, volume, frequency, note
                # Happy = High pitch (e.g. 'C5' or freq ~800)
                self.miro_pub.pub_tone(duration=0.5, volume=200, frequency=800)
                rospy.sleep(1.0) # Don't spam sounds
        except Exception:
            pass
        finally:
            self.is_making_sound = False

    def play_sad_sound(self):
        self.is_making_sound = True
        try:
            with self.lock:
                # Sad = Low pitch
                self.miro_pub.pub_tone(duration=1.0, volume=150, frequency=200)
                rospy.sleep(1.5)
        except Exception:
            pass
        finally:
            self.is_making_sound = False

if __name__ == "__main__":
    try:
        mc = MoodController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass