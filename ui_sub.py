#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import pyautogui  # For simulating keyboard events

def initiate_callback(msg):
    """
    Callback function for /initiate topic.
    Simulates 's' keypress using pyautogui.
    """
    if msg.data == 's':
        rospy.loginfo("Received 's' from /initiate, simulating keyboard input.")
        pub.publish("s")

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("interface_listener", anonymous=True)
    pub_stop = rospy.Publisher("recording_stop", String, queue_size=10)
    # Subscribe to the /initiate topic and set the callback function
    rospy.Subscriber("/initiate", String, initiate_callback)
    pub = rospy.Publisher("recording_start", String, queue_size=10)
    rospy.loginfo("Interface Listener Node Started. Listening for 's'...")
    rospy.spin()  # Keep the node running and processing callbacks

