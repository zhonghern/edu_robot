#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr #type: ignore
import os

# Initialize ROS node
rospy.init_node('stt', anonymous=True)
recognizer = sr.Recognizer()
song_name_publisher = rospy.Publisher('song_name', String, queue_size=10)

def convert_audio_to_text(file_path):
    """Converts a WAV file to text using speech recognition."""
    try:
        file = file_path.data
        with sr.AudioFile(file) as source:
            rospy.loginfo(f"Processing file: {file}")
            audio = recognizer.record(source)
            return recognizer.recognize_google(audio)
    except Exception as e:
        rospy.logerr(f"Speech recognition error: {e}")
        return None

def get_song_name(file_path):
    """Converts a WAV file to text using speech recognition."""
    try:
        file = file_path.data
        with sr.AudioFile(file) as source:
            rospy.loginfo(f"Processing file: {file}")
            audio = recognizer.record(source)
            pub_songname.publish(recognizer.recognize_google(audio))
    except Exception as e:
        rospy.logerr(f"Speech recognition error: {e}")
        return None

def callback(data):
    query = convert_audio_to_text(data)
    query = query.strip()
    if query.lower() == "music":
        pub_music.publish(query)
    else:
        pub.publish(query)

if __name__ == "__main__":
    try:
        pub_music = rospy.Publisher("ini_music", String, queue_size=10)
        pub_songname = rospy.Publisher("song_name", String, queue_size=10)
        pub = rospy.Publisher('student_question', String, queue_size=10)
        rospy.Subscriber('/audio_topic', String, callback)
        rospy.Subscriber("song_topic", String, get_song_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS node shutting down.")



