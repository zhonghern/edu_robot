#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import time
import pyaudio# type: ignore
import wave

def callback(data):
    text = data.data
    rospy.loginfo(f"TTS Output: {text}")    
    soundhandle.say(text)

def put_the_phone_down(data):
    # Audio configuration
    CHUNK = 1024
    # Initialize PyAudio
    audio = pyaudio.PyAudio()
    f = wave.open(r"/home/tey/catkin_ws/src/edu_robot/audio/PUT THE PHONE DOWN.wav","rb")
    stream = audio.open(format = audio.get_format_from_width(f.getsampwidth()),  
                channels = f.getnchannels(),  
                rate = f.getframerate(),  
                output = True)  
    #read data  
    data = f.readframes(CHUNK)  
    
    #play stream  
    while data:  
        stream.write(data)  
        data = f.readframes(CHUNK)  
    
    #stop stream  
    stream.stop_stream()  
    stream.close()  
    
    #close PyAudio  
    audio.terminate() 


if __name__ == '__main__':
    rospy.init_node('tts_node')
    
    # Initialize SoundClient with a slight delay for safety
    soundhandle = SoundClient()
    time.sleep(1)  # Ensure sound client initializes correctly
    rospy.loginfo("SoundClient initialized successfully.")
    rospy.Subscriber('tts_input', String, callback)
    rospy.Subscriber('Phone_detection', String, put_the_phone_down)
    rospy.spin()

