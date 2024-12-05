#!/usr/bin/env python3

import io
from google.oauth2 import service_account # type: ignore
from google.cloud import speech
import rospy
from std_msgs.msg import String

client_file = '/home/tey/catkin_ws/src/edu_robot/src/sa_speech.json'

credentials = service_account.Credentials.from_service_account_file(client_file)
client = speech.SpeechClient(credentials=credentials)

def get_transcribe_text(audio_file):
	audio_file = '/home/tey/catkin_ws/src/edu_robot/audio/output.wav'
	with io.open(audio_file, 'rb') as f:
    		content = f.read()
    		audio = speech.RecognitionAudio(content=content)#type: ignore
    		config = speech.RecognitionConfig(#type: ignore
        		encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        		sample_rate_hertz=16000,
        		language_code='en-US'
    		)
    		response = client.recognize(config=config, audio=audio)#type: ignore
	for result in response.results:#type: ignore
        	return result.alternatives[0].transcript
	return "Error: No transcription available"#type: ignore

def get_transcribe_songname(audio_file):
	audio_file = '/home/tey/catkin_ws/src/edu_robot/audio/songname.wav'
	with io.open(audio_file, 'rb') as f:
    		content = f.read()
    		audio = speech.RecognitionAudio(content=content)#type: ignore
    		config = speech.RecognitionConfig(#type: ignore
        		encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        		sample_rate_hertz=16000,
        		language_code='en-US'
    		)
    		response = client.recognize(config=config, audio=audio)#type: ignore
	for result in response.results:#type: ignore
        	return result.alternatives[0].transcript
	return None#type: ignore

def callback(data):
	question = get_transcribe_text(data.data)
	rospy.loginfo(f"Received question: {question}")
	if question == "music":
		pub_music.publish(question)
	else:
		pub.publish(question)

# create function for getting song name 
def get_song_name(data):
	song_name = get_transcribe_songname(data.data)
	rospy.loginfo("song name received")
	pub_songname.publish(song_name)


if __name__ == "__main__":
	rospy.init_node("speech_to_text")
	pub = rospy.Publisher("student_question", String, queue_size=10)
	pub_music = rospy.Publisher("ini_music", String, queue_size=10)
	pub_songname = rospy.Publisher("song_name", String, queue_size=10)
	rospy.Subscriber("song_topic", String, get_song_name)
	rospy.Subscriber("audio_topic", String, callback)
	rospy.spin()  # Keep the node running