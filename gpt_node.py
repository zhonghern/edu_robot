#!/usr/bin/env python3
import rospy
from groq import Groq # type: ignore
from std_msgs.msg import String

# Initialize Groq client with API key
client = Groq(api_key='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')

def get_gpt_response(question):
    # Generate response using the Groq client
    completion = client.chat.completions.create(
        model="llama3-8b-8192",
        messages=[{"role": "user", "content": question}],
        temperature=1,
        max_tokens=50,
        top_p=1,
        stream=True,
    )
    
    # Concatenate chunks into a full response
    response = ""
    for chunk in completion:
        content = chunk.choices[0].delta.content
        if content is not None:  # Check if content is not None
            response += content
    return response

def callback(data):
    question = data.data
    rospy.loginfo(f"Received question: {question}")
    answer = get_gpt_response(question)
    rospy.loginfo(f"LLM Response: {answer}")
        
    
    # Publish the response to the TTS node
    pub.publish(answer)

if __name__ == '__main__':
    rospy.init_node('gpt_node')
    pub = rospy.Publisher('tts_input', String, queue_size=10)
    rospy.Subscriber('student_question', String, callback)
    rospy.spin()

