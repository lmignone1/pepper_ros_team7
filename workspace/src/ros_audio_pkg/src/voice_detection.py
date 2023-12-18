#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

import time
import speech_recognition as sr

pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)
rospy.init_node('voice_detection_node', anonymous=False)

# this is called from the background thread
def callback(recognizer, audio):
    data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
    data_to_send = Int16MultiArray()
    data_to_send.data = data
    pub.publish(data_to_send)

# Initialize a Recognizer
r = sr.Recognizer()
r.dynamic_energy_threshold = False
r.energy_threshold = 10 #Modify here to set threshold. Reference: https://github.com/Uberi/speech_recognition/blob/1b737c5ceb3da6ad59ac573c1c3afe9da45c23bc/speech_recognition/__init__.py#L332

for i, mic in enumerate(sr.Microphone.list_microphone_names()):
    if 'ReSpeaker 4 Mic Array' in mic:
        device_index = i

m = sr.Microphone(device_index=device_index,
                    sample_rate=16000,
                    chunk_size=1024)

# Calibration within the environment
# we only need to calibrate once, before we start listening
print("Calibrating...")
with m as source:
   r.adjust_for_ambient_noise(source,duration=3)  
print("Calibration finished")

# start listening in the background
# `stop_listening` is now a function that, when called, stops background listening
print("Recording...")
stop_listening = r.listen_in_background(m, callback)

rospy.spin()
