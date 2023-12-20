#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, Int16
import numpy as np

import time
import speech_recognition as sr

class Voice():

    def __init__(self, dynamic_energy_threshold=False, energy_threshold=10):
        # inizializzazione nodo ROS
        rospy.init_node('voice_detection_node', anonymous=False)
        
        # inizializzazione publisher
        self._pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)
        
        # inzializzazione subscriber
        rospy.Subscriber('mic_command', Int16, self._command_mic_callback)
        
        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = dynamic_energy_threshold
        self.r.energy_threshold = energy_threshold  #Modify here to set threshold. Reference: https://github.com/Uberi/speech_recognition/blob/1b737c5ceb3da6ad59ac573c1c3afe9da45c23bc/speech_recognition/__init__.py#L332

        self.m = None
        self.stop_listening = None


    # start listening in the background
    # `stop_listening` is now a function that, when called, stops background listening
    def _command_mic_callback(self, command):
        if command.data == 1:
            print('Start listening')
            self.stop_listening = self.r.listen_in_background(self.m, self._callback)
        else:
            print('Stop listening')
            self.stop_listening(wait_for_stop=False)
            self.stop_listening = None

    
    # this is called from the background thread
    def _callback(self, recognizer, audio):
        print('################# callback ####################à')
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
        data_to_send = Int16MultiArray()
        data_to_send.data = data
        self._pub.publish(data_to_send)


    def _create_mic(self, name = 'ReSpeaker 4 Mic Array'):
        device_index = None
        for i, mic in enumerate(sr.Microphone.list_microphone_names()):
            if name in mic:
                device_index = i
        
        device_index = 13

        if device_index is None:
            raise Exception('No microphone found')

        m = sr.Microphone(device_index=device_index,
                            sample_rate=16000,
                            chunk_size=1024)
        
        return m

    def start(self):
        self.m = self._create_mic()
        self._calibration()

        rospy.spin()

    def _calibration(self):
        # Calibration within the environment
        # we only need to calibrate once, before we start listening
        
        print("Calibrating...")
        with self.m as source:
            self.r.adjust_for_ambient_noise(source,duration=3)
        print("Calibration finished")


if __name__ == '__main__':
    voice = Voice()
    voice.start()
