#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, Int16
import numpy as np
from ros_audio_pkg.srv import TurnOn, TurnOnResponse, TurnOff, TurnOffResponse

import time
import speech_recognition as sr

class Voice():

    def __init__(self, dynamic_energy_threshold=False, energy_threshold=100, pause_threshold=1.0):
        # inizializzazione nodo ROS
        rospy.init_node('voice_detection_node', anonymous=False)
        
        # inizializzazione publisher
        self._pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)
        
        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = dynamic_energy_threshold
        self.r.energy_threshold = energy_threshold  #Modify here to set threshold. Reference: https://github.com/Uberi/speech_recognition/blob/1b737c5ceb3da6ad59ac573c1c3afe9da45c23bc/speech_recognition/__init__.py#L332
        self.r.pause_threshold = pause_threshold    # seconds of non-speaking audio before a phrase is considered complete
        self.m = None
        self.stop_listening = None
        self._state = False

    
    # this is called from the background thread
    def _callback(self, recognizer, audio):
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
        data_to_send = Int16MultiArray()
        data_to_send.data = data
        self._pub.publish(data_to_send)


    def _create_mic(self, name = 'ReSpeaker 4 Mic Array'):
        device_index = None
        for i, mic in enumerate(sr.Microphone.list_microphone_names()):
            if name in mic:
                device_index = i
        
        # device_index = 13

        if device_index is None:
            raise Exception('No microphone found')

        m = sr.Microphone(device_index=device_index,
                            sample_rate=16000,
                            chunk_size=1024)
        
        return m

    def _turn_on(self, _):
        if self._state == True:
            return TurnOnResponse('Already listening')
        self._state = True
        print('Start listening')
        self.stop_listening = self.r.listen_in_background(self.m, self._callback)
        return TurnOnResponse('ACK')
        
    def _turn_off(self, _):
        if self._state == False:
            return TurnOffResponse('Already stopped')
        self._state = False
        print('Stop listening')
        self.stop_listening()
        return TurnOffResponse('ACK')

    def start(self):
        self.m = self._create_mic()
        #self._calibration()

        rospy.Service('turn_on_mic', TurnOn, self._turn_on)
        rospy.Service('turn_off_mic', TurnOff, self._turn_off)
        self.stop_listening = self.r.listen_in_background(self.m, self._callback)
        self.stop_listening()
        rospy.spin()

    def _calibration(self):
        # Calibration within the environment
        # we only need to calibrate once, before we start listening
        
        print("Calibrating...")
        with self.m as source:
            self.r.adjust_for_ambient_noise(source,duration=10)
            
        print("Calibration finished, calibration value is: ", self.r.energy_threshold)


if __name__ == '__main__':
    voice = Voice()
    voice.start()
