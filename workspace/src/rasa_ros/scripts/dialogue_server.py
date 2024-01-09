#!/usr/bin/env python3
from rasa_ros.srv import Dialogue, DialogueResponse

import rospy
import requests
from std_msgs.msg import Int16
from ros_audio_pkg.srv import TurnOn, TurnOnResponse, TurnOff, TurnOffResponse
from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse


class DialogueServer():

    def __init__(self, pepper):
        rospy.init_node('dialogue_service')

        self.get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
        self._pepper = pepper

        if pepper:
            rospy.wait_for_service('turn_on_mic')
            self._turn_on_mic = rospy.ServiceProxy('turn_on_mic', TurnOn)

            rospy.wait_for_service('turn_off_mic')
            self._turn_off_mic = rospy.ServiceProxy('turn_off_mic', TurnOff)

            rospy.wait_for_service('/tts')
            self.tts_service = rospy.ServiceProxy('tts', Text2Speech)
        else:
            self._turn_on_mic = None
            self._turn_off_mic = None
            self.tts_service = None


    def handle_service(self, req):
        
        text = req.input_text
        
        if self._pepper:
            start = rospy.Time.now()
            self._turn_off_mic()
        
        message = {
        "sender": 'bot',
        "message": text
                    }
        
        r = requests.post(self.get_answer_url, json=message)
        response = DialogueResponse()

        if(text == "/restart"):
            response.answer = "restarting"
        else:
            response.answer = ""
            
            for i in r.json():
                response.answer += i['text'] + ' ' if 'text' in i else ''
            
            if self._pepper:
                tts_req = Text2SpeechRequest()
                tts_req.speech = response.answer
                self.tts_service(tts_req)

                time_last_utterance = len(response.answer.split()) * 0.25   
                end = rospy.Time.now()

                if (end - start).to_sec() < time_last_utterance:
                    rospy.sleep(time_last_utterance - (end - start).to_sec())

                self._turn_on_mic()
        
        return response

    def start(self):
        rospy.Service('dialogue_server', Dialogue, self.handle_service)
        rospy.logdebug('Dialogue server READY.')
        rospy.spin()


if __name__ == '__main__':
    PEPPER = True
    try: 
        node = DialogueServer(PEPPER)
        node.start()
    except rospy.ROSInterruptException as e:
        pass
