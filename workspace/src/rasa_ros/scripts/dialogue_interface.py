#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse, DialogueRequest
from std_msgs.msg import Int16MultiArray, String

from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse

class TerminalInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal

    '''

    def get_text(self):
        return input("[IN]:  ") 

    def set_text(self,text):
        print("[OUT]:",text)

def main():
    rospy.init_node('speaking')
    print('Node speaking started')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('/dialogue_server', Dialogue)

    rospy.wait_for_service('/tts')
    tts_service = rospy.ServiceProxy('/tts', Text2Speech)

    # terminal = TerminalInterface()

    while not rospy.is_shutdown():

        message = rospy.wait_for_message('voice_txt', String)
        message = message.data

        if message == 'exit': 
            break
        
        try:
    
            print("[IN]:", message)
            bot_request = DialogueRequest()
            bot_request.input_text = message
            bot_answer = dialogue_service(bot_request)
            print("[OUT]:", bot_answer.answer)

            msg = Text2SpeechRequest()
            msg.speech = bot_answer.answer
            
            tts_service(msg)

            print('fine ciclo')
        

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass