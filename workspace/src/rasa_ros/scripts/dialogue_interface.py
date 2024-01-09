#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse, DialogueRequest
from std_msgs.msg import Int16MultiArray, String, Int16
from ros_audio_pkg.srv import TurnOn, TurnOnResponse, TurnOff, TurnOffResponse
from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse


class DialogueInterface():

    def __init__(self) -> None:
        # inizializzazione del nodo
        rospy.init_node('dialogue_interface')
        print('Node dialogue interface started')
        
        # inizializzazione dei servizi
        rospy.wait_for_service('dialogue_server')
        self.dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)


    def start(self):
        self._pub_tablet = rospy.Publisher('tablet_template', Int16, queue_size=1)
        
      
        while not rospy.is_shutdown():
            
            print('Waiting for detection')
            rospy.wait_for_message('detection', Int16)
            
            self._pub_tablet.publish(1)
            engagement = DialogueRequest()
            engagement.input_text = 'Hello'
            self.dialogue_service(engagement)

            print("Engagement")

            while True:
                
                try:
                    user_txt = rospy.wait_for_message('voice_txt', String, timeout=TIMEOUT_VOICE) # lunghezza ultima frase fatta pronunciare a Pepper + costante 
                except rospy.ROSException:
                    break
                
                try:
                    print("[IN]:", user_txt)
                    user_req = DialogueRequest()
                    user_req.input_text = user_txt.data.lower()                    
                    bot_answer = self.dialogue_service(user_req)
                    print("[OUT]:", bot_answer.answer)
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                    break
                
                try:
                    rospy.wait_for_message('detection', Int16, timeout=TIMEOUT_DETECTOR) 
                except rospy.ROSException:
                    break                                                         # altrimenti procedi perche il tempo atteso è stato superiore o uguale al tempo di pronuncia
            
            print('restart bot')
            self._pub_tablet.publish(0)
            restart_req = DialogueRequest()
            restart_req.input_text = '/restart'
            resp = self.dialogue_service(restart_req)
            print(resp.answer)
           



if __name__ == '__main__':
    TIMEOUT_VOICE = 30
    TIMEOUT_DETECTOR = 5

    try: 
        node = DialogueInterface()
        node.start()
    except rospy.ROSInterruptException:
        pass