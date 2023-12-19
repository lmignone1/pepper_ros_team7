#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse, DialogueRequest
from std_msgs.msg import Int16MultiArray, String, Int16

from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse

conversation = False

def rcv_detection(msg):
    global conversation

    # if conversation == False:
    #     if msg.data == 1:
    #         conversation = True
    # elif conversation == True:
    #     if msg.data == 0:
    #         conversation = False


    if msg.data == 1:
        conversation = True
    else:
        conversation = False


def main():
    rospy.init_node('speaking')
    print('Node speaking started')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('/dialogue_server', Dialogue)

    rospy.wait_for_service('/tts')
    tts_service = rospy.ServiceProxy('/tts', Text2Speech)

    # terminal = TerminalInterface()
   
    rospy.Subscriber('detection', Int16, rcv_detection)

    while not rospy.is_shutdown():
        
        print('conversation: ', conversation)

        if conversation == True:
            message = Text2SpeechRequest()
            message.speech = 'Hello folks'
            tts_service(message)
            print('Saluto pr la prima volta')

            i = 0
            while True:
                


                if conversation == False:
                    print('RESTART:')
                    restart = DialogueRequest()
                    restart.input_text = '/restart'
                    resp = dialogue_service(restart)
                    print('RISP bot al restart', resp.answer)
                    break
                
                try:
                    message = rospy.wait_for_message('voice_txt', String, timeout=30) # lunghezza parola + costante 
                except rospy.ROSException:
                    message = None

                if message is None:
                    print('RESTART:')
                    restart = DialogueRequest()
                    restart.input_text = '/restart'
                    resp = dialogue_service(restart)
                    print('RISP bot al restart', resp.answer)
                    break

                message = message.data.lower()

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
                
                i += 1
                print('fine iterazione ', i)
                
            print('sono fuori dall iterazione')

            
        
        rospy.sleep(1)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass