#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse, DialogueRequest
from std_msgs.msg import Int16MultiArray, String, Int16
from ros_audio_pkg.srv import TurnOn, TurnOnResponse, TurnOff, TurnOffResponse
from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse


class Speaking():

    def __init__(self) -> None:
        # inizializzazione del nodo
        rospy.init_node('pepper_speaking_node')
        print('Node speaking pepper started')
        
        # inizializzazione dei servizi
        rospy.wait_for_service('dialogue_server')
        self.dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)

        rospy.wait_for_service('/tts')
        self.tts_service = rospy.ServiceProxy('tts', Text2Speech)

        rospy.wait_for_service('turn_on_mic')
        self._turn_on_mic = rospy.ServiceProxy('turn_on_mic', TurnOn)

        rospy.wait_for_service('turn_off_mic')
        self._turn_off_mic = rospy.ServiceProxy('turn_off_mic', TurnOff)

        # inizializzazione dei publisher
        # self._mic = rospy.Publisher('mic_command', Int16, queue_size=1)
        # inizializzazione dei subscriber
        # rospy.Subscriber('detection', Int16, self._rcv_detection)

        # se true allora la persona parla con il robot altrimenti non ci sono persone
        # self.conversation = None


    # def _rcv_detection(self, detected):
    #     if detected.data == 1:
    #         self.conversation = True
    #     else:
    #         self.conversation = False

    def _make_request(self, text, request_obj):
        if isinstance(request_obj, DialogueRequest):
            request_obj.input_text = text
            return request_obj
        
        if isinstance(request_obj, Text2SpeechRequest):
            request_obj.speech = text
            return request_obj
        

        


    def start(self):
        while not rospy.is_shutdown():
        
            # print('conversation: ', self.conversation)

            rospy.wait_for_message('detection', Int16)

            engagement_utterance = 'Hello folks'
            engagement = self._make_request(engagement_utterance, Text2SpeechRequest())
            self.tts_service(engagement)
            print('Io robot ingaggio la persona che ho davanti')
            
            time_last_utterance = len(engagement_utterance.split()) * 0.25 # rappresenta un approssimazione del tempo impiegato da pepper per pronunciare l ultima frase fornita
            
            

            i = 0
            while True:
                
                
                try:
                    self._turn_on_mic()
                    user_txt = rospy.wait_for_message('voice_txt', String, timeout=TIMEOUT_VOICE) # lunghezza ultima frase fatta pronunciare a Pepper + costante 
                    self._turn_off_mic()
                except rospy.ROSException:
                    print('RESTART')
                    restart_req = self._make_request('/restart', DialogueRequest())
                    resp = self.dialogue_service(restart_req)
                    print('RISPOSTA del bot al restart', resp.answer)
                    self._turn_off_mic()
                    break
                
                try:
                    start = rospy.Time.now()
                    print("[IN]:", user_txt)
                    user_req = self._make_request(user_txt.data.lower(), DialogueRequest())
                    
                    bot_answer = self.dialogue_service(user_req)
                    
                    print("[OUT]:", bot_answer.answer)

                    bot_answer = self._make_request(bot_answer.answer, Text2SpeechRequest())
                    time_last_utterance = len(bot_answer.speech.split()) * 0.25 # pepper impiega 100 parole/minuto (100 : 60s = #parole : Xs)
                    self.tts_service(bot_answer)

                    print('fine ciclo')
                
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                
                try:
                    rospy.wait_for_message('detection', Int16, timeout=TIMEOUT_DETECTOR) 
                except rospy.ROSException:
                    print('RESTART')
                    restart_req = self._make_request('/restart', DialogueRequest())
                    resp = self.dialogue_service(restart_req)
                    print('RISPOSTA del bot al restart', resp.answer)
                    self._turn_off_mic()
                    break
                end = rospy.Time.now()
                i += 1
                print('fine iterazione ', i)
                print('STO ASPETTANDO time_last_utterance: ', str(time_last_utterance))
                print('TEMPO SPESO SERVIZI: ', str((end - start).to_sec()))
                # print('NOSTRO TEMPO DI ATTESA 2: ', str(time_last_utterance - (end - start).to_sec()))
                if (end - start).to_sec() >= time_last_utterance:
                   # print('NOSTRO TEMPO DI ATTESA 2: ', str((end - start).to_sec()))
                    # rospy.sleep((end - start).to_sec())
                    pass
                else:
                    print('NOSTRO TEMPO DI ATTESA 1: ', str(time_last_utterance - (end - start).to_sec()))
                    rospy.sleep(time_last_utterance - (end - start).to_sec())

            print('sono fuori dal ciclo piu interno')



if __name__ == '__main__':
    TIMEOUT_VOICE = 30
    TIMEOUT_DETECTOR = 5

    try: 
        speaking = Speaking()
        speaking.start()
    except rospy.ROSInterruptException:
        pass