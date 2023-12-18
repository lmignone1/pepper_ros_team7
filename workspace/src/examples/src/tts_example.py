#!/usr/bin/python3

import rospy
from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse

class Handler:
    '''
    The constructor creates the service proxy object, which is able to make the robot speak
    '''
    def __init__(self):
        self.tts = rospy.ServiceProxy("/tts", Text2Speech)

    '''
    This method calls the Text to Speech service and sends it the desired text to be played.
    '''
    def call(self, text: str):
        msg = Text2SpeechRequest()
        msg.speech = text
        resp = self.tts(text)
        rospy.loginfo(resp.ack)

if __name__ == "__main__":
    NODE_NAME = "tts_node_example"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    text = "Hi! I'm Pepper, the robotic guardian of the mall. Do you want to ask me anything?" + "\\pau=3000\\" +\
    "Bye" + "\\pau=3000\\" +\
    "Would you like to provide additional details to your counting task answering a few questions?" + "\\pau=3000\\" +\
    "It was a plaisure to help you" + "\\pau=3000\\" + \
    "I'm here to help you finding who you are looking for, but before you need to give me some other details by answering a few questions upperColour, lowerColour, kindOfPeople, hat, bag, place, duration, comparative, not_upper, not_lower" + "\\pau=3000\\" +\
    "Are you interested in males, females or people in general?" + "\\pau=3000\\" +\
    "What colour should be the upper clothing of the people involved in the count?" + "\\pau=3000\\" +\
    "What colour should be the lower clothing of the people involved in the count?" + "\\pau=3000\\" +\
    "Are you interested in people with hat, without hat, or both?" + "\\pau=3000\\" +\
    "Are you interested in people with bag, without bag, or both?" + "\\pau=3000\\" +\
    "Do you want to consider the entire mall, sisa or caldarelli?" + "\\pau=3000\\" +\
    "Are you looking for somebody who is male or female?" + "\\pau=3000\\" +\
    "What is the main colour of the upper clothes?" +  "\\pau=3000\\" +\
    "What is the main colour of the lower clothes?" +  "\\pau=3000\\" +\
    "Are you looking for someone with a hat or without?" +  "\\pau=3000\\" +\
    "Are you looking for someone with a bag or without?" +  "\\pau=3000\\" +\
    "I can give you information about this mall. In particular, I can count and locate people based on their physical attributes. Tell me your question!" + "\\pau=3000\\" +\
    "I'm sorry, I didn't understand you. Repeat, please?" + "\\pau=3000\\" +\
    "Sorry, my mistake. Could you please repeat your counting task request?" + "\\pau=3000\\" +\
    "Sorry, my mistake. Could you please repeat your localization task request?" + "\\pau=3000\\" +\
    "Okay, so... tell me your request, please" + "\\pau=3000\\" +\
    "Okay, don't mind"
    
    # text to speech della classe ActionConfirmationCount
    text2 =  "So, if I understand well, you want to count females with red upper clothes, without blue lower clothes, with a hat, without a bag, who have been in sisa for more than 10 minutes. Is it right?" + "\\pau=3000\\" +\
    "So, if I understand well, you want to count people with upper clothes of any colour, with lower clothes of any colour, without a hat, without a bag. Is it right?" + "\\pau=3000\\" +\
    "So, if I understand well, you want to count males without upper clothes, with black lower clothes, with a hat, with a bag, who have been in caldarelli for less than 5 minutes. Is it right?"


  
    text3 = "I have found one person that matches your localization request. Such person has passed through sisa 23 times and has spent a total of 50 seconds there, and has passed through caldarelli 34 times and has spent a total of 12 seconds there." + "\\pau=3000\\" +\
    "I have found 3 people that match your localization request.\
    The first person has passed through sisa 23 times and has spent a total of 45 seconds there, and has passed through caldarelli 2 times and has spent a total of 2 seconds there.\
    The second person has passed through sisa 3 times and has spent a total of 25 seconds there, and has passed through caldarelli times and has spent a total of 4 seconds there.\
    The third person has passed through sisa 0 times and has spent a total of 0 seconds there, and has passed through caldarelli 3 times and has spent a total of 3 seconds there." + "\\pau=3000\\" +\
    "I have not found anyone that matches your counting request" + "\\pau=3000\\" +\
    "I have found one person that matches your counting request" + "\\pau=3000\\" +\
    "I have found 27 people that match your counting request"

    # text to speech della classe ActionConfirmationLocation
    text4 = "So, if I understand well, you want to localize a female with a hat, with a bag, with red upper clothes, without blue lower clothes. Is it right?" + "\\pau=3000\\" +\
    "So, if I understand well, you want to localize a male without a hat, without a bag, with upper clothes of any colour, without black lower clothes. Is it right?" "\\pau=3000\\" +\
    "So, if I understand well, you want to localize a person with both a hat and a bag, with green upper clothes, with yellow lower clothes. Is it right?"

    
    
    handler.call(text)
