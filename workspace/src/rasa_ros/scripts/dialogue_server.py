#!/usr/bin/env python3
from rasa_ros.srv import Dialogue, DialogueResponse

import rospy
import requests
from std_msgs.msg import Int16


def handle_service(req):
    input_text = req.input_text   
    if input_text == '/restart':
        pub.publish(0)
    # Get answer        
    get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
    message = {
        "sender": 'bot',
        "message": input_text
    }

    r = requests.post(get_answer_url, json=message)
    response = DialogueResponse()
    response.answer = ""
    for i in r.json():
        response.answer += i['text'] + ' ' if 'text' in i else ''

    return response

def main():

    # Server Initialization
    
    s = rospy.Service('dialogue_server',
                        Dialogue, handle_service)

    rospy.logdebug('Dialogue server READY.')
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('dialogue_service')
    pub = rospy.Publisher('tablet_template', Int16, queue_size=1)
    try: 
        main()
    except rospy.ROSInterruptException as e:
        pass
