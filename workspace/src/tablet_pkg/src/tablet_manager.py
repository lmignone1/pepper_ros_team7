#!/usr/bin/python3

import rospy
import socket
from pepper_nodes.srv import LoadUrl, LoadUrlRequest, LoadUrlResponse
from std_msgs.msg import Int16, String
import os

class Handler:
    '''
    The constructor creates the service proxy object, which is able to display the desired URL on the tablet.
    '''
    def __init__(self):
        rospy.init_node('tablet_manager_node')

        rospy.wait_for_service("load_url")
        self.tablet_service = rospy.ServiceProxy("load_url", LoadUrl)
        self._ip = socket.gethostbyname(socket.gethostname())
        self._port = 5000

    '''
    This method calls the tablet service and sends it the URL of the web page to be displayed.
    '''
    def load_url(self, url):
        msg = LoadUrlRequest()
        msg.url = url
        resp = self.tablet_service(msg)
        rospy.loginfo(resp.ack)
    
    def start(self):
        rospy.Subscriber('tablet_template', Int16, self._show_url)
        rospy.Subscriber('voice_txt', String,  self._show_dialogue)
        
        url = f'http://{self._ip}:{self._port}/static/index'
        self.load_url(url)
        rospy.spin()
    
    def _show_url(self, msg):
        if msg.data == 1:
            url = f'http://{self._ip}:{self._port}/engagement'  # si attiva quando il servizio tts pubblica 1 in seguito alla frase 'Hello folks'
        else:
            url = f'http://{self._ip}:{self._port}/static/index'    # si attiva quando il servizio dialogue_server pubblica 0 in seguito al comando '/restart'

        self.load_url(url)

    def _show_dialogue(self, msg):
        url = f'http://{self._ip}:{self._port}/dialogue?text={msg.data}'
        self.load_url(url)

if __name__ == "__main__":

    handler = Handler()
    handler.start()

