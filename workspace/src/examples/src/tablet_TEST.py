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
        rospy.wait_for_service("load_url")
        self.tablet_service = rospy.ServiceProxy("load_url", LoadUrl)
        rospy.init_node('table_node_example')
        rospy.Subscriber('tablet_template', Int16, self._show_url)
        rospy.Subscriber('voice_txt', String,  self._show_dialogue)
        self._ip = socket.gethostbyname(socket.gethostname())

    '''
    This method calls the tablet service and sends it the URL of the web page to be displayed.
    '''
    def load_url(self, url):
        msg = LoadUrlRequest()
        msg.url = url
        resp = self.tablet_service(msg)
        rospy.loginfo(resp.ack)
    
    def start(self):
        url = "http://" + self._ip + ":5000/static/index"
        self.load_url(url)
        rospy.spin()
    
    def _show_url(self, msg):
        if msg.data == 1:
            print('Passaggio alla schermata di engagement')
            url = "http://" + self._ip + ":5000/engagement"
        else:
            print('Passaggio alla schermata di index')
            url = "http://" + self._ip + ":5000/static/index"

        self.load_url(url)

    def _show_dialogue(self, msg):
        print('Passaggio alla schermata di dialogo')
        url = "http://" + self._ip + ":5000/dialogue"
        with open(PATH, 'w') as f:
            f.write(msg.data)
        self.load_url(url)

if __name__ == "__main__":
    PATH = os.path.join(os.path.dirname(__file__), 'flask', 'payload.txt')

    handler = Handler()
    handler.start()

    # url = "http://" + ip + ":5000"
    # url = url + "/index1"
    # # url = r"https://www.diem.unisa.it/"
    # handler.load_url(url)
