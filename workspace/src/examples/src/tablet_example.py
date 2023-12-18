#!/usr/bin/python3

import rospy
from pepper_nodes.srv import LoadUrl, LoadUrlRequest, LoadUrlResponse

class Handler:
    '''
    The constructor creates the service proxy object, which is able to display the desired URL on the tablet.
    '''
    def __init__(self):
        self.tablet_service = rospy.ServiceProxy("load_url", LoadUrl)

    '''
    This method calls the tablet service and sends it the URL of the web page to be displayed.
    '''
    def load_url(self, url):
        msg = LoadUrlRequest()
        msg.url = url
        resp = self.tablet_service(msg)
        rospy.loginfo(resp.ack)

if __name__ == "__main__":
    NODE_NAME = "table_node_example"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    url = r"https://www.diem.unisa.it/"
    handler.load_url(url)
