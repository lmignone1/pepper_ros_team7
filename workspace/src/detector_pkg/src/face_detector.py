#!/usr/bin/python3
import numpy as np
import cv2
import tensorflow as tf
import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import ros_numpy 
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

class Detector():

    def __init__(self):
        # inizializzazione nodo ROS
        rospy.init_node('face_detector')
        self._pub = None
        
        # creazione path per rete neurale
        path = os.path.dirname(__file__)
        faceProtoPath = path + "/opencv_face_detector.pbtxt"
        faceModelPath = path + "/opencv_face_detector_uint8.pb"
        
        # caricamento rete neurale
        self._faceNet = cv2.dnn.readNet(faceModelPath, faceProtoPath)
        
        # inizializzazione deque
        self._deque = deque(maxlen=MAXLEN)
        for _ in range(MAXLEN): 
            self._deque.append(0) 
        
        self._bridge = CvBridge()
        print("inizializzazione completata")
        
      


    def _getFaceBox(self, frame, conf_threshold=0.8):
        height = frame.shape[0]
        width = frame.shape[1]
        #swapRB =True
        # flag which indicates that swap first and last channels in 3-channel image is necessary.
        #crop = False
        # flag which indicates whether image will be cropped after resize or not
        # If crop is false, direct resize without cropping and preserving aspect ratio is performed
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], True, False)
        self._faceNet.setInput(blob)
        detections = self._faceNet.forward()

        print(detections.shape)
        
        bboxes = []
            
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]  
            if confidence > conf_threshold and detections[0, 0, i, 5]<1 and detections[0, 0, i, 6]<1:
                x1 = int(detections[0, 0, i, 3] * width)
                y1 = int(detections[0, 0, i, 4] * height)
                x2 = int(detections[0, 0, i, 5] * width)
                y2 = int(detections[0, 0, i, 6] * height)
                bboxes.append([x1, y1, x2, y2])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), int(round(height/300)), 8)
                print(f"Rectangle drawn at coordinates: Top Left ({x1}, {y1}), Bottom Right ({x2}, {y2})")
                cv2.imshow("Demo", frame)
                cv2.waitKey(1)

        if len(bboxes) == 0:
            print("No face detected")
            cv2.imshow("Demo", frame)
            cv2.waitKey(1)
            detected = 0
        else:
            detected = 1
        
        return detected, bboxes


    def _rcv_image(self, frame):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(frame)
            detected, _ = self._getFaceBox(cv_image, CONFIDENCE_TH)
            self._deque.append(detected)
        except CvBridgeError as e:
            print('Image conversion failed')
    
    def start(self):
        rospy.Subscriber("/in_rgb", Image, self._rcv_image)
        self._pub = rospy.Publisher('detection', Int16, queue_size=1)
        
        while not rospy.is_shutdown():
            res = sum(i for i in self._deque)

            if res >= self._deque.maxlen/2:
                self._pub.publish(1)
            
            rospy.sleep(INTERVAL_TIME)


if __name__ == '__main__':
    FPS = 20    # frame rate della camera
    INTERVAL_TIME = 0.5   # il tempo che intercorre mediamente tra una pubblicazione e la successiva sul topic /detection
    MAXLEN = int(INTERVAL_TIME * FPS)          # trade-off between performance and responsiveness
    CONFIDENCE_TH = 0.7  

    try:
        detector = Detector()
        detector.start()
    except KeyboardInterrupt:
        print("Shutting down")


