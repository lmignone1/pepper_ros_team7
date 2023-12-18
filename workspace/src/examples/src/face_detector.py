#!/usr/bin/python3
import numpy as np
import cv2
import tensorflow as tf
import os
import rospy
from sensor_msgs.msg import Image
import ros_numpy 
from cv_bridge import CvBridge, CvBridgeError

faceProto = '/home/lorx/catkin_ws/src/cogrob_pepper_nodes/src/examples/src/opencv_face_detector.pbtxt'
faceModel = '/home/lorx/catkin_ws/src/cogrob_pepper_nodes/src/examples/src/opencv_face_detector_uint8.pb'
faceNet = cv2.dnn.readNet(faceModel, faceProto)


def getFaceBox(net, frame, conf_threshold=0.8):
    frameOpencvDnn = frame.copy()
    frameHeight = frameOpencvDnn.shape[0]
    frameWidth = frameOpencvDnn.shape[1]
    #swapRB =True
    # flag which indicates that swap first and last channels in 3-channel image is necessary.
    #crop = False
    # flag which indicates whether image will be cropped after resize or not
    # If crop is false, direct resize without cropping and preserving aspect ratio is performed
    blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], True, False)
    net.setInput(blob)
    detections = net.forward()
    print(detections.shape)
    bboxes = []
        
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold and detections[0, 0, i, 5]<1 and detections[0, 0, i, 6]<1:
            x1 = int(detections[0, 0, i, 3] * frameWidth)
            y1 = int(detections[0, 0, i, 4] * frameHeight)
            x2 = int(detections[0, 0, i, 5] * frameWidth)
            y2 = int(detections[0, 0, i, 6] * frameHeight)
            bboxes.append([x1, y1, x2, y2])
            cv2.rectangle(frameOpencvDnn, (x1, y1), (x2, y2), (0, 255, 0), int(round(frameHeight/300)), 8)
            print(f"Rectangle drawn at coordinates: Top Left ({x1}, {y1}), Bottom Right ({x2}, {y2})")
            cv2.imshow("Demo", frameOpencvDnn)
            cv2.waitKey(1)

    if len(bboxes) == 0:
        print("No face detected")
        cv2.imshow("Demo", frameOpencvDnn)
        cv2.waitKey(1)

    return frameOpencvDnn, bboxes


def rcv_image(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e) 
    frameFace, bboxes = getFaceBox(faceNet, cv_image)


rospy.init_node('face_detector')
si = rospy.Subscriber("in_rgb", Image, rcv_image)
# si = rospy.Subscriber("image_raw", Image, rcv_image)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")


