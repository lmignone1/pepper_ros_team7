#!/usr/bin/python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def publish_image():
    """Capture frames from a camera and publish it to the topic /image_raw
    """
    image_pub = rospy.Publisher("/in_rgb", Image, queue_size=1)
    bridge = CvBridge()
    capture = cv2.VideoCapture("/dev/video0")

    while not rospy.is_shutdown():
        # Capture a frame
        ret, img = capture.read()
        if not ret:
            rospy.ERROR("Could not grab a frame!")
            break
        # Publish the image to the topic "/in_rgb"
        try:
            img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
            image_pub.publish(img_msg)
            # rospy.sleep(0.1)
        except CvBridgeError as error:
            print(error)

if __name__=="__main__":
    rospy.init_node("my_cam", anonymous=True)
    print("Image is being published to the topic /in_rgb ...")
    publish_image()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
