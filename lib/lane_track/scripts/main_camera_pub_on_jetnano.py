#!/usr/bin/env python

import cv2,rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge=CvBridge()

cap=cv2.VideoCapture("/dev/maincamera",cv2.CAP_V4L2)

cameraPub=rospy.Publisher("maincamera_image",Image,queue_size=10)

rospy.init_node("main_camera_pub")

while not rospy.is_shutdown():
    ret,frame=cap.read()
    frame=cv2.flip(frame,-1)
    cameraPub.publish(bridge.cv2_to_imgmsg(frame,"bgr8"))

    if cv2.waitKey(1)&0xFF==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()