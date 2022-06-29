#!/usr/bin/env python

import cv2,rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge=CvBridge()

cap=cv2.VideoCapture(0)

rospy.init_node("camera_pub",anonymous=True)
cameraPub=rospy.Publisher("camera_pub",Image,queue_size=10)

while not rospy.is_shutdown():
    ret,frame=cap.read()
    cameraPub.publish(bridge.cv2_to_imgmsg(frame,"bgr8"))
    cv2.imshow("frame",frame)
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()