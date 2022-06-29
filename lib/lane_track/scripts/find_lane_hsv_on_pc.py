#!/usr/bin/env python

import cv2,time,rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16MultiArray,Int16

bridge=CvBridge()

HSVLower=np.array([0,0,0])
HSVUpper=np.array([255,255,255])

def hLow1(value):
    HSVLower[0]=value
def hHigh1(value):
    HSVUpper[0]=value
def sLow1(value):
    HSVLower[1]=value
def sHigh1(value):
    HSVUpper[1]=value
def vLow1(value):
    HSVLower[2]=value
def vHigh1(value):
    HSVUpper[2]=value
def white_save_params(x):
    if x == 1:
        whitepubData=[int(HSVLower[0]),int(HSVLower[1]),int(HSVLower[2]),int(HSVUpper[0]),int(HSVUpper[1]),int(HSVUpper[2])]
        whitepubArr=Int16MultiArray(data=whitepubData)
        g_hsvParamsPub.publish(whitepubArr)
        cv2.setTrackbarPos('SAVE White','lineHSV',0)
        print(whitepubData)
        print("save Ok.")
def white_renew(y):
    if y==1:
        renew=1
        renewPub.publish(renew)
        time.sleep(0.05)
        cv2.setTrackbarPos('White renew','lineHSV',0)
        print("renew Ok.")
    if y==0:
        renew=0
        renewPub.publish(renew)

def findHSV(img):
    HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    HSV=cv2.inRange(HSV,HSVLower,HSVUpper)
    return HSV

def callback(msg):
    frame=bridge.imgmsg_to_cv2(msg,"bgr8")
    hsv=findHSV(frame)
    res = cv2.bitwise_and(frame,frame,mask=hsv)
    numpy_horizontal=np.concatenate((frame, res), axis=1)
    cv2.imshow("lineHSV",numpy_horizontal)
    cv2.waitKey(1)

cv2.namedWindow('lineHSV',cv2.WINDOW_AUTOSIZE| cv2.WINDOW_KEEPRATIO)
cv2.createTrackbar('H Low','lineHSV',0,255,hLow1)
cv2.createTrackbar('H High','lineHSV',255,255,hHigh1)
cv2.createTrackbar('S Low','lineHSV',0,255,sLow1)
cv2.createTrackbar('S High','lineHSV',255,255,sHigh1)
cv2.createTrackbar('V Low','lineHSV',0,255,vLow1)
cv2.createTrackbar('V High','lineHSV',255,255,vHigh1)
cv2.createTrackbar('SAVE White','lineHSV',0,1,white_save_params)
cv2.createTrackbar('White renew','lineHSV',0,1,white_renew)

rospy.init_node("find_white_hsv",anonymous=True)
g_hsvParamsPub=rospy.Publisher("white_hsv_params",Int16MultiArray,queue_size=10)
renewPub=rospy.Publisher("white_hsv_renew",Int16,queue_size=10)
cameraSub=rospy.Subscriber("lane_image",Image,callback)
rospy.spin()