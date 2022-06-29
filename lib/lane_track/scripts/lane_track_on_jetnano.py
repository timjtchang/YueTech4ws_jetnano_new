#!/usr/bin/env python

import cv2
import rospy
import os
import numpy as np
from std_msgs.msg import Int16, Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

HOME = os.getenv('HOME')
HSV_PATH = HOME+'/catkin_ws/src/lib/lane_track/csv'
GREEN_CSVFILE = HSV_PATH+'/white_hsv.csv'

hsvLower = np.array([0, 0, 0])
hsvUpper = np.array([255, 255, 255])
grayScale = [220, 255]

cap = cv2.VideoCapture("/dev/maincamera", cv2.CAP_V4L2)
# cap=cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

roi_params = [
    (0, (h*70)/100),
    ((w*15)/100, (h*10)/100),
    ((w*85)/100, (h*10)/100),
    (w, (h*70)/100)
]

start, renew = 0, 0

offsetPub = rospy.Publisher("lane_status", Int16MultiArray, queue_size=10)
cameraPub = rospy.Publisher("lane_image", Image, queue_size=10)

rospy.init_node("lane_track_node")


def readHSV():
    global hsvLower, hsvUpper
    with open(GREEN_CSVFILE, 'r') as w_csvfile:
        data = np.genfromtxt(w_csvfile, delimiter=',', skip_header=1)
        hsvLower = np.array(data[0])
        hsvUpper = np.array(data[1])
    print(hsvLower)
    print(hsvUpper)


def fusionMask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.inRange(hsv, hsvLower, hsvUpper)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.inRange(gray, grayScale[0], grayScale[1])
    mask = cv2.bitwise_or(hsv, gray)
    # return hsv
    return mask, hsv, gray


def hsvMask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsvLower, hsvUpper)
    return mask


def roiMask(img, roi):
    mask = np.zeros_like(img)
    match_mask_color = (255, 255, 255)
    cv2.fillPoly(mask, np.array([roi], np.int32), match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def blurMask(img, kernel):
    return cv2.GaussianBlur(img, (kernel, kernel), 0)


def cannyMask(img, low_threhold=50, high_threshold=100):
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(img, kernel, iterations=1)
    dilation = cv2.dilate(erosion, kernel, iterations=1)
    return cv2.Canny(dilation, low_threhold, high_threshold)


def hough_lines(img, rho=1, theta=np.pi/180, threshold=30, min_line_len=10, max_line_gap=6):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array(
        []), minLineLength=min_line_len, maxLineGap=max_line_gap)
    return lines


def get_slope(x1, y1, x2, y2):
    slope_y = float(y2-y1)
    slope_x = float(x2-x1)
    if slope_x == 0:
        slope_x = 1
    return slope_y/slope_x


def group_lines_and_draw(img, lines, roi=roi_params):
    left_x, left_y, right_x, right_y = [], [], [], []
    det_slope = 0.4

    # print(roi[0])

    # min_y = int((img.shape[0]*10)/100)
    # max_y = int((img.shape[0]*70)/100)
    min_y=int(roi[1][1])
    max_y=int(roi[0][1])
    max_x = int((img.shape[1]))
    left_x_limit = int((img.shape[1]*30)/100)
    right_x_limit = int((img.shape[1]*70)/100)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                # print("%d %d %d %d" % (x1, y1, x2, y2))
                slope = get_slope(x1, y1, x2, y2)
                # print("slope %.2f" % slope)
                if slope < -det_slope:
                    left_x.extend([x1, x2])
                    left_y.extend([y1, y2])
                if slope > det_slope:
                    right_x.extend([x1, x2])
                    right_y.extend([y1, y2])
        if len(left_x) != 0:
            poly_left = np.poly1d(np.polyfit(left_y, left_x, deg=1))
            left_x_start = int(poly_left(max_y))
            left_x_end = int(poly_left(min_y))
            if left_x_end > left_x_limit:
                left_x_end = left_x_limit
            if left_x_start > left_x_limit:
                left_x_start = 0
                left_line = 2
            # if left_x_start>left_x_limit:

            else:
                left_line = 1
        else:
            left_x_start, left_x_end, left_line = 0, 0, 0

        if len(right_x) != 0:
            poly_right = np.poly1d(np.polyfit(right_y, right_x, deg=1))
            right_x_start = int(poly_right(max_y))
            right_x_end = int(poly_right(min_y))
            if right_x_end < right_x_limit:
                right_x_end = right_x_limit
            if right_x_start < right_x_limit:
                right_x_start = max_x
                right_line = 1
            else:
                right_line = 2
        else:
            right_x_start, right_x_end, right_line = max_x, max_x, 0
    else:
        left_x_start, left_x_end, left_line = 0, 0, 0
        right_x_start, right_x_end, right_line = max_x, max_x, 0

    line_status = left_line+right_line
    # rospy.loginfo("Line Status:%d"%line_status)

    # rospy.loginfo("left_start:%d left_end:%d right_start:%d right_end:%d"%(left_x_start,left_x_end,right_x_start,right_x_end))

    line_img = draw_lines(img, [[
        [left_x_start, max_y, left_x_end, min_y],
        [right_x_start, max_y, right_x_end, min_y]
    ]], thickness=5)
    return line_img, left_x_start, left_x_end, right_x_start, right_x_end, line_status


def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
    if lines is None:
        return
    img = np.copy(img)
    img_channels = img.shape[2]
    line_img = np.zeros(
        (img.shape[0], img.shape[1], img_channels), dtype=np.uint8)
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
    return img


def drawROI(img, roi, color=(0, 255, 0), thickness=2):
    cv2.line(img, roi[0], roi[1], color, thickness)
    cv2.line(img, roi[1], roi[2], color, thickness)
    cv2.line(img, roi[2], roi[3], color, thickness)
    cv2.line(img, roi[3], roi[0], color, thickness)


def startCallback(data):
    global start
    start = data.data


def renewCallback(data):
    global renew
    renew = data.data


statSub = rospy.Subscriber("Key", Int16, startCallback)
renewSub = rospy.Subscriber("white_hsv_renew", Int16, renewCallback)

readHSV()
while not rospy.is_shutdown():
    ret, frame = cap.read()
    # frame=cv2.flip(frame,-1)
    if renew == 1:
        readHSV()
    if start == 1:
        blur = blurMask(frame, 5)
        fusion, hsv, gray = fusionMask(blur)
        roi = roiMask(hsv, roi_params)
        canny = cannyMask(roi)
        lines = hough_lines(canny)
        lineImg, lx1, lx2, rx1, rx2, lineExist = group_lines_and_draw(
            frame, lines)
        center_x = int(w/2)
        offset_1 = int((lx1+rx1)/2)
        offset_2 = int((lx2+rx2)/2)
        offset = int(((offset_1+offset_2)/2)-center_x)
        cv2.line(lineImg, (center_x, (h*45)/100), (center_x,
                 (h*65)/100), (255, 255, 0), 2, cv2.LINE_AA)
        cv2.line(lineImg, (offset+center_x, (h*40)/100),
                 (offset+center_x, (h*70)/100), (0, 0, 255), 2, cv2.LINE_AA)

        # cv2.line(lineImg, (int(w*30)/100, 0), (int(w*30)/100,
        #          h), (255, 0, 0), 2, cv2.LINE_AA)
        # cv2.line(lineImg, (int(w*70)/100, 0), (int(w*70)/100,
        #          h), (255, 0, 0), 2, cv2.LINE_AA)

        pubData = [offset, lineExist]
        drawROI(lineImg, roi_params)
        # print(offset)

        # if lines is not None:
        #     allLine = draw_lines(frame, lines)
        #     cameraPub.publish(bridge.cv2_to_imgmsg(allLine, "rgb8"))

        # cv2.imshow("All Line Image",allLine)
        # cv2.imshow("HSV",hsv)
        # cv2.imshow("gray",gray)
        # cv2.imshow("fusion Image",fusion)
        # cv2.imshow("blur Image",blur)
        # cv2.imshow("roi Image",roi)
        # cv2.imshow("canny Image",canny)
        # cv2.imshow("Line Image",lineImg)

        # cameraPub.publish(bridge.cv2_to_imgmsg(canny,"mono8"))
        # cameraPub.publish(bridge.cv2_to_imgmsg(gray,"mono8"))
        # cameraPub.publish(bridge.cv2_to_imgmsg(fusion,"mono8"))
        cameraPub.publish(bridge.cv2_to_imgmsg(lineImg, "rgb8"))
    else:
        pubData = [0, 0]
        cameraPub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

    pubArr = Int16MultiArray(data=pubData)
    offsetPub.publish(pubArr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
