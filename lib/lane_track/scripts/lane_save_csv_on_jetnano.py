#!/usr/bin/env python
import csv,rospy,os
from std_msgs.msg import Int16MultiArray

HOME=os.getenv('HOME')
HSV_PATH=HOME+'/catkin_ws/src/AICar/lane_track/csv'
WHITE_CSVFILE=HSV_PATH+'/white_hsv.csv'

def white_paramsCallback(msg):
    with open(WHITE_CSVFILE,'w') as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow(['H','S','V'])
        writer.writerow([str(msg.data[0]),str(msg.data[1]),str(msg.data[2])])
        writer.writerow([str(msg.data[3]),str(msg.data[4]),str(msg.data[5])])
        rospy.loginfo("White HSV Params save ok.")

rospy.init_node('lane_save_hsv',anonymous=True)
g_hsvParamsSub=rospy.Subscriber('white_hsv_params',Int16MultiArray,white_paramsCallback)
rospy.spin()