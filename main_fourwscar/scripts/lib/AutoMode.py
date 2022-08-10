#!/usr/bin/env python

'''
 *  AutoMode.py - to check and modify data regarding auto mode
 *  Copyright (c) 2022 Realplus Tech.
 *   
 *  A module to check and modify data regarding auto mode
 *
 *  
 *  Designed by Tim J. May/2022
 *
 *  Paramater:  | 0        | 1                 | 2               | 3     | 4      | 5            | 6
 *		| Ifauto   | front_motor_MAX   | back_motor_MAX  | Gear  | Mode   | Direction_FB | Direction_RL
 *
 *  Note:  Mode( moving type ) 1: only for front wheel 2:same direction with the back wheels 3: opposite direction with the back wheels
 *
 *  Command:    | 0                  | 1                  | 2              | 3              | 4                | 5         
 *		| front_motor_speed  | back_motor_speed   | LF_Servo_angle | RF_Servo_Angle | LB_Servo_Angle   | RB_Servo_Angle 
 *
 *
 *  METHOD:
 *
 *  		ifAuto( ifauto, autohold ) - return True or False  # determine if the vehicle is in the auto mode
 *
 * 		getAutoPara( para ) - return para  # implement the auto mode data such as offline to modify parameter

'''

import rospy, cv2
import numpy as np
import sys
import time
from std_msgs.msg import Int16, Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge=CvBridge()

global auto_data
global isCamera
isCamera = 0

global HSVLower
global HSVUpper
HSVLower=np.array([0,0,0])
HSVUpper=np.array([255,255,255])

auto_data = np.array( [0, 0] )
pubKey = rospy.Publisher('Key', Int16, queue_size=10)

global HSVPublisher
global renewPublisher
HSVPublisher = rospy.Publisher('white_hsv_params',Int16MultiArray, queue_size=10)
renewPublisher = rospy.Publisher('white_hsv_renew',Int16,queue_size=10) 

global ImgSubscriber

def setHSV():
	
	global HSVLower
	global HSVUpper

	global renewPublisher
	global HSVPublisher

	whitepubData=[int(HSVLower[0]),int(HSVLower[1]),int(HSVLower[2]),int(HSVUpper[0]),int(HSVUpper[1]),int(HSVUpper[2])]
	whitepubArr=Int16MultiArray(data=whitepubData)

        HSVPublisher.publish(whitepubArr)
	renewPublisher.publish(1)
	


def watchCamera( tmp ):
	
	global isCamera
	isCamera = tmp
	
def showImg( msg ):
	
	global isCamera

	if( isCamera ):
		frame=bridge.imgmsg_to_cv2(msg,"bgr8")
		cv2.imshow("frame", frame)
		cv2.waitKey(1)

def getDataForAutoMode( data ):

	global auto_data
	auto_data = data.data
	
	
	'''
	print( data.data[0] )     # line offset
	print( "\n" )
	print( data.data[1] )	  # if line exist
	'''


def ifAuto( ifauto, autohold ):
	

	global AutoSubscriber
	global renewPublisher
	
	renewPublisher.publish(0)

	print "ifauto=", ifauto, "autohold=", autohold

	if( ifauto ==  autohold and ifauto == 1.0 ):
		
		pubKey.publish( 1 )
		return ifauto

	elif( ifauto == autohold and ifauto == 0.0):
		
		pubKey.publish( 0 )
		return ifauto

	else:

		if( ifauto == 1.0 and autohold == 0.0 ):

			pubKey.publish( 1 )
			AutoSubscriber = rospy.Subscriber("lane_status", Int16MultiArray, getDataForAutoMode ) 
			#ImgSubscriber = rospy.Subscriber("lane_image", Image, showImg ) 
			print "subscribe"

		elif( ifauto == 0.0  and autohold == 1.0 ):

			pubKey.publish( 0 )
			AutoSubscriber.unregister()
			#ImgSubscriber.unregister()

			print "unregister"
		elif( ifauto == -1 ):
			print " STOP !! "
			return 0

		else:

			sys.exit( "error in ifAuto" )

		return ifauto

ImgSubscriber = rospy.Subscriber("lane_image", Image, showImg )

'''
***************************************************************

	implement auto data to modify parameters

***************************************************************
'''

def getAutoPara( para ):

	global auto_data

	lane_width = 224.0
	

	if( auto_data[1]): 
		para[6] = float(auto_data[0])/ (lane_width/2)

	print "offset=", auto_data[0], "RF=", para

	return para

if __name__ == '__main__':

	para = np.array( [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ] )
	cmd = np.array( [ 0, 0, 0, 0] )

	tmp = 0
	autohold = 0
	
	while True:


		tmp += 1

		if( tmp%5 == 0):
			para[0] = (para[0]+1)%2


		if( ifAuto( para[0], autohold) ):

			para = getAutoPara( para )
			autohold = 1	


		else:
			autohold = 0

		#cmd = parapro.generateCmd( para )
		#JetToVehicle.publish( cmd )

		print para[0]

		time.sleep(2)


		
		
	



