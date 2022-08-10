#!/usr/bin/env python

'''
 *  main.py - the main structure to communicate among stm32, ui, controller
 *  Copyright (c) 2022 Realplus Tech.
 *   
 *  An archetecture to manage all dataflow among stm32 motor side, UI side, and contoller
 *  
 *  Designed by Tim J. May/2022
 *
 *  Paramater:  | 0        | 1                 | 2               | 3     | 4      | 5            | 6
 *		| Ifauto   | front_motor_MAX   | back_motor_MAX  | Gear  | Mode   | Direction_FB | Direction_RL

 *  Note:  Mode( moving type ) 1: only for front wheel 2:same direction with the back wheels 3: opposite direction with the back wheels

    Command:    | 0                  | 1                  | 2              | 3              | 4                | 5         
		| front_motor_speed  | back_motor_speed   | LF_Servo_angle | RF_Servo_Angle | LB_Servo_Angle   | RB_Servo_Angle 

'''
import rospy
import time
import sys
import numpy as np
import os

from std_msgs.msg import Int32, Int32MultiArray, Int16MultiArray, Int16, Float32MultiArray

from lib import controller as ctr
from lib import AutoMode as auto
from lib import ParameterProcessor as parapro

global RECORD_ADDRESS
HOME=os.getenv('HOME')
RECORD_ADDRESS = HOME+'/catkin_ws/src/main_fourwscar/scripts/record.txt'


rospy.init_node('mainOnJetsonNano', anonymous=True)
pubKey = rospy.Publisher('Key', Int16, queue_size=10)

'''
***********************************************************************

	send parameter to UI

***********************************************************************
'''

def sendParameter( para ):
	
	pub = rospy.Publisher('Parameter', Float32MultiArray, queue_size=10)
	pubdata = Float32MultiArray(data=para)
	pub.publish( pubdata )

'''
*************************************************************************

	update parameter from UI

*************************************************************************
'''


def updateParameter(data):
	
	global para 
	para = np.array( data.data )

	sendParameter( para )

UISubscriber = rospy.Subscriber("ParameterFromUI", Float32MultiArray, updateParameter)
SonarSubscriber = rospy.Subscriber("Sonar", Int16MultiArray, parapro.setSonar)

'''
***********************************************************************

	main

***********************************************************************
'''

if __name__ == '__main__':

	global para 
	para = np.array( [ 0.0, 90.0, 90.0, 0.0, 1.0, 0.0, 0.0 ] )  	# default parameter

	para = parapro.readPara( RECORD_ADDRESS )			# read record 

	ctr.setDefaultPara( para )					# set default parameter for controller
	
	parapro.setSonarLimit( 5 ) 					# set sonar limit value
	parapro.setReflectTime( 0.6 )					# set sonar relect Time

	parapro.setGearCoefficient( 0.5 )				# set coeffcient for getting servo angle by calculating gear

	cmd = np.array( [ 0, 0, 0, 0 ] )
	autohold = 0.0

	#auto.watchCamera(1)

	try:	
		sendParameter( para )					# send parameter to UI

		while not rospy.is_shutdown():

			para = ctr.updateParaFromController( para )	# update new parameter from controller
			para = parapro.checkPara( para )		# check if the value of parmeters is out of bounds
			sendParameter( para )				# send parameter to UI	
	
			if( auto.ifAuto( para[0], autohold) ):		# if in the auto mode
				para = auto.getAutoPara( para )		# modify parameters with auto data
				autohold = 1.0	

			else:
				autohold = 0.0
			
			sonar = parapro.getSonar()			# update sonar value
			print "sonar value=", sonar

			print "para=", para	

			cmd = parapro.generateCmd( para, sonar )	# generate command by translating parameter
			print "cmd=", cmd
			parapro.pubCmd( cmd )				# send command to stm32
			parapro.writePara( para, RECORD_ADDRESS  )
			time.sleep(0.05)

	except rospy.ROSInterruptException:
		pass

