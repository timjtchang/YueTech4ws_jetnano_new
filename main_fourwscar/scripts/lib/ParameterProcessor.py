#!/usr/bin/env python
'''
 *  ParameterProcesser.py - to process parameter for either chekcing if it is out of bounds or applying it to generate command
 *  Copyright (c) 2022 Realplus Tech.
 *   
 *  A module to process parameter
 *
 *  
 *  Designed by Tim J. May/2022
 *
 *  Paramater:  | 0        | 1             | 2           | 3     | 4      | 5         | 6         | 7         | 8         | 9            | 10
 *		| Ifauto   | front_motor   | back_motor  | Gear  | Mode   | Servo_LF  | Servo_RF  | Servo_LB  | Servo_RB  | Direction_FB | Direction_RL
 *
 *  Note:  Mode( moving type ) 1: only for front wheel 2:same direction with the back wheels 3: opposite direction with the back wheels
 *
 *  Command:    | 0                  | 1                  | 2              | 3              | 4                | 5         
 *		| front_motor_speed  | back_motor_speed   | LF_Servo_angle | RF_Servo_Angle | LB_Servo_Angle   | RB_Servo_Angle 
 *
 *		 90 Left (\\)  0 striaight (||)   -90 Right (//)
 *
 *
 *  METHOD:
 *
 *  		para checkPara( para ) - to check if the servo and motor speed are out of bounds, to implement gear, and modify paramenters
 *		
 *
 *		cmd generateCmd( para ) - to generate command for controling the vehicle by caculating the parameter 		

'''

import numpy as np
import time
import sys
import rospy
from std_msgs.msg import Int32MultiArray

global motor_speed_limit
motor_speed_limit = 300

global FIXED_ANGLE
FIXED_ANGLE = 0
global ANGLE
ANGLE = 90

def setMaxSpeed( speed ):
	
	global motor_speed_limit
	motor_speed_limit = speed
	
	return motor_speed_limit


def modifyGear( para ):

	if( para[3] < 1 ):
		
		para[3] = 1

	elif( para[3] > 3 ):
		para[3] = 1

	return para


def limitMotor( motor_speed ):

	global motor_speed_limit

	if( motor_speed >= 0  and motor_speed <= motor_speed_limit ):

		return motor_speed

	elif( motor_speed > motor_speed_limit ):

		return motor_speed_limit

	else:

		return 0


def checkPara( para ):


	para = modifyGear( para )

	for i in range( 1, 3 ):

		para[i] = limitMotor( para[i] )


	return para

def setFixedAngle( angle ):
	
	global FIXED_ANGLE
	global ANGLE

	FIXED_ANGLE = 1
	ANGLE = angle
	
	return ANGLE


def generateCmd( para ):

	global cmd
	cmd = np.array( [ 0, 0, 0, 0, 0, 0  ] )

	global FIXED_ANGLE
	global ANGLE

	if( FIXED_ANGLE == 0 ):
		ANGLE = 90

	elif( FIXED_ANGLE == 1 ):
		
		if( para[6] == 0 ):
			para[6] = 0
		
		elif( para[6] > 0 ):
			para[6] = 1

		elif( para[6] < 0 ):
			para[6] = -1


	else:
		sys.exit( "angle error" )

	if( para[0] == -1 ): #emergency stop
		return np.array( [ -1, -1, -1, -1, -1, -1  ] )

	'''

	speed

	'''

	cmd[0] = para[1]*para[5]
	cmd[1] = para[2]*para[5]

	'''

	mode & servo

	'''

	cmd[2] = para[6]*ANGLE   # angle*direction*(-1)
	cmd[3] = para[6]*ANGLE

	if( para[4] == 1 ):

		cmd[4] = 0
		cmd[5] = 0

	elif( para[4] == 2):

		cmd[4] = para[6]*ANGLE
		cmd[5] = para[6]*ANGLE

	elif( para[4] == 3 ):

		cmd[4] = -para[6]*ANGLE
		cmd[5] = -para[6]*ANGLE

	else:

		sys.exit( "mode error" )


	return cmd

def pubCmd( cmd ):
	
	cmdpub = rospy.Publisher('JetToStm32', Int32MultiArray, queue_size=10)

	pubdata = Int32MultiArray(data=cmd)
	cmdpub.publish( pubdata )

	return

if __name__ == '__main__':

	para = np.array( [ 0.0, 100.0, 100.0, 0.0, 1.0, 90.0, 1.0, 1.0] ) 
	cmd = np.array( [ 0, 0, 0, 0, 0, 0 ] )
	
	while True:
		

		for i in range(-1, 2 ):

			para[9] = i

			for t in range( -1, 2 ):

				para[10] = t
				para = checkPara( para )
				cmd  = generateCmd( para )

				print "para = ", para
				print "cmd=", cmd

				time.sleep(3)

		'''
		for i in range( -50, 400, 10 ):


			para[5] = i
			para[1] = i

			print "orginial", para
			para = checkPara( para )

			print "new",para
			time.sleep(1)
		'''

	





















