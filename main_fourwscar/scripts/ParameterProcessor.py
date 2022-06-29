#!/usr/bin/env python

'''
*  Paramater:  | 0        | 1             | 2           | 3     | 4      | 5         | 6         | 7         | 8         | 9            | 10
		       | Ifauto   | front_motor   | back_motor  | Gear  | Mode   | Servo_LF  | Servo_RF  | Servo_LB  | Servo_RB  | Direction_FB | Direction_RL

   Command:    | 0                  | 1                  | 2              | 3              | 4                | 5         
		       | front_motor_speed  | back_motor_speed   | LF_Servo_angle | RF_Servo_Angle | LB_Servo_Angle   | RB_Servo_Angle 


		       90 Left (\\)  0 striaight (||)   -90 Right (//)
'''

import numpy as np
import time
import sys

global motor_speed_limit
motor_speed_limit = 300


def limitServo( servo_degree ):

	if( servo_degree >= -90 and servo_degree <= 90):

		return servo_degree

	elif( servo_degree < -90):

		return -90

	elif( servo_degree > 90 ):

		return 90

def modifyGear( para ):

	if( para[3] < 0 ):
		para[3] = 0

	if( para[3] == 0 ):

		return para

	else:

		para[7] = para[5]*para[3]
		para[8] = para[6]*para[3]

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

	for i in range( 5,9 ):
		para[i] = limitServo( para[i] )

	return para


def generateCmd( para ):

	global cmd
	cmd = np.array( [ 0, 0, 0, 0, 0, 0  ] )

	if( para[0] == -1 ): #emergency stop
		return np.array( [ -1, -1, -1, -1, -1, -1  ] )

	'''

	speed

	'''

	cmd[0] = para[1]*para[9]
	cmd[1] = para[2]*para[9]

	'''

	mode & servo

	'''

	cmd[2] = para[5]*para[10]*(-1)    # angle*direction*(-1)
	cmd[3] = para[6]*para[10]*(-1)

	if( para[4] == 1 ):

		cmd[4] = 0
		cmd[5] = 0

	elif( para[4] == 2):

		cmd[4] = para[7]*para[10]*(-1)
		cmd[5] = para[8]*para[10]*(-1)

	elif( para[4] == 3 ):

		cmd[4] = para[7]*para[10]*(-1)*(-1)
		cmd[5] = para[8]*para[10]*(-1)*(-1)

	else:

		sys.exit( "mode error" )


	return cmd
def pubCmd( cmd ):

	if( cmd[0] == -1 and cmd[1] == -1 ):
		# pub ( 0 , 0, 0, 0, 0, 0 )
		sys.exit( " Stop !! " )
	return

if __name__ == '__main__':

	para = np.array( [ 0, 100, 100, 0, 1, 90, 90, 90, 90, -1, 1] ) 
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

	





















