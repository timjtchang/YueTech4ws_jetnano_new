'''
*  Paramater:  | 0        | 1             | 2           | 3     | 4      | 5         | 6         | 7         | 8         | 9            | 10
		       | Ifauto   | front_motor   | back_motor  | Gear  | Mode   | Servo_LF  | Servo_RF  | Servo_LB  | Servo_RB  | Direction_FB | Direction_RL

   Command:    | 0                  | 1                  | 2              | 3              | 4                | 5         
		       | front_motor_speed  | back_motor_speed   | LF_Servo_angle | RF_Servo_Angle | LB_Servo_Angle   | RB_Servo_Angle 

	
'''

import rospy
import numpy as np
import sys
import time
from std_msgs.msg import Int16, Int16MultiArray

global auto_data

auto_data = np.array( [0, 0] )
pubKey = rospy.Publisher('Key', Int16, queue_size=10)

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

	if( ifauto ==  autohold ):

		return ifauto

	else:

		if( ifauto == 1 and autohold == 0 ):

			pubKey.publish( 1 )
			AutoSubscriber = rospy.Subscriber("lane_status", Int16MultiArray, getDataForAutoMode ) 
			print "subscribe"

		elif( ifauto == 0  and autohold == 1 ):

			pubKey.publish( 0 )
			AutoSubscriber.unregister()
			print "unregister"
		elif( ifauto == -1 ):
			print " STOP !! "
			return 0

		else:

			sys.exit( "error in ifAuto" )

		return ifauto

def getAutoPara( para ):

	global auto_data

	print "to modify auto data"
	
	print auto_data

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


		
		
	



