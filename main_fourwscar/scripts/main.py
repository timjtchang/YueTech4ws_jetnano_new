#!/usr/bin/env python

'''
 *  main.py - the main structure to communicate among stm32, ui, controller
 *  Copyright (c) 2022 Realplus Tech.
 *   
 *  An archetecture to manage all dataflow among stm32 motor side, UI side, and contoller
 *  
 *  Designed by Tim J. May/2022
 *
 *  Paramater:  | 0        | 1             | 2           | 3     | 4      | 5         | 6         | 7         | 8         | 9            | 10
 *		| Ifauto   | front_motor   | back_motor  | Gear  | Mode   | Servo_LF  | Servo_RF  | Servo_LB  | Servo_RB  | Direction_FB | Direction_RL

 *  Note:  Mode( moving type ) 1: only for front wheel 2:same direction with the back wheels 3: opposite direction with the back wheels

    Command:    | 0                  | 1                  | 2              | 3              | 4                | 5         
		| front_motor_speed  | back_motor_speed   | LF_Servo_angle | RF_Servo_Angle | LB_Servo_Angle   |

'''
import rospy
import time
import sys
import numpy as np
import controller as ctr
import AutoMode as auto
import ParameterProcessor as parapro
from std_msgs.msg import Int32, Int32MultiArray, Int16MultiArray, Int16


rospy.init_node('mainOnJetsonNano', anonymous=True)
pubKey = rospy.Publisher('Key', Int16, queue_size=10)

def updateParameter(data):
	
	global para 
	para = np.array( data.data )

	sendParameter( para )

	
UISubscriber = rospy.Subscriber("ParameterFromUI", Int32MultiArray, updateParameter)


def sendParameter( para ):
	
	pub = rospy.Publisher('Parameter', Int32MultiArray, queue_size=10)
	pubdata = Int32MultiArray(data=para)
	pub.publish( pubdata )
		

if __name__ == '__main__':

	global para 
	para = np.array( [ 0, 200, 200, 0, 1, 30, 30, 30, 30, 0, 0 ] )
	ctr.setDefaultPara( para )

	cmd = np.array( [ 0, 0, 0, 0 ] )
	autohold = 0

	try:	
		sendParameter( para )

		while not rospy.is_shutdown():

			para = ctr.updateParaFromController( para )
			para = parapro.checkPara( para )
			sendParameter( para )

			print "para=", para

			if( auto.ifAuto( para[0], autohold) ):
				para = auto.getAutoPara( para )
				autohold = 1	

			else:
				autohold = 0

			cmd = parapro.generateCmd( para )
			print "cmd=", cmd
			parapro.pubCmd( cmd )
			time.sleep(0.05)

	except rospy.ROSInterruptException:
		pass

