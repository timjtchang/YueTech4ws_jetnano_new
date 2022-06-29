#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

def callback(data):
	
	print( data.data )

def listener():
	
	rospy.init_node('subParameter', anonymous=True)
	rospy.Subscriber("Parameter", Int32MultiArray, callback)
	print( "hello" )

	rospy.spin()

if __name__ == '__main__':
	listener()
