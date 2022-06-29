#!/usr/bin/env python

'''
 *  controller.py - For YunTech project, reading data from controller and interecting with main code
 *  Copyright (c) 2022 Realplus Tech.
 *  
 *  Dependent on numpy and pygame ( installed the libraries before using )
 *   
 *  To get command from controller and update Parameter transmitted from main code
 *
 *  Default key value of controller can be modified by assgining value to member varaible below:
 * 
 *  	BUTTON_LB  = 4
 *	BUTTON_RB  = 5
 *	BUTTON_Y   = 3
 *	BUTTON_X   = 2
 *	BUTTON_B   = 1
 *	BUTTON_A   = 0
 *	BUTTON_MIN = 6
 *	BUTTON_PLU = 7 
 *
 *	AXIS_Y = 1
 *	AXIS_X = 0 	
 * 
 *  A member function:
 *	
 *      para updateParaFromController( para )   // to get command from controller and update parameter
 *						// expected to be put in a loop
 *  
 *  Designed by Tim J. May/2022

'''

import pygame
import sys
import numpy as np
import time
import copy

pygame.init()
pygame.joystick.init()

controller = pygame.joystick.Joystick(0)
controller.init()

# Three types of controls: axis, button, and hat
axis = {}
button = {}
hat = {}

# Labels for DS4 controller axes
AXIS_Y = 1
AXIS_X = 0


# Labels for DS4 controller buttons
# Note that there are 14 buttons (0 to 13 for pygame, 1 to 14 for Windows setup)
BUTTON_LB  = 4
BUTTON_RB  = 5
BUTTON_Y   = 3
BUTTON_X   = 2
BUTTON_B   = 1
BUTTON_A   = 0
BUTTON_MIN = 6
BUTTON_PLU = 7

# Labels for DS4 controller hats (Only one hat control)
HAT_1 = 0

# Assign initial data values
# Axes are initialized to 0.0
for i in range(controller.get_numaxes()): 

	axis[i] = 0.0
	
# Buttons are initialized to False
for i in range(controller.get_numbuttons()): 

	button[i] = False
	
# Hats are initialized to 0
for i in range(controller.get_numhats()): 
 
	hat[i] = (0, 0)

def setDefaultPara( tmp_para ):

	global default_para
	default_para = copy.deepcopy( tmp_para )


def updateParaFromController( para ):


	# Get events
	for event in pygame.event.get():
		if event.type == pygame.JOYAXISMOTION:

		    	axis[event.axis] = round(event.value,3)

		elif event.type == pygame.JOYBUTTONDOWN:
				    	
			button[event.button] = True

		elif event.type == pygame.JOYBUTTONUP:

		    	button[event.button] = False

		elif event.type == pygame.JOYHATMOTION:
		    				
			hat[event.hat] = event.value
	
	# update mode parameter

	if( button[ BUTTON_Y ] ):

		para[4] = 1
		
	elif( button[ BUTTON_X ] ):

		para[4] = 2

	elif( button[ BUTTON_B ] ):

		para[4] = 3
	
	# update Ifauto patameter

	if( button[ BUTTON_A ] ):

		if( para[0] ):
		    para[0] = 0

		elif( para[0] == 0 ):
		    para[0] = 1

		time.sleep(0.15)

	# update Gear parameter

	if( button[ BUTTON_PLU ] ):
		
		global default_para
		
		return copy.deepcopy( default_para )

	if( button[ BUTTON_RB ] ):

		para[3] = para[3]+1
		time.sleep(0.15)

	elif( button[ BUTTON_LB ] ):

		para[3] = para[3]-1
		time.sleep(0.15)

	# update Direction Forward or Backward

	if(   axis[ AXIS_Y ] == 0 ):
		
		para[9] = 0

	elif( axis[ AXIS_Y ]>0 ):

		para[9] = -1

	elif( axis[ AXIS_Y ] < 0 ):

		para[9] = 1

	# update Direction Right or Left 

	if(   axis[ AXIS_X ] == 0 ):

		para[10] = 0

	elif( axis[ AXIS_X ] > 0 ):

		para[10] = 1

	elif( axis[ AXIS_X ] < 0 ):

		para[10] = -1

    
	# quit python when Button Minus pushed
    	
	if( button[BUTTON_MIN] ):
		
		para = np.array( [ -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ] )

	return para


if __name__ == '__main__':

	para = np.array( [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ] )
	
	while True:
		
		para = updateParaFromController( para )

		# Limited to 30 frames per second to make the display not so flashy
		clock = pygame.time.Clock()
	        clock.tick(30)

		print "Mode=", para[4], "\n"
		print "Ifauto=", para[0], "\n"
		print "Gear=", para[3], "\n"
		print "Direction_FB=", para[9], "\n"
		print "Direction_RL=", para[10], "\n"
	










