#!/usr/bin/env python

# just for test and will be deleted in the formal version

import controller as ctr
import pygame
import numpy as np

if __name__ == '__main__':

	para = np.array( [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ] )
	
	while True:
		
		para = ctr.updateParaFromController( para )

		# Limited to 30 frames per second to make the display not so flashy
		clock = pygame.time.Clock()
	    	clock.tick(30)

		print "Mode=", para[4], "\n"
		print "Ifauto=", para[0], "\n"
		print "Gear=", para[3], "\n"
		print "Direction_FB=", para[9], "\n"
		print "Direction_RL=", para[10], "\n"
