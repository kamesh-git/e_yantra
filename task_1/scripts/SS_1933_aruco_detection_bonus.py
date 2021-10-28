#!/usr/bin/env python3

############## Task1.1 - ArUco Detection ##############
### YOU CAN EDIT THIS FILE FOR DEBUGGING PURPOSEs, SO THAT YOU CAN TEST YOUR ArUco_library.py AGAINST THE VIDEO Undetected ArUco markers.avi###
### BUT MAKE SURE THAT YOU UNDO ALL THE CHANGES YOU HAVE MADE FOR DEBUGGING PURPOSES BEFORE TESTING AGAINST THE TEST IMAGES ###

import numpy as np
import cv2
import cv2.aruco as aruco
import time
from SS_1933_aruco_library import *


cam=cv2.VideoCapture("aruco.mp4")

while True:
	time.sleep(0.01)
	_,img = cam.read()
	Detected_ArUco_markers = detect_ArUco(img)									## detecting ArUco ids and returning ArUco dictionary
	angle = Calculate_orientation_in_degree(Detected_ArUco_markers)				## finding orientation of aruco with respective to the menitoned scale in problem statement
	img = mark_ArUco(img,Detected_ArUco_markers,angle)						## marking the parameters of aruco which are mentioned in the problem statement
	result_image = ".png"
	cv2.imshow(result_image,img)					## saving the result image
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cv2.destroyAllWindows()