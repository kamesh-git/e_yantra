#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

	Detected_ArUco_markers = {}
    ## enter your code here ##
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	corner, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	print(corner,ids)
	try:	
		for i in range(len(ids)):
			Detected_ArUco_markers[str(*ids[i])]=corner[i].astype('i')
	except:
		pass
	return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	## enter your code here ##
	for i in list(Detected_ArUco_markers.keys()):
		corners=Detected_ArUco_markers[i]
		center = corners[0]
		mid=(int((corners[0][0][0]+corners[0][1][0])/2),int((corners[0][0][1]+corners[0][1][1])/2))
		M = cv2.moments(center)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		coor=((cX,cY),mid)
		ang = math.degrees(math.atan2(coor[0][1]-coor[1][1],coor[1][0]-coor[0][0]))
		ang=round(ang)
		print("ang",ang)
		ang=(ang+360)%360

		ArUco_marker_angles[i]=ang


	print("\n\nang",ArUco_marker_angles)
	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##

	print(ArUco_marker_angles)
	for i in list(Detected_ArUco_markers.keys()):
		corners=Detected_ArUco_markers[i]
		center = corners[0]
		mid=(int((corners[0][0][0]+corners[0][1][0])/2),int((corners[0][0][1]+corners[0][1][1])/2))
		M = cv2.moments(center)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		ang=ArUco_marker_angles[i]
		cv2.circle(img, (cX, cY), 1, (0, 0, 255), 8)
		cv2.circle(img,(corners[0][0][0],corners[0][0][1]), 6, (128,128,128), -1)
		cv2.circle(img,(corners[0][1][0],corners[0][1][1]), 6, (0,255,0), -1)
		cv2.circle(img,(corners[0][2][0],corners[0][2][1]), 6, (255,105,180), -1)
		cv2.circle(img,(corners[0][3][0],corners[0][3][1]), 6, ((255,255,255)), -1)
		cv2.putText(img,i,(cX+10,cY),cv2.FONT_HERSHEY_COMPLEX,0.8,(0,0,255),thickness=2)
		cv2.putText(img,str(ang),(cX-130,cY),cv2.FONT_HERSHEY_COMPLEX,0.8,(0,0,255),thickness=2)
		cv2.line(img, (cX,cY),mid,(255,0,0) ,2)
	# img=cv2.resize(img,(1000,800))
	return img


