#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
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

    ## enter your code here ##
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	corner, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	corner=corner[0].astype('i')
	ang,x,y = Calculate_orientation_in_degree(corner)
	return int(str(*ids[0])),x,y,ang


def Calculate_orientation_in_degree(corners):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	## enter your code here ##

	center = corners[0]
	mid=(int((corners[0][0][0]+corners[0][1][0])/2),int((corners[0][0][1]+corners[0][1][1])/2))
	M = cv2.moments(center)
	cX = int(M["m10"] / M["m00"])
	cY = int(M["m01"] / M["m00"])
	coor=((cX,cY),mid)
	ang = math.degrees(math.atan2(coor[0][1]-coor[1][1],coor[1][0]-coor[0][0]))
	ang=round(ang)
	ang=(ang+360)%360
	return ang,cX,cY


class image_proc():

	# Initialise everything
	def __init__(self,aruco_obj):
		self.aruco_obj=aruco_obj
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker


	# Callback function of camera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return
			
	def publish_data(self):
		self.marker_pub.publish(self.marker_msg)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    mar_msg=image_proc_obj.marker_msg
    frequency = rospy.Rate(10)
    while True:
        try:
            mar_msg.id,mar_msg.x,mar_msg.y,mar_msg.yaw=detect_ArUco(image_proc_obj.img)
        except:
            pass
        image_proc_obj.publish_data()
        frequency.sleep()
        if cv2.waitKey(1)& 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    rospy.spin()
    