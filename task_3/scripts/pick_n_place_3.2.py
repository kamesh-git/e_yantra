#!/usr/bin/env python3

from pickle import TRUE
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import cv2
import numpy as np
from std_msgs.msg import String
import math
import time
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import *



class aruco_library():

    def __init__(self):
        self.Detected_ArUco_markers={}
        self.ArUco_marker_angles={}
        self.img = np.empty([]) # This will contain your image frame from camera



        



    def detect_ArUco(self,img):
        ## function to detect ArUco markers in the image using ArUco library
        ## argument: img is the test image
        ## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
        ## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
        ## 				{0: array([[315, 163],
        #							[319, 263],
        #				[219, 267],
        #							[215,167]], dtype=float32)}

        ## enter your code here ##
        img=img.copy()
        # cv2.imshow("detect_aruco",img)
        # if cv2.waitKey(1) & 0xFF == ord('r'):
        #     cv2.destroyAllWindows()

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corner, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        try:	
            for i in range(len(ids)):
                self.Detected_ArUco_markers[str(*ids[i])]=corner[i].astype('i')
        except:
            self.Detected_ArUco_markers={}
        if ids != None : 
            self.img = self.Calculate_orientation_in_degree(img,self.Detected_ArUco_markers)
        else:
            self.img = img
        cv2.imshow("aruco_lib",img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()



    def Calculate_orientation_in_degree(self,img,Detected_ArUco_markers):
        ## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
        ## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
        ## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
        ##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
        ##			function should return: {1: 120 , 2: 164}

        ## enter your code here ##
        for i in list(Detected_ArUco_markers.keys()):
            corners=Detected_ArUco_markers[i]
            center = corners[0]
            mid=(int((corners[0][0][0]+corners[0][1][0])/2),int((corners[0][0][1]+corners[0][1][1])/2))
            M = cv2.moments(center)
            self.cX = int(M["m10"] / M["m00"])
            self.cY = int(M["m01"] / M["m00"])
            coor=((self.cX,self.cY),mid)
            ang = math.degrees(math.atan2(coor[0][1]-coor[1][1],coor[1][0]-coor[0][0]))
            ang=round(ang)
            ang=(ang+360)%360

            self.ArUco_marker_angles[i]=ang
            img=self.mark_ArUco(img,Detected_ArUco_markers,self.ArUco_marker_angles)
            return img


    def mark_ArUco(self,img,Detected_ArUco_markers,ArUco_marker_angles):
        ## function to mark ArUco in the test image as per the instructions given in problem statement
        ## arguments: img is the test image 
        ##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
        ##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
        ## return: image namely img after marking the aruco as per the instruction given in problem statement

        ## enter your code here ##

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
            cv2.putText(img,str(cX)+str(",")+str(cY),(cX,cY-70),cv2.FONT_HERSHEY_COMPLEX,0.8,(0,0,255),thickness=2)
            cv2.line(img, (cX,cY),mid,(255,0,0) ,2)
        return img

 


class image_proc():

	# Initialise everything
	def __init__(self,aruco_obj):
		self.aruco_obj=aruco_obj
		
		# Making a publisher 
		
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
			# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/eDrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
			# -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		


	# Callback function of camera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.img=cv2.resize(self.img,(400,400))
		except CvBridgeError as e:
			print(e)
			return			
		self.aruco_obj.detect_ArUco(self.img)

			

            
class pick_n_place():
    def __init__(self,aruco_obj,image_proc_obj):
        rospy.init_node('pick_n_place',anonymous=True)
        self.aruco_obj=aruco_obj
        self.image_proc_obj=image_proc_obj
        self.state=State()
        self.reached=False   # used in position function to check if the drone reached the setpoints
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)    # used in goto function
        self.local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20.0)
        self.in_range=False     #to check if the box in range



    def statecb(self,msg):
        self.state=msg




    # arming or disarming the drone
    def setArm(self,arm_bool):
        if arm_bool:
            res="ARM"
        else:
            res="DISARM"
        while not (self.state.armed==arm_bool):
            rospy.wait_for_service('mavros/cmd/arming')
            try:
                armService=rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
                armService(arm_bool)
            except rospy.ServiceException as e:
                print("failed to arm:%s"%e)
            print(res+"ING...",self.state.armed)
            self.rate.sleep()
        print(res+"ED!!!")





    # setting any mode to the drone
    def setMode(self,mode_val):
        while not (self.state.mode==mode_val):    
            rospy.wait_for_service('mavros/set_mode')
            try:
                modeService=rospy.ServiceProxy('mavros/set_mode',SetMode)
                modeService(custom_mode=mode_val)
            except rospy.ServiceException as e:
                print("failed to setmode:%s"%e)
            print(mode_val+" not yet activated, current_mode:"+self.state.mode)
            self.rate.sleep()
        print(mode_val+" activated")




    # navigating drone to required setpoint !!!!!!!!!!!!!
    def goto(self,setpoints,vel,grip_activ=False):
        self.reached=False
        self.setpoints=setpoints
        self.vel=vel
        self.local_pose=rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.position)
        while not self.reached:
            # img=self.aruco_obj.img
            # cv2.imshow("original",self.image_proc_obj.img)
            # cv2.imshow("aruco img",img)
            # if cv2.waitKey(1) & 0xFF == ord('w'):
            #     cv2.destroyAllWindows()
            self.local_pos_pub.publish(self.setpoints)
            self.local_vel_pub.publish(self.vel)
            self.rate.sleep()
            if len(list(self.aruco_obj.Detected_ArUco_markers.keys())) and grip_activ:   #True- when the box found in camera
                print("entered box file")
                self.box_grip()
                grip_activ=False
        self.local_pose.unregister()


    def position(self,msg):
        self.x=self.setpoints.pose.position.x
        self.y=self.setpoints.pose.position.y
        self.z=self.setpoints.pose.position.z
        self.xn=msg.pose.position.x
        self.yn=msg.pose.position.y
        self.zn=msg.pose.position.z
        tol=0.1
        # print(self.x,self.y,self.z,self.xn,self.yn,self.zn)
        if (self.xn<self.x+tol and self.xn>self.x-tol) and (self.yn<self.y+tol and self.yn>self.y-tol) and (self.zn<self.z+tol and self.zn>self.z-tol):
            print("reached the stpoint")
            self.reached=True
 
    def box_grip(self):
       self.box_pos=PoseStamped()
       self.box_pos.pose.position.x,self.box_pos.pose.position.y,self.box_pos.pose.position.z=self.xn+1,self.yn+1,self.zn     #adding 1 because of inertia
       print(self.xn,self.yn,self.zn)
       for i in range(100):
           self.local_vel_pub.publish(self.vel)
           self.local_pos_pub.publish(self.box_pos)
           self.rate.sleep()
       while True:
           self.x_diff=(self.aruco_obj.cX-200)*0.01546153846153846       #g/p ratio:0.01546153846153846,0.015228426395939089
           self.y_diff=(200-self.aruco_obj.cY)*0.01546153846153846+0.5
           self.box_pos.pose.position.x=self.xn+self.x_diff
           self.box_pos.pose.position.y=self.yn+self.y_diff
           self.box_pos.pose.position.z=self.z
           print(self.x_diff,self.y_diff,list(self.aruco_obj.Detected_ArUco_markers.keys()))
           time.sleep(0.5)
           if self.x_diff!=0 and self.y_diff!=0 and len(list(self.aruco_obj.Detected_ArUco_markers.keys())):
                self.local_vel_pub.publish(self.vel)
                self.local_pos_pub.publish(self.box_pos)
                self.rate.sleep()
                print(".....",end="")
                self.aruco_obj.Detected_ArUco_markers={}
                # time.sleep(60)
           elif (self.x_diff>=0 and self.x_diff<=0.1) and (self.y_diff>=0 and self.y_diff<=0.4) and len(list(self.aruco_obj.Detected_ArUco_markers.keys())):
               break
       self.setMode("AUTO.LAND")
       self.gripper_check=rospy.Subscriber("/gripper_check",String,self.gripper_check_cb)
       print("landed on box")



        

        

    def gripper_check_cb(self,msg_bool):
        print(msg_bool)
        if msg_bool.data == "True":
            self.gripper_check.unregister()
            self.activate_gripper(True)
            time.sleep(5)
            self.setMode("OFFBOARD")
            for i in range(100):
                self.local_pos_pub.publish(self.box_pos)
                self.local_vel_pub.publish(self.vel)
                self.rate.sleep()

            # while not msg_bool.data == "True":
            #     self.local_pos_pub.publish(self.setpoints)
                # print("not in range")


    def activate_gripper(self,att_bool):
        rospy.wait_for_service('/activate_gripper')
        try:
            set_gripper=rospy.ServiceProxy('/activate_gripper',Gripper)
            resp=set_gripper(att_bool)
        except rospy.ServiceException as e:
            print("unable to call the service:%s"%e)
        if not resp:
            self.activate_gripper(att_bool)     
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


def main():
    aruco_obj=aruco_library()
    image_proc_obj=image_proc(aruco_obj)
    drone=pick_n_place(aruco_obj,image_proc_obj)
    rospy.Subscriber("/mavros/state",State, drone.statecb)

    setpoints=[]

    pos =PoseStamped()

    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.y = 1
    vel.linear.z = 1
    vel.linear.x = 1
    
    # Similarly add other containers 
    pos1 =PoseStamped()
    pos2 =PoseStamped()
    pos3 =PoseStamped()
    pos4 =PoseStamped()
    pos5 =PoseStamped()
    pos6 =PoseStamped()
    pos7 =PoseStamped()
    pos8 =PoseStamped()


    pos1.pose.position.x = 0
    pos1.pose.position.y = 0
    pos1.pose.position.z = 3
    setpoints.append(pos1)

    pos2.pose.position.x = 9
    pos2.pose.position.y = 0
    pos2.pose.position.z = 3
    setpoints.append(pos2)


    pos3.pose.position.x = 9
    pos3.pose.position.y = 0
    pos3.pose.position.z = 0
    setpoints.append(pos3)
    
    pos4.pose.position.x = 3
    pos4.pose.position.y = 0
    pos4.pose.position.z = 3
    setpoints.append(pos4)

    pos5.pose.position.x = 3
    pos5.pose.position.y = 3
    pos5.pose.position.z = 3
    setpoints.append(pos5)

    pos6.pose.position.x = 3
    pos6.pose.position.y = 3
    pos6.pose.position.z = 0
    setpoints.append(pos6)

    pos7.pose.position.x = 3
    pos7.pose.position.y = 3
    pos7.pose.position.z = 3
    setpoints.append(pos7)

    pos8.pose.position.x = 0
    pos8.pose.position.y = 0
    pos8.pose.position.z = 3
    setpoints.append(pos8)

    setpoints.append(pos)
    print("publishing dummy points....")
    for i in range(200):
        drone.local_pos_pub.publish(pos)
        drone.rate.sleep()
    drone.setArm(True)
    drone.setMode("OFFBOARD")
    drone.goto(pos1,vel)
    drone.goto(pos2,vel,True)
    drone.goto(pos3,vel)
    drone.setMode("AUTO.LAND")
    time.sleep(10)
    drone.activate_gripper(False)
    drone.setMode("OFFBOARD")
    drone.goto(pos2,vel)
    drone.goto(pos1,vel)
    drone.goto(pos,vel)
    drone.setMode("AUTO.LAND")
    drone.setArm(False)

    # while not rospy.is_shutdown():
    #     x,y,z=tuple([float(i) for i in input("enter coor:").split()])
    #     pos.pose.position.x = x
    #     pos.pose.position.y = y
    #     pos.pose.position.z = z
    #     drone.goto(pos,vel)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass