#!/usr/bin/env python3
from threading import Thread
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import cv2
import numpy as np
from std_msgs.msg import String,UInt8
import math
import time
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import *


class aruco_library():

    def __init__(self,drone_num):
        self.drone_num=drone_num
        self.Detected_ArUco_markers={}
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
        # img=img.copy()
        # cv2.imshow('img',img)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #         cv2.destroyAllWindows()
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corner, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        try:	
            i=0
            self.Detected_ArUco_markers[str(*ids[i])]=corner[i].astype('i')
            self.Calculate_orientation_in_degree(img,self.Detected_ArUco_markers)
        except:
            self.Detected_ArUco_markers={}



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
            M = cv2.moments(center)
            self.cX = int(M["m10"] / M["m00"])
            self.cY = int(M["m01"] / M["m00"])


 


class image_proc():

	# Initialise everything
	def __init__(self,aruco_obj,drone_num):
		self.aruco_obj=aruco_obj
		
		# Making a publisher 
		
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
			# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber(drone_num+"/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
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
    def __init__(self,aruco_obj,image_proc_obj,drone_num):
        self.drone_num=drone_num
        self.aruco_obj=aruco_obj
        self.image_proc_obj=image_proc_obj
        self.state=State()
        self.reached=False   # used in position function to check if the drone reached the setpoints
        self.local_pos_pub = rospy.Publisher(self.drone_num+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)    # used in goto function
        self.local_vel_pub = rospy.Publisher(self.drone_num+'/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
        self.local_pos_sub=rospy.Subscriber(self.drone_num+'/mavros/local_position/pose',PoseStamped,self.position)

        self.rate = rospy.Rate(20.0)
        self.in_range=False     #to check if the box in range
        self.loop_num=0



    def statecb(self,msg):
        self.state=msg




    # arming or disarming the drone
    def setArm(self,arm_bool):
        if arm_bool:
            res="ARM"
        else:
            res="DISARM"
        while not (self.state.armed==arm_bool):
            rospy.wait_for_service(self.drone_num+'/mavros/cmd/arming')
            try:
                armService=rospy.ServiceProxy(self.drone_num+'/mavros/cmd/arming',CommandBool)
                armService(arm_bool)
            except rospy.ServiceException as e:
                print("failed to arm:%s"%e)
            print(res+"ING...",self.drone_num)
            self.rate.sleep()
        print(res+"ED!!!")





    # setting any mode to the drone
    def setMode(self,mode_val):
        while not (self.state.mode==mode_val):    
            rospy.wait_for_service(self.drone_num+'/mavros/set_mode')
            try:
                modeService=rospy.ServiceProxy(self.drone_num+'/mavros/set_mode',SetMode)
                modeService(custom_mode=mode_val)
            except rospy.ServiceException as e:
                print("failed to setmode:%s"%e)
            print(mode_val+" not yet activated, current_mode:"+self.state.mode)
            self.rate.sleep()
        print(mode_val+" activated")



    def autoLand(self,tol=0.1):
        while not (self.state.mode=='AUTO.LAND' and self.zn<tol):    
            rospy.wait_for_service(self.drone_num+'/mavros/set_mode')
            try:
                modeService=rospy.ServiceProxy(self.drone_num+'/mavros/set_mode',SetMode)
                modeService(custom_mode='AUTO.LAND')
            except rospy.ServiceException as e:
                print("failed to setmode:%s"%e)
            print("current_mode:"+self.state.mode,'and not landed')
            self.rate.sleep()
        print(" Landed")



    # navigating drone to required setpoint !!!!!!!!!!!!!
    def goto(self,setpoints,vel,x=0,grip_activ=False,xy_tol=0.2,z_tol=0.2):
        self.setpoints=PoseStamped()
        self.vel=vel
        self.setpoints.pose.position.y=setpoints.pose.position.y
        self.setpoints.pose.position.z=setpoints.pose.position.z
        self.x=x
        self.setpoints.pose.position.x=x
        self.y=self.setpoints.pose.position.y
        self.z=self.setpoints.pose.position.z



        self.time_check=time.time()
        while not ((self.xn<self.x+xy_tol and self.xn>self.x-xy_tol) and (self.yn<self.y+xy_tol and self.yn>self.y-xy_tol) and (self.zn<self.z+z_tol and self.zn>self.z-z_tol)):
            self.local_pos_pub.publish(self.setpoints)
            self.local_vel_pub.publish(vel)
            self.rate.sleep()
            if len(list(self.aruco_obj.Detected_ArUco_markers.keys())) and grip_activ:   #True- when the box found in camera
                self.box_grip()
                return
        if self.x < setpoints.pose.position.x:
            setpoints.pose.position.z=3
            x+=4
            self.goto(setpoints,vel,x,True)
        print("reached the setpoint")



    def position(self,msg):
        self.xn=msg.pose.position.x
        self.yn=msg.pose.position.y
        self.zn=msg.pose.position.z


 
    def box_grip(self):
       self.correction()
       self.setpoints.pose.position.x,self.setpoints.pose.position.y=(self.xn,self.yn)
       self.activate_gripper(True)
       time.sleep(2)
       self.setMode('OFFBOARD')
       self.goto(self.setpoints,self.vel,self.setpoints.pose.position.x)
       self.goto(self.truck_pos,self.vel,self.truck_pos.pose.position.x+0.85)
       self.goto(self.truck_pos,self.vel,self.truck_pos.pose.position.x)
       self.autoLand(3)
       time.sleep(4)
       self.activate_gripper(False)
       self.setMode('OFFBOARD')
       self.goto(self.truck_pos,self.vel,self.truck_pos.pose.position.x,xy_tol=0.2,z_tol=0.5)
       self.in_range=False


    def correction(self):
       self.box_pos=PoseStamped()
       print("in the loop")
       while True:
           self.x_diff=(self.aruco_obj.cX-200)*0.01546153846153846      #g/p ratio:0.01546153846153846,0.015228426395939089
           self.y_diff=(200-self.aruco_obj.cY)*0.01546153846153846+0.4
           self.box_pos.pose.position.x=self.xn+self.x_diff
           self.box_pos.pose.position.y=self.yn+self.y_diff
           self.box_pos.pose.position.z=self.z
           print(self.x_diff,self.y_diff,list(self.aruco_obj.Detected_ArUco_markers.keys()))
        #    time.sleep(0.5)
           if (self.x_diff>=-0.04 and self.x_diff<=0.03) and (self.y_diff>=-0.015 and self.y_diff<=0.03) and len(list(self.aruco_obj.Detected_ArUco_markers.keys())):
               break
           elif (self.y_diff>=-0.5 and self.y_diff<=0.5):
            self.local_vel_pub.publish(self.vel)
            self.local_pos_pub.publish(self.box_pos)
            self.rate.sleep()
            print(".....",end="")
       print(self.xn,self.yn)
       self.truck_pos=ret_truck_pos(list(self.aruco_obj.Detected_ArUco_markers.keys())[0],self.drone_num)
       self.autoLand()
       self.gripper_check=rospy.Subscriber(self.drone_num+"/gripper_check",String,self.gripper_check_cb)
       print("landed on box")


    def gripper_check_cb(self,msg_bool):
        print(msg_bool)
        if msg_bool.data == "True":
            self.gripper_check.unregister()
            self.in_range=True
            self.loop_num=0
        if self.loop_num==50:
            self.gripper_check.unregister()
            self.setMode('OFFBOARD')
            self.correction()
            self.loop_num=0
        self.loop_num+=1


            

    def activate_gripper(self,att_bool):
        rospy.wait_for_service(self.drone_num+'/activate_gripper')
        while not self.in_range:
            pass
        try:
            set_gripper=rospy.ServiceProxy(self.drone_num+'/activate_gripper',Gripper)
            resp=set_gripper(att_bool)
        except rospy.ServiceException as e:
            print("unable to call the service:%s"%e)
        if not resp:
            self.activate_gripper(att_bool)     
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

class drones():
    def drone_1(self):
        self.drone1_aruco_obj=aruco_library('/edrone0')
        self.drone1_image_proc_obj=image_proc(self.drone1_aruco_obj,'/edrone0')



        pos =PoseStamped()

        pos.pose.position.x = 0
        pos.pose.position.y = 0
        pos.pose.position.z = 3

        # Set your velocity here
        vel = Twist()
        vel.linear.y = 5
        vel.linear.z = 5
        vel.linear.x = 5
        pos1=PoseStamped()

        self.drone1=pick_n_place(self.drone1_aruco_obj,self.drone1_image_proc_obj,'/edrone0')
        rospy.Subscriber("/edrone0/mavros/state",State, self.drone1.statecb)
        print("publishing dummy points....")
        for i in range(200):
            print(i)
            self.drone1.local_pos_pub.publish(pos)
            self.drone1.local_vel_pub.publish(vel)
            self.drone1.rate.sleep()
        self.drone1.setArm(True)
        self.drone1.setMode("OFFBOARD")
        self.drone1.goto(pos,vel,z_tol=0.5)
        for i in row_list_num:
            y=row_list_num.pop(0)*4-4
            pos1.pose.position.x = 100
            pos1.pose.position.y = y
            pos1.pose.position.z = 3
            self.drone1.goto(pos1,vel,xy_tol=0.1)

        self.drone1.goto(pos,vel)        
        self.drone1.autoLand()
        self.drone1.setArm(False)
        



    def drone_2(self):
        self.drone2_aruco_obj=aruco_library('/edrone1')
        self.drone2_image_proc_obj=image_proc(self.drone2_aruco_obj,'/edrone1')
        
        pos =PoseStamped()

        pos.pose.position.x = 0
        pos.pose.position.y = 0
        pos.pose.position.z = 5

        # Set your velocity here
        vel = Twist()
        vel.linear.y = 5
        vel.linear.z = 5
        vel.linear.x = 5
        pos1=PoseStamped()

        

        self.drone2=pick_n_place(self.drone2_aruco_obj,self.drone2_image_proc_obj,'/edrone1')
        rospy.Subscriber("/edrone1/mavros/state",State, self.drone2.statecb)
        print("publishing dummy points....")
        for i in range(200):
            self.drone2.local_pos_pub.publish(pos)
            self.drone2.rate.sleep()
        self.drone2.setArm(True)
        self.drone2.setMode("OFFBOARD")
        print(pos)
        self.drone2.goto(pos,vel,z_tol=0.5)
        for i in row_list_num:
            y=-((16-row_list_num.pop(0))*4)
            pos1.pose.position.x = 100
            pos1.pose.position.y = y
            pos1.pose.position.z = 4
            self.drone2.goto(pos1,vel,xy_tol=0.2)
        self.drone2.goto(pos,vel)        
        self.drone2.autoLand()
        self.drone2.setArm(False)


    def camera_feed(self):
        while True:
            cv2.imshow("drone 1",self.drone1_image_proc_obj.img)
            cv2.imshow("drone 2",self.drone2_image_proc_obj.img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

def row_list(msg):
    row_list_num.append(msg.data)
    print(row_list_num)


def ret_truck_pos(box_id,drone_num):
    truck_pos=PoseStamped()
    if drone_num=='/edrone0':
        if box_id=='1':
            x_cor,y_cor=red_truck_pos.pop(0)
            truck_pos.pose.position.x = 60.05-x_cor
            truck_pos.pose.position.y = 63.75+y_cor
            truck_pos.pose.position.z = 6
        if box_id=='2':
            x_cor,y_cor=blue_truck_pos.pop(0)        
            truck_pos.pose.position.x = 17.4-x_cor
            truck_pos.pose.position.y = -8.4+y_cor
            truck_pos.pose.position.z = 6
    if drone_num=='/edrone1':
        if box_id=='1':
            x_cor,y_cor=red_truck_pos.pop(0)
            truck_pos.pose.position.x = 60.05-x_cor
            truck_pos.pose.position.y = 3.75+y_cor
            truck_pos.pose.position.z = 6
        if box_id=='2':
            x_cor,y_cor=blue_truck_pos.pop(0)        
            truck_pos.pose.position.x = 17.4-x_cor
            truck_pos.pose.position.y = -68.4+y_cor
            truck_pos.pose.position.z = 6

    print(truck_pos)
    return truck_pos


    



print("new file")
row_list_num=[]
rospy.init_node('multi_box',anonymous=True)
drones_obj=drones()

truck_pos=[]
for x in range(4):
    for y in range(3):
        for k in range(2):
            truck_pos.append((x*0.85,y*1.23))
blue_truck_pos=truck_pos.copy()
red_truck_pos=truck_pos.copy()



# initializing the thread
# while True:
#     if len(row_list_num):  
t1=Thread(target=drones_obj.drone_1)
t2=Thread(target=drones_obj.drone_2)
t3=Thread(target=drones_obj.camera_feed)

# starting the threads
t1.start()
time.sleep(1)
t2.start()
rospy.Subscriber("/spawn_info",UInt8, row_list)
time.sleep(10)
t3.start()

# waiting for the threads to complete
t1.join()
t2.join()
t3.join()


print("completed....")