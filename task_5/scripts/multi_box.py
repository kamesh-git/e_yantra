#!/usr/bin/env python3
'''
* Team Id : 1933
* Author List : Kamesh,Harshavardhan,sarvesh,pranav kumar
* Filename: SS_1933_bonuc_configuration.py
* Theme: Strawberry Stacker -- Specific to eYRC 
* Functions: detect_ArUco,Calculate_orientation_in_degree,setArm,setMode,autoLand,goto,position,box_grip,correction,activate_gripperdrone_1,drone_2,row_list,ret_truck_pos
* Global Variables:previous_truck_pos,prev_box_spawn,edrone0_row_list_num,edrone1_row_list_num,drones_obj,blue_truck_pos,red_truck_pos
'''


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



        

    '''
    * Function Name:detect_ArUco
    * Input:img
    * Output:dictionary-which posses id's of aruco marker as keys and their respective corners as values
    * Logic: this function detects aruco marker with the help of imported aruco libraries and returns the dictionary of aruco details
    * Example Call: self.aruco_obj.detect_ArUco(self.img)

    '''

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
            self.dist_aruco=((corner[0][0][1][0]-corner[0][0][3][0])**2 + (corner[0][0][1][1]-corner[0][0][3][1])**2)**0.5
            self.pix2cm_ratio=0.2856191880599769/self.dist_aruco

            self.Calculate_orientation_in_degree(img,self.Detected_ArUco_markers)
        except:
            self.Detected_ArUco_markers={}


    '''
    * Function Name:Calculate_orientation_in_degree
    * Input: image , dictionary returned from detect_ArUco function
    * Output:dictionary-which posses id's of aruco marker as keys and their respective angles from horizontal axis as values
    * Logic: we used opencv moments function to find centroid of aruco marker then drawn a line
             straight to one of the border of aruco square and found the angle from the line to the horizontal axis
    * Example Call:self.Calculate_orientation_in_degree(img,self.Detected_ArUco_markers)
    '''

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
        self.box_exist=0



    def statecb(self,msg):
        self.state=msg




    # arming or disarming the drone given by eyantra
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





    # setting offboard mode to the drone given by eyantra
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


    # setting autoland mode to the drone given by eyantra

    def autoLand(self,tol=1):
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
    '''
    * Function Name: goto
    * Input:  setpoints where the drone should go next,x coordinate of setpoint,grip_active to say the drone whether to scan the 
              aruco or not,xy_tol for setting the tollerance for xy axis , z_tol for setting the tollerance for z axis
    * Output: returns nothing just to push the drone to given setpoint
    * Logic: it just publish the given setpoint via mavros and check whether the drone reaches it
    * Example Call: self.goto(setpoints,100,True,0.5,0.3) except setpoints others are optional
    '''
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
                if self.box_exist==200:
                    self.box_exist=0
                    continue
                return
        if self.x < setpoints.pose.position.x:
            x+=4
            self.goto(setpoints,vel,x,True,xy_tol=0.4,z_tol=0.4)
        print("reached the setpoint")

    '''
    * Function Name: position
    * Input: msg from the /mavros/local_position/pose subscriber as a call back function
    * Output: returns nothing but set's the x,y,z current coordinates of drone
    * Logic: just an call back function
    * Example :self.position
    '''
    
    def position(self,msg):
        self.xn=msg.pose.position.x
        self.yn=msg.pose.position.y
        self.zn=msg.pose.position.z


    '''
    * Function Name: position
    * Input: nothing
    * Output: returns nothing just makes the drone to grab and deliver the box
    * Logic: after finding the aruco in the camera makes the drone to correct itself to the top of aruco box
             and land the drone to grab it and take it to the delivery truck
    * Example : self.box_grip()
    '''
    def box_grip(self):
       self.correction()
       self.setpoints.pose.position.x,self.setpoints.pose.position.y=(self.xn,self.yn)
       self.activate_gripper(True)
       self.truck_pos=ret_truck_pos(self.box_id,self.drone_num)
       self.setMode('OFFBOARD')
       self.goto(self.setpoints,self.vel,self.setpoints.pose.position.x,z_tol=0.5)
       self.goto(self.truck_pos,self.vel,self.truck_pos.pose.position.x+1,z_tol=0.5)
       self.goto(self.truck_pos,self.vel,self.truck_pos.pose.position.x)
       self.autoLand(2)
       self.activate_gripper(False)
       self.setMode('OFFBOARD')
       self.goto(self.truck_pos,self.vel,self.truck_pos.pose.position.x+4,xy_tol=0.5,z_tol=0.5)
       self.in_range=False

    '''
    * Function Name: correction
    * Input: nothing
    * Output: it will make the drone exactly to top of the box with aruco and check is that remaining in a range for 5 seconds
    * Logic: we do have the exact diagonal length of aruco marker box(from the gazebo)
             and by finding the diagonal length from the camera feed we will find the ratio of pixel to
             cm and with that ratio we will find how much distance (in cm) does the aruco got deviated from the center of the camera
             and make the drone to correct the distance and check the error to be in a range for 5 seconds 
    * Example: self.correction() 
    '''

    def correction(self):
       self.box_pos=PoseStamped()
       self.time_check=time.time()
       self.box_exist=time.time()
       while True:
           self.x_diff=(self.aruco_obj.cX-200)*self.aruco_obj.pix2cm_ratio     #g/p ratio:0.01546153846153846,0.015228426395939089
           self.y_diff=(200-self.aruco_obj.cY)*self.aruco_obj.pix2cm_ratio+0.4
           self.box_pos.pose.position.x=self.xn+self.x_diff
           self.box_pos.pose.position.y=self.yn+self.y_diff
           self.box_pos.pose.position.z=self.z




           if (self.x_diff>=-0.1 and self.x_diff<=0.1) and (self.y_diff>=-0.1 and self.y_diff<=0.1):
                if time.time() > self.time_check + 4:
                    if len(list(self.aruco_obj.Detected_ArUco_markers.keys())):
                       self.box_id=list(self.aruco_obj.Detected_ArUco_markers.keys())[0]
                       break
                else:
                    self.local_vel_pub.publish(self.vel)
                    self.local_pos_pub.publish(self.box_pos)
                    self.rate.sleep()
           elif (self.y_diff>=-1 and self.y_diff<=1):
               self.time_check=time.time()
               self.local_vel_pub.publish(self.vel)
               self.local_pos_pub.publish(self.box_pos)
               self.rate.sleep()

       print(self.xn,self.yn)
       self.autoLand()
       self.gripper_check=rospy.Subscriber(self.drone_num+"/gripper_check",String,self.gripper_check_cb)
       print("landed on box")

    '''
    * Function Name: gripper_check_cb
    * Input: an callback function from the /gripper_check subscriber and takes the published value as input
    * Output: checks the drone whether in the range to grab the box 
    * Logic: an call back functiion
    * Example : self.gripper_check_cb
    '''
    def gripper_check_cb(self,msg_bool):
        print(msg_bool)
        if msg_bool.data == "True":
            self.gripper_check.unregister()
            self.in_range=True
            self.loop_num=0
        if self.loop_num==20:
            self.gripper_check.unregister()
            self.setMode('OFFBOARD')
            self.correction()
            self.loop_num=0
        self.loop_num+=1


    '''
    * Function Name: activate_gripper
    * Input: att_bool to grab the box True-for grabbing, False- releasing the box
    * Output: nothing
    * Logic: just check if the box in range to grab and wait for the drone to grab after this function calls
             when the drone in range this function sends message to drone to grab the box and for releasing 
             it just releases without any checks
    * Example : self.activate_gripper(True)
    '''

    def activate_gripper(self,att_bool):
        while not self.in_range:
            pass
        rospy.wait_for_service(self.drone_num+'/activate_gripper')
        try:
            set_gripper=rospy.ServiceProxy(self.drone_num+'/activate_gripper',Gripper)
            resp=set_gripper(att_bool)
        except rospy.ServiceException as e:
            print("unable to call the service:%s"%e)
        if not resp:
            self.activate_gripper(att_bool)     
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

class drones():
    '''
    * Function Name: drone_1
    * Input: nothing 
    * Output: controls every moment of edrone0
    * Logic: an function thread which creates individual objects for above classes and access it individually for grabbing
             sending setpoints and etc...
    * Example : drone_1()
    '''
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
        for i in range(160):
            print(i)
            self.drone1.local_pos_pub.publish(pos)
            self.drone1.local_vel_pub.publish(vel)
            self.drone1.rate.sleep()
        self.drone1.setArm(True)
        self.drone1.setMode("OFFBOARD")
        self.drone1.goto(pos,vel,z_tol=0.5)
        list_size=0
        while True:
            for i in edrone0_row_list_num:
                if list_size<len(edrone0_row_list_num):
                    y=i*4-4
                    pos1.pose.position.x = 100
                    pos1.pose.position.y = y
                    pos1.pose.position.z = 3
                    self.drone1.goto(pos1,vel,xy_tol=0.1)
                    list_size+=1


        

    '''
    * Function Name: drone_2
    * Input: nothing 
    * Output: controls every moment of edrone1
    * Logic: an function thread which creates individual objects for above classes and access it individually for grabbing
             sending setpoints and etc...
    * Example : drone_2()
    '''
    
    def drone_2(self):
        self.drone2_aruco_obj=aruco_library('/edrone1')
        self.drone2_image_proc_obj=image_proc(self.drone2_aruco_obj,'/edrone1')
        
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

        

        self.drone2=pick_n_place(self.drone2_aruco_obj,self.drone2_image_proc_obj,'/edrone1')
        rospy.Subscriber("/edrone1/mavros/state",State, self.drone2.statecb)
        print("publishing dummy points....")
        for i in range(160):
            self.drone2.local_pos_pub.publish(pos)
            self.drone2.rate.sleep()
        self.drone2.setArm(True)
        self.drone2.setMode("OFFBOARD")
        self.drone2.goto(pos,vel,xy_tol=0.3,z_tol=0.5)
        list_size=0
        while True:
            for i in edrone1_row_list_num:
                if list_size < len(edrone1_row_list_num):
                    y=-((16-i)*4)
                    pos1.pose.position.x = 100
                    pos1.pose.position.y = y
                    pos1.pose.position.z = 3
                    self.drone2.goto(pos1,vel,xy_tol=0.2,z_tol=0.5)
                    list_size+=1



'''
* Function Name: row_list
* Input: an callback function for /spawn_info subscriber which takes the row number of boxes as input
* Output: returns nothing just sets the row numbers individually for each drones to go next as a list
* Logic: as we have individual list for each drones the function checks which list has more length and appends the 
         number opposite accordingly and posses the previous msg value if previous row number and current row number
         are same collision problem may occur to avoid those it appends to the other drones list
* Example : row_list_num
'''
def row_list(msg):
    global prev_box_spawn
    if len(edrone0_row_list_num)>len(edrone1_row_list_num):
        if prev_box_spawn == msg.data:
            edrone0_row_list_num.append(msg.data)
        else:
            edrone1_row_list_num.append(msg.data)
    else:
        if prev_box_spawn==msg.data:
            edrone1_row_list_num.append(msg.data)
        else:
            edrone0_row_list_num.append(msg.data)
    prev_box_spawn=msg.data
    print(edrone0_row_list_num,edrone1_row_list_num)


'''
    * Function Name: ret_truck_pos
    * Input: takes box_id of which the drone scanned before , drone_num to identify which drone called this function
    * Output: returns the truck cell coordinate to place the box 
    * Logic: as each drone posses different coordinate system we should send the coordinate accordingly
             so by taking the box_id and edrone number it sends the box position accordingly
    * Example : ret_truck_pos('1','/edrone0')
    '''
def ret_truck_pos(box_id,drone_num):
    global previous_truck_pos
    truck_pos=PoseStamped()
    if drone_num=='/edrone0':
        if box_id=='1':
            x_cor,y_cor=red_truck_pos.pop(0)
            truck_pos.pose.position.x = 60.05-x_cor
            truck_pos.pose.position.y = 63.75+y_cor
            truck_pos.pose.position.z = 6.5
        if box_id=='2':
            x_cor,y_cor=blue_truck_pos.pop(0)        
            truck_pos.pose.position.x = 17.4-x_cor
            truck_pos.pose.position.y = -8.4+y_cor
            truck_pos.pose.position.z = 6.5
    if drone_num=='/edrone1':
        if box_id=='1':
            x_cor,y_cor=red_truck_pos.pop(0)
            truck_pos.pose.position.x = 60.05-x_cor
            truck_pos.pose.position.y = 3.75+y_cor
            truck_pos.pose.position.z = 6.5
        if box_id=='2':
            x_cor,y_cor=blue_truck_pos.pop(0)        
            truck_pos.pose.position.x = 17.4-x_cor
            truck_pos.pose.position.y = -68.4+y_cor
            truck_pos.pose.position.z = 6.5
    if previous_truck_pos[1] != drone_num:
        print('entered loop1')
        if (previous_truck_pos[0].pose.position.x==truck_pos.pose.position.x):
            print('entered loop2')
            print(previous_truck_pos[0].pose.position.y,truck_pos.pose.position.y+60)
            if (previous_truck_pos[0].pose.position.y==float("{:.2f}".format(truck_pos.pose.position.y+60))) or previous_truck_pos[0].pose.position.y==float("{:.2f}".format(truck_pos.pose.position.y-60)):
                print('entered loop3')
                red_truck_pos.append((x_cor,y_cor)) if box_id == '1' else blue_truck_pos.append((x_cor,y_cor))
                for i in range(2):
                    print('entered loop4')
                    if box_id == '1':
                        red_truck_pos.append(red_truck_pos.pop(0))
                        print(red_truck_pos)
                    else:
                        blue_truck_pos.append(blue_truck_pos.pop(0))
                        print(blue_truck_pos)
                truck_pos=ret_truck_pos(box_id,drone_num)
                print(truck_pos)

    previous_truck_pos=[truck_pos,drone_num,box_id]
    print(previous_truck_pos)

    return truck_pos




 # main and the place to initialize global variables
previous_truck_pos=[PoseStamped(),'','']
prev_box_spawn=0
edrone0_row_list_num=[]
edrone1_row_list_num=[]
rospy.init_node('multi_box',anonymous=True)
drones_obj=drones()

truck_pos=[]
for x in range(4):
    for y in range(3):
        for k in range(2):
            truck_pos.append((x*0.85,y*1.23))
blue_truck_pos=truck_pos.copy()
red_truck_pos=truck_pos.copy()



# initialinzing the threads 
t1=Thread(target=drones_obj.drone_1)
t2=Thread(target=drones_obj.drone_2)

# starting the threads
t1.start()
time.sleep(1)
t2.start()
rospy.Subscriber("/spawn_info",UInt8, row_list)


# waiting for the threads to complete
t1.join()
t2.join()


print("completed....")