#!/usr/bin/env python3


import numpy as np
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import *


class pick_n_place:
    def __init__(self):
        rospy.init_node('pick_n_place',anonymous=True)
        self.state=State()
        self.reached=False   # used in position function
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)    # used in goto function
        self.local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20.0)
        self.in_range=False



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
    def goto(self,setpoints,vel):
        self.reached=False
        self.setpoints=setpoints
        self.vel=vel
        self.local_pose=rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.position)
        while not self.reached:
            self.local_pos_pub.publish(self.setpoints)
            self.local_vel_pub.publish(self.vel)
            self.rate.sleep()
        print("reached the setpoint....")
        self.local_pose.unregister()


    def position(self,msg):
        x=self.setpoints.pose.position.x
        y=self.setpoints.pose.position.y
        z=self.setpoints.pose.position.z
        xn=msg.pose.position.x
        yn=msg.pose.position.y
        zn=msg.pose.position.z
        tol=0.1
        print(x,y,z,xn,yn,zn)

        if (xn<x+tol and xn>x-tol) and (yn<y+tol and yn>y-tol) and (zn<z+tol and zn>z-tol):
            print(x,y,z,xn,yn,zn)
            self.reached=True        



    def gripper_check_cb(self,msg_bool):
        print(msg_bool)
        if msg_bool.data == "True":
            time.sleep(5)
            self.setMode("OFFBOARD")
            while not msg_bool.data == "True":
                self.local_pos_pub.publish(self.setpoints)
                print("not in range")
            self.setMode("AUTO.LAND")
            self.in_range=True


    def activate_gripper(self,att_bool):
        rospy.wait_for_service('/activate_gripper')
        try:
            set_gripper=rospy.ServiceProxy('/activate_gripper',Gripper)
            resp=set_gripper(att_bool)
        except rospy.ServiceException as e:
            print("unable to call the service:%s"%e)
        if not resp:
            self.activate_gripper()      
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!





# MAIN function!!!!!!
def main():
    drone=pick_n_place()
    rospy.Subscriber("/mavros/state",State, drone.statecb)

    setpoints=[]

    pos =PoseStamped()

    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.y = 0.5
    vel.linear.z = 0.5
    vel.linear.x = 0.5
    
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

    pos2.pose.position.x = 3
    pos2.pose.position.y = 0
    pos2.pose.position.z = 3
    setpoints.append(pos2)


    pos3.pose.position.x = 3
    pos3.pose.position.y = 0
    pos3.pose.position.z = -0.2
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
    drone.goto(setpoints[0],vel)
    drone.goto(setpoints[1],vel)
    drone.goto(setpoints[2],vel)
    drone.setMode("AUTO.LAND")
    print("time to sleep....")
    gripper_check=rospy.Subscriber("/gripper_check",String,drone.gripper_check_cb)
    while not drone.in_range:
        print("drone not in range to pick")
    gripper_check.unregister()
    drone.activate_gripper(True)
    drone.setMode("OFFBOARD")
    drone.goto(setpoints[3],vel)
    drone.goto(setpoints[4],vel)
    drone.goto(setpoints[5],vel)
    drone.setMode("AUTO.LAND")
    time.sleep(10)
    drone.activate_gripper(False)
    drone.setMode("OFFBOARD")
    drone.goto(setpoints[6],vel)
    drone.goto(setpoints[7],vel)
    drone.setMode("AUTO.LAND")
    time.sleep(10)
    drone.setArm(False)


main()