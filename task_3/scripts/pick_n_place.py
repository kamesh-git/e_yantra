import numpy as np
import time
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class offboard_control:


    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)


    
    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies 
    def disArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)
    def offboard_set_mode(self):

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        try:
            set_mode=rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            set_mode(custom_mode='OFFBOARD')
        # and print fail message on failure
        except rospy.ServiceException as e:
            print("Failedto set offboard:%s"%e)


    def auto_land_mode(self):
        try:
            land_set_mode=rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            land_set_mode(custom_mode='AUTO.LAND')
        # and print fail message on failure
        except rospy.ServiceException as e:
            print("Failedto set AutoLand:%s"%e)



class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def position(self,msg):
        global setpoints
        global check
        global t=0
        global ac=0
        try:

            if int(msg.pose.position.x) == setpoints[3].pose.position.x and  int(msg.pose.position.y)==setpoints[3].pose.position.y and int(msg.pose.position.z)==setpoints[3].pose.position.z:
               t=1
                
            if int(msg.pose.position.x) == setpoints[check].pose.position.x and  int(msg.pose.position.y)==setpoints[check].pose.position.y and int(msg.pose.position.z)==setpoints[check].pose.position.z:
                check+=1          
        except:
            pass
    def pick(self,msg):
        try:
            if(t==1):
                if(msg.gripstate==True):
                    ac=1

    
                





def main():
    global check
    check =0
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    act=rospy.Publisher('/activate_gripper',activate,queue_size=1)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    global setpoints
    setpoints=[]  #List to setpoints

    # Similarly initialize other publishers 

    # Create empty message containers 
    pos =PoseStamped()
    gripstate=False
    activate=False

    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0
    setpoints.append(pos)

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 5.0
    vel.linear.y = 5.0
    vel.linear.z = 5.0
    
    # Similarly add other containers 
    pos1 =PoseStamped()
    pos2 =PoseStamped()
    pos3 =PoseStamped()
    pos4 =PoseStamped()
    pos5 =PoseStamped()
    pos6 =PoseStamped()


    pos1.pose.position.x = 0
    pos1.pose.position.y = 0
    pos1.pose.position.z = 3
    setpoints.append(pos1)

    pos2.pose.position.x = 3
    pos2.pose.position.y = 0
    pos2.pose.position.z = 3
    setpoints.append(pos2)


    posl.pose.position.x = 3
    posl.pose.position.y = 0
    posl.pose.position.z = 0
    setpoints.append(posl)

    


    pos3.pose.position.x = 3
    pos3.pose.position.y = 3
    pos3.pose.position.z = 3
    setpoints.append(pos3)

    pos4.pose.position.x = 3
    pos4.pose.position.y =3
    pos4.pose.position.z = 0
    setpoints.append(pos4)

    pos5.pose.position.x = 0
    pos5.pose.position.y = 0
    pos5.pose.position.z = 3
    setpoints.append(pos5)
    
    pos6.pose.position.x = 0
    pos6.pose.position.y = 0
    pos6.pose.position.z = 0
    setpoints.append(pos6)
    





    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    local_pose=rospy.Subscriber('/mavros/local_position/pose',PoseStamped,stateMt.position)
    local_gripper=rospy.Subscriber('/gripper_check',gripstate,stateMt.pick)
    


    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    print("arm")
    for i in range(200):
        local_pos_pub.publish(pos)
        rate.sleep()

    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

        # Switching the state to  mode
    while not stateMt.state.mode=="OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
        print ("OFFBOARD mode not yet activated, current mode:",stateMt.state.mode)

    print ("OFFBOARD mode activated")
    
    # Publish the setpoints 
    
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here 
        '''
        print("moving towards setpoint:",check)
        if check == 7:
            local_pose.unregister()
            break
        if ac==1:
            activate=True

        pos=setpoints[check]
        act.publish(activate)
        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        rate.sleep()

    while not stateMt.state.mode=="AUTO.LAND":
        print ("AutoLand mode not yet activated, current mode:",stateMt.state.mode)
        ofb_ctl.auto_land_mode()
        rate.sleep()
    time.sleep(5)
    while stateMt.state.armed:
        ofb_ctl.disArm()
        rate.sleep()
    print("DisArmed!!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    


