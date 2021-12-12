#!/usr/bin/env python3

import rospy
import mavros
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode) 
local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)


pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 10

vel = Twist()
vel.linear.x = 0
vel.linear.y = 0
vel.linear.z = 10


def position_control():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        while not current_state.armed:
            arming_client(True)
            rate.sleep()
            print("armed status",current_state.armed)
        print("armed",current_state.armed)
        while current_state.mode != "OFFBOARD":
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            rate.sleep()
            print("mode status",current_state.mode)

        print("mode",current_state.mode)
        # if current_state.mode != "OFFBOARD":
        #     set_mode_client(base_mode=0, custom_mode="OFFBOARD")
        #     print("mode",current_state.mode)
        # else:
        #     if not current_state.armed:
        #         arming_client(True)

        # older versions of PX4 always return success==True, so better to check Status instead
        print("1")
        print(prev_state.armed)
        print(current_state.armed)
        if prev_state.armed != current_state.armed:
            print("2")
            print(prev_state.armed)
            print(current_state.armed)
        # else:
        #     arming_client(True)
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            print("3")
            print(prev_state.armed)
            print(current_state.armed)
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass