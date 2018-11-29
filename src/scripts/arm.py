#! /usr/bin/env python
import rospy
import subprocess
import os
import sys
from std_msgs.msg import Float64

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty
import time

# A Pose with reference coordinate frame and timestamp
# Method comes from: geometry_msgs
cur_pose = PoseStamped()


def pos_cb(msg):
    global cur_pose
    cur_pose = msg


# These can be deleted
mode_proxy = None
arm_proxy = None

# One of the first calls you will likely execute in a rospy program.
# Anonymous will add numbers after multi.
rospy.init_node('multi', anonymous=True)

# Comm for drones:
# Set FCU operation mode (FCU = Flight Controll Unit)
mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)
# Change Arming status.
arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

print('communication initialization complete')

try:
    print('Waiting for message...')
    # Relative Altitude (Will give you the altitude of the Drone)
    data = rospy.wait_for_message('mavros/global_position/rel_alt', Float64, timeout=5)
except:
    pass


print("wait for service")
rospy.wait_for_service('mavros/set_mode')
print("got service")

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    success = None
    try:
        success = mode_proxy(1,'OFFBOARD')
    except rospy.ServiceException as e:
        print ("mavros/set_mode service call failed: %s" %e)

    success = None
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        success = arm_proxy(True)
    except rospy.ServiceException as e:
        print ("mavros1/set_mode service call failed: %s"%e)
