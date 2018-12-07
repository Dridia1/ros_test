#! /usr/bin/env python

import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy
import sys
import tf


class TestLoop:
    curr_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 0.5
    sim_ctr = 1
    des_pose = PoseStamped()
    isReadyToFly = False

    def __init__(self, H, V, yaw):
        # 0: print('yaw:' + str(yaw))
        # 1: print('h:' + str(H))
        # 1: print('v: ' + str(V))
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.locations = numpy.matrix([[H, H, V, q[0], q[1], q[2], q[3]],
                                       [-H, H, V, q[0], q[1], q[2], q[3]],
                                       [-H, -H, V, q[0], q[1], q[2], q[3]],
                                       [H, -H, V, q[0], q[1], q[2], q[3]], ])
        print(self.locations)
        print('/mavros/setpoint_position/local')
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.mocap_cb)
        rospy.Subscriber('/mavros/state', State, callback=self.state_cb)

        rospy.Subscriber('mavros/global_position/rel_alt', Float64, callback=self.report_alt)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_pose)
        shape = self.locations.shape

        while not rospy.is_shutdown():
            print(self.sim_ctr, shape[0], self.waypointIndex)
            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1

            if self.isReadyToFly:
                des_x = self.locations[self.waypointIndex, 0]
                des_y = self.locations[self.waypointIndex, 1]
                des_z = self.locations[self.waypointIndex, 2]

                self.des_pose.pose.position.x = des_x
                self.des_pose.pose.position.y = des_y
                self.des_pose.pose.position.z = des_z

                self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
                self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
                self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
                self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]

                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z

                azimuth = math.atan2(0 - curr_y, 20 - curr_x)
                quaternion = tf.transformations.quaternion_from_euler(0, 0, azimuth)
                print(quaternion)
                self.des_pose.pose.orientation.x = quaternion[0]
                self.des_pose.pose.orientation.y = quaternion[1]
                self.des_pose.pose.orientation.z = quaternion[2]
                self.des_pose.pose.orientation.w = quaternion[3]

                dist = math.sqrt(
                    (curr_x - des_x) * (curr_x - des_x) + (curr_y - des_y) * (curr_y - des_y) + (curr_z - des_z) * (
                                curr_z - des_z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            print('Publish this: ' + str(self.des_pose))
            pose_pub.publish(self.des_pose)
            rate.sleep()

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def mocap_cb(self, msg):
        # print msg
        self.curr_pose = msg

    def report_alt(self, msg):
        # print current alt
        print('Altitude: ' + str(msg))

    def state_cb(self, msg):
        print(msg.mode)
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            print("readyToFly")


if __name__ == "__main__":
    TestLoop(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
