#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from sensor_msgs.msg import LaserScan
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String
import math
import time


subscribe = rospy.Subscriber('scan', LaserScan, callback)
kps = 1.1
kds = 0.00065
kis = 0.05
uk1s = 0.0
er_1s = 0.0
er_2s = 0.0



def callback(points):
    frontleft = points.ranges[0:79]
    frontright = points.ranges[280:359]
    left = []
    right = []
    global kps, kds, kis, uk1s, er_1s, er_2s
    for i in frontleft:
        if 0 < i < 10:
            left.append(i)
    for j in frontright:
        if 0 < j < 10:
            right.append(j)
    errors = numpy.mean(right) - numpy.mean(left)

    k_1s = kps + kis + kds
    k_2s = -kps - (2.0 * kds)
    k_3s = kds

    uks = uk1s + (k_1s * errors) + (k_2s * er_1s) + (k_3s * er_2s)
    uks = uks
    uk1s = uks
    er_2s = er_1s
    er_1s = errors

    velocity.linear.x = 0.1

    if errors == 0:
        velocity.angular.z = 0.0
    else:
        velocity.angular.z = -uks
    return velocity
ht

def main_callback(data):
    global rotate, forward
    rotate = data.detections[0].pose.pose.pose.position.x - 0.089
    forward = data.detections[0].pose.pose.pose.position.z
    twist_object = Twist()
    Kp_forward = 0.2
    Kp_rotate = 2
    count = 0
    if forward < 0.15:
        twist_object.linear.x = 0
    else:
        twist_object.linear.x = forward * Kp_forward
        if rotate < 0:
            count = 1
        if rotate > 0:
            count = -1
        twist_object.angular.z = count * Kp_rotate * abs(rotate) / 1.9

    publish.publish(twist_object)



rospy.init_node('wall_follower', anonymous=True)
vel = Twist()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    publish.publish(vel)
    rate.sleep()
