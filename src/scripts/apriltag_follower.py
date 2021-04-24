#!/usr/bin/env python3
import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from move_robot import MoveTurtlebot3
import numpy as np
from std_msgs.msg import Int16

x_diff = 0
depth_diff = 0

class Apriltag_follower(object):

    def __init__(self):
        self.bridge = CvBridge()
        #to use the CvBridge, you will need to transfer the message from compressed to raw (follow the instructions)

        self.publish = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/tag_detections_image', Image, self.camera_callback)
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)
        self.stop_sign_detect = rospy.Subscriber("/detect_stop",Int16,self.stop_detection)

    def stop_detection(self,msg):
        global stop_detected
        stop_detected = msg.data

    def camera_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imshow("Scan", cv_image)
        cv2.waitKey(1)

    def callback(self, data):
        global x_diff, depth_diff
        try:
            global april_tag_pub
            global stop_detected

            if stop_detected ==1 and april_tag_pub ==0:
                now = time.time()
                diff=0
                while diff<5:
                    current = time.time()
                    diff = current - now
                april_tag_pub = 1

            x_diff = data.detections[0].pose.pose.pose.position.x        #compensate the width to determine the right x difference
            depth_diff = data.detections[0].pose.pose.pose.position.z           #the z position represents the depth from the camera cooridnates
            vel_msg = Twist()

            #the proportianal variables values
            Kp_depth_diff = 0.15                                                #the proportional value
            Kp_x_diff = 0.5                                                     #the proportional value
            distance_threshold = 1

            max_lin_speed = 0.2
            max_ang_speed = 0.2

            vel_msg.angular.z = -1*x_diff * Kp_x_diff

            if ((vel_msg.angular.z < max_ang_speed) and (vel_msg.angular.z > -1*max_ang_speed)):
                vel_msg.angular.z = vel_msg.angular.z
            elif vel_msg.angular.z > max_ang_speed:
                vel_msg.angular.z = max_ang_speed
            elif vel_msg.angular.z < -1*max_ang_speed:
                vel_msg.angular.z = -1*max_ang_speed

            #the velocity message
            if (depth_diff < distance_threshold) and (depth_diff > 0):          #the robot will maintain distance of 10 cm
                vel_msg.linear.x = 0
                vel_msg.angular.z = vel_msg.angular.z
            elif depth_diff > distance_threshold:
                vel_msg.linear.x = np.minimum((depth_diff * Kp_depth_diff),max_lin_speed)
                vel_msg.angular.z = vel_msg.angular.z
            else:
                vel_msg.linear.x = 0
                vel_msg.angular.z = vel_msg.angular.z

            if april_tag_pub ==1:
                self.publish.publish(vel_msg)
                print("I am publishing")

        except IndexError:
            rospy.loginfo('Can not detect the tags')


def main():
    global stop_detected
    global april_tag_pub
    april_tag_pub =0
    stop_detected =0
    rospy.init_node('april_tag_node', anonymous=True)
    Apriltag_follower()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    april_tag_pub = 0
    while True:
        main()
