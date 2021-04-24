#!/usr/bin/env python3
import rospy
import time
import roslaunch
import numpy as np
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

class stopsign:
    def __init__(self):
        rospy.init_node('stopsignprobability', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.stop_sign_detect_pub = rospy.Publisher('/detect_stop', Int16, queue_size=10)
        self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.newprediction)
        self.detect_line_sub = rospy.Subscriber("/detect_line",Int16,self.line_detection)
        self.rate = rospy.Rate(10)

    def line_detection(self,msg):
        global line_detection
        line_detection = msg.data

    def newprediction(self,bounding_box):
        self.rate.sleep()
        global stop_sign_detect
        prediction = bounding_box.bounding_boxes
        for box in prediction:
            identified_class=box.Class
            probability = float(box.probability)
            area = abs(box.xmax-box.xmin)*abs(box.ymax-box.ymin)
            if ((identified_class == 'stop sign') and (probability >= 0.5) and (area >=5000)):        #change based on the calibration
                now = time.time()
                diff=0
                while diff<4:
                    current = time.time()
                    diff = current - now
                stop_sign_detect = 1
                self.stop_sign_detect_pub.publish(stop_sign_detect)
            break
            rospy.sleep(0.1)

stopsign()
vel_msg = Twist()
line_detection = 0
stop_sign_detect = 0
while True:
    stopsign()
    if line_detection ==1:
        stopsign()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/atefemran/AuE8935_Course/ros_local_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"])
        launch.start()
        while stop_sign_detect==0:
            rospy.sleep(0.1)
            stopsign()
    if stop_sign_detect==1:
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(3)
