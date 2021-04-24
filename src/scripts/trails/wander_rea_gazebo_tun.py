#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import struct
import numpy as np
from std_msgs.msg import Int16


# the logic is dviding the wander condition to three modes based ont he surroundings
# this is being established by dividing the lidar into 3 regions and selecting the
# mode based on the front readings:
# 1) The wander mode - the min lidar reading from the fron region is above the threshold, the TB will try to center its position
# 2) Slow Down - When the min front measurment is less than the threshold, the linear speed is decreased
# 3) Critical - when its very close to obstcal, stops and rotates away from obstcale

class avoidance:

    def __init__(self):
        # unique node (using anonymous=True).
        rospy.init_node('avoidance', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.new_measurment)
        self.rate = rospy.Rate(10)
        self.max= 2.5

        # self.detect_line_sub = rospy.Subscriber("/detect_line",Int16,self.line_detection)

    def new_measurment(self,lidar_readings):
        self.rate.sleep()

        #processing the lidar data
        lidar_processed_data = []
        for i in lidar_readings.ranges:
            if i==0 or i>3.5:
                lidar_processed_data.append(3.5)
            else:
                lidar_processed_data.append(i)
        self.left_mean = np.minimum(np.mean(lidar_processed_data[20:60]),self.max)
        self.right_mean = np.minimum(np.mean(lidar_processed_data[300:340]), self.max)

        self.front = np.minimum((np.mean(lidar_processed_data[0:25])+np.mean(lidar_processed_data[335:360]))/2,self.max)

        self.front_l_cr = np.minimum((np.mean(lidar_processed_data[0:35])),self.max)
        self.front_r_cr = np.minimum((np.mean(lidar_processed_data[325:360])),self.max)

        # self.front_l_cr = np.minimum((np.mean(lidar_processed_data[0:30])),self.max)
        # self.front_r_cr = np.minimum((np.mean(lidar_processed_data[330:360])),self.max)

    #
    # def line_detection(self,msg):
    #         global line_detection
    #         line_detection = msg.data

    def move(self):
        rospy.sleep(3)              #Needed while running from launch file
        vel_msg = Twist()

        #Tuning Parameters
        # (1) threshhold, (2) critical, (3) speeds (linear and angular), (4) difference min and mean
        speed_lin = 0.2
        speed_ang = 0.4

        threshold = 1            #in real world make it 0.5 | Gazebo 0.5
        critical = 0.3              #in real world make it 0.3 | Gazebo 0.2

        #let's move it
        vel_msg.linear.x = speed_lin
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

        # The indefinite loop
        while True:
            # Let's descover what is surrounding us! (kp value)
            # diff = self.left-self.right                 #used when critical
            diff_mean_raw = (self.left_mean-self.right_mean)   #used in regular conditions

            if (diff_mean_raw<speed_ang) and (diff_mean_raw>(-1*speed_ang)):
                diff_mean = diff_mean_raw
            elif diff_mean_raw>speed_ang:
                diff_mean = speed_ang
            elif diff_mean_raw<(-1*speed_ang):
                diff_mean = -1*speed_ang

            print("--------------------------------------------------")
            print("diff_mean", diff_mean)
            print("self.front_l_cr", self.front_l_cr)
            print("self.front_r_cr", self.front_r_cr)

            #what if we will hit something ahead --> reduce the linear speed and rotate .. take it easy :)
            if (self.front_l_cr<critical) or (self.front_r_cr<critical):
                vel_msg.linear.x = 0
                vel_msg.angular.z = diff_mean
                print('critical')
            elif (self.front_l_cr<threshold and self.front_l_cr>critical) or (self.front_r_cr<threshold and self.front_r_cr>critical):
                # Reduce linear speed and rotate
                vel_msg.linear.x = np.minimum((self.front*speed_lin/threshold), speed_lin)
                vel_msg.angular.z = 0.7*diff_mean
                print('threshhold')
            else:
                vel_msg.linear.x = speed_lin
                vel_msg.angular.z = 0.2*diff_mean
                print('else')

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        # ctrl + C, the node will stop.
        rospy.spin()

try:
    x = avoidance()
    x.move()

except rospy.ROSInterruptException:
    pass

#Atef Emran @Apr,2021
