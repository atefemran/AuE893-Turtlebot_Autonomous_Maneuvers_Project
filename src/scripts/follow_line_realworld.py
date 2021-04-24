#!/usr/bin/env python3
import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int16


class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detect_line_publisher = rospy.Publisher('/detect_line', Int16, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        #self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",Image,self.camera_callback)
        self.stop_sign_detect = rospy.Subscriber("/detect_stop",Int16,self.stop_detection)

    def stop_detection(self,msg):
        global stop_detected
        stop_detected = msg.data

    def camera_callback(self, data):
        global stop_detected
        global stopped_before
        global counter_line_detect
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #np_arr = np.fromstring(data.data, np.uint8)
        #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        # crop_start=(height/2)+200
        # crop_end=(height/2)+400
        crop_start=400
        crop_end=500
        # crop_start=(height/2)+200
        # crop_end=(height/2)+400
        crop_img = cv_image[int(crop_start):int(crop_end)][1:int(width)]
        #crop_img = cv_image[340:360][1:640]

        # photo center
        centerx, centery = width/2, ((crop_start+crop_end)/2-crop_start)

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV.
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        global line_detection
        line_detection=0
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            counter_line_detect = counter_line_detect +1
            if counter_line_detect >= 75:
                line_detection=1
            self.detect_line_publisher.publish(line_detection)             # to let other nodes know that line is detected
        except ZeroDivisionError:
            cx, cy = 10000, 10000
            self.detect_line_publisher.publish(line_detection)             # to let other nodes know that line is not detected


        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        vel_msg = Twist()

        #calculating the difference between the center of the image and the centroid
        diff_y= centery-cy
        diff_x= centerx-cx

        # Proportional controller value calculation for linear speed
        if diff_y < -8000 :
            vel_msg.linear.x = 0
        else:
            #vel_msg.linear.x = 0
            vel_msg.linear.x = 0.15


        # Proportional controller value calculation for angulat speed
        if diff_x < -8000 :
            vel_msg.angular.z = 0
        else:
            #vel_msg.angular.z = 0
            vel_msg.angular.z = 0.32*diff_x/100

        # print("centerioid",cx,cy)
        # print("center",centerx,centery)
        # print(vel_msg.linear.x)

        if ((line_detection==1) and (stop_detected==0)):
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            self.velocity_publisher.publish(vel_msg)
        elif (stop_detected == 1) and (stopped_before == 1) and (line_detection==1):
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            self.velocity_publisher.publish(vel_msg)
        elif stop_detected==1 and stopped_before==0:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

            #stop for 3 seconds at stop sign
            now = time.time()
            diff=0
            while diff<3:
                current = time.time()
                diff = current - now
            stop_detected=0
            stopped_before=1

    def clean_up(self):
        cv2.destroyAllWindows()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

def main():
    rospy.init_node('line_following_node', anonymous=True)
    stop_detection =0
    line_follower_object = LineFollower()
    rate = rospy.Rate(10)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

counter_line_detect = 0
stop_detected =0
stopped_before =0
if __name__ == '__main__':
        main()

#Atef Emran @Apr,2021
