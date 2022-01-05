#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Lane Follower V4
Author: Ahmet Fatih Akcan
Date: 20.08.2021

Update Notes:
    - PID tunings has been updated
    - Kinematics has been updated
    - Perspective cam angles has been updated
    - Steady speed and Maximum speed has been updated
    - After the updates, the long time running test is succeeded. (+12 hours)
    
To do:
    - New detailed kinematics
    - New detailed algorithm for curve detection and lane following
    
"""

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from simple_pid import PID
import time
import math

class LaneFollower():
    def __init__(self):
        rospy.init_node("lane_test")
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.camCallback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
#        self.pid = PID(0.012, 0.008, 1.3, output_limits=(-1.5, 1.5))
        self.steady_speed = 1.2
        self.max_speed = 1.6
        
#        self.pid = PID(0.016, 0.0015, 0.00005, output_limits=(-self.max_speed, self.max_speed))
        self.pid = PID(0.018, 0.0005, 0.000007, output_limits=(-self.max_speed, self.max_speed)) #-self.max_speed, self.max_speed
#        self.pid.output_limits((-300, 300))
        self.start_time = 0
        self.target_acc_first = True
        self.target_dec_first = True
        self.first_vel_dec = True
        print("inits ok")

        rospy.spin()
        
    def start(self, time):
        return -(math.cos(math.pi*time) - 1) / 2
    
    def accelerate(self, time):
        if time < 0.5 : return 2*time*time
        else : return 1 - math.pow(-2*time + 2, 2) / 2
        
    def target_acc(self, target_vel):
        if self.target_acc_first == True:
            self.target_acc_first = False
            self.target_acc_startTime = time.time()
            
        current_time = time.time()
        diff_time = current_time - self.target_acc_startTime
        if diff_time > 1.5 : diff_time = 1.5
        return self.accelerate(diff_time*2/3) * (target_vel-self.steady_speed)

    def decelerate(self, time):
          return math.sin((time * math.pi) / 2);
        
    def target_dec(self, target_vel):
        if self.target_dec_first == True:
            self.target_dec_first = False
            self.target_dec_startTime = time.time()
            
        current_time = time.time()
        diff_time = current_time - self.target_dec_startTime
        if diff_time > 0.3 : diff_time = 0.3
        
        if self.first_vel_dec == True:
            self.first_vel_dec = False
            first_vel = self.first_vel_dec        
            diff_vel = first_vel - target_vel
        
        ret = first_vel - self.decelerate(diff_time*3.33333) * diff_vel
        if ret<self.steady_speed : return self.steady_speed
        else : return ret
    
    def camCallback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
#        k1 = np.float32([[215, 270], [425, 270], [0, 375], [640, 375]])
#        k1 = np.float32([[215, 260], [425, 260], [0, 375], [640, 375]])
#        k2 = np.float32([[30,0], [610,0], [30,480], [610,480]])
        k1 = np.float32([[175, 255], [465, 265], [0, 375], [640, 375]])
        k2 = np.float32([[60,0], [580,0], [40,480], [600,480]])

        M = cv.getPerspectiveTransform(k1, k2)
        perspective = cv.warpPerspective(img, M, (640,480))

        hsv = cv.cvtColor(perspective, cv.COLOR_BGR2HSV)
        
        h,w,d = perspective.shape
        cv.circle(perspective, (int(w/2), int(h/2)), 5, (0,0,255), -1)
        
        min_yellow = np.array([20,75,75])
        max_yellow = np.array([40,255,255])        
        
        mask = cv.inRange(hsv, min_yellow, max_yellow)
        result = cv.bitwise_and(img, img, mask=mask)
        #        print("masking ok")
        
        M = cv.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv.circle(perspective, (cx,cy), 5, (255,0,0), -1)
           
            if self.start_time == 0 : self.start_time = time.time()

            self.pid.setpoint = cx
            output = self.pid(w/2)
            print(output)
            
            self.current_time = time.time()
            self.diff_time = self.current_time - self.start_time
            if self.diff_time > 5 :
#                if abs(cx-w/2) < 20 :
                if abs(output) < 0.20 :
#                    self.vel.linear.x = 2
                    self.vel.linear.x = self.steady_speed + self.target_acc(self.max_speed)
                else:
#                    self.vel.linear.x = 1.2
                    lin_x = self.target_dec(self.steady_speed)
                    if lin_x < self.steady_speed : lin_x = self.steady_speed
                    self.vel.linear.x = lin_x
                    self.target_acc_first = True
                    self.target_dec_first = True
                    self.first_vel_dec = True
            else:
                self.vel.linear.x = 0 + self.start(self.diff_time / 5)
                
#            if self.vel.linear.x > 1.2 : self.angle_coef = self.vel.linear.x * 5/6
#            elif self.vel.linear.x > 1.5 : self.angle_coef = self.vel.linear.x * 4/5
#            elif self.vel.linear.x > 1.7 : self.angle_coef = self.vel.linear.x * 3/4
#            else : self.angle_coef = 1
            if self.vel.linear.x > self.steady_speed:
                self.vel.angular.z = -output * (1+(abs(self.vel.linear.x-self.steady_speed)/self.steady_speed) )
            else:
                self.vel.angular.z = -output
#            self.vel.linear.x = 0 #for test
#            self.vel.angular.z = 0 #for test
            self.pub.publish(self.vel)

        else:
            print("no action taken")
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.start_time = 0
            self.pub.publish(self.vel)
        
#        cv.imshow("Original", img)
        cv.imshow("Perspective", perspective)
        cv.waitKey(1)

                
LaneFollower()  		
