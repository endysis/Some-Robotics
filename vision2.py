#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class image_converter:

    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Image window2", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
     
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image, self.callback)
        
        # Publish to the cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist()
        
        #self.pub = rospy.Publisher('/result_topic',String)

        
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
                
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Set image threshold
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((30, 100, 100)),
                                 numpy.array((60, 255, 255)))                                 
       
       # 40 - 90 Green
       # 0 - 5 Red
       # 30 - 60 Yellow
      # 110 - 130 Blue
                              
                              
        ##SAMPLE RANGES##
       ## print numpy.mean(hsv_img[:, :, 0])
        ##self.pub.publish(str(numpy.mean(hsv_img[:, :, 0])))

       ## print numpy.mean(hsv_img[:, :, 1])
        ##self.pub.publish(str(numpy.mean(hsv_img[:, :, 1])))
        
        ##print numpy.mean(hsv_img[:, :, 2])
        ##self.pub.publish(str(numpy.mean(hsv_img[:, :, 2])))


        
        
                                                  
        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
                                             cv2.RETR_TREE,
                                             cv2.CHAIN_APPROX_SIMPLE)
        
        # Gets da moments                                          
        mom = cv2.moments(hsv_thresh)
        print mom                              
                                     
        # width height of image                             
        h, w, d = hsv_img.shape

        for c in hsv_contours:
            a = cv2.contourArea(c)
            print 'A: ' + str(a)
            if a > 0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
                
                # Finds the center of the contour area
                centerConX = int(mom['m10']/mom['m00'])
                
                
                # No idea what this means got from the workshop code                
                angVel = centerConX - w/2
                
                # Start control
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(angVel)/100
                self.cmd_vel.publish(self.twist)
                                
                
        print '===='
        cv2.imshow("Image window", cv_image)
        cv2.imshow("Image window2",hsv_thresh)
        

        
        def forward_kinematics(w_l, w_r):
            wheel_radius = 1
            robot_radius = 1
            c_l = wheel_radius * w_l
            c_r = wheel_radius * w_r
            v = (c_l + c_r) / 2
            a = (c_l - c_r) / robot_radius
            return (v, a)
        



image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()





























