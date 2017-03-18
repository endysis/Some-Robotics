#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32
from move_base_msgs.msg import MoveBaseActionFeedback
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist



#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#import actionlib
#from actionlib_msgs.msg import *
#from geometry_msgs.msg import Pose, Point, Quaternion


from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped


       # 40 - 90 Green
       # 0 - 5 Red
       # 30 - 60 Yello
       # 110 - 130 Blue


class image_converter():
    
    isMoving = True
    isTurning = False
    
    redGo = False
    greenGo = False
    blueGo = False
    yellowGo = False    
    
    redFound = False
    greenFound = False
    blueFound = False
    yellowFound = False
    
    
    
    redDistance = 0
    greenDistance = 0
    blueDistance = 0
    yellowDistance = 0
    
    
    
    
    def __init__(self):
        
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Image window2", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image, self.callback)
        self.scan_sub = rospy.Subscriber('/turtlebot/scan',LaserScan,self.laserCallBack)
        self.rescallback = rospy.Subscriber("/turtlebot/move_base/feedback", MoveBaseActionFeedback, self.resrobot)
        
        #Publish to the cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist()       
        
        self.goal_sent = False
        #self.move_base = actionlib.SimpleActionClient("/turtlebot/move_base",MoveBaseAction)
        #self.move_base.wait_for_server()
        
        
        self.basePub = rospy.Publisher("/turtlebot/move_base_simple/goal", PoseStamped, queue_size = 0)
        
        
        
    def resrobot(self, value):
            if value.status.status == 1:
                self.isMoving = True
                
                
 
    def moveToPoint(self,posX,posY,posZ):
       global isMoving        
       goal = PoseStamped()
       goal.header.frame_id = "/map"
       goal.header.stamp = rospy.Time.now()
       goal.pose.position.x = posX
       goal.pose.position.y = posY
       goal.pose.orientation.w = 1
       self.basePub.publish(goal)
        
        
        
    def goToColour(self, lin, ang, home):
            
        self.twist.linear.x = lin
        self.twist.angular.z = -float(ang)/100
        self.cmd_vel.publish(self.twist)        
        
        
        
    def goto(self,pos,quat):
        #Sending a goal
        self.goal_sent = True
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        
        print goal
        
        # The moving comences
        self.move_base.send_goal(goal)
                        
        # Allow Turtlebot 60s to complete the task
        success = self.move_base.wait_for_result(rospy.Duration(60))
                        
        state = self.move_base.get_state()
        print state
        result = False
        
        if success and state == GoalStatus.SUCCEEDED:
            #The robot has made to goal
            result = True
            
        else:
            self.move_base.cancel_goal()
            
            
        self.goal_sent = False
        return result
        
        
                
    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
        
        
    
    def laserCallBack(self,data):        
        range_ahead = data.ranges[len(data.ranges)/2]
        #print "range ahead:  %0.1f" % range_ahead
        
        
        
    def callback(self, data):
        
        global isMoving
        
        global redFound
        global greenFound
        global blueFound
        global yellowFound
        
        global redGo
        global greenGo
        global blueGo
        global yellowGo
    
    
    
        global redDistance
        global greenDistance
        global blueDistance
        global yellowDistance
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
            
        
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        
        # Set image threshold
        hsv_threshRED = cv2.inRange(hsv_img,
                                 numpy.array((0, 100, 100)),
                                 numpy.array((4, 255, 255)))                         
                            
        hsv_threshGREEN = cv2.inRange(hsv_img,
                                 numpy.array((40, 100, 100)),
                                 numpy.array((90, 255, 255)))                          
                        
        hsv_threshYELLOW = cv2.inRange(hsv_img,
                                 numpy.array((30, 100, 100)),
                                 numpy.array((40, 255, 255)))                           
                                 
        hsv_threshBLUE = cv2.inRange(hsv_img,
                                 numpy.array((110, 100, 100)),
                                 numpy.array((130, 255, 255)))
                                 
                                
        mask = hsv_threshRED + hsv_threshGREEN + hsv_threshYELLOW + hsv_threshBLUE
                        
                        
                               
        hsv_contoursR, hierachy = cv2.findContours(hsv_threshRED.copy(),
                                             cv2.RETR_TREE,
                                             cv2.CHAIN_APPROX_SIMPLE)
                                            
                                            
        hsv_contoursG, hierachy = cv2.findContours(hsv_threshGREEN.copy(),
                                             cv2.RETR_TREE,
                                             cv2.CHAIN_APPROX_SIMPLE)
                                    

        hsv_contoursY, hierachy = cv2.findContours(hsv_threshYELLOW.copy(),
                                             cv2.RETR_TREE,
                                             cv2.CHAIN_APPROX_SIMPLE)
                                
                                
        hsv_contoursB, hierachy = cv2.findContours(hsv_threshBLUE.copy(),
                                             cv2.RETR_TREE,
                                             cv2.CHAIN_APPROX_SIMPLE)



        # Gets da moments                                          
        momR = cv2.moments(hsv_threshRED)
        momG = cv2.moments(hsv_threshGREEN)
        momY = cv2.moments(hsv_threshYELLOW)
        momB = cv2.moments(hsv_threshBLUE)
        #print mom
    
    
        # width height of image                             
        h, w, d = hsv_img.shape
        
        
        if self.isMoving == False and self.isTurning == False:
            print("Is not moving or turning")
            self.moveToPoint(0,0,0)
 
#        print("Callback running")
 
        #if redDistance > 
        
        
        # So we need a way to pick the max distance and go to it, if there are two or more there
        
        self.redGo = True
        self.redFound = True
        
            
        for c in hsv_contoursR:
            a = cv2.contourArea(c)
            self.redDistance = a
            cv2.drawContours(cv_image, c, -1, (255, 0, 0))
            # Finds the center of the contour area
            centerConX = int(momR['m10']/momR['m00'])
            # No idea what this means got from the workshop code                
            angVel = centerConX - w/2
            #print("Red")
            print " Red Distance %d" % self.redDistance
            if(self.redFound == True and self.redGo == True):
                self.goToColour(0.2,angVel,1)
                
                    
         
                        
                        
        for c in hsv_contoursG:
            a = cv2.contourArea(c)
            self.greenDistance = a
            #print 'A: ' + str(a)
            cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
            centerConX = int(momG['m10']/momG['m00'])
            print "Green Distance %d" % self.greenDistance
            
            
        
        for c in hsv_contoursY:
            a = cv2.contourArea(c)
            #print 'A: ' + str(a)
            if a > 0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
                centerConX = int(momY['m10']/momY['m00'])
                print("Yellow")

                
                
        for c in hsv_contoursB:
            a = cv2.contourArea(c)
            #print 'A: ' + str(a)
            if a > 0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
                centerConX = int(momB['m10']/momB['m00'])
                print("Blue")
                
          
           
        #print '===='
        cv2.imshow("Image window", cv_image)
        cv2.imshow("Image window2",mask)
        



        
        def turnAround(self):
            self.isTurning = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = 11
            self.cmd_vel.publish(self.twist)
            self.isTurning = False
        

            
        
                
            
            




if __name__ == '__main__':
    try:
        rospy.init_node('image_converter',anonymous=False)
      #  navigator = image_converter()
        
      # position = {'x':1, 'y' : -5, 'z':0.0}
     #   quartonion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        
        
    #    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    #    success = navigator.goto(position,quartonion)
       
     #   if success:
      #      rospy.loginfo("The robot has reached its position")
        
     #   else:
      #      rospy.loginfo("The robot has not reached its position")
            
            
       # rospy.sleep(1)
        

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")





image_converter()
#rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
















































