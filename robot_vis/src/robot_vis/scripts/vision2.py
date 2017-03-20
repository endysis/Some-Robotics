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
    
    isMoving = False
    isTurning = False
    
    redComplete = False
    greenComplete = False
    blueComplete = False
    yellowComplete = False    
    
    redFound = False
    greenFound = False
    blueFound = False
    yellowFound = False
    
    stopCheck = False
    isCenter = False
    
    colourArr = []
    
    laserScan = 0
    redDistance = 0
    greenDistance = 0
    blueDistance = 0
    yellowDistance = 0
    
    redArea = 0
    greenArea = 0
    yellowArea = 0
    blueArea = 0
    
    coorValues = [[-3.5,-4.4],[1.47,-4.25],[1.15,-1.25]]  
    
    currTarget = coorValues[0]
    
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
        self.basePub = rospy.Publisher("/turtlebot/move_base_simple/goal", PoseStamped, queue_size = 0)
        
        
        
    def resrobot(self, value):
            
            global coorValues
            
            currX = value.position.x
            currY = value.position.y
        
            if value.status.status == 1:
                self.isMoving = True
                print ("It is moving",value.status.status)
            else:
                print ("It is not moving",value.status.status)
                
                if currX > (self.coorValues[0][0] - 1) and currX < (self.coorValues[0][0] + 1) and currY > (self.coorValues[0][1] - 1) and currY < (self.coorValues[0][1] + 1): 
                    self.coorValues.pop(0)
                    self.isMoving = False
                    
                
                
                
 
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
        global laserScan   
        global stopCheck
        global colourArr
        global isCenter
        
        global redComplete
        global greenComplete
        global blueComplete
        global yellowComplete          
        
        global redArea
        global greenArea
        global yellowArea
        global blueArea          
        
        
        if self.stopCheck == False:
            self.twist.linear.x = lin
            self.twist.angular.z = -float(ang)/100
            self.cmd_vel.publish(self.twist)
            self.isCenter = True
            #print ("Should be moving")
        
        
        if self.laserScan > 0.5 and self.laserScan <= 1:
            self.stopCheck = True
            if self.colourArr[0] == "Red" and self.redArea > 48000:
                self.redComplete = True
                self.colourArr.pop(0)
                
            elif self.colourArr[0] == "Green" and self.greenArea > 48000:
                self.greenComplete = True
                self.colourArr.pop(0)
                
            elif self.colourArr[0] == "Blue" and self.blueArea > 48000:
                self.blueComplete = True
                self.colourArr.pop(0)
                
            elif self.colourArr[0] == "Yellow" and self.yellowArea > 48000:
                self.yellowComplete = True
                self.colourArr.pop(0)
                
            self.stopCheck = False
            #print ("New colour is %s",self.colourArr[0])
            
            # Pops off red, then goes straght to green but because green equals 0 now, straight away
            # it is marked as complete




        
    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
        
    
    
    def laserCallBack(self,data):      
        global laserScan
        self.laserScan = data.ranges[len(data.ranges)/2]
        #print ("range ahead:  %0.1f" % self.laserScan)
        
        
        
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
    
        global colourArr
    
        global redDistance
        global greenDistance
        global blueDistance
        global yellowDistance
        
        
        global redArea
        global greenArea
        global yellowArea
        global blueArea          
        
        global coorValues   
        
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            print ()
            
        
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
        
 
#        print("Callback running")
 
        #if redDistance > 
        
        # So in order for the robot to got to a colour it has to be found, there has to not be completed and it needs to be the first element in the memory array
         
        
        if self.isMoving == False and self.isTurning == False:
            print("Is not moving or turning")
            self.moveToPoint(self.coorValues[0][0],self.coorValues[0][1],0)


        if self.redFound == True:        
            
            for c in hsv_contoursR:
                a = cv2.contourArea(c)
                self.redArea = a
                print ("Area %d",self.redArea)
                if a > 0:
                    if self.redFound == False:
                        self.colourArr.append('Red')
                        print ("Added red")
                        print (self.colourArr[0])
                        self.redFound = True
                    self.redDistance = a
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0))  # Finds the center of the contour area
                    centerConX = int(momR['m10']/momR['m00']) # No idea what this means got from the workshop code 
                    angVel = centerConX - w/2
                    #print("Red")
                    #print (" Red Distance %d" % self.redDistance)
                    if(self.redFound == True and self.redComplete == False):
                        if(self.colourArr[0] == "Red"):
                             self.goToColour(0.5,angVel,1)

                          
            for c in hsv_contoursG:
                a = cv2.contourArea(c)
                self.greenArea = a
                if a > 0:
                    if self.greenFound == False:
                        self.colourArr.append('Green')
                        print ("Added Green")
                        self.greenFound = True
                self.greenDistance = a
                #print 'A: ' + str(a)
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
                centerConX = int(momG['m10']/momG['m00'])
                angVel = centerConX - w/2
                if(self.greenFound == True and self.greenComplete == False):
                        if(self.colourArr[0] == "Green"):
                             self.goToColour(0.5,angVel,1)
                       
            
            for c in hsv_contoursY:
                a = cv2.contourArea(c)
                self.yellowArea = a
                #print 'A: ' + str(a)
                if a > 0:
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
                    centerConX = int(momY['m10']/momY['m00'])
                    print("Yellow")
    
    
                                    
            for c in hsv_contoursB:
                a = cv2.contourArea(c)
                self.blueArea = a
                #print 'A: ' + str(a)
                if a > 0:
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
                    centerConX = int(momB['m10']/momB['m00'])
                    print("Blue")
                
         
        #print '===='
        cv2.imshow("Image window", cv_image)
        cv2.imshow("Image window2",mask)
       
        
        print (self.colourArr)
            
                
            
            
        
        def turnAround(self):
            self.isTurning = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = 11
            self.cmd_vel.publish(self.twist)
            self.isTurning = False
        
        
          
        

if __name__ == '__main__':
    try:
        rospy.init_node('image_converter',anonymous=False)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")


image_converter()
#rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
















































