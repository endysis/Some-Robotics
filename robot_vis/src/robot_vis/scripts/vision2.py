#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
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
    isFinding = False
    
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
    
    enableFind = False
    
    completeMess = False
    
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
    
    robotTurnCount = 0
    
    coorValues = [[-1.28,-4.51],[1.5,-4.2],[1,-1],[-3.96,1.908],[-4.24,0.52],[1.1,2.05],[-0.81,3.74],[1.18,4.65],[3.28,4.38]]  
    #coorValues = [[1.46,-4.0],[0.74,-0.9],[-0.85,-1.39],[-4.12,-0.007],[-4.19,1.48],[-3.86,-1.67],[-0.66,1.85],[-0.8,3.6]] 
    
    #coorValues = [[1.46,-4.0],[0.74,-0.9],[2.686,1.105],[-4.12,-0.007],[-4.0,1.85],[-1.9,-1.7],[-0.8,3.6]] 
    
    currTarget = coorValues[0]
    
    def __init__(self):
        
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Image window2", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image, self.callback)
        self.scan_sub = rospy.Subscriber('/turtlebot/scan',LaserScan,self.laserCallBack)
        self.rescallback = rospy.Subscriber("/turtlebot/move_base/feedback", MoveBaseActionFeedback, self.resrobot)
        self.result = rospy.Subscriber("/turtlebot/move_base/result",MoveBaseActionResult,self.resultBack)
        
        #Publish to the cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist()       
        
        self.basePub = rospy.Publisher("/turtlebot/move_base_simple/goal", PoseStamped, queue_size = 100)
        
   
    def resrobot(self, value):
            global coorValues
            currX = value.feedback.base_position.pose.position.x
            currY = value.feedback.base_position.pose.position.y
            
            #print(value.status.status)            
            
            if value.status.status == 1:
                self.isMoving = True
                #print ("It is moving",value.status.status)
      
            else:
                print ("It is not moving",value.status.status)
                
#                if currX > (self.coorValues[0][0] - 1) and currX < (self.coorValues[0][0] + 1) and currY > (self.coorValues[0][1] - 1) and currY < (self.coorValues[0][1] + 1): 
#                    print ("Robot is at its destination")                    
#                    self.coorValues.pop(0)
#                    self.isMoving = False
                    
                                
    def resultBack(self,value):
        global coorValues        
        global enableFind
        if value.status.status == 3:
            print("Landmark Reached")
            self.enableFind = True  
            self.coorValues.pop(0)
            #print("New Coordinates")
        else:
            self.enableFind = True  
            self.coorValues.pop(0)
            print("Has reached else statment")
            
            
        
 
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
        global enableFind
        
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
                self.redFound = False # For the turning statment
                self.colourArr.pop(0)
                
            elif self.colourArr[0] == "Green" and self.greenArea > 48000:
                self.greenComplete = True
                self.greenFound = False
                self.colourArr.pop(0)
                
            elif self.colourArr[0] == "Blue" and self.blueArea > 48000:
                self.blueComplete = True
                self.blueFound = False
                self.colourArr.pop(0)
                
            elif self.colourArr[0] == "Yellow" and self.yellowArea > 48000:
                self.yellowComplete = True
                self.yellowFound = False
                self.colourArr.pop(0)
                
            self.stopCheck = False
            #print ("New colour is %s",self.colourArr[0])
            
            # Pops off red, then goes straght to green but because green equals 0 now, straight away
            # it is marked as complete

    
    def laserCallBack(self,data):      
        global laserScan
        self.laserScan = data.ranges[len(data.ranges)/2]
        #print ("range ahead:  %0.1f" % self.laserScan)
        
        
    def turn(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 2
        self.cmd_vel.publish(self.twist) 

        
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
        
        global isFinding    
        
        global robotTurnCount
        
        global completeMess
        
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
 
 
        if self.isMoving == False and self.enableFind == False:
            if(not self.coorValues):
                if self.completeMess == False:
                    print ("Search Complete")
                    self.completeMess = True
            else:
                #print("Entered Move_base Command")
                #print (self.coorValues)
                self.moveToPoint(self.coorValues[0][0],self.coorValues[0][1],0)
        
        

        if self.enableFind == True:        
            
            if(self.redComplete == False):            
                for c in hsv_contoursR:
                    a = cv2.contourArea(c)
                    self.redArea = a
                    print (a)
                    if a > 5000:  # Makes sure the robot only goes to nodes within its area
                        
                        if self.redFound == False:
                            self.colourArr.append('Red')
                            print ("Found red")
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


            if(self.greenComplete == False):              
                for c in hsv_contoursG:
                    a = cv2.contourArea(c)
                    self.greenArea = a
                    #print 'G: ' + str(a)
                    if a > 5000:
                        if self.greenFound == False:
                            self.colourArr.append('Green')
                            print ("Found Green")
                            self.greenFound = True
                    self.greenDistance = a
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
                    centerConX = int(momG['m10']/momG['m00'])
                    angVel = centerConX - w/2
                    if(self.greenFound == True and self.greenComplete == False):
                            if(self.colourArr[0] == "Green"):
                                 self.goToColour(0.5,angVel,1)
                       
            if(self.yellowComplete == False):
                for c in hsv_contoursY:
                    a = cv2.contourArea(c)
                    self.yellowArea = a
                    #print 'Y: ' + str(a)
                    if a > 5000:
                        if self.yellowFound == False:
                            self.colourArr.append('Yellow')
                            print ("Found Yellow")
                            self.yellowFound = True
                    self.greenDistance = a
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
                    centerConX = int(momY['m10']/momY['m00'])
                    angVel = centerConX - w/2
                    if(self.yellowFound == True and self.yellowComplete == False):
                            if(self.colourArr[0] == "Yellow"):
                                 self.goToColour(0.5,angVel,1)
    
            if(self.blueComplete == False):                        
                for c in hsv_contoursB:
                    a = cv2.contourArea(c)
                    self.blueArea = a
                    #print 'B: ' + str(a)
                    if a > 5000:
                        if self.blueFound == False:
                            self.colourArr.append('Blue')
                            print ("Found Blue")
                            self.blueFound = True
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0))              
                    centerConX = int(momB['m10']/momB['m00'])
                    angVel = centerConX - w/2
                    if(self.blueFound == True and self.blueComplete == False):
                            if(self.colourArr[0] == "Blue"):
                                 self.goToColour(0.5,angVel,1)
                
         
             
            if(self.redFound == False and self.greenFound == False and self.yellowFound == False and self.blueFound == False): # Turns if the robot sees nothing in its sights
                self.turn()
                self.robotTurnCount += 1
                
                if self.robotTurnCount >= 11:
                    self.robotTurnCount = 0
                    self.enableFind = False
                    self.isMoving = False
                    
                    
            
         
        #print '===='
        #cv2.imshow("Image window", cv_image)
        cv2.imshow("Image window2",mask)
       
        

if __name__ == '__main__':
    try:
        rospy.init_node('image_converter',anonymous=False)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")


image_converter()
#rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
















































