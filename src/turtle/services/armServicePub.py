'''
Created on Apr 6, 2018
@author: gaignier
using the arbotix python package to command the arm:
http://docs.ros.org/jade/api/arbotix_python/html/classarbotix__python_1_1arbotix_1_1ArbotiX.html#a456f38adeb1075dcdd24ff7f1ad6a1cf
much better than the usual subscribing method
'''

import rospy
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from arbotix_python.joints import getJointsFromURDF
from arbotix_python.joints import getJointLimits


class Servo:
    
    def __init__(self, name, min_angle, max_angle):
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.position = 0
        
        
    def setPosition(self, pos):
        self.position = pos
        
    def getPosition(self):
        return self.position
    
        
class ArmPub:
    
    DEFAULT_SPEED = 20
    # predifined positions. More an be added the same way
    # a list of angles for each servo (first one is the gripper then the motors in order)
    # from these positions we can define variants:
    # with gripper closed. First parameter = 2.6
    # on the left 90. Second parameter = PI/2
    # on the right 90. Second parameter = -PI/2
    POSITION_TAKE = [0,0,0.38,0.28, 0.88, -1.7]
    POSITION_PUT = [2.6,0,0.38,0.28, 0.88, -1.7]
    #POSITION_2 = [0,0,1.07,-0.59, 0.88, -1.7]
    POSITION_RELAX = [0,0,-1.21,1.5,0.64, 0]
    POSITION_BRING = [2.6,0,-0.57,0.12,1.5,-1.7]
    
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        joint_defaults = getJointsFromURDF()
        
        self.publishers = list()
        self.servos = list() # list of the joints
        
        joints = rospy.get_param('/arbotix/joints', dict())
        for name in sorted(joints.keys()):
            # pull min and max angles for each servo
            min_angle, max_angle = getJointLimits(name, joint_defaults)
            serv = Servo(name, min_angle, max_angle)
            self.servos.append(serv)
            # create publisher
            self.publishers.append(rospy.Publisher(name+'/command', Float64,queue_size=5))
        
        # now we can subscribe
        rospy.Subscriber('joint_states', JointState)
        
    # pos est un float (angle en radian) 
    # les valeurs min et max sont stockees dans les objets servo (liste)
    # il y a une valeur par moteur (publishers dans l'ordre joint1 a 5 plus gripper
    def moveServo(self, pub, pos):
        d = Float64()
        print pos
        d.data = pos
        for i in range(1,10):
            pub.publish(d)
        time.sleep(1)
        
 
    def goToPosition(self, positions):
        # send joint updates
        for pub, pos in zip(self.publishers, positions):
            print pos
            self.moveServo(pub, pos)
            
    def closeGripper(self):
        max_angle = self.servos[0].max_angle
        pub = self.publishers[0]
        print " max angle for gripper = " , max_angle
        self.moveServo(pub,max_angle)
    
    def openGripper(self):
        min_angle = self.servos[0].min_angle
        pub = self.publishers[0]
        print " min angle for gripper = " , min_angle
        self.moveServo(pub,min_angle)
        
    def take(self):
        self.goToPosition(self.POSITION_TAKE)
        # waits 2 seconds before closing the gripper
        time.sleep(2)
        self.closeGripper()
        # waits 2 seconds before taking the object off the support
        time.sleep(2)
        self.goToPosition(ArmPub.POSITION_BRING)
        
    def put(self):
        self.goToPosition(self.POSITION_PUT)
        # waits 2 seconds before closing the gripper
        time.sleep(2)
        self.openGripper()
        # waits 2 seconds before taking the object off the support
        time.sleep(2)
        self.goToPosition(ArmPub.POSITION_BRING)
        