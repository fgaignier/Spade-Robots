'''
Created on Apr 6, 2018

@author: gaignier

using the arbotix python package to command the arm:
http://docs.ros.org/jade/api/arbotix_python/html/classarbotix__python_1_1arbotix_1_1ArbotiX.html#a456f38adeb1075dcdd24ff7f1ad6a1cf
much better than the usual subscribing method

'''

import rospy
import math
import time

from arbotix_python.joints import getJointsFromURDF
from arbotix_python.joints import getJointLimits

from arbotix_python.arbotix import ArbotiX
from math import radians

class Servo:
    
    def __init__(self, name, min_angle, max_angle, ticks, neutral, rad_per_tick, invert):
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.position = 0
        self.ticks = ticks
        self.neutral = neutral
        self.rad_per_tick = rad_per_tick
        self.invert = invert
        
    def setPosition(self, pos):
        self.position = pos
        
    def getPosition(self):
        return self.position
    
    def angleToTicks(self, angle):
        ticks = self.neutral + (angle/self.rad_per_tick)
        if self.invert:
            ticks = self.neutral - (angle/self.rad_per_tick)
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return int(round(ticks))
    
class Arm:
    
    DEFAULT_SPEED = 20
    # predifined positions. More an be added the same way
    # a list of angles for each servo (first one is the gripper then the motors in order)
    # from these positions we can define variants:
    # with gripper closed. First parameter = 2.6
    # on the left 90. Second parameter = PI/2
    # on the right 90. Second parameter = -PI/2
    POSITION_1 = [0,0,0.38,0.28, 0.88, -1.7]
    POSITION_2 = [0,0,1.07,-0.59, 0.88, -1.7]
    POSITION_RELAX = [0,0,-1.21,1.5,0.64, 0]
    POSITION_BRING = [2.6,0,0,math.pi/2,0,0]
    
    def __init__(self, port="/dev/ttyUSB0"):
        # creating an ArbotiX device (the arm) on port (normally "/dev/ttyUSB0")
        self.device = ArbotiX(port)
        
        joint_defaults = getJointsFromURDF()
        self.servos = list() # list of the joints
        
        joints = rospy.get_param('/arbotix/joints', dict())
        for name in sorted(joints.keys()):
            # pull data to convert from Angle to ticks
            n = "/arbotix/joints/"+ name +"/"
            ticks = rospy.get_param(n+"ticks", 1024)
            neutral = rospy.get_param(n+"neutral", ticks/2)
            invert = rospy.get_param(n+"invert", False)
            ranges = rospy.get_param(n+"range", 300)
            rad_per_tick = radians(ranges)/ticks
            
            # pull min and max angles for each servo
            min_angle, max_angle = getJointLimits(name, joint_defaults)
            serv = Servo(name, min_angle, max_angle, ticks, neutral, rad_per_tick, invert)
            self.servos.append(serv)
        
    # moves a single servo at a given speed to a given angle (in radians)
    # by default the speed is set at 20
    def moveServo(self, idserv, angle , speed=DEFAULT_SPEED):
        serv = self.servos[idserv]
        self.device.setSpeed(idserv, speed)
        ticks = serv.angleToTicks(angle)
        self.device.setPosition(idserv, ticks)
    
    
    def goToPosition(self, position):
        i = 0
        for pos in position:
            print "moving servo: ", i
            self.moveServo(i,pos)
            i = i+1
            
    def closeGripper(self):
        max_angle = self.servos[0].max_angle
        self.moveServo(0,max_angle)
    
    def openGripper(self):
        min_angle = self.servos[0].min_angle
        self.moveServo(0,min_angle)
        
    def take(self):
        self.goToPosition(self.POSITION_1)
        # waits 2 seconds before closing the gripper
        time.sleep(2)
        self.closeGripper()
        # waits 2 seconds before taking the object off the support
        time.sleep(2)
        self.goToPosition(Arm.POSITION_BRING)
        
    def put(self):
        self.goToPosition(self.POSITION_1)
        # waits 2 seconds before closing the gripper
        time.sleep(2)
        self.openGripper()
        # waits 2 seconds before taking the object off the support
        time.sleep(2)
        self.goToPosition(Arm.POSITION_BRING)
        