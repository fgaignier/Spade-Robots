'''
Created on Apr 5, 2018

@author: gaignier
'''

import rospy
from geometry_msgs.msg import Twist

from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax
from arbotix_python.joints import getJointsFromURDF
from arbotix_python.joints import getJointLimits
from sensor_msgs.msg import JointState

from arbotix_python.arbotix import ArbotiX
from math import radians
import time

class Servo:
    
    def __init__(self, name, min_angle, max_angle, ticks, neutral, rad_per_tick):
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.position = 0
        self.ticks = ticks
        self.neutral = neutral
        self.rad_per_tick = rad_per_tick
        
    def setPosition(self, pos):
        self.position = pos
        
    def getPosition(self):
        return self.position
    
    def angleToTicks(self, angle):
        ticks = self.neutral + (angle/self.rad_per_tick)
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return int(round(ticks))
    
class Arm:
    
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        joint_defaults = getJointsFromURDF()

        
        self.publishers = list()
        self.relaxers = list()
        self.servos = list() # list of the joints
        
        joints = rospy.get_param('/arbotix/joints', dict())
        for name in sorted(joints.keys()):
            # pull data to convert from Angle to ticks
            n = "/arbotix/joints/"+ name +"/"
            ticks = rospy.get_param(n+"ticks", 1024)
            neutral = rospy.get_param(n+"neutral", ticks/2)
            range = rospy.get_param(n+"range", 300)
            rad_per_tick = radians(range)/ticks
            
            # pull angles
            min_angle, max_angle = getJointLimits(name, joint_defaults)
            serv = Servo(name, min_angle, max_angle, ticks, neutral, rad_per_tick)
            self.servos.append(serv)

            # create publisher
            self.publishers.append(rospy.Publisher(name+'/command', Float64, queue_size=5))
            if rospy.get_param('/arbotix/joints/'+name+'/type','dynamixel') == 'dynamixel':
                self.relaxers.append(rospy.ServiceProxy(name+'/relax', Relax))
            else:
                self.relaxers.append(None)
            
        # now we can subscribe
        #rospy.Subscriber('joint_states', JointState)
        
    def moveServo(self, pos):
        d = Float64()
        print pos
        d.data = pos
        pub = self.publishers[2]
        pub.publish(d)
        
    def moveArm(self, positions, forward, turn):
        # send joint updates
        for pub, pos in zip(self.publishers, positions):
            d = Float64()
            # d.data est un float (angle en radian) 
            # les valeurs min et max sont stockees dans les objets servo (liste)
            # il y a une valeur par moteur (publishers dans l'ordre joint1 a 5 plus gripper
            print pos
            d.data = pos
            pub.publish(d)
      
    """
    def getPosition(self, name):
        for serv in self.servos:
            idx = msg.name.index(serv.name)
            serv.setPosition(msg.position[idx])
    """
        
    def newMoveServo(self, device, idserv):
        serv = self.servos[idserv]
        device.setSpeed(idserv, 20)
        ticks = serv.angleToTicks(0.7854)
        print ticks
        device.setPosition(idserv, ticks)
    
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
        
        
if __name__ == "__main__":
    rospy.init_node('arm')
    d = ArbotiX(port="/dev/ttyUSB0")
    arm = Arm()
    arm.newMoveServo(d, 2)
    
    '''
    # premiere methode... pas mal mais n'arrive pas a fixer la vitesse
    arm = Arm()
    positions = [0,0,0,0,0,0]
    forward = 5
    turn = 5
    pos = 0.0
    for i in range(0,10):
        arm.moveServo(pos)
        time.sleep(0.5)
        pos = pos + 0.1


    for i in range(0,10):
        arm.moveArm(positions, forward, turn)
    time.sleep(4)
    i = 0.0
    while i < 0.6:
    positions[0] = 2
    positions[2] = i #moteur 2 (position 3)
        positions[3] = i #moteur 3 (position 4)
    positions[4] = i #moteur 4 (position 5)
    positions[5] = i*1.5 #moteur 5 (position 6)
    #positions[6]
        arm.moveArm(positions, forward, turn)
    time.sleep(0.2)
    i = i+0.1
    '''