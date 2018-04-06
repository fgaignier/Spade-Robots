'''
Created on Feb 27, 2018

@author: gaignier
'''

# will propose some higher level Nao actions
# than provided by the apis
# this module is complementary to Naofluent.nao
from naoqi import ALProxy
from plan.Pathplan import PathPlan
import almath
from almathswig import Pose2D
import naoutil.naoenv as naoenv
from naoutil.broker import Broker
from fluentnao.nao import Nao

import time
import math

class NaoActions(object):
    
    def __init__(self, nao_ip, nao_port, map_path):
        # gets a simplified map for Nao
        # since thereis no navigation in Nao, we give it a simple
        # but efficient one
        self.map = PathPlan()
        self.map.loadFromFile(map_path)
        # at start Nao is at this theoritical position
        self.position = Pose2D(0,0,0)
        
        self.motionProxy = ALProxy("ALMotion", nao_ip, nao_port)
        self.postureProxy = ALProxy("ALRobotPosture", nao_ip, nao_port)
        self.trackerProxy = ALProxy("ALTracker", nao_ip, nao_port)
        try:
            self.faceProxy = ALProxy("ALFaceDetection", nao_ip, nao_port)
        except Exception, e:
            print "Error when creating face detection proxy:"
            print str(e)
            exit(1)
        # but we store as well the real original positon
        # Nao is constantly deviating from its theoritical trajectory
        # thus we must adjust 
        self.startDelta = self.getInitialDelta()
        # controls joints and Locomotion
        self.initNaoFluent(nao_ip, nao_port)
        
        
    def initNaoFluent(self, nao_ip, nao_port):
        self.broker = Broker('bootstrapBroker', naoIp=nao_ip, naoPort=nao_port)
        # FluentNao
        self.naofluent = Nao(naoenv.make_environment(None))
    
    def getInitialDelta(self):
        useSensorValues = True
        realpos = self.motionProxy.getRobotPosition(useSensorValues)
        return Pose2D(self.position.x - realpos[0], self.position.y - realpos[1],0)
    
    # probleme. Le referentiel tourne avec NAO.
    # donc il faut revoir tous les mouvements
    def goto(self, dest):
        # Wake up robot
        self.motionProxy.wakeUp()
        # Send robot to Pose Init
        self.postureProxy.goToPosture("StandInit", 0.5)

        start = self.position
        print "starting position: " , start
        points = self.map.getPointsFromTo(start, dest)
        prev = start
        alpha = 0
        for nextp in points:
            print "next position targeted: ", nextp
            dX = nextp.x - prev.x
            dY = nextp.y - prev.y
            XF = dX*math.cos(alpha) - dY*math.sin(alpha)
            YF = dX*math.sin(alpha) - dY*math.cos(alpha)
            nextpf = Pose2D(XF,YF, 0)
            alpha = self.getAngle(prev, nextpf)
            
            move = Pose2D(XF, YF, alpha)
         
            print "move: " , move 
            self.motionProxy.moveTo(move.x, move.y, move.theta)
            prev = move
            alpha = 0
            
    # from 2 Pose2D points
    # in radians
    def getAngle(self, X, Y):
        dx = Y.x-X.x
        dy = Y.y-X.y

        if dy == 0:
            return 0
        else:
            ratio = dx/dy
            if ratio < 0:
                sign = -1
            else:
                sign = 1
            return sign*(math.pi/2 - math.atan(math.fabs(ratio)))
    
    # returns a corrected move Pose2D to correct the deviance of Nao
    # to be tested
    def correctTrajectory(self, move):
        useSensorValues = True
        realpos = self.motionProxy.getRobotPosition(useSensorValues)
        delta = Pose2D(self.position.x - realpos[0], self.position.y - realpos[1],0)
        #print "delta of position: ", delta
        #print "corrected move: " , Pose2D(move.x + self.startDelta.x-delta.x, move.y + self.startDelta.y - delta.y, move.theta)
        #return Pose2D(move.x + self.startDelta.x-delta.x, move.y + self.startDelta.y - delta.y, move.theta)
    
    # does not work properly.
    # we set the position at (0,0,0) at start and store the new position after each move
    def getPosition(self):
        print("getPosition function")
        useSensorValues = False
        initialPos = self.motionProxy.getRobotPosition(useSensorValues)
        print "initial position = ", initialPos

    def speak(self, sentence):
        self.naofluent.say(sentence)

    def test(self):
        offset = -30 * almath.TO_RAD
        offset2 = -35 * almath.TO_RAD
        
        self.motionProxy.setStiffnesses("LArm", 1.0)
        self.motionProxy.setStiffnesses("RArm", 1.0)
        #self.motionProxy.changeAngles(["LShoulderPitch"], [offset], 1)
        #self.motionProxy.changeAngles(["LShoulderRoll"], [offset2], 1)
        #self.motionProxy.changeAngles(["RShoulderPitch"], [offset], 1)
        #self.motionProxy.changeAngles(["RShoulderRoll"], [offset2], 1)
        
        self.motionProxy.angleInterpolation(["LShoulderPitch"], [offset], [2], True)
        self.motionProxy.angleInterpolation(["LShoulderRoll"], [offset2], [2], True)
        self.motionProxy.angleInterpolation(["RShoulderPitch"], [offset], [2], True)
        self.motionProxy.angleInterpolation(["RShoulderRoll"], [offset2], [2], True)

    # angles a calculer en fonction de la hauteur de Nao (constante a noter
    # et la hauteur de l'objet
    # si la hauteur de l'objet est inferieure a celle de Nao, utiliser le package legs
    # pour se baisser
    def take(self, height, width):
        # soit l la longueur du bras
        # soit H la longueur de l'epaule au sol
        # angle entre les points (l, 0) et [(H-h), sqrt(l*l - (H-h)*(H-h))]
        # converitr en degres => *180/PI
        # determiner les 3 parametres. Les deux derniers sont des angles
        # le premier pour la position verticale (en degres)
        # le second a voir. surement la position horizontale (en degres)
        self.naofluent.stiff()
        self.naofluent.arms.forward(0, -30, -35)
        #self.naofluent.arms.out(0,45, -45)
        self.naofluent.hands.open()
        self.naofluent.arms.forward(0, -20, -35)
        self.naofluent.go()
        #self.motionProxy.moveTo(-0.2, 0, 0)
 
    def put(self, height):
        self.naofluent.arms.forward(20, -45, 25)
        self.naofluent.hands.open(5)
        self.naofluent.wrists.turn_in(20,90)
        # and relax the arms
        self.naofluent.arms.relax()

    ##################################################################################################
            
    def trackObject(self):
    
        fractionMaxSpeed = 0.8
        # Go to posture stand
        self.postureProxy.goToPosture("StandInit", fractionMaxSpeed)
    
        # Add target to track.
        targetName = "RedBall"
        diameterOfBall = 4
        self.trackerProxy.registerTarget(targetName, diameterOfBall)
    
        # set mode
        mode = "Move"
        self.trackerProxy.setMode(mode)
    
        # Then, start tracker.
        self.trackerProxy.track(targetName)
    
        print "ALTracker successfully started, now show a red ball to robot!"
        print "Use Ctrl+c to stop this script."
    
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print
            print "Interrupted by user"
            print "Stopping..."
    
        # Stop tracker, go to posture Sit.
        self.trackerProxy.stopTracker()
        self.trackerProxy.unregisterAllTargets()
        self.postureProxy.goToPosture("Sit", fractionMaxSpeed)
        self.motionProxy.rest()
    
        print "ALTracker stopped."

