#from numpy.distutils.fcompiler.g95 import G95FCompiler

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist
from turtle.services.positionService import PositionService

# for quarternion generation
from tf import transformations
import math

class GoToPose:
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("OK")

    def push(self, distance, direction=1):
        print "push the box on ", distance
        self.move(distance, direction)
        
    def move(self, distance, direction):
        print "move on ", distance, "with direction ", direction
        #we'll send a goal to the robot to move 3 meters forward
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = distance #distance
        goal.target_pose.pose.orientation.w = direction #direction

        #start moving
        self.move_base.send_goal(goal)
        #allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to goAt for some reason")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved 3 meters forward")

    # if not given quaternion set by default to 0 values
    def goTo(self, pos, quat={"r1":0, "r2": 0, "r3":0, "r4": 0}):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        
        # Start moving
        print "sending goal", pos, quat
        self.move_base.send_goal(goal)
        print "goal sent"
        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))
        print "moving base finished"
        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()
        
        # will be used to evaluate the difference between goal and reality
        print "goal sent", pos, quat
        print "goal reached:"
        print PositionService.getCurrentPositionAsMap()
        
        self.goal_sent = False
        return result
    
    #rotate from an angle in degre
    def rotateD(self, angle_in_degre):
        curr_pos = PositionService.getCurrentPositionAsMap()
        quat = Position.generateQuaternionD(angle_in_degre)
        self.goTo(curr_pos, quat)

    #rotate from an angle in radian
    def rotateR(self, angle_in_radian):
        curr_pos = PositionService.getCurrentPositionAsMap()
        quat = Position.generateQuaternionR(angle_in_radian)
        self.goTo(curr_pos, quat)
        
    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

class Position:
    
    # takes an angle in degre and returns the quaternion associated
    @staticmethod
    def generateQuaternionD(angle_in_degre):
        #convert euler to quaternion and save in new variable quat
        quat = transformations.quaternion_from_euler(0, 0, math.radians(angle_in_degre))
        #return quaternion
        return Quaternion(quat[0], quat[1], quat[2], quat[3])
    
    # takes an angle in degre and returns the quaternion associated
    @staticmethod
    def generateQuaternionR(angle_in_radian):
        #convert euler to quaternion and save in new variable quat
        quat = transformations.quaternion_from_euler(0, 0, angle_in_radian)
        #return quaternion
        return Quaternion(quat[0], quat[1], quat[2], quat[3])
    
    #returns an angle to the desired direction in radians
    @staticmethod
    def diffInDirection(target, position):
        diff = Position.getDirection(target) - Position.getDirection(position)
        print "difference in angle between expected and realized = ", diff 
        return diff
    
    @staticmethod
    def getDirection(position):
        quat = position["quaterion"]
        quater =  Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4'])
        #angles = transformations.euler_from_quaternion(quater)
        angles = transformations.euler_from_quaternion([quat['r1'], quat['r2'], quat['r3'], quat['r4']])
        print "angles of position = ", angles
      
        print "direction (in radians) = ", angles[2] 
        return angles[2]
    
    # returns the X and Y difference between two positions
    @staticmethod
    def diffInPosition(target, position):
        diffX = target["position"]["x"] - position["position"]["x"]
        diffY = target["position"]["y"] - position["position"]["y"]
        
        print "difference in X in meters", diffX
        print "difference in Y in meters", diffY
        
        #convert in centimeters
        diffX = int(round(diffX*100))
        diffY = int(round(diffY*100))
        
        return [diffX,diffY]
    
# offers the same methods as GoToPos, but with a different method
# we implemented both to compare and chose the best one
class Move:
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    def push(self, distance, angle=0):
        print "push the box on ", distance
        self.move(distance, angle)
        print "finished pushing the box"
        #self.shutdown()

    # enables to go in a given direction (negative distance for backwards)
    # give distance in centimeters and angle in degree
    def move(self, distance, angle):
        print "move on ", distance, "with direction ", angle
        #first rotate
        self.rotateD(angle)
        
        r = rospy.Rate(10)
        move_cmd = Twist()
        # let's go forward at 0.1 m/s
        if distance < 0:
            move_cmd.linear.x = -0.1
        else:
            move_cmd.linear.x = 0.1
            
        distance = abs(distance)
        
        for i in range(distance):
            self.cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
        print "finished moving"
        #self.shutdown()
        
    #rotate from an angle in degre
    def rotateD(self, angle_in_degre):
        self.rotateR(math.radians(angle_in_degre))
        
    def rotateR(self, angle_in_radian):
        r = rospy.Rate(10)
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = angle_in_radian
        rospy.loginfo("Turning")
        for x in range(0,10):
            self.cmd_vel.publish(turn_cmd)
            r.sleep()
                    
    # corection of position
    # first we detect if we are far off
    # then go back to the X angle direction
    # then we correct the X position
    # then we rotate to the desire direction
    # then we correct the y position
    def correctPosition(self, target):
        
        # must first check if difference is big enough 
        current_position = PositionService.getCurrentPositionAsMap()

        # rotates to face the X direction
        direction= Position.getDirection(current_position)
        self.rotateR(-direction)
        
        # we ask each time for the position since any move has an impact
        current_position = PositionService.getCurrentPositionAsMap()
        diff = Position.diffInPosition(target, current_position)
        
        # need to deal with negative Y
        y = target["position"]["y"]
        diffX = diff[0]
        
        # first move        
        self.move(diffX,0)
        
        # get desired angle
        current_position = PositionService.getCurrentPositionAsMap()
        #direction= Position.diffInDirection(target, current_position)
        direction = Position.getDirection(target)
        self.rotateR(direction)
 
        # we ask each time for the position since any move has an impact
        current_position = PositionService.getCurrentPositionAsMap()
        diff = Position.diffInPosition(target, current_position)
        
        # need to deal with negative Y
        diffY = diff[1]
        
        if y < 0 and direction <0:
            self.move(-1*diffY,0)
        else:
            self.move(diffY,0)
    
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
        
        

        
        