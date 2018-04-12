#from numpy.distutils.fcompiler.g95 import G95FCompiler

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist


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

    
    
    # if not given quaterion set by default to 0 values
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

        self.goal_sent = False
        return result
   
    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

class Move:
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    def push(self, distance, angle):
        print "push the bow on ", distance
        self.move(distance, angle)
        print "finished pushing the box"

    # enables to go in a given direction (negative distance for backwards)
    # give distance and angle in radians
    def move(self, distance, angle):
        print "move on ", distance
        r = rospy.Rate(10)
        move_cmd = Twist()
        if distance < 0:
            move_cmd.linear.x = -0.1
        else:
            move_cmd.linear.x = 0.1
            
        distance = abs(distance)*10
        move_cmd.angular.x = angle/distance

        for i in range(distance):
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        print "finished moving"
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
        
        

        
        