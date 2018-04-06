#from numpy.distutils.fcompiler.g95 import G95FCompiler

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax
from arbotix_python.joints import getJointsFromURDF
from arbotix_python.joints import getJointLimits
from sensor_msgs.msg import JointState


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
        
        # simple move services
        #self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    # will be used for goto and pushTo (with different speed
    def goto(self, pos, quat):
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

    def push(self, distance):
        print "push the bow on ", distance
        r = rospy.Rate(10)
        move_cmd = Twist()
        move_cmd.linear.x = -0.1
        move_cmd.angular.z = 0
        for i in range(distance):
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        print "finished pushing the box"

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
        
        
class Arm:
    
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        joint_defaults = getJointsFromURDF()

        # pas sur que ca serve
        dynamixels = rospy.get_param('/arbotix/dynamixels', dict())
      
        self.publishers = list()
        self.relaxers = list()

        joints = rospy.get_param('/arbotix/joints', dict())
        for name in sorted(joints.keys()):
            # pull angles
            min_angle, max_angle = getJointLimits(name, joint_defaults)
            print "Angles for joint " + name, min_angle, max_angle
            # create publisher
            self.publishers.append(rospy.Publisher(name+'/command', Float64, queue_size=5))
            if rospy.get_param('/arbotix/joints/'+name+'/type','dynamixel') == 'dynamixel':
                self.relaxers.append(rospy.ServiceProxy(name+'/relax', Relax))
            else:
                self.relaxers.append(None)
            
        # now we can subscribe
        rospy.Subscriber('joint_states', JointState, self.stateCb)
        
    def moveArm(self, positions, forward, turn):
        # send joint updates
        for pub, pos in zip(self.publishers, positions):
            d = Float64()
            # d.data est un float entre -2.61 et + 2.61 (max)
            # il y a une valeur par moteur (publishers dans l'ordre joint1 a 5 plus gripper
            d.data = pos
            pub.publish(d)
        # send base updates
        # linear {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}
        # a priori toujours a 0 pour les 6 valeurs
        t = Twist()
        t.linear.x = forward/150.0; t.linear.y = 0; t.linear.z = 0
        if forward > 0:
            t.angular.x = 0; t.angular.y = 0; t.angular.z = -turn/50.0
        else:
            t.angular.x = 0; t.angular.y = 0; t.angular.z = turn/50.0
            
        self.cmd_vel.publish(t)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
        
        