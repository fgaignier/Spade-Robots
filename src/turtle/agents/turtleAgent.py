
import rospy
#rospy.init_node("samira")

import json
import spade

from spadeutils.spadeAgent import Agent
from communicator import Communicator
from turtle.agents.goals import Goals
from spadeutils.behaviours.system import SystemCore

from turtle.behaviours.goToPoseBehaviour  import GoToPoseBehaviour
from turtle.behaviours.waitMessageBehaviour import WaitMessageBehaviour
from turtle.behaviours.informNaoBehaviour import InformNaoBehaviour
from turtle.behaviours.getPositionBehaviour import GetPositionBehaviour
from turtle.behaviours.transformPositionBehaviour import TransformPositionBehaviour
from turtle.behaviours.moveToWithFeedBackBehaviour import MoveToWithFeedBackBehaviour
from turtle.behaviours.testIfNearOtherBehaviour import TestIfNearOtherBehaviour
from turtle.behaviours.moveBehaviour import moveBehaviour
from turtle.behaviours.pushBehaviour import pushBehaviour
from turtle.behaviours.takeBehaviour import takeBehaviour
from turtle.behaviours.informBehaviour import InformBehaviour

from plan.PlanExecutor import PlanExecutor
#from turtle.services.armService import Arm
from turtle.services.armServicePub import ArmPub

class TurtleAgent(Agent):

 
    def __init__(self, name, host, password, graphplan, actionplan, knowledge):
        self.communicator = Communicator()
        
        print "will init rosnode with name: " + name
        rospy.init_node(name)
        
        # init arm
        #self.arm = Arm()
        #self.arm.goToPosition(Arm.POSITION_RELAX)
        
        self.arm = ArmPub()
        self.arm.goToPosition(ArmPub.POSITION_RELAX)
        
        
        behaviours = self.communicator.getBehaviours()
        behaviours.append(SystemCore())
        
        # does not use the overwritten SpadeAgent
        # calls directly the constructor of Spade Agent
        Agent.__init__(self, name, host, password, graphplan, actionplan, behaviours)
        
        
        with open(knowledge) as knowledge:
            self.setData("knowledge", json.load(knowledge)[self.localName])
        self.setData("goals", {"pose": {}})
        self.setData("naoAid",spade.AID.aid("nao1@"+host,
                                 ["xmpp://nao1@"+host]))
        other = self.getData("knowledge")["other"] + "@" + host
        otherAid = spade.AID.aid(other, ["xmpp://" + other])
        self.setData("otherTurtleAid", otherAid)
        self.setData("otherState", Goals.otherStatus["nonReady"])
        print self.getData("knowledge")
        
        # stores parameters received by waitMessageBehaviour
        # in case it is a plan
        self.plan = ""
        # stores the parameters of an order
        self.parameters = list()
        # stores who gave the order
        self.messageSender = ""
        self.name = name
        
        
    def _setup(self):
        Agent._setup(self)
        print "agent starting"
        fsmStates = {"waitForMessage": 1, "goToPose": 2, "informNao": 3,
                     "getPosition": 4, "transformPosition": 5,
                     "moveWithFeedBack": 6, "testIfNear": 7, "move": 8, "planExecute": 9,
                     "take": 10, "push": 11, "inform": 12, "put": 13}
        
        fsm = spade.Behaviour.FSMBehaviour()

        fsm.registerFirstState(WaitMessageBehaviour(fsmStates["waitForMessage"]),fsmStates["waitForMessage"])
        fsm.registerState(GoToPoseBehaviour(fsmStates["goToPose"]),fsmStates["goToPose"])
        fsm.registerState(InformNaoBehaviour(fsmStates["informNao"]), fsmStates["informNao"])
        fsm.registerState(GetPositionBehaviour(fsmStates["getPosition"]), fsmStates["getPosition"])
        fsm.registerState(TransformPositionBehaviour(fsmStates["transformPosition"]),fsmStates["transformPosition"])
        fsm.registerState(MoveToWithFeedBackBehaviour(fsmStates["moveWithFeedBack"]),
                          fsmStates["moveWithFeedBack"])
        fsm.registerState(TestIfNearOtherBehaviour(fsmStates["testIfNear"]),
                          fsmStates["testIfNear"])
        fsm.registerState(moveBehaviour(fsmStates["move"]), fsmStates["move"])
        fsm.registerState(PlanExecutor(), fsmStates["planExecute"])
        fsm.registerState(pushBehaviour(fsmStates["push"]), fsmStates["push"])
        fsm.registerState(takeBehaviour(fsmStates["take"]), fsmStates["take"])
        fsm.registerState(InformBehaviour(fsmStates["inform"]), fsmStates["inform"])
        fsm.registerState(InformBehaviour(fsmStates["put"]), fsmStates["put"])
        
        
        # transition succession if message received is goNear
        fsm.registerTransition(fsmStates["waitForMessage"], fsmStates["getPosition"], WaitMessageBehaviour.goNear)
        fsm.registerTransition(fsmStates["getPosition"], fsmStates["transformPosition"], 0)
        fsm.registerTransition(fsmStates["transformPosition"], fsmStates["moveWithFeedBack"], 0)

        fsm.registerTransition(fsmStates["moveWithFeedBack"], fsmStates["testIfNear"],
                               MoveToWithFeedBackBehaviour.GOAL_REACHED)
        fsm.registerTransition(fsmStates["moveWithFeedBack"], fsmStates["informNao"],
                               MoveToWithFeedBackBehaviour.GOAL_NON_REACHED)

        fsm.registerTransition(fsmStates["testIfNear"], fsmStates["informNao"],
                               TestIfNearOtherBehaviour.Near)
        fsm.registerTransition(fsmStates["testIfNear"], fsmStates["getPosition"],
                               TestIfNearOtherBehaviour.NotNear)
        
        """ newly added fonctionnalities
            simple actions: goTo, move, take, push
            complex actions requesting a plan: planExecute
        """
        # transition succession if message received is goTo
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["goToPose"],WaitMessageBehaviour.goTo)
        fsm.registerTransition(fsmStates["goToPose"],fsmStates["inform"],0)
        fsm.registerTransition(fsmStates["inform"], fsmStates["waitForMessage"], 0)
        
        # transition succession if message received is move
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["move"],WaitMessageBehaviour.move)
        fsm.registerTransition(fsmStates["move"],fsmStates["inform"],0)
        fsm.registerTransition(fsmStates["inform"], fsmStates["waitForMessage"], 0)

        # transition succession if message received is push
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["push"],WaitMessageBehaviour.push)
        fsm.registerTransition(fsmStates["push"],fsmStates["inform"],0)
        fsm.registerTransition(fsmStates["inform"], fsmStates["waitForMessage"], 0)
        
        # transition succession if message received is take
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["take"],WaitMessageBehaviour.take)
        fsm.registerTransition(fsmStates["take"],fsmStates["inform"],0)
        fsm.registerTransition(fsmStates["inform"], fsmStates["waitForMessage"], 0)
        
         # transition succession if message received is put
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["put"],WaitMessageBehaviour.put)
        fsm.registerTransition(fsmStates["put"],fsmStates["inform"],0)
        fsm.registerTransition(fsmStates["inform"], fsmStates["waitForMessage"], 0)
        
        # plan execution
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["planExecute"],WaitMessageBehaviour.planExecute)
        fsm.registerTransition(fsmStates["planExecute"],fsmStates["inform"],0)
        fsm.registerTransition(fsmStates["inform"], fsmStates["waitForMessage"], 0)
        
        self.addBehaviour(fsm, None)

        
