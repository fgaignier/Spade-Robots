
import rospy
#rospy.init_node("samira")

import json
import spade

from spadeutils.spadeAgent import Agent
from communicator import Communicator
from turtle.agents.goals import Goals
from spadeutils.behaviours.system import SystemCore

from turtle.behaviours.goToBehaviour  import GoToPoseBehaviour
from turtle.behaviours.waitMessageBehaviour import WaitMessageBehaviour
from turtle.behaviours.informNaoBehaviour import InformNaoBehaviour
# regarder si on peut effacer cete classe
#from turtle.behaviours.communication import Communication
from turtle.behaviours.getPositionBehaviour import GetPositionBehaviour
from turtle.behaviours.transformPositionBehaviour import TransformPositionBehaviour
from turtle.behaviours.moveToWithFeedBackBehaviour import MoveToWithFeedBackBehaviour
from turtle.behaviours.testIfNearOtherBehaviour import TestIfNearOtherBehaviour
from turtle.behaviours.PushTheBoxBehaviour import PushTheBoxBehaviour
from turtle.behaviours.MoveArmBehaviour import MoveArmBehaviour

class TurtleAgent(Agent):

 
    def __init__(self, name, host, password, graphplan, actionplan, knowledge):
        self.communicator = Communicator()
        
        print "will init rosnode with name: " + name
        rospy.init_node(name)
        #self.communicator._receiver = receiveBehaviour
        behaviours = self.communicator.getBehaviours()
        behaviours.append(SystemCore())
        Agent.__init__(self, name, host, password, graphplan, actionplan, "", behaviours)
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
        
    def _setup(self):
        Agent._setup(self)
        print "agent starting"
        fsmStates = {"waitForMessage": 1, "goToPose": 2, "informNao": 3,
                     "getPosition": 4, "transformPosition": 5,
                     "moveWithFeedBack": 6, "testIfNear": 7, "pushTheBox": 8, "moveArm":9}
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
        fsm.registerState(PushTheBoxBehaviour(fsmStates["pushTheBox"]),
                          fsmStates["pushTheBox"])
        fsm.registerState(MoveArmBehaviour(fsmStates["moveArm"]),
                          fsmStates["moveArm"])

        # transition succession if message received is goTo
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["goToPose"],WaitMessageBehaviour.goTo)
        fsm.registerTransition(fsmStates["goToPose"], fsmStates["informNao"], 0)
        fsm.registerTransition(fsmStates["informNao"], fsmStates["waitForMessage"], 0)

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
        
        # rajouter pushBigBox 
        # d'abord goTo puis pousser (bien sur il faut synchroniser avec l'autre tortue)
        # attention il y a deja des behaviours qui semblent pas mal: 
        # TellOtherIamReady ......TestIfOtherReady
        
        # transition succession if message received is push
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["pushTheBox"],WaitMessageBehaviour.pushTheBox)
        fsm.registerTransition(fsmStates["pushTheBox"], fsmStates["informNao"], 0)
        fsm.registerTransition(fsmStates["informNao"], fsmStates["waitForMessage"], 0)

        # transition succession if message received is movearm
        fsm.registerTransition(fsmStates["waitForMessage"],fsmStates["moveArm"],WaitMessageBehaviour.moveArm)
        fsm.registerTransition(fsmStates["moveArm"], fsmStates["waitForMessage"], 0)
        
        self.addBehaviour(fsm, None)

        
