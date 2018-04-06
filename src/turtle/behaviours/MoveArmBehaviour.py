
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import Arm

class MoveArmBehaviour(OneShotBehaviour):
    GOAL_REACHED = 0
    GOAL_NON_REACHED = 0
    #navigator = GoToPose()
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        self.navigator = Arm()

    def process(self):
        print "MoveArmBehaviour"
        # need to understand how to move the arm
        #self.navigator.push(40)
            
    def onEnd(self):
        print "end of PushTheBoxBehaviour"
        self.navigator.shutdown()   