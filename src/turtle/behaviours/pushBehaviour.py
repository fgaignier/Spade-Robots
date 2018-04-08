
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import Move

class pushBehaviour(OneShotBehaviour):
    GOAL_REACHED = 0
    GOAL_NON_REACHED = 0
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        self.navigator = Move()

    def process(self):
        print "PushBehaviour"
        distance = self.myAgent.distance
        self.navigator.push(distance)
            
    def onEnd(self):
        print "end of PushBehaviour"
        self.navigator.shutdown()   