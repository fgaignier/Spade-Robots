
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import Move, GoToPose

class pushBehaviour(OneShotBehaviour):
    GOAL_REACHED = 0
    GOAL_NON_REACHED = 0
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        # chose here what method you want to use
        self.navigator = Move()
        #self.navigator = GoToPose()

    def process(self):
        print "PushBehaviour"
        distance = int(self.myAgent.parameters[0])
        #angle = self.myAgent.parameters[1]
        self.navigator.push(distance)
            
    def onEnd(self):
        print "end of PushBehaviour"
        self.navigator.shutdown()   