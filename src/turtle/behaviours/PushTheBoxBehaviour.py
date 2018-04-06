
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import Move

class PushTheBoxBehaviour(OneShotBehaviour):
    GOAL_REACHED = 0
    GOAL_NON_REACHED = 0
    #navigator = GoToPose()
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        self.navigator = Move()

    def process(self):
        print "PushTheBoxBehaviour"
        # should get this parameter from theagent and pass it over
        self.navigator.push(40)
            
    def onEnd(self):
        print "end of PushTheBoxBehaviour"
        self.navigator.shutdown()   