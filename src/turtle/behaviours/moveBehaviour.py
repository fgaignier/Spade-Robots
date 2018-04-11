
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import Move

class moveBehaviour(OneShotBehaviour):
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        self.navigator = Move()

    def process(self):
        print "moveBehaviour"
        params = self.myAgent.moveParams
        distance = params[0]
        angle = params[1]
        print "moving on distance ", distance, "with angel ", angle 
        self.navigator.move(distance, angle)
            
    def onEnd(self):
        print "end of moveBehaviour"
        self.navigator.shutdown()   