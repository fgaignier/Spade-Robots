
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import Move, GoToPose

class moveBehaviour(OneShotBehaviour):
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        # chose here what method you want to use
        self.navigator = Move()
        #self.navigator = GoToPose()

    def process(self):
        print "moveBehaviour"
        params = self.myAgent.parameters
        distance = int(params[0])
        angle = float(params[1])
        self.navigator.move(distance, angle)
            
    def onEnd(self):
        print "end of moveBehaviour"
        self.navigator.shutdown()   