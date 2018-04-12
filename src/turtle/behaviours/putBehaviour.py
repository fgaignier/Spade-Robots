

from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour

class takeBehaviour(OneShotBehaviour):
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        self.arm = self.myAgent.arm

    def process(self):
        print "PutBehaviour"
        self.arm.put()
        