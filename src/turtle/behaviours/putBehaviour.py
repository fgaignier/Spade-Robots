

from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour

class putBehaviour(OneShotBehaviour):
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        #self.arm = self.myAgent.arm

    def process(self):
        print "PutBehaviour"
        self.myAgent.arm.put()
        