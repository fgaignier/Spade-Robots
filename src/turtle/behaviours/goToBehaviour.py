

from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import GoToPose

class goToBehaviour(OneShotBehaviour):
    GOAL_REACHED = 0
    GOAL_NON_REACHED = 0
    #navigator = GoToPose()
    
    def __init__(self, name, goal):
        OneShotBehaviour.__init__(self, name)
        self.navigator = GoToPose()
        self.goal = goal

    def process(self):
        print "GoToBehaviour"
        # to be fixed. Quaterion should be given as a parameter
        # default value will be used
        if self.navigator.goTo(self.goal):
            self._exitcode = goToBehaviour.GOAL_REACHED
            print  "goal reached"
        else:
            self._exitcode = goToBehaviour.GOAL_NON_REACHED
            print "goal non reached"
            
    def onStop(self):
        self.navigator.shutdown()        