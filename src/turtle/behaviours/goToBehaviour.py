from turtle.agents.goals import Goals
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import GoToPose

class GoToPoseBehaviour(OneShotBehaviour):
    GOAL_REACHED = 0
    GOAL_NON_REACHED = 0
    #navigator = GoToPose()
    
    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        self.navigator = GoToPose()

    def process(self):
        print "GoToPoseBehaviour"
        pose = self.myAgent.getData("goals")["pose"]
        if self.navigator.goto(pose["position"], pose["quaterion"]):
            self.myAgent.getData("currentGoal")["status"] = Goals.state["suceeded"]
            self._exitcode = GoToPoseBehaviour.GOAL_REACHED
            print  "goal reached"
        else:
            self.myAgent.getData("currentGoal")["status"] = Goals.state["failed"]
            self._exitcode = GoToPoseBehaviour.GOAL_NON_REACHED
            print "goal non reached"
            
    #def onStop(self):
    #self.navigator.shutdown()        