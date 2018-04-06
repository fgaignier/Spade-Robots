from turtle.agents.goals import Goals
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import GoToPose

class MoveToWithFeedBackBehaviour(OneShotBehaviour):
    GOAL_REACHED = 0
    GOAL_NON_REACHED = 1
    #navigator = GoToPose()

    def __init__(self, name):
        OneShotBehaviour.__init__(self, name)
        self.navigator = GoToPose()
        
    def process(self):
        print "MoveToWithFeedBackBehaviour"
        pose = self.myAgent.getData("goals")["pose"]
        print "i must go to ", pose
        if self.navigator.goto(pose["position"], pose["quaterion"]):
            self.myAgent.getData("currentGoal")["status"] = Goals.state["suceeded"]
            self._exitcode = MoveToWithFeedBackBehaviour.GOAL_REACHED
            print  "goal reached"
        else:
            self.myAgent.getData("currentGoal")["status"] = Goals.state["failed"]
            self._exitcode = MoveToWithFeedBackBehaviour.GOAL_NON_REACHED
            print "goal non reached"