from turtle.agents.goals import Goals
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.navigationService import GoToPose, Move

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
        if self.navigator.goTo(pose["position"], pose["quaterion"]):
            # even if the move is a success, we need to adjust
            # stop the navigator
            self.navigator.shutdown()
            # correction: will assess the reached position against thetraget and ajust if necessary
            corrector = Move()
            corrector.correctPosition(pose)
            corrector.shutdown()
            
            self.myAgent.getData("currentGoal")["status"] = Goals.state["suceeded"]
            self._exitcode = GoToPoseBehaviour.GOAL_REACHED
            print  "goal reached"
        else:
            self.myAgent.getData("currentGoal")["status"] = Goals.state["failed"]
            self._exitcode = GoToPoseBehaviour.GOAL_NON_REACHED
            print "goal non reached"
            
    def onStop(self):
        self.navigator.shutdown()        