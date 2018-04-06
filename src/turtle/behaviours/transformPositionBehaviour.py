from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour

class TransformPositionBehaviour(OneShotBehaviour):

    def process(self):
        otherPosition = self.myAgent.getData("otherPosition")
        otherPosition["position"]["y"] -= 0.5
        self.myAgent.getData("goals")["pose"] = otherPosition