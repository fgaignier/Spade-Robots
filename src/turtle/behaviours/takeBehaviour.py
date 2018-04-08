

from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from turtle.services.armService import Arm
import time

class takeBehaviour(OneShotBehaviour):
    
    def __init__(self, name, position):
        OneShotBehaviour.__init__(self, name)
        self.arm = Arm()
        self.position = position

    def process(self):
        print "TakeBehaviour"
        self.arm.goToPosition(self.position)
        # waits 2 seconds before closing the gripper
        time.sleep(2)
        self.arm.closeGripper()
        # waits 2 seconds before taking the object off the support
        time.sleep(2)
        self.arm.goToPosition(Arm.POSITION_BRING)
        