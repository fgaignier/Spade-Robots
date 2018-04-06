'''
Created on Apr 2, 2018

@author: gaignier
'''
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour

class endPlanBehaviour(OneShotBehaviour):
    
        
    def onStart(self):
        pass
    
    def process(self):
        self._exitcode = 0
