'''
Created on Feb 24, 2018

@author: gaignier
'''

from spadeAgent import Agent
from plan import PlanExecutorOld.PlanExecutor

class SpadeTestAgent(Agent):
    def __init__(self, name, host, secret, graphplan, actionplan, pathplan):

        #initialState = "[handempty(nao1), clear(a), at(nao1, room1), at(a, room2), free(plateau), at(plateau, room2)]"
        #goal = "[handempty(nao1), on(a,plateau)]"
        #robot = "test2"
        
        Agent.__init__(self, name, host, secret, graphplan, actionplan, [])
        
        #plan = self.planer.getPlan(initialState, goal, robot)
        plan = self.planer.getActionPlan("getCoffee")
        print(plan)
        pe = PlanExecutorOld(plan, "samira")
        pe.printBehaviour()
        self.addBehaviour(pe, None)