'''
Created on Feb 21, 2018

@author: gaignier
'''

import spade
import traceback
from collections import defaultdict
from plan.GraphPlan import GraphPlan

# Spade Agent with enhanced funtionalities
class Agent(spade.Agent.Agent):

    def __init__(self, name, host, secret,  graphplan, actionplan, behaviours):
        super(Agent, self).__init__(name + "@" + host, secret)
        self.localName = name
        self.host = host
        self.name = self.localName + "@" + self.host
        self.behaviours = dict()
        self.beliefListeners = set()
        self.eventListeners = set()
        self.data = defaultdict(lambda: None)
        self.believes = defaultdict(lambda: False)
        for behaviour in behaviours:
            self.behaviours[behaviour.getName()] = behaviour
        self.initPlaner(graphplan, actionplan)
        
    # will add a GraphPlan planer to the Agent
    def initPlaner(self, graphplan, actionplan):
        # add a GraphPlan planer to the agent
        self.planer = GraphPlan(self)
        # loads the GraphPlan (swi-prolog) to the Agent
        self.planer.loadFromFile(graphplan, self.planer.FILE_TYPE_PLAN)
        # loads the possible actions of the agent
        self.planer.loadFromFile(actionplan, self.planer.FILE_TYPE_ACTIONS)
        
    def initBehaviours(self):
        for behaviour in self.behaviours.itervalues():
            if behaviour.haveTemplate():
                self.addBehaviour(behaviour, behaviour.getTemplate())
            else:
                self.addBehaviour(behaviour)


    def _setup(self):
        print("_setup from Agent called")
        try:
            self.initBehaviours()
        except:
            traceback.print_exc()

    def addBeliefListener(self, listener):
        self.beliefListeners.add(listener)

    def removeBeliefListener(self, listener):
        if listener in self.beliefListeners:
            self.beliefListeners.remove(listener)

    def addEventListener(self, listener):
        self.eventListeners.add(listener)

    def removeEventListener(self, listener):
        if listener in self.eventListeners:
            self.eventListeners.remove(listener)

    def raiseEvent(self, eventType, event=None):
        for listener in self.eventListeners:
            listener.onEvent(eventType, event)

    def setData(self, key, value):
        self.data[key] = value

    def getData(self, key):
        return self.data[key]

    def getTaskExecutor(self):
        executor = "TaskExecutor"
        if executor in self.behaviours:
            return self.behaviours[executor]
        else:
            print "Error: No TaskExecutor behaviour found in agent"