'''
Created on Feb 24, 2018

@author: gaignier
'''
#from spadeutils.behaviours.spadeBehaviours import FSMBehaviour
from spade.Behaviour import FSMBehaviour

from spadeutils.behaviours.sendOrderBehaviour import sendOrderBehaviour
from spadeutils.behaviours.endPlanBehaviour import endPlanBehaviour
from common.Vocabulary import Vocabulary


class PlanExecutor(FSMBehaviour):
    
    TRANSITION_SUCCESS = 0
    TRANSITION_ERROR = 1
    
    def __init__(self, plan, robot):
        super(PlanExecutor, self).__init__()
        self.plan = plan
        self.fsmStates = {}
        # we need to know if each action of the plan is assigned to the robot or not
        self.robot = robot
        self.create()
        
    def onStart(self):
        print "PlanExecutor started"

    def onEnd(self):
        print "PlanExecutor ended"

    def create(self):
        print "will generate an ad hoc FSMBehaviour to execute the plan"
        i = 0
        # iterate over the steps
        for key in self.plan:
            actions = self.plan[key]
            # iterate over the actions in a step (could be executed in a different order)
            for act in actions:
                self.addAction(act, i)
                i = i+1
        self.fsmStates["end"] = i
        print "register last state: ", i
        self.registerLastState(endPlanBehaviour("end"), i)
        self.createTransitions()
        
    def addAction(self, act, i):
        act_parsed = Vocabulary.parseMessage(act)
        executors = []
        params = []
        name = act_parsed["object"]
        for val in act_parsed["params"]:
            if val in Vocabulary.ROBOT_LIST:
                executors.append(val)
            else:
                params.append(val)
        
        # test who shall execute the action
        # if many, each is notified
        forme = False
        try:
            executors.index(self.robot)
            forme = True
        except:
            forme = False
            
        if(forme):
            self.fsmStates[name] = i
            behaviourName = name + "Behaviour"
            behaviour = globals()[behaviourName]
            instance = behaviour(*params)
            if(i == 0):
                self.registerFirstState(instance, self.fsmStates[name])
            else:
                self.registerState(instance, self.fsmStates[name])
        else:
            stateName = "distribute" + name + str(i)
            self.fsmStates[stateName] = i
            print "state name to be added: " + stateName + " with number " + str(i)
            behaviourName = "sendOrderBehaviour"
            #behaviour = globals()[behaviourName]
            #instance = behaviour(name, executors, params)
            #instance = eval(behaviourName)(name, executors, params)
            print "send order behaviour created with executors: ", executors
            instance = sendOrderBehaviour(name, executors, params)
            
            if(i == 0):
                print "register first state: ", self.fsmStates[stateName]
                self.registerFirstState(instance, self.fsmStates[stateName])
            else:
                print "register state: ", self.fsmStates[stateName]
                self.registerState(instance, self.fsmStates[stateName])

    def createBehaviourCall(self, name, params):
        result = name + "Behaviour(" + name
        for val in params:
            result = result + ","
            result = result + val
        result = result + ")"
        return result
    
    def createSendBehaviourCall(self, name, params):
        result = "sendOrderBehaviour(" + name
        for val in params:
            result = result + ","
            result = result + val
        result = result + ")"
        return result
    
    def createTransitions(self):
        for i in range(0,len(self.fsmStates)-1):
            print "transition from " + str(i) + " to " + str(i+1) + " at condition: " + str(PlanExecutor.TRANSITION_SUCCESS) 
            self.registerTransition(i, i+1, PlanExecutor.TRANSITION_SUCCESS)     
            
    def printBehaviour(self):
        print("states: ", self.fsmStates)     
    