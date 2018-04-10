'''
Created on Feb 24, 2018

@author: gaignier
'''
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from Queue import Queue
from turtle.services.navigationService import GoToPose



class PlanExecutor(OneShotBehaviour):
    
    def __init__(self):
        OneShotBehaviour.__init__(self,"PlanExecutor")
        self.waitingActions = Queue()

    def onStart(self):
        print "PlanExecutor new started"

    def onEnd(self):
        print "PlanExecutor new ended"

    def addAction(self, task):
        self.waitingActions.put(task)

    def addPlan(self, plan):
        for key in plan:
            actions = plan[key]
            for act in actions:
                self.addAction(self.parseAction(act))
                
    def parseAction(self, act):
        spl1 = act.split("(")
        spl2 = spl1[1].split(")")
        spl3 = spl2[0].split(",")
        name = spl1[0]
        params = list()
        for p in spl3:
            params.append(p)
        return action(name, params, self.myAgent)    
        

    def process(self):
        self.addPlan(self.myAgent.plan)
        while not self.waitingActions.empty():
            action = self.waitingActions.get(True)
            try:
                action.run()
            except AttributeError:
                print("no such robot action: " + action.name)
                print("entire plan cancelled")
                self.waitingActions.queue.clear()
      
class action(object):
    def __init__(self, name, parameters, turtleAgent):
        self.name = name
        self.parameters = parameters
        self.myAgent = turtleAgent
        self.arm = turtleAgent.arm
        
    # dynamic call of function
    # if the function does not exist, the exception is caught in the behaviour
    # and the action queue emptied
    def run(self):
        funct = getattr(self, self.name)
        funct(*self.parameters)
            
    # we need to make sure the names and profiles of Nao actions
    # are identical to the one defined with STRIPS
    # else an error is caught and the plan totally cancelled   
    # here we make the bridge between external STRIPS calls and internal calls      
    def goTo(self, robot, placeFrom, placeTo):
        print(robot + " going from: " + placeFrom + " To: " + placeTo)
        knowledge = self.myAgent.getData("knowledge")
        to = knowledge["pose"][placeTo]
        print "going to ", to["position"], to["quaterion"]
        
        goService = GoToPose()
        goService.goTo(to["position"], to["quaterion"])
        
    def take(self, robot, obj, place):
        print(robot + " taking: " + obj + " At: " + place)
        self.arm.take()
        
    def put(self, robot, obj, place):
        print(robot + " putting: " + obj + " On: " + place)
        self.arm.put()
        
