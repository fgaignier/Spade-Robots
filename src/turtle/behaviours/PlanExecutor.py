'''
Created on Feb 24, 2018

@author: gaignier
'''
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from Queue import Queue
from turtle.services.navigationService import GoToPose
from turtle.services.navigationService import Move
from common.Vocabulary import Vocabulary

import spade


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
        #myFullName = self.myAgent.getname()
        #spl1 = myFullName.split('@')
        #myName = spl1[0]
        myName = self.myAgent.name
        print "in plan executor: myName= ", myName
        self.addPlan(self.myAgent.plan)
        while not self.waitingActions.empty():
            action = self.waitingActions.get(True)
            robot = action.getParameter(0)
            try:
                if robot == myName:
                    print "will execute action myself"
                    action.run()
                else:
                    print "will send action to ", robot
                    action.dispatch()
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
        
    def getParameter(self, number):
        return self.parameters[number]
    
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
        
    def take(self, robot, obj, place, pos):
        print(robot + " taking: " + obj + " At: " + place + " On: " + pos)
        self.arm.take()
        
    def put(self, robot, obj, place):
        print(robot + " putting: " + obj + " On: " + place)
        self.arm.put()
    
    def push(self, robot, obj, distance):
        print(robot + " pushing: " + obj + " On: " + distance)
        #moveService = Move()
        moveService = GoToPose()
        moveService.push(distance)
        
    def move(self, robot, dist, angle):
        print(robot + " moving on distance: " + dist + " With angle: " + angle)
        #moveService = Move()
        moveService = GoToPose()
        moveService.move(dist, angle)
        
    # this is when the first parameter is different from the robot executing the plan
    # the action is sent to the robot in charge of executing the task
    def dispatch(self):
        print "sending action request to: ", self.parameters[0]
        ct = Vocabulary.buildMessage(self.name, self.parameters)
        host = self.myAgent.host
        print "hostname is : " + host
        dest = Vocabulary.getAid(self.parameters[0], host)
        print("message: ", ct, "to ", dest)
        message = spade.ACLMessage.ACLMessage()
        message.setPerformative(Vocabulary.REQUEST)
        message.setOntology(Vocabulary.TURTLEMOVE)
        message.setContent(ct)
        message.addReceiver(dest)
        self.myAgent.communicator.sendMessage(message)
        message = self.myAgent.communicator.waitForMessage(None)
        if message.getPerformative() == Vocabulary.INFORM and message.getContent() == Vocabulary.DONE:
            return True
        else:
            return False  
        
