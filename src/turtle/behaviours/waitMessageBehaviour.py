from turtle.agents.goals import  Goals
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from common.Vocabulary import Vocabulary

class WaitMessageBehaviour(OneShotBehaviour):

    goTo = 0
    goNear = 1
    push = 2
    planExecute = 3
    move = 4
    take = 5
    put = 6

    def process(self):
        print "wait for message behaviour"
        while True:
            message = self.myAgent.communicator.waitForMessage(None)
            print "waitForMessageBehaviour: message received by turtle ", message.getContent(), message.getOntology(), message.getPerformative()
            print "sent by ", message.getSenderName()
            print "I am ", self.myAgent.getName()
            self.myAgent.messageSender = message.getSenderName()
            
            # first we check the performative. REQUEST for actions, QUERY-IF for questions, INFORM for information
            if(message.getPerformative() == Vocabulary.REQUEST):
                #then we check the ontology
                if(message.getOntology() == Vocabulary.TURTLEMOVE):
                    content = Vocabulary.parseMessage(message.getContent())
                    if(content['object'] == Vocabulary.GOTO):
                        destination = content['params'][0]
                        print destination
                        if destination in self.myAgent.getData("knowledge")["pose"].keys():
                            print "goto message received"
                            self.myAgent.setData("goals", {"pose": self.myAgent.getData("knowledge")["pose"][destination]})
                            self.myAgent.setData("currentGoal", {"status": Goals.state["inProgress"],
                                                         "action": destination})
                            self._exitcode = WaitMessageBehaviour.goTo
                            break
                    if(content['object'] == Vocabulary.GATHER):
                        print "goNear message received"
                        self.myAgent.setData("currentGoal", {"state": Goals.state["inProgress"],
                                                         "action": "goNear"})
                        self._exitcode = WaitMessageBehaviour.goNear
                        break
                    if(content['object'] == Vocabulary.MOVE):
                        print "move message received"
                        self.myAgent.parameters = content['params']
                        self._exitcode = WaitMessageBehaviour.move
                        break
                    if(content['object'] == Vocabulary.TAKE):
                        print "take message received"
                        self.myAgent.parameters = content['params']
                        self._exitcode = WaitMessageBehaviour.take
                        break
                    if(content['object'] == Vocabulary.PUSH):
                        print "push message received"
                        self.myAgent.parameters = content['params']
                        self._exitcode = WaitMessageBehaviour.push
                        break
                    if(content['object'] == Vocabulary.PUT):
                        print "put message received"
                        self.myAgent.parameters = content['params']
                        self._exitcode = WaitMessageBehaviour.put
                        break
                if(message.getOntology() == Vocabulary.GETPLAN):
                    content = Vocabulary.parseMessage(message.getContent())
                    if(content['object'] == Vocabulary.GETCOFFEE):
                        for i in range(1,3):
                            plan = self.myAgent.planer.getActionPlan(Vocabulary.GETCOFFEE)
                            if plan:
                                break;
                        print(plan)
                        self.myAgent.plan = plan
                        self._exitcode = WaitMessageBehaviour.planExecute
                    if(content['object'] == Vocabulary.TAKE):
                        for i in range(1,3):
                            plan = self.myAgent.planer.getActionPlan(Vocabulary.TAKE)
                            if plan:
                                break;
                        print(plan)
                        self.myAgent.plan = plan
                        self._exitcode = WaitMessageBehaviour.planExecute
                    break
        print "wait for message behaviour exited"
