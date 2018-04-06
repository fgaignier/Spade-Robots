from turtle.agents.goals import  Goals
from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from common.Vocabulary import Vocabulary

class WaitMessageBehaviour(OneShotBehaviour):

    goTo = 0
    goNear = 1
    pushTheBox = 2
    moveArm = 3

    def process(self):
        print "wait for message behaviour"
        while True:
            message = self.myAgent.communicator.waitForMessage(None)
            print "waitForMessageBehaviour: message received by turtle ", message.getContent(), message.getOntology(), message.getPerformative()
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
                # Turtle push the box   
                if(message.getOntology() == Vocabulary.TURTLEPUSH):
                    print "push the box message received"
                    self._exitcode = WaitMessageBehaviour.pushTheBox
                    break
                if(message.getOntology() == Vocabulary.TURTLEARM):
                    content = Vocabulary.parseMessage(message.getContent())
                    if(content['object'] == Vocabulary.MOVEARM):
                        # lire les bons parametres
                        destination = content['params'][0]
                        print destination
                        
        print "wait for message behaviour exited"
