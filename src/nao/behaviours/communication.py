from collections import defaultdict
from spadeutils.behaviours.spadeBehaviours import Behaviour
from spade.ACLMessage import ACLMessage
from spade.AID import aid
#import random
import spade
from common.Vocabulary import Vocabulary
#import json


class Communicator(Behaviour):
    def __init__(self):
        template = spade.Behaviour.MessageTemplate(spade.Behaviour.ACLTemplate())
        super(Communicator, self).__init__("Communicator", template)
        self.aids = dict()
        self.ontologyActions = defaultdict(lambda: None)

    def getSenderName(self, sender):
        for item in self.aids.iteritems():
            if item[1].getName() == sender.getName():
                return item[0]
        return None

    def getOtherTurtleName(self, turtleName):
        for turtle in self.aids.iterkeys():
            if turtle != turtleName:
                return turtle
        return None

    def onHumanMoveMessage(self, aclMessage):
        self.myAgent.log("Sender: " + aclMessage.getSender().getName() + " Content: " + aclMessage.getContent(), "Communicator")
        self.myAgent.raiseEvent("EVENT_MOVE", aclMessage)

    def onTurtleMove(self, aclMessage):
        self.myAgent.log("Sender: " + aclMessage.getSender().getName() + " Content: " + aclMessage.getContent(), "Communicator")
        self.myAgent.raiseEvent("EVENT_TURTLE_MOVE", aclMessage)

    def onTurlePush(self, aclMessage):
        self.myAgent.log("Sender: " + aclMessage.getSender().getName() + " Content: " + aclMessage.getContent(), "Communicator")
        self.myAgent.raiseEvent("EVENT_TURTLE_PUSH", aclMessage)

    def onStart(self):
        print "Communicator behaviour started"
        self.aids[Vocabulary.TURTLE1] = Vocabulary.getAid(Vocabulary.TURTLE1, self.myAgent.host)
        self.aids[Vocabulary.TURTLE2] = Vocabulary.getAid(Vocabulary.TURTLE2, self.myAgent.host)
        #self.aids["samira"] = aid("samira@" + self.myAgent.host, ["xmpp://" + "samira@" + self.myAgent.host])
        #self.aids["raphael"] = aid("raphael@" + self.myAgent.host, ["xmpp://" + "raphael@" + self.myAgent.host])

        self.ontologyActions["turtleMove"] = self.onTurtleMove
        self.ontologyActions["turtlePush"] = self.onTurlePush
        self.ontologyActions["cameraOntology"] = self.onHumanMoveMessage

    def onEnd(self):
        print "Communicator behaviour ended"

    def process(self):

        msg = self._receive(True)
        if msg is not None:
            try:
                ontologyAction = self.ontologyActions[msg.getOntology()]
                if ontologyAction is not None:
                    ontologyAction(msg)
                else:
                    self.myAgent.log("No action for ontology: " + msg.getOntology(), "Communicator")
            except ValueError:
                pass

    # tels the turtle to go to a given position: destinationName
    def sendMoveOrder(self, turtleName, destinationName):
        msg = ACLMessage()
        msg.setOntology(Vocabulary.TURTLEMOVE)
        msg.setPerformative(Vocabulary.REQUEST)
        content = Vocabulary.GOTO + "(" + destinationName + ")"
        msg.setContent(content)
        msg.addReceiver(self.aids[turtleName])
        self.myAgent.log(msg)
        self.myAgent.send(msg)
    # tels the turtle to push on a given distance
    # prerequisit. The turtle is at the right position
    def sendPushOrder(self, turtleName, distance):
        msg = ACLMessage()
        msg.setOntology(Vocabulary.PUSH)
        msg.setPerformative(Vocabulary.REQUEST)
        content = Vocabulary.PUSH + "(" + distance + ")"
        msg.setContent(content)
        msg.addReceiver(self.aids[turtleName])
        self.myAgent.send(msg)

    def sendGatherOrder(self):
        msg = ACLMessage()
        msg.setOntology(Vocabulary.TURTLEMOVE)
        msg.setPerformative(Vocabulary.REQUEST)
        msg.setContent(Vocabulary.GATHER + "()")
        turtles = list()
        turtles.extend(self.aids.values())
        turtle = self.aids["samira"]
        self.myAgent.log(turtle.getName(),"GATHER")
        msg.addReceiver(turtle)
        self.myAgent.send(msg)

