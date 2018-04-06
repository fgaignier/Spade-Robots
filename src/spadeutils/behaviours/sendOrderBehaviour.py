'''
Created on Mar 17, 2018

@author: gaignier
'''

from spade.ACLMessage import ACLMessage
from common.Vocabulary import Vocabulary

from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour

class sendOrderBehaviour(OneShotBehaviour):
    
    def __init__(self, name, executors, params):
        OneShotBehaviour.__init__(self, name)
        #self.name = name
        self.executors = executors
        self.params = params
        self.aids = dict()
        print "send Order Behaviour inited"
     
    def process(self):
        print "process send order behaviour: " , self.executors
        self.loadAddressBook()
        msg = ACLMessage()
        msg.setOntology(Vocabulary.TURTLEMOVE)
        msg.setPerformative(Vocabulary.REQUEST)
        content = self.createContent()
        msg.setContent(content)
        for dest in self.executors:
            msg.addReceiver(self.aids[dest])
        #self.myAgent.log(msg)
        self.myAgent.send(msg)
        print "sent message: " + msg.getContent()

    def createContent(self):
        result = self.name + "("
        for val in self.params:
            result = result + val + ","
        result = result.rstrip(',')
        result = result + ")"
        return result
        
    def onEnd(self):
        print "send order behaviour ended"
        self._exitcode = 0

    def loadAddressBook(self):
        self.aids[Vocabulary.TURTLE1] = Vocabulary.getAid(Vocabulary.TURTLE1, self.myAgent.host)
        self.aids[Vocabulary.TURTLE2] = Vocabulary.getAid(Vocabulary.TURTLE2, self.myAgent.host)
        self.aids[Vocabulary.NAO1] = Vocabulary.getAid(Vocabulary.NAO1, self.myAgent.host)
        self.aids[Vocabulary.NAO2] = Vocabulary.getAid(Vocabulary.NAO2, self.myAgent.host)
