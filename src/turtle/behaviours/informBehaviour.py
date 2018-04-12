from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
from common.Vocabulary import Vocabulary

import spade

class InformBehaviour(OneShotBehaviour):

    def process(self):
        
        sender = self.myAgent.messageSender
        print "inform ", sender, " that the action has been completed"
        
        message = spade.ACLMessage.ACLMessage()
        message.setPerformative(Vocabulary.INFORM)
        message.setContent(Vocabulary.DONE)
        message.addReceiver(sender)
        self.myAgent.communicator.sendMessage(message)
        self._exitcode = 0