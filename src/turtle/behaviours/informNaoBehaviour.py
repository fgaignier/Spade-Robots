from spadeutils.behaviours.spadeBehaviours import OneShotBehaviour
import spade
import json
from common.Vocabulary import Vocabulary

class InformNaoBehaviour(OneShotBehaviour):

    def process(self):
        print "inform Nao Behaviour"
        message = spade.ACLMessage.ACLMessage()
        message.setPerformative(Vocabulary.INFORM)
        message.setContent(json.dumps(self.myAgent.getData("currentGoal")))
        print "will inform Nao of ", message.getContent()
        message.addReceiver(self.myAgent.getData("naoAid"))
        message.setOntology(Vocabulary.TURTLEMOVE)
        self.myAgent.communicator.sendMessage(message)
        self._exitcode = 0