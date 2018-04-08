'''
Created on Mar 17, 2018

@author: gaignier
'''
from spade.AID import aid

class Vocabulary(object):
    
    # performatives
    # actions
    REQUEST = "request"
    #questions
    QUESTION = "query-if"
    # information
    INFORM = "inform"
    
    # actions (message body)
    GOTO = "goTo"
    PUSH = "push"
    GATHER = "goNear"
    GETCOFFEE = "getCoffee"
    
    #ontologies
    TURTLEMOVE = "turtleMove"
    TURTLEPUSH = "turtlePush"
    GETPLAN = "getPlan"
    
    #robots available
    NAO1 = "nao1"
    NAO2 = "nao2"
    TURTLE1 = "samira"
    TURTLE2 = "raphael"
    
    ROBOT_LIST = ["nao1", "nao2", "samira", "raphael"]
    
    @staticmethod
    def parseMessage(msg):
        spl1 =msg.split("(")
        spl2 = spl1[1].split(")")
        spl3 = spl2[0].split(",")
        name = spl1[0]
        params = list()
        for p in spl3:
            params.append(p)
        result = dict()
        result['object'] = name
        result['params'] = params
        return result
    
    @staticmethod
    def getAid(name, host):
        return aid(name + host, ["xmpp://" + name + "@" + host])
            