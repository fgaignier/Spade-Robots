'''
Created on Feb 21, 2018

@author: gaignier
'''
import sys
import json
from nao.agents.NaoAgent import NaoAgent

if __name__ == "__main__":

    if len(sys.argv) < 3:
        print "Error: No config file"
        print "Usage: python NaoStarter.py path/to/config/file1.ini path/to/config/file1.ini"
        exit(1)

    config1 = json.loads(open(sys.argv[1]).read())
    config2 = json.loads(open(sys.argv[2]).read())
    
    print config1["agentName"]
    print config1["naoIp"]

    #print config2["agentName"]
    #print config2["naoIp"]
    nao1Agent = NaoAgent(config1["agentName"], config1["plateformIp"], config1["plateformSecret"], config1["naoIp"],
                         config1["graphplan"], config1["actionsplan"], config1["pathplan"])
    nao1Agent.start()
    
    #nao2Agent = NaoAgent(config2["agentName"], config2["plateformIp"], config2["plateformSecret"], config2["naoIp"])
    #nao2Agent.start()