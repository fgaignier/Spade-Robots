'''
Created on Mar 9, 2018

@author: gaignier
'''
import sys
import json
from turtle.agents.turtleAgent import TurtleAgent

if __name__ == "__main__":

    print "START"
    if len(sys.argv) < 3:
        print "Error: No config file"
        print "Usage: python NaoStarter.py path/to/config/file1.ini path/to/config/file1.ini"
        exit(1)


    config1 = json.loads(open(sys.argv[1]).read())
    config2 = json.loads(open(sys.argv[2]).read())
    
    print config1["agentName"]
    print config1["plateformIp"]

    samira = TurtleAgent(config1["agentName"], config1["plateformIp"], config1["plateformSecret"],
                         config1["graphplan"], config1["actionsplan"], config1["knowledge"])
    try:
        samira.start()
    except:
        print "termination of program"
        
    """
    print config2["agentName"]
    print config2["plateformIp"]
    raphael = TurtleAgent(config2["agentName"], config2["plateformIp"], config2["plateformSecret"],
                         config2["graphplan"], config2["actionsplan"], config2["knowledge"])
    raphael.start()
    """