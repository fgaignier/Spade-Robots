'''
Created on Mar 9, 2018

@author: gaignier
'''
import sys
import json
from turtle.agents.turtleAgent import TurtleAgent

if __name__ == "__main__":

    print "START"
    if len(sys.argv) < 1:
        print "Usage: python NaoStarter.py path/to/config/turtle.ini"
        exit(1)


    config1 = json.loads(open(sys.argv[1]).read())
    
    print config1["agentName"]
    print config1["plateformIp"]

    turtle = TurtleAgent(config1["agentName"], config1["plateformIp"], config1["plateformSecret"],
                         config1["graphplan"], config1["actionsplan"], config1["knowledge"])
    try:
        turtle.start()
    except KeyboardInterrupt:
        print "termination of program"
        
