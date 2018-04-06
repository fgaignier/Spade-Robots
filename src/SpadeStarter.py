'''
Created on Feb 24, 2018

@author: gaignier
'''
import sys
import json

from spadeutils.SpadeTest import SpadeTestAgent 

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print "Error: No config file"
        print "Usage: python SpadeStarter.py path/to/config/file1.ini"
        exit(1)

    config1 = json.loads(open(sys.argv[1]).read())
    
    spadeTest = SpadeTestAgent(config1["agentName"], config1["plateformIp"], config1["plateformSecret"],
                               config1["graphplan"], config1["actionsplan"], config1["pathplan"])
    spadeTest.start()
    
    