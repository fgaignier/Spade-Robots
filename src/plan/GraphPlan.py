'''
Created on Feb 24, 2018

@author: gaignier
'''

# this class uses swipl to load any PROLOG GraphPlan implementation
# and associated actions
# the method getPlan()
# returns a fully instanciated plan : a dictionary (= hashmap)  of lists
# mapping done by plan step and maps a list of one or several actions.
# if several actions in a step, these actions can be performed in parallel 
# initialState: a list defining the initial state
# goal, a list defining partially the final state
# robot, the type of robot. Actions are different for the various types of robots
# if you want to get a global plan for redistribution, use: all and define your actions with the same

import time

class GraphPlan(object):
    
    FILE_TYPE_PLAN = "plan"
    FILE_TYPE_ACTIONS = "actions"
    
    def __init__(self, a):
        self.a = a
        self.a.configureKB("SWI", None, "swipl")
        self.planLoaded = False
        self.actionsLoaded = False
        
    # loads prolog statements from file
    # two types of files admitted: plan or actions
    # if anything else is loaded, the planer will not return a plan   
    def loadFromFile(self, filename, file_type):
        with open(filename) as f:
            print("charging file into SPADE prolog engine: " + filename)
            rule = ""
            for line in f:
                # removes \r, \n from the string
                line = line.replace('\r', '')
                line = line.replace('\n', '')
                # removes white spaces at the beginning and end
                line = line.strip(' ')
                # removes tabulation at the beginning and end
                line = line.strip('\t')
                # and again the spaces to avoid \t space cases
                line = line.strip(' ')
                # skips empty lines and comments
                #print(list(line))
                if len(line) == 0 or line[0] == "%":
                    #print("skip: " + line)
                    continue
                else:
                    rule = rule + line
                    # we reach the end of the prolog statement.
                    # will therefore add the rule
                    if rule.endswith("."):
                        rule = rule.rstrip('.')
                        rule = "(" + rule + ")"
                        #print(" will add rule: " + rule)
                        self.a.addBelieve(rule)
                        rule = ""
        if file_type == self.FILE_TYPE_PLAN:
            self.planLoaded = True
        elif file_type == self.FILE_TYPE_ACTIONS:
            self.actionsLoaded = True
        # need a little time for the entire rules to be active in memory
        time.sleep(2) 
        
        
    
    # returns a fully instanciated plan : a dictionary (= hashmap)  of lists
    # mapping done by plan step and maps a list of one or several actions.
    # if several actions in a step, these actions can be performed in parallel 
    # initialState: a list defining the initial state
    # goal, a list defining partially the final state
    # robot, the type of robot. Actions are different for the various types of robots
    # if you want to get a global plan for redistribution, use: all and define your actions with the same
    def getPlan(self, initialState, goal, robot):
        if self.planLoaded == True and self.actionsLoaded == True:
            request = "plan(" + initialState + "," + goal + "," + robot + ", P)"
            print("requested plan: " + request)
            plan = self.a.askBelieve( request )
            #print (plan)
            if plan == False:
                print("could not find a plan.")
                return False
            else:
                return self.splitPlan(plan)
        
    def getActionPlan(self, action):
        if self.planLoaded == True and self.actionsLoaded == True:
            request = action + "(P)"
            print("requested plan: " + request)
            plan = self.a.askBelieve( request )
       
            if plan == False:
                time.sleep(5)
                request = "plan_" + request
                print("requested plan: " + request)
                plan = self.a.askBelieve( request )
                if plan == False:
                    print("could not find a plan")
                    return False
                else:
                    return self.splitPlan(plan)
            else:
                return self.splitPlan(plan)
        
    # returns a dictionary of steps with a list of action for each step
    # steps are indexed from 1 to n
    def  splitPlan(self, plan):
        # get the plan (the first one in case our planer returns many, the others are ignored
        actionList = list(plan[0]["P"])
        actionList = actionList[1:len(actionList)-1]
        result = dict()
        i = 1
        begin_action = False
        index = i
        for char in actionList:
            if char == "[": #start a step containing a list of actions
                begin_action = True
                index = i
                current_action = []
                continue
            elif char == "]": # end a step containing a list of actions
                begin_action = False
                i = i+1
            elif char == ")": # end of action
                begin_action = False
                current_action.append(char) # we still write the closing bracket
                action = ''.join(current_action)
                if index in result:
                    result[index].append(action)
                else:
                    result[index] = [action]
            elif char == "," and begin_action == False:
                begin_action = True
                current_action = []
                continue # we do not keep the separator
            if begin_action == True:
                current_action.append(char)
            else:
                continue
        return result
        
        
        