'''
Created on Feb 26, 2018

@author: gaignier
'''

import json
#import almath.Pose2D
import almath
from almathswig import Pose2D

# General Graph Structure and coordinates 
# Will be used to have a route network for Nao
# This is a dictionary whose keys are the nodes of the graph. 
# For each key, the corresponding value is a list containing the nodes 
# that are connected by a direct arc from this node. 
# The graph is built with letters as nodes.
# Each letter corresponds to a *pose2D cordinate stored in coord
class PathPlan:
    def __init__(self):
        self.graph = {}
        self.coord = {}
        
    def addArc(self, initial, destination):
        if initial in self.graph:
            self.graph[initial].append(destination)
        else:
            self.graph[initial] = [destination]
            
    def addCoord(self, node, coord):
        self.coord[node] = coord

    def setNeighbours(self, node, neighbours):
        self.graph[node] = neighbours        
        
    def getNeighbours(self, node):
        return self.graph[node]
        
    def find_shortest_path(self, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not self.graph.has_key(start):
            return None
        shortest = None
        for node in self.graph[start]:
            if node not in path:
                newpath = self.find_shortest_path(node, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest
    
    def find_all_paths(self, start, end, path=[]):
        path = path + [start]
        if start == end:
            return [path]
        if not self.graph.has_key(start):
            return []
        paths = []
        for node in self.graph[start]:
            if node not in path:
                newpaths = self.find_all_paths(self, node, end, path)
                for newpath in newpaths:
                    paths.append(newpath)
        return paths
    
    # since in practice the robot will never be on a node of the graph to start
    # and the destination will never really be a node of the graph
    # we calculate the nearest position
    # then according to the shortest path, only two moves need to be added:
    # go to start position and go to final destination
    
    def getNearestNetworkPoint(self, position):
        mini = +1000
        nearNode = None
        for node in self.graph:
            pos = position.distance(self.coord[node])
            if pos < mini:
                mini = pos
                nearNode = node
        #res = self.coord[nearNode]
        return nearNode
        
    
    def loadFromFile(self, fileName):
        print("will load office map from: " + fileName)
        g_setup = json.loads(open(fileName).read())   
                            
        graphJson = g_setup["Graph"]
        coordJson = g_setup["Points"]
        
        #print(graphJson)
        # juste ajouter les points aux structures
        for key in graphJson:
            val = graphJson[key]
            self.setNeighbours(key, val)

        for key in coordJson:
            val = coordJson[key]
            coord = Pose2D(val["x"], val["y"], 0.0)
            self.addCoord(key, coord)
        
    # takes Pose2D elements as real start and end
    # this method returns a list of points the robot has to
    # follow
    def getPointsFromTo(self, start, end):  
        print "start: ", start
        print "end: ", end  
        print self.getNearestNetworkPoint(start)
        print self.getNearestNetworkPoint(end)
        
        nodePath = self.find_shortest_path(self.getNearestNetworkPoint(start), self.getNearestNetworkPoint(end))
        print(nodePath)
        result = []
        #result.append(start)
        for node in nodePath:
            result.append(self.coord[node])
        result.append(end)
        return result
    
if __name__ == "__main__":
    
    test = PathPlan()
    test.loadFromFile("/home/gaignier/workspace/TER/src/utils/officeGraphMap.ini")
    print(test.graph)
    print(test.coord)
    start = Pose2D(0.2, 6.5, 0.0)
    end = Pose2D(6.1, 9.0,0.0)
    closeToStart = test.getNearestNetworkPoint(start)
    closeToEnd = test.getNearestNetworkPoint(end)
    print(closeToStart)
    print(closeToEnd)
    print(test.find_shortest_path(closeToStart, closeToEnd))
    print(test.getPointsFromTo(start, end))
    
         
        
    