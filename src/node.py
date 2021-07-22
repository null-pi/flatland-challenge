import math

# class for creating nodes in the graph search
class Node:
    
    """
    function to initialise node
    
    parameters:
    loc             : location of the agent, a tuple(x, y)
    direction       : diretion of the agent, an int between 0 and 3 inclusive
    goal            : goal of the agent, a tuple(x, y)
    parent          : parent of the agent, a node None if left empty
    heuristic_func  : heuristic function for informed search
    heuristic_weight: weight of the heuristic
    """
    def __init__(self, loc, direction, goal, parent=None, heuristic_func=None, heuristic_weight=1):
        self.loc = loc
        self.direction = direction
        self.goal = goal
        self.parent = parent
        
        if parent != None:
            self.g = self.parent.g + 1
            self.time = self.parent.time + 1
        else:
            self.g = 0
            self.time = 0
            
        if heuristic_func != None:
            self.h = heuristic_func(start_loc=loc, start_direction=direction, goal=goal)
        else:
            self.h = 0
            
        self.heuristic_weight = heuristic_weight
        self.f = self.g + (self.h * self.heuristic_weight)
        self.children = []
        self.interval = [self.time, math.inf]
      
    """
    function for comparing two nodes
    
    parameters:
    element : node to be compared with itself
    
    returns:
    boolean
    """  
    def __eq__(self, element):
        return element != None and self.loc == element.loc and self.direction == element.direction and self.time == element.time