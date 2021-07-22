from os import path
from lib_piglet.utils.tools import eprint
import glob, os, sys

#import necessary modules that this python scripts need.
try:
    from flatland.envs.rail_env import RailEnv
    from controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator, remote_evaluator
except:
    eprint("Cannot load flatland modules! Make sure you activated flatland virtual environment with 'conda activate flatland-rl'")
    exit(1)

from node import Node

"""
function for getting valid nodes that can be traversed from the current node

parameters:
node            : current node
env             : current rail environment
heuristic_func  : heuristic function for calculating heuristic value
heuristic_weight: weight for the heuristic value
wait            : whether a waiting node needs to be created

returns:
current node with populated children list
"""
def transition_nodes(node, env, heuristic_func, heuristic_weight=1, wait=True):
    # get available transitions from Rail_Env object.
    valid_transitions = env.rail.get_transitions(node.loc[0], node.loc[1], node.direction)
    
    # find valid position and direction for the agent for the current rail environment
    for i in range(0,len(valid_transitions)):
        if valid_transitions[i]:
            new_x = node.loc[0]
            new_y = node.loc[1]
            action = i
            if action == Directions.NORTH:
                new_x -= 1
            elif action == Directions.EAST:
                new_y += 1
            elif action == Directions.SOUTH:
                new_x += 1
            elif action == Directions.WEST:
                new_y -= 1
            
            # add new node to the current node's children
            new_node = Node(loc=(new_x, new_y), direction=action, goal=node.goal, parent=node, heuristic_func=heuristic_func, heuristic_weight=heuristic_weight)
            if not new_node in node.children:
                node.children.append(new_node)
    
    # if an wait action needs to be done, add the waiting node to the current node's children
    if wait:            
        new_node = Node(loc=node.loc, direction=node.direction, goal=node.goal, parent=node, heuristic_func=heuristic_func, heuristic_weight=heuristic_weight)      
        if not new_node in node.children:
            node.children.append(new_node)
                
    return node

"""
function for creating a node list corresponding to a valid path from stop to node

parameters:
node    : last node from which path needs to be created
stop    : node upto which path needs to be created

returns:
list of nodes which corresponds to a valid path from stop to node
"""
def create_path(node, stop):
    path = []   # initialising path

    # iterate until stop node is reached
    while node != stop:
        path.append(node)
        node = node.parent
        
    path.reverse()
    
    return path

"""
function to calculate the manhattan distance

parameters:
start   : starting position, a tuple(x, y)
goal    : goal position, a tuple(x, y)

returns:
manhattan distance between two points
"""
def manhattan_distance(start_loc, start_direction, goal):
    return abs(start_loc[0] - goal[0]) + abs(start_loc[1] - goal[1])

"""
function for finding path for bidrectional astar

parameters:
search_node : current node either from the start or goal
closed_list : list of all traversed nodes either for goal or start
goal        : goal position of the current node
env         : current rail environment
type        : what is the type of the closed_list

returns:
list of nodes which corresponds to a valid path from start to goal or None
"""
def find_path(search_node, closed_list, goal, env, type='start'):
    # iterate over all nodes in closed list until a node similar to search node is found
    i = len(closed_list) - 1
    while i >= 0:
        
        # check if the search node is the same as the node in closed list expect in opposite direction
        if search_node.loc == closed_list[i].loc and (search_node.direction % 2) == (closed_list[i].direction % 2) \
        and (search_node.direction % 4) != (closed_list[i].direction % 4):
            
            # assign values to variables depending on the type
            if type == 'start':
                goal_node = search_node.parent
                node = closed_list[i]
                new_path = create_path(closed_list[i], None)
            elif type == 'goal':
                goal_node = closed_list[i].parent
                node = search_node
                new_path = create_path(search_node, None)
            
            # iterate until either a path is found or it is sure that no such path exists
            route_exist = True
            while route_exist:
                route_exist = False
                
                # if node does not have any children, find the children
                if len(node.children) == 0:
                    node = transition_nodes(node=node, env=env, heuristic_func=manhattan_distance, wait=False)

                # iterate over all child in the current node's children
                for child in node.children:
                    # a common node is found
                    if goal_node.loc == child.loc:
                        new_path.append(child)
                        
                        # if goal is not reached, make variables ready for next iteration
                        if goal_node.loc != goal:
                            node = child
                            goal_node = goal_node.parent
                            route_exist = True
                        else:
                            return new_path

            i = -1
        
        i -= 1
        
    return None