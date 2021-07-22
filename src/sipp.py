# important necessary classes and functions
import math
from node import Node
from queue_assignment import Queue
from essentials import transition_nodes
from heuristic import HeuristicCalc
from copy import deepcopy
from flatland.envs.rail_env import RailEnv

# class for solving single agent using sipp
class SIPP:
    """
    function for creating nodes with safe intervals from a node

    parameters:
    node            : node for which nodes with safe intervals needs to be created
    time            : earliest starting time of the node
    max_len         : maximum length for checking safe intervals for the given node
    existing_paths  : list of paths for all previously completed agents
    heuristic_func  : heuristic function for calculating heuristic
    heuristic_weight: weight for the heuristic

    returns:
    list of nodes with respective safe intervals
    """
    def configure_state(self, node, time, max_len, existing_paths, heuristic_func, heuristic_weight):
        safe_interval_nodes = []    # nodes with their respective safe intervals
        node.parent = None          # removing any relation with other nodes

        # iterating from time to max_len for finding safe intervals of the node
        for i in range(time, max_len):

            # checking for conflict with other agents in that specific time
            conflict = False
            for p in existing_paths:
                if 0 <= i - 1 < len(p) and p[i - 1] == node.loc:
                    conflict = True

                if 0 <= i < len(p) and p[i] == node.loc:
                    conflict = True

                if 0 <= i + 1 < len(p) and p[i + 1] == node.loc:
                    conflict = True

            if conflict:
                # if a safe interval for the node has been found
                # add the new node with respective safe interval to the list 
                if node.interval[1] != math.inf:
                    safe_interval_nodes.append(node)
                    
                # create a new node each time a conflict occurs
                node = Node(loc=node.loc, direction=node.direction, goal=node.goal, heuristic_func=heuristic_func, heuristic_weight=heuristic_weight)

            else:
                # if safe interval for the new node has not been calculated
                # change node properties to align with safe interval for the node
                if node.interval[1] == math.inf:
                    node.interval[0] = i
                    node.time = i
                    node.g = i
                    node.f = node.g + (node.h * node.heuristic_weight)

                node.interval[1] = i

        safe_interval_nodes.append(node)

        return safe_interval_nodes

    """
    function to create a node list which corresponds to a valid path

    parameters:
    node    : last node from which path needs to be created
    stop    : node upto which path needs to be created

    returns:
    list of nodes which corresponds to a valid path from stop to node
    """
    def create_path(self, node, stop):
        path = [node]   # initialising list

        # iterate until parent of current node equals to stop node
        while node.parent != stop:
            # finding the wait time for the parent of the node
            i = node.interval[0] - node.parent.interval[0]

            # node's parent waiting for specified waiting time
            while i > 0:
                path.append(node.parent)
                i -= 1
            node = node.parent

        path.reverse()

        return path

    """
    function for finding a valid path for an agent from start to goal

    parameters:
    start           : start position of the agent
    start_direction : start direction of the agent
    goal            : goal position of the agent
    local_env       : rail environment
    existing_paths  : list of paths for all previously completed agents
    heuristic_calc  : calculating heuristic value
    heuristic_weight: weight for the heuristic
    max_time        : maximum time to run the function

    returns:
    list of nodes which corresponds to a valid path from stop to node
    """
    def get_path(self, start: tuple, start_direction: int, goal: tuple, local_env: RailEnv, existing_paths: list, heuristic_calc: HeuristicCalc, \
        heuristic_weight: int=1, max_time: int=0):
        queue = Queue()                             # queue for storing nodes
        dict_safe_interval_nodes = {}               # dictionary for storing nodes with safe intervals for a specific nodes

        # finding the maximum length of among the existing paths
        max_len = 0
        for p in existing_paths:
            if len(p) > max_len:
                max_len = len(p)

        # creating nodes with safe intervals for the start node
        search_node = Node(loc=start, direction=start_direction, goal=goal, heuristic_func=heuristic_calc.calculate_heuristic, \
            heuristic_weight=heuristic_weight)
        node = search_node
        safe_interval_nodes = self.configure_state(node, search_node.time, max_len * 2, existing_paths, heuristic_calc.calculate_heuristic, \
            heuristic_weight=heuristic_weight)

        # stroing the nodes with safe intervals inside the dictionary for the start node
        key = heuristic_calc.create_key(loc=node.loc, direction=node.direction)
        dict_safe_interval_nodes[key] = safe_interval_nodes

        # only keep the start node with safe interval starting at 0
        for node in safe_interval_nodes:
            if node.loc == start and node.interval[0] == 0:
                queue.push(node)

        search_node = queue.pop()

        # iterate until queue becomes exhausted or goal is reached
        while search_node != None and search_node.loc != search_node.goal:

            # setting children of search node to all the reachable nodes from the search node
            search_node = transition_nodes(node=search_node, env=local_env, heuristic_func=heuristic_calc.calculate_heuristic, \
                heuristic_weight=heuristic_weight, wait=False)

            # iterate over all child in the children list
            for child in search_node.children:
                node = child  # deep copying, otherwise any change will also change the original copy

                # searching for nodes with safe interval for the current node
                # if found the exisiting nodes with safe interval for the current node are used
                # otherwise nodes with safe intervals are created for the current node
                key = heuristic_calc.create_key(loc=node.loc, direction=node.direction)
                if key in dict_safe_interval_nodes:
                    safe_interval_nodes = dict_safe_interval_nodes[key]
                else:
                    safe_interval_nodes = self.configure_state(node, child.time, max_len * 2, existing_paths, heuristic_calc.calculate_heuristic, \
                        heuristic_weight=heuristic_weight)
                    dict_safe_interval_nodes[key] = safe_interval_nodes

                # iterate over all nodes with safe intervals
                for node in safe_interval_nodes:
                    # if the child nodes safe interval does not align with the movement from the current node
                    # i.e. either the safe interval of the child node start after two time steps from the end of current node's safe interval
                    # or the safe interval of the child node ends before two time steps from the start of current node's safe interval
                    if search_node.interval[1] + 1 < node.interval[0] or search_node.interval[0] + 1 > node.interval[1]:
                        continue

                    # make necessary changes to the child node for being consistent with the current node
                    new_node = deepcopy(node)
                    new_node.parent = search_node
                    if new_node.interval[0] < search_node.interval[0] + 1:
                        new_node.interval[0] = search_node.interval[0] + 1
                        new_node.time = search_node.interval[0] + 1
                        new_node.g = search_node.interval[0] + 1
                        new_node.f = new_node.g + (new_node.h * new_node.heuristic_weight)

                    queue.push(new_node)

            search_node = queue.pop()

        # return empty list if queue is exhausted
        if search_node == None:
            return []

        # return list of nodes which correspond a valid path
        return self.create_path(search_node, None)