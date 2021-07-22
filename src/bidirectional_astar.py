from node import Node
from queue_assignment import Queue
from essentials import transition_nodes, find_path, manhattan_distance
from heuristic import HeuristicCalc
from flatland.envs.rail_env import RailEnv
from math import inf
from time import time

class BidirectionalAStar:
    """
    function for checking conflict

    parameters:
    node_list       : list of nodes which corresponds to a valid path from start to goal
    heuristic_calc  : calculating heuristic value
    existing_paths  : list of paths for all previously completed agents

    returns:
    list of nodes which corresponds to a valid path from start to goal or []
    """
    def conflict_check(self, node_list, heuristic_calc, existing_paths):
        # iterate over all nodes and adding h_value if it does not exist
        i = len(node_list) - 1
        h_value = 0
        while i >= 0:
            key = heuristic_calc.create_key(loc=node_list[i].loc, direction=node_list[i].direction)
            if not key in heuristic_calc.heuristic_value:
                heuristic_calc.heuristic_value[key] = h_value

            i -= 1
            h_value += 1

        # checking for conflict by iterating over all paths traversed by previously completed agents
        conflict = False
        i = 0
        while i < len(node_list):
            p = 0
            while p < len(existing_paths):
                if 0 <= i - 1 < len(existing_paths[p]) and existing_paths[p][i - 1] == node_list[i].loc:
                    conflict = True
                if 0 <= i < len(existing_paths[p]) and existing_paths[p][i] == node_list[i].loc:
                    conflict = True
                if 0 <= i + 1 < len(existing_paths[p]) and existing_paths[p][i + 1] == node_list[i].loc:
                    conflict = True
                    
                if conflict:
                    p = len(existing_paths)
                    i = len(node_list)
                    
                p += 1
                
            i += 1
            
        if conflict:
            return []
        else:
            return node_list

    
    def get_path(self, start: tuple, start_direction: int, goal: tuple, env: RailEnv, heuristic_func, heuristic_calc: HeuristicCalc, \
        existing_paths: list = [], heuristic_weight: int = 1, wait: bool = False, max_time: int = inf):
        closed_list_start = []
        queue_start = Queue()
        queue_start.push(Node(loc=start, direction=start_direction, goal=goal, heuristic_func=heuristic_func, heuristic_weight=heuristic_weight))

        goal_direction = None
        for goal_check_dir in range(4):
            valid_transitions = env.rail.get_transitions(goal[0], goal[1], goal_check_dir)
            
            if any(valid_transitions):
                goal_direction = goal_check_dir

        closed_list_goal = []
        queue_goal = Queue()
        queue_goal.push(Node(loc=goal, direction=goal_direction, goal=start, heuristic_func=heuristic_func, heuristic_weight=heuristic_weight))

        search_node_start = queue_start.pop()
        search_node_goal = queue_goal.pop()
        beginning = time()
        while time() - beginning < max_time and search_node_goal != None and search_node_start != None:
            # find path by combining node generated from goal and any node in traversed node from start
            node_list = find_path(search_node=search_node_goal, closed_list=closed_list_start, goal=goal, env=env, type='start')
            if node_list != None:
                # check for conflicts in the new path and add actual heuristic value if it does not exist
                path = self.conflict_check(node_list=node_list, heuristic_calc=heuristic_calc, existing_paths=existing_paths)
                if len(path) > 0:
                    return path
            
            # find path by combining node generated from start and any node in traversed node from goal
            node_list = find_path(search_node=search_node_start, closed_list=closed_list_goal, goal=goal, env=env, type='goal')
            if node_list != None:
                # check for conflicts in the new path and add actual heuristic value if it does not exist
                path = self.conflict_check(node_list=node_list, heuristic_calc=heuristic_calc, existing_paths=existing_paths)
                if len(path) > 0:
                    return path
            
            # add traversed node to the closed list for both start and goal
            closed_list_start.append(search_node_start)
            closed_list_goal.append(search_node_goal)

            # search whether actual heuristic value exists for that state
            # calculate heuristic value based on that
            key = heuristic_calc.create_key(loc=search_node_start.loc, direction=search_node_start.direction)
            if not key in heuristic_calc.heuristic_value:
                # find the traversable node from the current node
                search_node_start = transition_nodes(node=search_node_start, env=env, heuristic_func=heuristic_func, \
                    heuristic_weight=heuristic_weight, wait=wait)

            else:
                # find the traversable node from the current node
                search_node_start = transition_nodes(node=search_node_start, env=env, heuristic_func=heuristic_calc.calculate_heuristic, \
                    heuristic_weight=heuristic_weight, wait=wait)


            # iterate over all child in node coming from start position       
            for child in search_node_start.children:
                # check for conflict if the node has not been traversed
                if not child in closed_list_start:
                    conflict = False
                    p = 0
                    while p < len(existing_paths):
                        if 0 <= child.time - 1 < len(existing_paths[p]) and existing_paths[p][child.time - 1] == child.loc:
                            conflict = True

                        if 0 <= child.time < len(existing_paths[p]) and existing_paths[p][child.time] == child.loc:
                            conflict = True

                        if 0 <= child.time + 1 < len(existing_paths[p]) and existing_paths[p][child.time + 1] == child.loc:
                            conflict = True
                            
                        if conflict:
                            p = len(existing_paths)
                            
                        p += 1
                    
                    if not conflict:    
                        queue_start.push(child)

            # find the traversable node from the current node
            search_node_goal = transition_nodes(node=search_node_goal, env=env, heuristic_func=heuristic_func, \
                heuristic_weight=heuristic_weight, wait=False)
            
            # iterate over all child in node coming from start position
            for child in search_node_goal.children:
                if not child in closed_list_goal:
                    queue_goal.push(child)

            search_node_start = queue_start.pop()
            search_node_goal = queue_goal.pop()

        return []