from node import Node
from queue_assignment import Queue
from essentials import transition_nodes, create_path
from flatland.envs.rail_env import RailEnv
from time import time
from math import inf

class AStar:
    def get_path(self, start: tuple, start_direction: int, goal: tuple, env: RailEnv, heuristic_func, existing_paths: list = [], \
        heuristic_weight: int = 1, wait: bool = False, max_time: int = inf):
        queue = Queue()     # priority queue for prioritising nodes
        closed_list = []    # list of nodes for not repeating them
        
        # adding the start node to the queue
        queue.push(Node(loc=start, direction=start_direction, goal=goal, heuristic_func=heuristic_func, heuristic_weight=heuristic_weight))
        search_node = queue.pop()   # getting the prioritised node
        
        beginning = time()
        while search_node != None and search_node.loc != goal and time() - beginning < max_time:
            closed_list.append(search_node)

            # get all valid node traversable from the current node in current node's children
            search_node = transition_nodes(node=search_node, env=env, heuristic_func=heuristic_func, heuristic_weight=heuristic_weight, wait=wait)

            # iterate over all child nodes of the current node
            for child in search_node.children:
                # checking for conflict with other agents in that specific time
                conflict = False
                for p in existing_paths:
                    if 0 <= child.time - 1 < len(p) and p[child.time - 1] == child.loc:
                        conflict = True

                    if 0 <= child.time < len(p) and p[child.time] == child.loc:
                        conflict = True

                    if 0 <= child.time + 1 < len(p) and p[child.time + 1] == child.loc:
                        conflict = True

                if not child in closed_list and not conflict:
                    queue.push(child)

            search_node = queue.pop()   # getting the prioritised node
            
        if search_node == None:
            return []
        else:
            return create_path(node=search_node, stop=None)