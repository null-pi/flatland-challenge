from node import Node
from queue_assignment import Queue
from essentials import transition_nodes, find_path, manhattan_distance

class HeuristicCalc:
    def __init__(self, env):
        self.heuristic_value = {}
        self.env = env

    """
    function for calculating actual heuristic value for each node

    parameters:
    start_loc       : start location for the node
    start_direction : start direction for the node
    goal            : goal for the node

    returns:
    actual heuristic value for the node
    """
    def calculate_heuristic(self, start_loc, start_direction, goal):
        # create key for searching the actual heuristic value
        search_key = self.create_key(loc=start_loc, direction=start_direction)

        # when there is no actual heuristic value for the node
        if not search_key in self.heuristic_value:
            # initialising required data structures for start node
            closed_list_start = []
            queue_start = Queue()
            queue_start.push(Node(loc=start_loc, direction=start_direction, goal=goal, heuristic_func=manhattan_distance))

            # finding goal direction
            goal_direction = None
            for goal_check_dir in range(4):
                valid_transitions = self.env.rail.get_transitions(goal[0], goal[1], goal_check_dir)
                
                if any(valid_transitions):
                    goal_direction = goal_check_dir

            # initialising required data structures for goal node
            closed_list_goal = []
            queue_goal = Queue()
            queue_goal.push(Node(loc=goal, direction=goal_direction, goal=start_loc, heuristic_func=manhattan_distance))

            search_node_start = queue_start.pop()
            search_node_goal = queue_goal.pop()

            # iterate until a path is found from start to goal
            new_path = None
            while new_path == None:
                # creating a path with goal node and closed list of start by finding a common node
                new_path = find_path(search_node=search_node_goal, closed_list=closed_list_start, goal=goal, env=self.env, type='start')
                if new_path != None:
                    break

                # creating a path with start node and closed list of goal by finding a common node
                new_path = find_path(search_node=search_node_start, closed_list=closed_list_goal, goal=goal, env=self.env, type='goal')
                if new_path != None:
                    break

                closed_list_start.append(search_node_start)
                closed_list_goal.append(search_node_goal)

                # finding new nodes from the start node
                search_node_start = transition_nodes(node=search_node_start, env=self.env, heuristic_func=manhattan_distance, wait=False)
                for child in search_node_start.children:
                    if not child in closed_list_start:
                        queue_start.push(child)

                # finding new nodes from the start node
                search_node_goal = transition_nodes(node=search_node_goal, env=self.env, heuristic_func=manhattan_distance, wait=False)
                for child in search_node_goal.children:
                    if not child in closed_list_goal:
                        queue_goal.push(child)

                search_node_start = queue_start.pop()
                search_node_goal = queue_goal.pop()

            # calculating actual heuristic value for all the nodes in the path
            h_value = 0
            idx = len(new_path) - 1
            while idx >= 0:
                key = self.create_key(loc=new_path[idx].loc, direction=new_path[idx].direction)

                if (key in self.heuristic_value and self.heuristic_value[key] > h_value) or not key in self.heuristic_value:
                    self.heuristic_value[key] = h_value

                h_value += 1
                idx -= 1

        return self.heuristic_value[search_key]

    """
    function for creating key to search for actual heuristic value

    parameters:
    loc         : location of the node
    direction   : direction of the node

    returns:
    a string uniquely identifying the node
    """
    def create_key(self, loc, direction):
        return str((loc[0] * self.env.width) + loc[1]) + str(direction)

    """
    function to calculate the manhattan distance

    parameters:
    start_loc       : start location for the node
    start_direction : start direction for the node
    goal            : goal for the node

    returns:
    manhattan distance between two points
    """
    def manhattan_distance(self, start_loc, start_direction, goal):
        return abs(start_loc[0] - goal[0]) + abs(start_loc[1] - goal[1])