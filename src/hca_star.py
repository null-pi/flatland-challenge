from bidirectional_astar import BidirectionalAStar
from heuristic import HeuristicCalc
from flatland.envs.rail_env import RailEnv

class HCAStar:
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
    max_time        : time to run the function

    returns:
    list of nodes which corresponds to a valid path from stop to node
    """
    def get_path(self, start: tuple, start_direction: int, goal: tuple, local_env: RailEnv, existing_paths: list, heuristic_calc: HeuristicCalc, \
        heuristic_weight: int=1, max_time: int=0):
        bi_astar = BidirectionalAStar()
        node_list = bi_astar.get_path(start=start, start_direction=start_direction, goal=goal, env=local_env, \
            heuristic_func=heuristic_calc.manhattan_distance, heuristic_calc=heuristic_calc, existing_paths=existing_paths, \
            heuristic_weight=heuristic_weight)

        if len(node_list) == 0:
            node_list = bi_astar.get_path(start=start, start_direction=start_direction, goal=goal, env=local_env, \
            heuristic_func=heuristic_calc.calculate_heuristic, heuristic_calc=heuristic_calc, existing_paths=existing_paths, \
            heuristic_weight=heuristic_weight, wait=True, max_time=max_time)

        return node_list