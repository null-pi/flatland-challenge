from heuristic import HeuristicCalc
from sipp import SIPP
from hca_star import HCAStar
from copy import deepcopy
from math import inf

"""
function for running different algorithms with prioritised agent list

parameters:
local_env       : current rail environment
heuristic_weight: weight for heuristic value
max_time        : maximum time to let the function run
planner_type    : type of search algorithm to run

returns:
list of paths for all agents
"""
def prioritised_planner(local_env, heuristic_weight=1, max_time=inf, planner_type = 'sipp'):
    # initialising list of paths and heuristic calcs for all agents
    path_all = []
    heuristic_calc = []
    for agent in range(len(local_env.agents)):
        path_all.append([])
        heuristic_calc.append(HeuristicCalc(local_env))

    # creating the list of agents
    agent_list = list(range(len(local_env.agents)))

    # iterate over all agents in the agent list
    idx = 0
    while idx < len(agent_list):
        # getting agent and its starting postion, direction and goal position
        agent = agent_list[idx]
        start_loc = local_env.agents[agent].initial_position
        start_direction = local_env.agents[agent].initial_direction
        goal = local_env.agents[agent].target

        # setting planner based on planner type
        if planner_type == 'sipp':
            planner = SIPP()
        elif planner_type == 'hca':
            planner = HCAStar()
            
        # finding the node list from the planner
        node_list = planner.get_path(start=start_loc, start_direction=start_direction, goal=goal, local_env=local_env, \
            existing_paths=path_all, heuristic_calc=heuristic_calc[agent], heuristic_weight=heuristic_weight, max_time=max_time)

        # create the path from node list
        path_all[agent] = []
        for curr_node in node_list:
            path_all[agent].append(curr_node.loc)

        idx += 1

        # if the path is empty arrange the agent list such that it puts the conflicting agent at the first
        # and start from the beginning by reinitialising list of all paths
        if path_all[agent] == []:
            # creating new agent list by putting conflicting agent at the start of the list
            new_agent_list = [agent]
            for curr_agent in agent_list:
                if curr_agent != agent:
                    new_agent_list.append(curr_agent)

            agent_list = deepcopy(new_agent_list)

            # reinitialising list of all paths
            path_all = []
            for _ in agent_list:
                path_all.append([])

            idx = 0
            max_time += 1

    return path_all