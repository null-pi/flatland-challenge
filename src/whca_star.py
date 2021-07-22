from node import Node
from queue_assignment import Queue
from essentials import transition_nodes, create_path
from heuristic import HeuristicCalc
from time import time

class WHCAStar:
    """
    function for creating key for closed list

    parameters:
    agent_list  : list of agents
    count       : time bound

    returns:
    string
    """
    def create_closed_list_key(self, agent_list, count):
        key = ''
        for agent in agent_list:
            key += str(agent)

        return key + str(count)

    """
    function for getting a list of paths for all agents

    parameters:
    local_env       : current rail environment
    time_limit      : time limit for the function to run
    existing_paths  : list of paths of previously completed agents or list of empty list for all agents
    agent_list      : list of agent
    heuristic_weight: weight of heuristic value
    max_time        : time bound for each iteration

    returns:
    list of paths for all agents
    """
    def get_path(self, local_env, time_limit, existing_paths, agent_list, heuristic_weight=1, max_time=5):
        start_node_all = {}                         # list of all agent's start node
        heuristic_calc_all = {}                     # list of all agent's actual heuristic value calculator
        done_all = {}                               # list of all agent's done status
        heuristic_weight = heuristic_weight         # weight of the heuristic value

        # an alternative exit when time runs out
        alternative_exit = []
        for p in existing_paths:
            alternative_exit.append(p)

        # populate above lists with relevant data
        for agent in agent_list:
            heuristic_calc_all[agent] = HeuristicCalc(local_env)
            done_all[agent] = False

            # get start state of each agent
            loc = local_env.agents[agent].initial_position
            direction = local_env.agents[agent].initial_direction
            goal = local_env.agents[agent].target

            start_node = Node(loc=loc, direction=direction, goal=goal, heuristic_func=heuristic_calc_all[agent].calculate_heuristic, \
                heuristic_weight=heuristic_weight)
            start_node_all[agent] = start_node

        max_time = max_time                         # maximum time for the function to run
        closed_list_all_time = {}                   # dictionary for closed_list_all list in each time bound
        start_node_all_time = {0: start_node_all}   # dictionary for start_node_all in each time bound
        done_all_time = {0: done_all}               # dictionary for done_all in each time bound
        done_all_position = {}                      # dictionary for storing goal node for each agent
        agent_list_all_time = {0: agent_list}       # dictionary for storing agent list in specific time bound
        count = 0                                   # counting time bound
        beginning = time()                          # start the counting of time

        # iterate until solution is found or time limit is up
        finished = False
        while not finished:

            # return the alternative exit list when time is up
            if time() - beginning > time_limit:
                return alternative_exit

            unreachable = False     # initialising unreachable
            start_node_all = {}     # list of all agent's start node
            done_all = {}           # list of all agent's done status

            # initializing all lists
            for agent in agent_list:
                done_all[agent] = False

            # getting the agent list in current time bound
            agent_list = []
            for agent in agent_list_all_time[count]:
                agent_list.append(agent)

            # creating existing paths for all agents before current time bound
            for agent in agent_list:
                if done_all_time[count][agent]:
                    existing_paths[agent] = create_path(done_all_position[agent], None)
                else:
                    existing_paths[agent] = create_path(start_node_all_time[count][agent], None)

            # iterate over each agent in agent list
            for agent in agent_list:
                # when that agent is not done
                if not done_all_time[count][agent]:

                    # create agent closed list for avoiding conflicted node
                    agent_closed_list = []
                    if (count + 1) in agent_list_all_time:
                        closed_list_key_0 = self.create_closed_list_key(agent_list, count)
                        closed_list_key_1 = self.create_closed_list_key(agent_list_all_time[count + 1], count + 1)
                        closed_list_key = closed_list_key_0 + closed_list_key_1

                        # getting the conflicted nodes and push it to the closed list
                        if closed_list_key in closed_list_all_time and agent in closed_list_all_time[closed_list_key]:
                            for closed_node in closed_list_all_time[closed_list_key][agent]:
                                agent_closed_list.append(closed_node)

                    # creating prioritised queue for searching
                    agent_queue = Queue()
                    agent_queue.push(start_node_all_time[count][agent])
                    search_node = agent_queue.pop()

                    # check whether current search node conflicts with other agents
                    conflict = False
                    p = 0
                    while p < len(existing_paths):
                        # check whether conflict occurs at time - 1
                        if p != agent and search_node != None and 0 <= search_node.time - 1 < len(existing_paths[p]) and \
                            existing_paths[p][search_node.time - 1].loc == search_node.loc:
                            conflict = True
                        
                        # check whether conflict occurs at time
                        if p != agent and search_node != None and 0 <= search_node.time < len(existing_paths[p]) and \
                            existing_paths[p][search_node.time].loc == search_node.loc:
                            conflict = True

                        # check whether conflict occurs at time + 1
                        elif p != agent and search_node != None and 0 <= search_node.time + 1 < len(existing_paths[p]) and \
                            existing_paths[p][search_node.time + 1].loc == search_node.loc:
                            conflict = True

                        if conflict:
                            p = len(existing_paths)

                        p += 1

                    if conflict:
                        search_node = None

                    # time bound search for current search node
                    while search_node != None and (search_node.time - (count * max_time)) < max_time:
                        # adding current search node to the closed list
                        agent_closed_list.append(search_node)

                        # check whether search node is the goal node
                        if search_node.loc == search_node.goal:
                            # path_all_time[count][agent].append(search_node)
                            done_all_position[agent] = search_node
                            done_all[agent] = True
                            break

                        # create children for the search node
                        search_node = transition_nodes(node=search_node, env=local_env, heuristic_func=heuristic_calc_all[agent].calculate_heuristic, \
                            heuristic_weight=heuristic_weight + count)

                        # check for conflictin child
                        for child in search_node.children:
                            conflict = False
                            p = 0
                            while p < len(existing_paths):
                                # check for conflict with other agents at time - 1
                                if p != agent and 0 < child.time - 1 < len(existing_paths[p]) and existing_paths[p][child.time - 1].loc == child.loc:
                                    conflict = True

                                # check for conflict with other agents at time
                                if p != agent and child.time < len(existing_paths[p]) and existing_paths[p][child.time].loc == child.loc:
                                    conflict = True

                                # check for conflict with other agents at time + 1
                                if p != agent and child.time + 1 < len(existing_paths[p]) and existing_paths[p][child.time + 1].loc == child.loc:
                                    conflict = True

                                if conflict:
                                    p = len(existing_paths)

                                p += 1

                            # check whether collision occured or child is already visited
                            if not conflict and not child in agent_closed_list:
                                agent_queue.push(child)

                        # creating path for agent
                        search_node = agent_queue.pop()

                    # if agent reached goal state create the existing path for it
                    if done_all[agent]:
                        existing_paths[agent] = create_path(done_all_position[agent], None)
                        continue

                    # if search node is None do necessary state management
                    if search_node == None:
                        unreachable = True

                        if count > 0:
                            closed_list_key_0 = self.create_closed_list_key(agent_list_all_time[count - 1], count - 1)
                            closed_list_key_1 = self.create_closed_list_key(agent_list_all_time[count], count)
                            closed_list_key = closed_list_key_0 + closed_list_key_1

                            if not closed_list_key in closed_list_all_time:
                                closed_list_all_time[closed_list_key] = {}

                            if not agent in closed_list_all_time[closed_list_key]:
                                closed_list_all_time[closed_list_key][agent] = []

                            # add the initial position because this leads to a conflicting path
                            closed_list_all_time[closed_list_key][agent].append(start_node_all_time[count][agent])

                            # shuffle agent list to put the conflicting agent at first
                            agent_list_all_time[count] = [agent]
                            for agent_num in agent_list:
                                if agent_num != agent:
                                    agent_list_all_time[count].append(agent_num)
                            count -= 1

                        break

                    else:
                        existing_paths[agent] = create_path(search_node, None)
                        start_node_all[agent] = search_node

                # if the agent is done create a new existing_paths for that agent
                else:
                    existing_paths[agent] = create_path(done_all_position[agent], None)
                    done_all[agent] = True

            if not unreachable:
                count += 1
                start_node_all_time[count] = start_node_all

                # remove the done agent from re-expanding if any conflict happens
                done_list = []
                for key in done_all:
                    done_list.append(done_all[key])

                    if done_all[key]:
                        # removing completed agent from all agent list
                        for t in agent_list_all_time:
                            if key in agent_list_all_time[t]:
                                agent_list_all_time[t].remove(key)

                        if key in agent_list:
                            agent_list.remove(key)

                        # adding the path in alternative exit
                        alternative_exit[key] = existing_paths[key]
                
                if all(done_list):
                    finished = True

                done_all_time[count] = done_all

                # create a new agent list if it is not found
                if not count in agent_list_all_time:
                    agent_list_all_time[count] = []
                    for agent in agent_list:
                        agent_list_all_time[count].append(agent)

        return existing_paths