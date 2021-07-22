
from os import path
from prioritised_planner import prioritised_planner
from lib_piglet.utils.tools import eprint
import glob, os, sys

#import necessary modules that this python scripts need.
try:
    from flatland.envs.rail_env import RailEnv
    from controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator, remote_evaluator
except:
    eprint("Cannot load flatland modules! Make sure you activated flatland virtual environment with 'conda activate flatland-rl'")
    exit(1)


#########################
# Debugger and visualizer options
#########################

# Set these debug option to True if you want more information printed
debug = False
visualizer = False

#########################
# Reimplementing the content in get_path() function.
#
# Return a list of paths. A path is a list of (x,y) location tuples.
# The path should be conflict free.
#########################

from prioritised_planner import prioritised_planner
from whca_star import WHCAStar

def get_path(local_env: RailEnv):
    path_all = prioritised_planner(local_env=local_env, planner_type='sipp')
    
    # whca = WHCAStar()
    # agent_list = list(range(len(local_env.agents)))
    # existing_paths = []
    # for _ in agent_list:
    #     existing_paths.append([])

    # existing_paths = whca.get_path(local_env, 5 * len(agent_list), existing_paths, agent_list)

    # path_all = []
    # for p in existing_paths:
    #     path = []
    #     for curr_node in p:
    #         path.append(curr_node.loc)
    #     path_all.append(path)

    return path_all


#####################################################################
# Instantiate a Remote Client
#####################################################################
remote_mode = False

if len(sys.argv) > 1:
    if sys.argv[1] == "--remote-mode":
        remote_mode = True

if remote_mode:
    remote_evaluator(get_path)
else:
    script_path = os.path.dirname(os.path.abspath(__file__))
    test_cases = glob.glob(os.path.join(script_path, "question2_test_case/*.pkl"))
    evaluator(get_path, test_cases, debug, visualizer, 3)