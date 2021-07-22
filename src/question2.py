"""
This is the python script for question 1. In this script, you are required to implement a single agent path-finding algorithm
"""
# TODO: flatland virtual env check. debug mode option.

from numpy.core.fromnumeric import searchsorted
from numpy.lib.function_base import copy
from lib_piglet.utils.tools import eprint
import glob, os


#import necessary modules that this python scripts need.
try:
    from flatland.envs.rail_env import RailEnv
    from controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator
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
# Return a list of (x,y) location tuples which connect the start and goal locations.
# The path should avoid conflicts with existing paths.
#########################

from sipp import SIPP
from hca_star import HCAStar
from heuristic import HeuristicCalc

def get_path(start: tuple, start_direction: int, goal: tuple, local_env: RailEnv, agent_id: int, existing_paths: list):
    heuristic_calc = HeuristicCalc(local_env)   # heuristic calculation class
    sipp = SIPP()
    node_list = sipp.get_path(start=start, start_direction=start_direction, goal=goal, local_env=local_env, existing_paths=existing_paths, \
        heuristic_calc=heuristic_calc, heuristic_weight=1)
    # hca_star = HCAStar()
    # node_list = hca_star.get_path(start=start, start_direction=start_direction, goal=goal, local_env=local_env, existing_paths=existing_paths, \
    #     heuristic_calc=heuristic_calc, heuristic_weight=1, max_time=300)

    path = []
    for curr_node in node_list:
        path.append(curr_node.loc)

    return path


#########################
# You should not modify any codes below. You can read it know how we ran flatland environment.
########################

script_path = os.path.dirname(os.path.abspath(__file__))
test_cases = glob.glob(os.path.join(script_path,"question2_test_case/*.pkl"))
evaluator(get_path,test_cases,debug,visualizer,2)
















