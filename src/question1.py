"""
This is the python script for question 1. In this script, you are required to implement a single agent path-finding algorithm
"""
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
#########################

from astar import AStar
from bidirectional_astar import BidirectionalAStar
from heuristic import HeuristicCalc

# This function return a list of location tuple as the solution.
# @param start A tuple of (x,y) coordinates
# @param start_direction An Int indicate direction.
# @param goal A tuple of (x,y) coordinates
# @param env The flatland railway environment
# @return path A list of (x,y) tuple.
def get_path(start: tuple, start_direction: int, goal: tuple, env: RailEnv):
    heuristic_calc = HeuristicCalc(env)
    astar = AStar()
    node_list = astar.get_path(start=start, start_direction=start_direction, goal=goal, env=env, heuristic_func=heuristic_calc.manhattan_distance) 
    # bi_astar = BidirectionalAStar()
    # node_list = bi_astar.get_path(start=start, start_direction=start_direction, goal=goal, env=env, heuristic_func=heuristic_calc.manhattan_distance, \
    #     heuristic_calc=heuristic_calc)
    path = []
    for curr_node in node_list:
        path.append(curr_node.loc)

    return path


#########################
# You should not modify any codes below. You can read it know how we ran flatland environment.
########################

script_path = os.path.dirname(os.path.abspath(__file__))
test_cases = glob.glob(os.path.join(script_path,"question1_test_case/*.pkl"))
evaluator(get_path,test_cases,debug,visualizer,1)

















