# heuristics/gridmap_h.py
#
# Heuristics for gridmap.
#
# @author: mike
# @created: 2020-07-22
#

import math

def pigelet_heuristic(current_state, goal_state):
    return manhattan_heuristic(current_state, goal_state)

def manhattan_heuristic(current_state, goal_state):
    return NotImplementedError

def straight_heuristic(current_state, goal_state):
    return NotImplementedError

def octile_heuristic(current_state, goal_state):
    return NotImplementedError