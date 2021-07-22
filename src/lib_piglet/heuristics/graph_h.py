# heuristics/graph.py
#
# Heuristics for graph map.
#
# @author: mike
# @created: 2020-07-22
#

import math

def pigelet_heuristic(current_state, goal_state):
    return straight_heuristic(current_state, goal_state)

# In graph map this heuristic may not admissible if the distance derived by give coordinate is lager than given edge weight.
def straight_heuristic(current_state, goal_state):
    return NotImplementedError