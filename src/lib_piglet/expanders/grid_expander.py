# expander/grid_expander.py
# 
# Expand function for the 4-connected gridmap domain.
#
# Given a current search node, the expander checks the set of valid grid actions 
# and generates search node successors for each.
#
# @author: dharabor
# @created: 2020-07-15
#

from lib_piglet.search.search_node import search_node
from lib_piglet.expanders.base_expander import base_expander
from lib_piglet.domains.gridmap import gridmap
from lib_piglet.domains.grid_action import  Move_Actions, grid_action
from lib_piglet.constraints.grid_constraints import grid_constraint_table, grid_reservation_table

class grid_expander(base_expander):
    domain_: gridmap
    succ_: list
    constraint_table_: grid_constraint_table  = None

    def __init__(self, map : gridmap, constraint_table: grid_constraint_table = None):
        self.domain_ = map
        # memory for storing successor (state, action) pairs
        self.succ_ = [None] * 4 
        self.constraint_table_ = constraint_table


    # identify successors of the current node
    #
    # @param current: The current node
    # @return : a list tuple (new_state, gridaction).
    def expand(self, current: search_node):
        
        self.succ_.clear()
        ################
        # Implement your codes here
        ################

        return self.succ_[:]

    # return a list with all the applicable/valid actions
    # at tile (x, y)
    # @param loc A (x,y) coordinate tuple
    # @return a list of gridaction object.
    def get_actions(self, loc: tuple):
        retval = []

        #######
        # Implement your codes here
        ######

        return retval

    # Given a current location tuple, and move action
    # @param current_state A tuple of (x,y)
    # @param move A int indicate move direction
    # @return (x,y) location tuple.
    def __move(self, curr_state: tuple, move):

        #######
        # Implement your codes here
        #######


        return 0, 0

    def __str__(self):
        return str(self.domain_)


