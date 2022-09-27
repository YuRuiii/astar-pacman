# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import math

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def myHeuristic(state, problem=None):
    """
        you may need code other Heuristic function to replace  NullHeuristic
        """
    "*** YOUR CODE HERE ***"
    return util.manhattanDistance(state, problem.goal)

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first.

        Your search algorithm needs to return a list of actions that reaches the
        goal. Make sure to implement a graph search algorithm.

        To get started, you might want to try some of these simple commands to
        understand the search problem that is being passed in:

        print("Start:", problem.getStartState())
        print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
        print("Start's successors:", problem.getSuccessors(problem.getStartState()))
        """
    "*** YOUR CODE HERE ***"

    node_start = problem.getStartState()
    
    open_list = [node_start]
    closed_list = []

    g = {node_start: 0}
    h = {node_start: heuristic(node_start, problem)}
    parent_info = {node_start: (None, None)}

    while open_list:
        # Take from the open list the node node_current with the lowest f
        node_current = None
        for node in open_list:
            min_f = math.inf
            if g[node] + h[node] <= min_f:
                node_current = node
                min_f = g[node] + h[node]
        open_list.remove(node_current)
        if problem.isGoalState(node_current):
            actions = []
            node_p = node_current
            while node_p in parent_info:
                if parent_info[node_p] == (None, None): # no_parent
                    break
                actions.insert(0, parent_info[node_p][1])
                node_p = parent_info[node_p][0]
            return actions
        successors = problem.getSuccessors(node_current)
        for successor in successors:
            node_successor = successor[0]
            successor_action = successor[1]
            successor_cost = successor[2]
            assert(node_current in g)
            successor_current_cost = g[node_current] + successor_cost

            if node_successor in open_list:
                assert(node_successor in g)
                if g[node_successor] <= successor_current_cost:
                    continue
            elif node_successor in closed_list:
                assert(node_successor in g)
                if g[node_successor] <= successor_current_cost:
                    continue
                open_list.remove(node_successor)
                closed_list.append(node_successor)
            else:
                open_list.append(node_successor)
                h[node_successor] = heuristic(node_successor, problem)
            g[node_successor] = successor_current_cost
            parent_info[node_successor] = (node_current, successor_action)
        closed_list.append(node_current)

    assert(0)

# Abbreviations
astar = aStarSearch