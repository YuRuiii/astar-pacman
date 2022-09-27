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

from argparse import Action
import util
from operator import attrgetter

class Node:
    def __init__(self):
        self.pos = (0, 0)
        self.parent = None
        self.actions = []
        self.g = 0
        self.h = 0
        self.f = 0

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
    return 0

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

    node_start = Node()
    node_start.pos = problem.getStartState()
    node_start.parent = None
    node_start.actions = []
    node_start.g = 0
    node_start.h = heuristic(node_start.pos)
    node_start.f = node_start.h
    
    open_list = [node_start]
    closed_list = []

    while open_list:
        node_current = min(open_list, key=attrgetter('f'))
        if problem.isGoalState(node_current.pos):
            return node_current.actions
        successors = problem.getSuccessors(node_current.pos)
        for successor in successors:
            successor_pos = successor[0]
            successor_action = successor[1]
            successor_cost = successor[2]
            successor_current_cost = node_current.g + successor_cost
            get_pos = attrgetter('pos')
            if successor_pos in [get_pos(ele) for ele in open_list]:

            skip = False


            for ele in open_list:
                if ele.pos == successor_pos and ele.g <= successor_current_cost:
                    skip = True
                    break
            if skip:
                continue
            else:
                for ele in closed_list:
                    if ele.pos == successor_pos and ele.g <= successor_current_cost:
                        closed_list.remove(ele)
                        open_list.append(ele)

# Abbreviations
astar = aStarSearch