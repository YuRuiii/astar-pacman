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
from operator import attrgetter

class State:
    def __init__(self, pos, parent, action, cost, heuristic):
        self.pos = (0, 0)
        self.parent = parent
        self.actions = parent.actions.append(action) if action and self.parent and self.parent.actions else []
        self.g = parent.g + cost if parent else 0
        self.h = heuristic(self.pos)
        self.f = self.g + self.h

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

def print_list(list):
    for _, ele in enumerate(list):
        print(ele.pos, end=" ")
    print("")

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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    print("(35,2) successor", problem.getSuccessors((35,2)))

    # 1. initialize the open list and the closed list, put the starting node on the open
    open_list = []
    closed_list = []
    start = State(
        pos=problem.getStartState(), 
        parent=None, 
        cost=0, 
        action=None,
        heuristic=heuristic
    )
    open_list.append(start)

    # 2. while the open list is not empty
    while open_list:
        # a) find the node with the least f on the open list, call it "q"
        q = min(open_list, key=attrgetter('f'))
        print_list(open_list)

        # b) pop q off the open list
        open_list.remove(q)

        # c) generate q's successors and set their parents to q
        q_successors = problem.getSuccessors(q.pos)

        # d) for each successor
        for s in q_successors:
            s_pos = s[0]
            s_action = s[1]
            s_cost = s[2]

            # i) if successor is the goal, stop search
            # ii) else compute both g and h for the successor
            state_s = State(pos=s_pos, parent=q, action=s_action, cost=s_cost, heuristic=heuristic)
            if problem.isGoalState(state_s.pos):
                return state_s.actions
        
            # iii) if a node with the same position as successor is in the OPEN list which has a lower f than successor, skip this successor
            skip = False
            for state_e in open_list:
                if state_e.pos == state_s.pos and state_e.f < state_s.f:
                    skip = True
                    break
            if skip:
                continue

            # iv) if a node with the same position as successor is in the CLOSED list which has a lower f than successor, skip this successor. otherwise, add the node to the open list
            for state_e in open_list:
                if state_e.pos == state_s.pos and state_e.f < state_s.f:
                    skip = True
                    break
            if skip:
                continue

            open_list.append(state_s)

        # e) push q on the closed list
        closed_list.append(q)

    exit(-1)
    # util.raiseNotDefined()

# Abbreviations
astar = aStarSearch