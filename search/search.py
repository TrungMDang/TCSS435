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
from util import Stack
from util import Queue
from util import PriorityQueue

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

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from game import Directions
    S = Directions.SOUTH
    W = Directions.WEST
    E = Directions.EAST
    N = Directions.NORTH
           
    path = list()
    parentChild = list()
    print "Problem: ", problem
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    if problem.isGoalState(problem.getStartState()):
        return None
    explored = list()
    frontier = Stack()
    frontier.push(problem.getStartState())
    while (not frontier.isEmpty()):
        state = frontier.pop()
        #print "Current state: ", state
        explored.append(state)
        if (problem.isGoalState(state)):
            #print "Found..."
            path = backtracking(problem, state, parentChild)
            return path
        for successor in problem.getSuccessors(state):
            #print "Successor: ", successor
            if (not successor[0] in explored):
                parentChild.append((state, successor[1], successor[0]))
                frontier.push(successor[0])
    return None

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"   
    path = list()
    parentChild = list()
    print "Problem: ", problem
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    if problem.isGoalState(problem.getStartState()):
        return None
    explored = list()
    frontier = Queue()
    frontier.push(problem.getStartState())
    while (not frontier.isEmpty()):
        state = frontier.pop()
        #print "Current state: ", state
        explored.append(state)
        if (problem.isGoalState(state)):
            #print "Found..."
            path = backtracking(problem, state, parentChild)
            return path
        for successor in problem.getSuccessors(state):
            #print "Successor: ", successor
            if (not successor[0] in explored):
                parentChild.append((state, successor[1], successor[0]))
               
                frontier.push(successor[0])
    return None
    
    #util.raiseNotDefined()
    
def backtracking(problem, goal, parentChild):
    #print "Backtracking..."
    path = list()
    parentL = list()
    childL = list()
    directionL = list()
    parentL = [x[0] for x in parentChild]
    directionL = [x[1] for x in parentChild]
    childL = [x[2] for x in parentChild]
    #print parentL, len(parentL)
    #print directionL, len(directionL)
    #print childL, len(childL)

    #print "Start: ", goal
    child = goal
    while (child != problem.getStartState()):
        indexParent = childL.index(child)
        parent = parentL[indexParent]
        #print "Parent: ", parent
        path.insert(0, directionL[indexParent])
        if (parent == problem.getStartState()):
            #print "Path: ", path
            return path
        indexChild = childL.index(parent)
        child = childL[indexChild]
        #print "New child: ", child
    #print "Path: ", path
    return path
    
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    path = list()
    parentChild = list()
    print "Problem: ", problem
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    if problem.isGoalState(problem.getStartState()):
        return None
    explored = list()
    frontier = PriorityQueue()
    #tuple = (problem.getStartState(), float("inf"))
    frontier.push(problem.getStartState(), float("inf"))
    while(not frontier.isEmpty()):
        state = frontier.pop()      #state = [(x, y), cost]
        explored.append(state)
        if (problem.isGoalState(state)):
            #print "Found..."
            path = backtracking(problem, state, parentChild)
            return path
        for successor in problem.getSuccessors(state):
            
            #print "Successor: ", successor
            if (not successor[0] in explored):
                cost = successor[2] + state[1]
                parentChild.append((state, successor[1], successor[0]))
                frontier.push(successor[0], cost)
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
def myHeuristic(state, problem):
    """
    Actually a manhattan heuristic and euclid heuristic copied from searchAgents.py
    Layout: bigMaze - manhattanHeuristic 481 nodes expanded
            bigMaze - Euclidean heuristic 485 nodes expanded
    """
    xy1 = state
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
##    xy1 = state
##    xy2 = problem.goal
##    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

def aStarSearch(problem, heuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    """
    Only A* can find dots in openMaze. Path looks like a staircase.
    """
    path = list()
    parentChild = list()
    print "Problem: ", problem
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    if problem.isGoalState(problem.getStartState()):
        return None
    explored = list()
    frontier = PriorityQueue()
    #tuple = (problem.getStartState(), float("inf"))
    frontier.push(problem.getStartState(), float("inf"))
    while(not frontier.isEmpty()):
        state = frontier.pop()      #state = [(x, y), cost]
        explored.append(state)
        if (problem.isGoalState(state)):
            #print "Found..."
            path = backtracking(problem, state, parentChild)
            return path
        for successor in problem.getSuccessors(state):
            #print "Successor: ", successor
            if (not successor[0] in explored):
                #print("State[1] ", state[1])
                cost = successor[2] + heuristic(state, problem) #cost = stepCost +currentNodeCost
                parentChild.append((state, successor[1], successor[0]))
                frontier.push(successor[0], cost)
    return None

    #util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
