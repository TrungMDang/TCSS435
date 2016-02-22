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

    directions = list()
    directions.append(S)
    directions.append(W)
    directions.append(E)
    directions.append(N)

    
    print problem
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    print "Directions: ", directions
    stack = Stack()
    pathQueue = Queue()
    explored = list()
    path = list()
    localPath = list()
    result = list()
    counter = 0
    parentChildMap = list()        
    path = recursiveDFS(problem, problem.getStartState(), None, explored, path)
    return path
            
    
def recursiveDFS(problem, start, direction, explored, path):
    print "current: ", start
    explored.append(start)
    if (problem.getStartState() != start):
        path = path + [direction]
    if (problem.isGoalState(start)):
        print "Found goal!"
        return path
    reverseList = list()
    for child in problem.getSuccessors(start):
        if not child[0] in explored:
            reverseList.insert(0, child) 
    for element in reverseList:
        new = recursiveDFS(problem, element[0], element[1], explored, path)            
        if new:
            return new
    return None        
                
##        while (not stack.isEmpty()):
##            #print "Path: ", path
##            next = stack.pop()
##            path.append(next[1])
##            print "Path:", path
##                  
####            if len(problem.getSuccessors(next[0])) != 0:
####                path.append(next[1])
####                print "new path: ", next[1]
##        
##
##            counter = counter + 1
##
##            print "next: ", next
##            print "Explored:", explored
##            if next[0] in explored:
##                print "has: ", next
##                "path.append(next[1])"
##                "path.pop()"
##                
##            else:
##                explored.append(next[0])
##                
##                "print ""Explored set: "", explored"
##                for next1 in problem.getSuccessors(next[0]):
##                    stack.push(next1)
##                    print "successors of: ", next1
##                    if len(problem.getSuccessors(next1[0])) == 0:
##                        print "no successors for: ", next1
##                        stack.pop()
##                        path.pop()
##                                  

    
    #util.raiseNotDefined()

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
        print "Current state: ", state
        explored.append(state)
        if (problem.isGoalState(state)):
            print "Found..."
            path = backtracking(problem, state, parentChild)
            return path
        for successor in problem.getSuccessors(state):
            if (not successor[0] in explored):
                parentChild.append((state, successor[1], successor[0]))
               
                frontier.push(successor[0])
    return None
    
    #util.raiseNotDefined()
    
def backtracking(problem, goal, parentChild):
    print "Backtracking..."
    path = list()
    parentL = list()
    childL = list()
    directionL = list()
    parentL = [x[0] for x in parentChild]
    directionL = [x[1] for x in parentChild]
    childL = [x[2] for x in parentChild]
    print parentL, len(parentL)
    print directionL, len(directionL)
    print childL, len(childL)

    print "Start: ", goal
    child = goal
    while (child != problem.getStartState()):
        indexParent = childL.index(child)
        parent = parentL[indexParent]
        #print "Parent: ", parent
        path.insert(0, directionL[indexParent])
        if (parent == problem.getStartState()):
            print "Path: ", path
            return path
        indexChild = childL.index(parent)
        child = childL[indexChild]
        #print "New child: ", child
    #print "Path: ", path
    return path
    
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

    

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
