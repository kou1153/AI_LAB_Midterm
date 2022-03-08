"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from game import Directions

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST

import util


def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 17
    frontier = util.Stack()
    frontier.push(problem.getStartState())
    visited = []
    path = []  # Directions
    currentPath = util.Stack()
    currentState = frontier.pop()
    while not problem.isGoalState(currentState):
        if currentState not in visited:
            visited.append(currentState)
            successors = problem.getSuccessors(currentState)
            for childNode, direction, stepCost in successors:
                frontier.push(childNode)
                tempPath = path + [direction]
                currentPath.push(tempPath)
        currentState = frontier.pop()
        path = currentPath.pop()
    return path


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 18
    frontier = util.Queue()
    frontier.push(problem.getStartState())
    visited = []
    path = []
    tempPath = []
    currentPath = util.Queue()
    currentState = frontier.pop()
    while not problem.isGoalState(currentState):
        if currentState not in visited:
            visited.append(currentState)
            successors = problem.getSuccessors(currentState)
            for childNode, direction, stepCost in successors:
                frontier.push(childNode)
                tempPath = path + [direction]
                currentPath.push(tempPath)
        currentState = frontier.pop()
        path = currentPath.pop()

    return path


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 19
    frontier = util.PriorityQueue()
    frontier.push(problem.getStartState(), 0)
    visited = []
    path = []
    tempPath = []
    currentPath = util.PriorityQueue()
    currentState = frontier.pop()
    while not problem.isGoalState(currentState):
        if currentState not in visited:
            visited.append(currentState)
            successors = problem.getSuccessors(currentState)
            for childNode, direction, stepCost in successors:
                tempPath = path + [direction]
                cost = problem.getCostOfActions(tempPath)
                if childNode not in visited:
                    frontier.push(childNode, cost)
                    currentPath.push(tempPath, cost)
        currentState = frontier.pop()
        path = currentPath.pop()

    return path


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def singleFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of single food search
    """
    # TODO 20
    pass


def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    pass


def aStarSearch(problem, heuristic=nullHeuristic):
    '''
    return a path to the goal
    '''
    # TODO 22


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
