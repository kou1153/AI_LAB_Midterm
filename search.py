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
    currentPath = util.PriorityQueue()
    currentState = frontier.pop()
    cost = 0
    while not problem.isGoalState(currentState):
        if currentState not in visited:
            visited.append(currentState)
            successors = problem.getSuccessors(currentState)
            for childNode, direction, stepCost in successors:
                tempPath = path + [direction]
                cost += 1
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


def singleFoodSearchHeuristic(state, problem=None): # state includes a pacman position and a food position
    """
    A heuristic function for the problem of single food search
    """
    # TODO 20
    position, foodPos = state[0], state[1]
    return ((position[0] - foodPos[0]) ** 2 + (position[1] - foodPos[1]) ** 2 ) ** 0.5


def multiFoodSearchHeuristic(state, problem=None): # state includes pac pos in [0] and food positions onwards
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    position = state[0]
    foodPos = []
    for i in range(len(state) - 1):
        foodPos.append(state[i + 1])
    totalH = 0
    for food in foodPos:
        totalH += ((position[0] - food[0]) ** 2 + (position[1] - food[1]) ** 2 ) ** 0.5
    return totalH


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
