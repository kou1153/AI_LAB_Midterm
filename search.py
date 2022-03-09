"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import problems
import search
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


def singleFoodSearchHeuristic(state, problem=None):  # state includes a pacman position and a food position
    """
    A heuristic function for the problem of single food search
    """
    # TODO 20


def multiFoodSearchHeuristic(state, problem=None):  # state includes pac pos in [0] and food positions onwards
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    position, foodGrid = state

    heuristic = 0
    foodList = foodGrid.asList()
    print(foodList)

    # calculate the distance from current node to food-containing nodes
    if len(foodList) > 0:
        closestPoint = findClosestPoint(position, foodList)
        farthestPoint = findFarthestPoint(position, foodList)

        closestPointIndex = closestPoint[0]
        farthestPointIndex = farthestPoint[0]

        currentNode = problem.startingGameState
        closestFoodNode = foodList[closestPointIndex]
        farthestFoodNode = foodList[farthestPointIndex]

        # distance between current location and closest manhattan node
        currentToClosest = mazeDistance(position, closestFoodNode, currentNode)

        # distance between closest manhattan node and farthest manhattan node
        closestToFarthest = mazeDistance(closestFoodNode, farthestFoodNode, currentNode)

        heuristic = currentToClosest + closestToFarthest
    return heuristic


def findClosestPoint(location, goalArray):
    closestPoint = 0
    closestPointCost = util.manhattanDistance(location, goalArray[0])

    for j in range(len(goalArray)):
        # calculate distance between current state to corner
        cornerLocation = goalArray[j]
        lengthToCorner = util.manhattanDistance(location, cornerLocation)

        if lengthToCorner < closestPointCost:
            closestPoint = j
            closestPointCost = lengthToCorner

    return (closestPoint, closestPointCost)


def findFarthestPoint(location, goalArray):
    farthestPoint = 0
    farthestPointCost = util.manhattanDistance(location, goalArray[0])

    for j in range(len(goalArray)):
        # calculate distance between current state to corner
        cornerLocation = goalArray[j]
        lengthToCorner = util.manhattanDistance(location, cornerLocation)

        if lengthToCorner > farthestPointCost:
            farthestPoint = j
            farthestPointCost = lengthToCorner

    return (farthestPoint, farthestPointCost)


def mazeDistance(point1, point2, gameState):
    """
    Returns the maze distance between any two points, using the search functions
    you have already built. The gameState can be any game state -- Pacman's
    position in that state is ignored.

    Example usage: mazeDistance( (2,4), (5,6), gameState)

    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = problems.SingleFoodSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.breadthFirstSearch(prob))


def aStarSearch(problem, heuristic=nullHeuristic):
    '''
    return a path to the goal
    '''
    # TODO 22
    frontier = util.PriorityQueue()
    frontier.push(problem.getStartState(), 0)
    currentState = frontier.pop()
    visited = []
    path = []
    currentPath = util.PriorityQueue()
    while not problem.isGoalState(currentState):
        if currentState not in visited:
            visited.append(currentState)
            successors = problem.getSuccessors(currentState)
            for childNode, direction, stepCost in successors:
                tempPath = path + [direction]
                cost = problem.getCostOfActions(tempPath) + heuristic(childNode, problem)
                if childNode not in visited:
                    frontier.push(childNode, cost)
                    currentPath.push(tempPath, cost)
        currentState = frontier.pop()
        path = currentPath.pop()
    return path


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
