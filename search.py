# """
# In search.py, you will implement generic search algorithms which are called by
# Pacman agents (in searchAgents.py).
# """
import problems
import search
from game import Directions, Actions

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST

import util
from datetime import datetime


def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 17
    timeStart = datetime.now()
    closedSet = set()
    fringe = util.Stack()
    fringe.push((problem.getStartState(), []))
    while not fringe.isEmpty():
        nodeState, path = fringe.pop()
        if problem.isGoalState(nodeState):
            print(datetime.now() - timeStart)
            return path
        if nodeState not in closedSet:
            closedSet.add(nodeState)
            for successor in problem.getSuccessors(nodeState):
                fringe.push((successor[0], path + [successor[1]]))
    return closedSet


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 18
    timeStart = datetime.now()
    closedSet = set()
    fringe = util.Queue()
    fringe.push((problem.getStartState(), []))
    while not fringe.isEmpty():
        nodeState, path = fringe.pop()
        if problem.isGoalState(nodeState):
            print(datetime.now() - timeStart)
            return path
        if nodeState not in closedSet:
            closedSet.add(nodeState)
            for successor in problem.getSuccessors(nodeState):
                fringe.push((successor[0], path + [successor[1]]))
    return closedSet


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 19
    timeStart = datetime.now()
    closedSet = {}
    fringe = util.PriorityQueue()
    fringe.push((problem.getStartState(), [], 0), 0)
    while not fringe.isEmpty():
        nodeState, path, cost = fringe.pop()
        if problem.isGoalState(nodeState):
            print(datetime.now() - timeStart)
            return path
        if (nodeState not in closedSet) or (cost < closedSet[nodeState]):
            closedSet[nodeState] = cost
            for successor in problem.getSuccessors(nodeState):
                fringe.push((successor[0], path + [successor[1]], cost + successor[2]), cost + successor[2])
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
    successors = problem.getSuccessors(state)
    foodPos = problem.getFoodPos()

    heuristic = util.manhattanDistance(successors[0][0], foodPos)

    return heuristic


def multiFoodSearchHeuristic(state, problem=None):  # state includes pac pos in [0] and food positions onwards
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    position, foodGrid = state

    heuristic = 0
    foodList = foodGrid.asList()

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

        # distance between the closest manhattan node and farthest manhattan node
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
    prob = problems.SingleFoodSearchProblem(gameState)
    return len(search.breadthFirstSearch(prob))


def aStarSearch(problem, heuristic=nullHeuristic):
    timeStart = datetime.now()
    frontier = util.PriorityQueue()

    def frontierAdd(frontier, node, cost):  # node is a tuple with format like : (state, cost, path)
        cost += heuristic(node[0], problem)  # f(n) = g(n) + h(n), heuristic(state, problem=None)
        frontier.push(node, cost)

    # initialize the frontier using the initial state of problem
    startState = (problem.getStartState(), 0, [])  # node is a tuple with format like : (state, cost, path)
    frontierAdd(frontier, startState, 0)  # frontierAdd(frontier, node, cost)

    # initialize the explored set to be empty
    explored = set()  # use set to keep distinct

    # loop do
    while not frontier.isEmpty():
        # choose a leaf node and remove it from the frontier
        (state, cost, path) = frontier.pop()

        # if the node contains a goal state then return the corresponding solution
        if problem.isGoalState(state):
            print(datetime.now() - timeStart)
            return path

        # add the node to the explored set
        if not state in explored:
            explored.add(state)

            # expand the chosen node, adding the resulting nodes to the frontier
            # ??? only if not in the frontier or explored set
            for childState, childAction, childCost in problem.getSuccessors(state):
                newCost = cost + childCost  # Notice! Can't use cost += childCost
                newPath = path + [childAction]  # Notice! Can't use path.append(childAction)
                newState = (childState, newCost, newPath)
                frontierAdd(frontier, newState, newCost)

    # if the frontier is empty then return failure
    return "There is nothing in frontier. Failure!"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
