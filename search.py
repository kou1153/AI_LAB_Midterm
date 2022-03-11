# """
# In search.py, you will implement generic search algorithms which are called by
# Pacman agents (in searchAgents.py).
# """
import problems
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
    visited = set()  # store visited states, use set to keep distinct
    frontier = util.Stack()  # init the frontier ((x,y), path) using a stack
    frontier.push((problem.getStartState(), []))

    while not frontier.isEmpty():
        currentState, path = frontier.pop()

        # if it is a goal state then return the corresponding solution
        if problem.isGoalState(currentState):
            return path

        if currentState not in visited:
            visited.add(currentState)
            for successor in problem.getSuccessors(currentState):
                frontier.push((successor[0], path + [successor[1]]))

    return "No path found"


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 18
    visited = set()  # store visited states, use set to keep distinct
    frontier = util.Queue()  # init the frontier ((x,y), path) using a queue
    frontier.push((problem.getStartState(), []))

    while not frontier.isEmpty():
        currentState, path = frontier.pop()

        # if it is a goal state then return the corresponding solution
        if problem.isGoalState(currentState):
            return path

        if currentState not in visited:
            visited.add(currentState)
            for successor in problem.getSuccessors(currentState):
                frontier.push((successor[0], path + [successor[1]]))

    return "No path found"


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 19
    visited = {}  # store visited states, use dict to keep distinct and access to key/value
    frontier = util.PriorityQueue()  # init the frontier ((pos, path, cost), cost) using a PriorityQueue
    frontier.push((problem.getStartState(), [], 0), 0)

    while not frontier.isEmpty():
        currentState, path, cost = frontier.pop()

        # if it is a goal state then return the corresponding solution
        if problem.isGoalState(currentState):
            return path

        if (currentState not in visited) or (cost < visited[currentState]):
            visited[currentState] = cost
            for successor in problem.getSuccessors(currentState):
                frontier.push((successor[0], path + [successor[1]], cost + successor[2]), cost + successor[2])

    return "No path found"


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
    """
    state here is a tuple (x, y) of pacmanPos
    Simply calculate the manhanttanDistance between pacmanPos to foodPos as heuristic 
    foodPos in single problem is always fixed
    """
    successors = problem.getSuccessors(state)
    foodPos = problem.getFoodPos()

    heuristic = util.manhattanDistance(successors[0][0], foodPos)

    return heuristic


def findClosestPoint(location, goalArray):
    """
    support for calculate mazeDistance
    """

    closestPoint = 0
    closestPointCost = util.manhattanDistance(location, goalArray[0])

    for j in range(len(goalArray)):
        # calculate distance between current state to corner
        cornerLocation = goalArray[j]
        lengthToCorner = util.manhattanDistance(location, cornerLocation)

        if lengthToCorner < closestPointCost:
            closestPoint = j
            closestPointCost = lengthToCorner

    return closestPoint, closestPointCost


def findFarthestPoint(location, goalArray):
    """
    support for calculate mazeDistance
    """

    farthestPoint = 0
    farthestPointCost = util.manhattanDistance(location, goalArray[0])

    for j in range(len(goalArray)):
        # calculate distance between current state to corner
        cornerLocation = goalArray[j]
        lengthToCorner = util.manhattanDistance(location, cornerLocation)

        if lengthToCorner > farthestPointCost:
            farthestPoint = j
            farthestPointCost = lengthToCorner

    return farthestPoint, farthestPointCost


def mazeDistance(point1, point2, gameState):
    """
    Returns the maze distance between any two points.
    support for multiFoodSearchHeuristic
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()

    if walls[x1][y1]:
        raise 'point1 is a wall: ' + str(point1)
    if walls[x2][y2]:
        raise 'point2 is a wall: ' + str(point2)

    prob = problems.SingleFoodSearchProblem(gameState)
    return len(breadthFirstSearch(prob))


def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    """
    state is a tuple (pacmanPos, foodGrid)
    Using maze distance to foodGrid as heuristic
    Heuristic is the addition of: (distance between current location and closest manhattan state) 
    + (distance between the closest manhattan state and farthest manhattan state)
    """
    pacmanPos, foodGrid = state
    foodList = foodGrid.asList()  # get a list of food coordinates
    heuristic = 0

    # calculate the distance from current pacmanPos to food-containing pos
    if len(foodList) > 0:
        currentState = problem.startingGameState

        # find the closest food
        closestFood = findClosestPoint(pacmanPos, foodList)
        closestFoodIndex = closestFood[0]
        closestFoodPos = foodList[closestFoodIndex]

        # find the farthest food
        farthestFood = findFarthestPoint(pacmanPos, foodList)
        farthestFoodIndex = farthestFood[0]
        farthestFoodPos = foodList[farthestFoodIndex]

        # distance between current location and closest manhattan state
        currentToClosest = mazeDistance(pacmanPos, closestFoodPos, currentState)

        # distance between the closest manhattan state and farthest manhattan state
        closestToFarthest = mazeDistance(closestFoodPos, farthestFoodPos, currentState)

        heuristic = currentToClosest + closestToFarthest

    return heuristic


def aStarSearch(problem, heuristic=nullHeuristic):
    frontier = util.PriorityQueue()

    def frontierAdd(frontier, state, cost):  # state is a tuple with format like : (state, cost, path)
        cost += heuristic(state[0], problem)  # f(n) = g(n) + h(n), heuristic(state, problem=None)
        frontier.push(state, cost)

    # initialize the frontier using the initial state of problem
    startState = (problem.getStartState(), 0, [])  # state is a tuple with format like : (state, cost, path)
    frontierAdd(frontier, startState, 0)  # frontierAdd(frontier, state, cost)

    # initialize the visited set to be empty
    visited = set()  # use set to keep distinct

    while not frontier.isEmpty():
        # choose a child state and remove it from the frontier
        (currentState, cost, path) = frontier.pop()

        # if it is a goal state then return the corresponding solution
        if problem.isGoalState(currentState):
            return path

        # add the state to the visited set
        if currentState not in visited:
            visited.add(currentState)

            # expand the chosen state, adding the resulting states to the frontier
            # ??? only if not in the frontier or visited set
            for childState, childAction, childCost in problem.getSuccessors(currentState):
                newCost = cost + childCost  
                newPath = path + [childAction]  
                newState = (childState, newCost, newPath)
                frontierAdd(frontier, newState, newCost)

    return "No path found"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
