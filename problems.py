import util
from game import Directions, Actions


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


# TODO
class SingleFoodSearchProblem(SearchProblem):
    """
    SingleFoodSearchProblem is used to find paths to a particular point on the board
    """

    def __init__(self, startingGameState, start=None, goal=(1, 1), warn=True, visualize=True):
        # TODO 1
        """
        Save start and goal positions (x, y)
        startingGameState: a GameState object (in pacman.py)
        costFunc: a function from a search state to a positive number
        goal: a position in the startingGameState
        """
        self.walls = startingGameState.getWalls()
        self.startState = startingGameState.getPacmanPosition()
        if start is not None:
            self.startState = start

        self.goal = goal
        self.visualize = visualize

        if warn and (startingGameState.getNumFood() != 1 or not startingGameState.hasFood(*goal)):
            print("The maze is not for single food search")

    def getStartState(self):
        # TODO 2
        return self.startState

    def isGoalState(self, state):
        # TODO 3
        return state == self.goal

    def getSuccessors(self, state):
        # TODO 4
        """
        For a given state, it will return a list of (successor, action, stepCost):
        - successor: successor to the current state
        - action: required action to get there
        - stepCost: the incremental cost of expanding to the successor
        """
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]: # check all directions
            x, y = state
            dx, dy = Actions.directionToVector(action)
            successor_x = int(x + dx)
            successor_y = int(y + dy)
            if not self.walls[successor_x][successor_y]:
                successor = (successor_x, successor_y)
                successors.append((successor, action, 1))

        return successors

    def getCostOfActions(self, actions):
        # TODO 5
        """
        Return the cost of a particular sequence of actions. If no illegal actions, return -1
        """
        if actions is None:
            # no actions
            return -1

        # x, y = self.getStartState()
        totalCost = 0
        for action in actions:
            # find the successor and check whether it's legal
            # dx, dy = Actions.directionToVector(action)
            # successor_x = int(x + dx)
            # successor_y = int(y + dy)
            totalCost += 1

        return totalCost


# TODO
class MultiFoodSearchProblem(SearchProblem):
    """
    MultiFoodSearchProblem is used to find a path that help pacman collect all the dots in the maze
    State in this way is a tuple (pacmanPos, dotGrid):
    - pacmanPos: specifies pacman's position (x, y)
    - dotGrid: a Grid (in game.py) contains 2 values True and False to specify dots on the map
    """

    def __init__(self, startingGameState):
        # TODO 6
        pacmanPos, dotGrid = startingGameState.getPacmanPosition(), startingGameState.getFood()
        self.startState = (pacmanPos, dotGrid)
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self.expanded = 0
        # Store information about heuristic
        self.heuristicInfo = {}

    def getStartState(self):
        # TODO 7
        return self.startState

    def isGoalState(self, state):
        # TODO 8
        return state[1].count() == 0

    def getSuccessors(self, state):
        # TODO 9
        """
        For a given state, it will return a list of (successor, action, stepCost):
        - successor: successor to the current state
        - action: required action to get there
        - stepCost = 1
        """
        successors = []
        self.expanded += 1
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(action)
            successor_x = int(x + dx)
            successor_y = int(y + dy)
            if not self.walls[successor_x][successor_y]:
                successor = state[1].copy()
                successor[successor_x][successor_y] = False
                successors.append((((successor_x, successor_y), successor), action, 1))

        return successors

    def getCostOfActions(self, actions):
        # TODO 10
        """
        Return the cost of a particular sequence of actions. If no illegal actions, return -1
        """
        # x, y = self.getStartState()[0]
        totalCost = 0
        for action in actions:
            # find the successor and check whether it's legal
            # dx, dy = Actions.directionToVector(action)
            # successor_x = int(x + dx)
            # successor_y = int(y + dy)
            totalCost += 1

        return totalCost
