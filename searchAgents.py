import random

import problems
import search
from game import Agent
from game import Directions


class GoWestAgent(Agent):
    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


class RandomAgent(Agent):
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        random.shuffle(actions)
        return actions[0]


class SearchAgent(Agent):
    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        # TODO 11
        self.agentActionIndex = -1
        problem = self.searchProblem(state)
        if self.AlgoCheck:
            if self.Single:
                self.agentActions = self.searchAlgorithm(problem, search.singleFoodSearchHeuristic)
            else:
                self.agentActions = self.searchAlgorithm(problem, search.multiFoodSearchHeuristic)
        else:
            self.agentActions = self.searchAlgorithm(problem)
        print(f"Total steps of Pacman took: {len(self.agentActions)}")

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        # TODO 12
        self.agentActionIndex += 1
        if len(self.agentActions) > self.agentActionIndex:
            return self.agentActions[self.agentActionIndex]
        else:
            return Directions.STOP


class BFSFoodSearchAgent(SearchAgent):
    # TODO 13
    def __init__(self):
        self.AlgoCheck = False
        self.searchProblem = problems.SingleFoodSearchProblem
        self.searchAlgorithm = search.breadthFirstSearch


class BFSMultipleFoodSearchAgent(SearchAgent):
    # TODO 13
    def __init__(self):
        self.AlgoCheck = False
        self.searchProblem = problems.MultiFoodSearchProblem
        self.searchAlgorithm = search.breadthFirstSearch


class DFSFoodSearchAgent(SearchAgent):
    # TODO 14
    def __init__(self):
        self.AlgoCheck = False
        self.searchProblem = problems.SingleFoodSearchProblem
        self.searchAlgorithm = search.depthFirstSearch


class DFSMultipleFoodSearchAgent(SearchAgent):
    # TODO 14
    def __init__(self):
        self.AlgoCheck = False
        self.searchProblem = problems.MultiFoodSearchProblem
        self.searchAlgorithm = search.depthFirstSearch


class UCSFoodSearchAgent(SearchAgent):
    # TODO 15
    def __init__(self):
        self.AlgoCheck = False
        self.searchProblem = problems.SingleFoodSearchProblem
        self.searchAlgorithm = search.uniformCostSearch


class UCSMultipleFoodSearchAgent(SearchAgent):
    # TODO 15
    def __init__(self):
        self.AlgoCheck = False
        self.searchProblem = problems.MultiFoodSearchProblem
        self.searchAlgorithm = search.uniformCostSearch


class AStarFoodSearchAgent(SearchAgent):
    # TODO 16
    def __init__(self):
        self.AlgoCheck = True
        self.Single = True
        self.searchProblem = problems.SingleFoodSearchProblem
        self.searchAlgorithm = search.aStarSearch


class AStarMultipleFoodSearchAgent(SearchAgent):
    # TODO 16
    def __init__(self):
        self.AlgoCheck = True
        self.Single = False
        self.searchProblem = problems.MultiFoodSearchProblem
        self.searchAlgorithm = search.aStarSearch
