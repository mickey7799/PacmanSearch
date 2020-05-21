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
import game
import util
import searchAgents


class PathNode:
    def __init__(self, parent_path_node, parent_path_node_to_here_direction, parent_path_node_to_here_cost, state):
        self.parentPathNode = parent_path_node
        self.parentPathNodeToHereDirection = parent_path_node_to_here_direction
        self.parentPathNodeToHereCost = parent_path_node_to_here_cost
        self.state = state

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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    ending_path_node = getEndingPathNodeByDFS(problem)
    # print( "End at : " + str(ending_path_node.state))
    # print( "Path : " + str(getPacmanPathFromEndingPathNode(ending_path_node)))
    return getPacmanPathFromEndingPathNode(ending_path_node)  # util.raiseNotDefined()


def getEndingPathNodeByDFS(problem):
    visited_state_set = set()
    state_stack = util.Stack()

    start_state = problem.getStartState()
    start_path_node = PathNode(None, None, None, start_state)
    visited_state_set.add(start_path_node)
    state_stack.push(start_path_node)

    while state_stack.isEmpty() is False:
        current_path_node = state_stack.pop()
        visited_state_set.add(current_path_node.state)
        if problem.isGoalState(current_path_node.state):
            return current_path_node
        successors = problem.getSuccessors(current_path_node.state)
        for successor_state, direction_to_successor, successor_cost in successors:
            # print("successor coordinates" +  str( coordinates))
            # print("successor direction" +  str( direction))
            # print("successor cost" +  str( cost))
            if successor_state not in visited_state_set:
                visited_state_set.add(successor_state)
                state_stack.push(PathNode(current_path_node, direction_to_successor, successor_cost, successor_state))


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    # print("Start at : " + str(problem.getStartState()))
    ending_path_node = getEndingPathNodeByBFS(problem)
    # print( "End at : " + str(ending_path_node.state))
    # print( "Path : " + str(getPacmanPathFromEndingPathNode(ending_path_node)))
    return getPacmanPathFromEndingPathNode(ending_path_node)
    # util.raiseNotDefined()


def getEndingPathNodeByBFS(problem):
    visited_state_set = set()
    state_queue = util.Queue()

    start_state = problem.getStartState()
    start_path_node = PathNode(None, None, None, start_state)
    visited_state_set.add(start_path_node)
    state_queue.push(start_path_node)

    while state_queue.isEmpty() is False:
        current_path_node = state_queue.pop()
        visited_state_set.add(current_path_node.state)
        if problem.isGoalState(current_path_node.state):
            return current_path_node
        successors = problem.getSuccessors(current_path_node.state)
        for successor_state, direction_to_successor, successor_cost in successors:
            # print("successor coordinates" +  str( coordinates))
            # print("successor direction" +  str( direction))
            # print("successor cost" +  str( cost))
            if successor_state not in visited_state_set:
                visited_state_set.add(successor_state)
                state_queue.push(PathNode(current_path_node, direction_to_successor, successor_cost, successor_state))


def getPacmanPathFromEndingPathNode(ending_path_node):
    s = util.Stack()
    current_path_node = ending_path_node
    s.push(current_path_node)
    while current_path_node.parentPathNode is not None:
        s.push(current_path_node.parentPathNode)
        current_path_node = current_path_node.parentPathNode

    pacman_path = []
    while s.isEmpty() is not True:
        current_path_node = s.pop()
        direction = current_path_node.parentPathNodeToHereDirection
        if direction is not None:
            pacman_path.append(direction)
    return pacman_path


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    # print("Start at : " + str(problem.getStartState()))
    ending_path_node = getEndingPathNodeByUCS(problem)
    # print( "End at : " + str(ending_path_node.state))
    # print( "Path : " + str(getPacmanPathFromEndingPathNode(ending_path_node)))
    return getPacmanPathFromEndingPathNode(ending_path_node)
    util.raiseNotDefined()


def getEndingPathNodeByUCS(problem):
    visited_state_set = set()
    state_priority_queue = util.PriorityQueue()

    start_state = problem.getStartState()
    start_path_node = PathNode(None, None, 0, start_state)
    visited_state_set.add(start_path_node)
    state_priority_queue.push(start_path_node, 0)

    while state_priority_queue.isEmpty() is False:
        current_path_node = state_priority_queue.pop()
        visited_state_set.add(current_path_node.state)
        if problem.isGoalState(current_path_node.state):
            return current_path_node
        successors = problem.getSuccessors(current_path_node.state)
        for successor_state, direction_to_successor, successor_cost in successors:
            # print("successor coordinates" +  str( coordinates))
            # print("successor direction" +  str( direction))
            # print("successor cost" +  str( cost))
            if successor_state not in visited_state_set:
                visited_state_set.add(successor_state)
                successor_path_node = PathNode(current_path_node, direction_to_successor, successor_cost,
                                               successor_state)
                state_priority_queue.push(successor_path_node, successor_cost)
                # state_priority_queue.update()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    ending_path_node = getEndingPathNodeByAStar(problem, heuristic)
    # print( "End at : " + str(ending_path_node.state))
    # print( "Path : " + str(getPacmanPathFromEndingPathNode(ending_path_node)))
    return getPacmanPathFromEndingPathNode(ending_path_node)


def getEndingPathNodeByAStar(problem, heuristic):
    WEIGHT = 2
    visited_state_set = set()
    best_g_map = {}
    state_priority_queue = util.PriorityQueue()
    start_state = problem.getStartState()
    start_path_node = PathNode(None, None, 0, start_state)
    visited_state_set.add(start_path_node.state)
    best_g_map[start_path_node.state] = 0
    state_priority_queue.push(start_path_node, 0)
    while state_priority_queue.isEmpty() is False:
        current_path_node = state_priority_queue.pop()
        visited_state_set.add(current_path_node.state)
        if problem.isGoalState(current_path_node.state):
           
            return current_path_node
        successors = problem.getSuccessors(current_path_node.state)
        for successor_state, direction_to_successor, successor_cost in successors:
            
            if (successor_state not in visited_state_set) or (
                    current_path_node.parentPathNodeToHereCost + successor_cost < best_g_map[successor_state]):
                visited_state_set.add(successor_state)
                best_g_map[successor_state] = current_path_node.parentPathNodeToHereCost + successor_cost
                successor_path_node = PathNode(current_path_node, direction_to_successor,
                                               current_path_node.parentPathNodeToHereCost + successor_cost,
                                               successor_state)
                state_priority_queue.push(successor_path_node,
                                          current_path_node.parentPathNodeToHereCost + successor_cost + WEIGHT * heuristic(
                                              successor_state, problem))



def iterativeDeepeningSearch(problem):
    """Search the deepest node in an iterative manner."""
    "*** YOUR CODE HERE FOR TASK 1 ***"
    ending_path_node = getEndingPathNodeByID(problem)
    # print( "End at : " + str(ending_path_node.state))
    # print( "Path : " + str(getPacmanPathFromEndingPathNode(ending_path_node)))
    return getPacmanPathFromEndingPathNode(ending_path_node)

    util.raiseNotDefined()


def getEndingPathNodeByID(problem):
    state_stack = util.Stack()
    depth = 0
    while True:
        visited_state_set = set()
        start_state = problem.getStartState()
        start_path_node = PathNode(None, None, 0, start_state)
        visited_state_set.add(start_path_node)
        state_stack.push(start_path_node)

        while state_stack.isEmpty() is False:
            current_path_node = state_stack.pop()
            visited_state_set.add(current_path_node.state)
            if problem.isGoalState(current_path_node.state):
                return current_path_node
            if current_path_node.parentPathNodeToHereCost + 1 <= depth:
                successors = problem.getSuccessors(current_path_node.state)
                for successor_state, direction_to_successor, successor_cost in successors:                
                    if successor_state not in visited_state_set:
                        visited_state_set.add(successor_state)
                        state_stack.push(PathNode(current_path_node, direction_to_successor,
                                                current_path_node.parentPathNodeToHereCost + 1,
                                                successor_state))

        depth += 1


def waStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has has the weighted (x 2) lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE FOR TASK 2 ***"
    # print("in wwwaStarSearch")
    # print(str(type(problem)))
    start_state = problem.getStartState()
    start_path_node = PathNode(None, None, 0, start_state)
    if type(problem) == searchAgents.CapsuleSearchProblem:
        if problem.isCapsuleEaten() is False:
            capsule_path_node = getCapsulePathNodeByWAStar(problem, start_path_node, heuristic)
            print("capsule found")
            ending_path_node = getEndingPathNodeByWAStar(problem, capsule_path_node, heuristic)
    else:
        ending_path_node = getEndingPathNodeByWAStar(problem, start_path_node, heuristic)
    # print( "End at : " + str(ending_path_node.state))
    # print( "Path : " + str(getPacmanPathFromEndingPathNode(ending_path_node)))

    return getPacmanPathFromEndingPathNode(ending_path_node)
    util.raiseNotDefined()


def getCapsulePathNodeByWAStar(problem, start_path_node, heuristic):
    WEIGHT = 2
    visited_state_set = set()
    best_g_map = {}
    state_priority_queue = util.PriorityQueue()

    visited_state_set.add(start_path_node.state)
    best_g_map[start_path_node.state] = 0
    state_priority_queue.push(start_path_node, 0)

    while state_priority_queue.isEmpty() is False:
        current_path_node = state_priority_queue.pop()
        # print(current_path_node.state)
        visited_state_set.add(current_path_node.state)

        if problem.isCapsuleState(current_path_node.state):
            return current_path_node
        successors = problem.getSuccessors(current_path_node.state)
        for successor_state, direction_to_successor, successor_cost in successors:
            # print("successor coordinates" +  str( coordinates))
            # print("successor direction" +  str( direction))
            # print("successor cost" +  str( cost))
            # position = successor_state[0]
            food_grid_position = getFoodPosition(current_path_node.state[1])
            # print(food_grid_position)
            if ((successor_state not in visited_state_set) or (
                    current_path_node.parentPathNodeToHereCost + successor_cost < best_g_map[successor_state])) and \
                    successor_state[0] not in food_grid_position:
                visited_state_set.add(successor_state)
                best_g_map[successor_state] = current_path_node.parentPathNodeToHereCost + successor_cost
                successor_path_node = PathNode(current_path_node, direction_to_successor,
                                               current_path_node.parentPathNodeToHereCost + successor_cost,
                                               successor_state)
                state_priority_queue.push(successor_path_node,
                                          current_path_node.parentPathNodeToHereCost + successor_cost + WEIGHT * searchAgents.capsuleHeuristic(
                                              successor_state, problem))

def getFoodPosition(foodGrid):
    food_position = foodGrid.asList()
    return food_position

def getEndingPathNodeByWAStar(problem, start_path_node, heuristic):
    WEIGHT = 2
    visited_state_set = set()
    best_g_map = {}
    state_priority_queue = util.PriorityQueue()

    visited_state_set.add(start_path_node.state)
    best_g_map[start_path_node.state] = 0
    state_priority_queue.push(start_path_node, 0)

    while state_priority_queue.isEmpty() is False:
        current_path_node = state_priority_queue.pop()
        visited_state_set.add(current_path_node.state)
        if problem.isGoalState(current_path_node.state):
            # for key in best_g_map:
            #     print(str(key) + " : " + str(best_g_map[key]))
            return current_path_node
        successors = problem.getSuccessors(current_path_node.state)
        for successor_state, direction_to_successor, successor_cost in successors:
            # print("successor coordinates" +  str( coordinates))
            # print("successor direction" +  str( direction))
            # print("successor cost" +  str( cost))
            if (successor_state not in visited_state_set) or (
                    current_path_node.parentPathNodeToHereCost + successor_cost < best_g_map[successor_state]):
                visited_state_set.add(successor_state)
                best_g_map[successor_state] = current_path_node.parentPathNodeToHereCost + successor_cost
                successor_path_node = PathNode(current_path_node, direction_to_successor,
                                               current_path_node.parentPathNodeToHereCost + successor_cost,
                                               successor_state)
                state_priority_queue.push(successor_path_node,
                                          current_path_node.parentPathNodeToHereCost + successor_cost + WEIGHT * heuristic(
                                              successor_state, problem))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
wastar = waStarSearch
