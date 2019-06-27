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
    # initialization
    current_state = problem.getStartState()
    if problem.isGoalState(current_state):
        return []
    actions = list()
    fringe = util.Stack()
    for successor in problem.getSuccessors(current_state):
        if problem.isGoalState(successor[0]):
            return [successor[1]]
        fringe.push((successor[0], successor[1], 1))
    explored = set()
    current_depth = 0   # record the depth in order to adjust actions
    # implement DFS
    while not fringe.isEmpty():
        explored.add(current_state)
        while current_state in explored:
            current_state, action, depth = fringe.pop()
            if depth <= current_depth:
                actions = actions[:depth-current_depth-1]   # adjust actions with depth
            current_depth = depth
        actions.append(action)
        for successor in problem.getSuccessors(current_state):
            if successor[0] not in explored:
                if problem.isGoalState(successor[0]):
                    actions.append(successor[1])
                    return actions
                fringe.push((successor[0], successor[1], current_depth+1))

    print 'Cannot find a solution!'
    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # initialization
    current_state = problem.getStartState()
    if problem.isGoalState(current_state):
        return []
    actions = list()
    current_index = -1   # record the index of list "actions"
    fringe = util.Queue()
    explored = set()
    state2data = dict()  # each key-value pair of state: parent_index
    for successor in problem.getSuccessors(current_state):
        if problem.isGoalState(successor[0]):
            return [successor[1]]
        fringe.push(successor)
        state2data[successor[0]] = current_index
    # implement BFS
    while not fringe.isEmpty():
        explored.add(current_state)
        while current_state in explored:
            current_state, action, _ = fringe.pop()
        actions.append([current_state, action])
        current_index += 1
        for successor in problem.getSuccessors(current_state):
            if successor[0] not in explored:
                if problem.isGoalState(successor[0]):
                    solution = [successor[1]]
                    while current_index >= 0:
                        state, action = actions[current_index]
                        solution.insert(0, action)
                        current_index = state2data[state]
                    return solution
                fringe.push(successor)
            if successor[0] not in state2data.keys():
                state2data[successor[0]] = current_index

    print 'Cannot find a solution!'
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # initialization
    current_state = problem.getStartState()
    if problem.isGoalState(current_state):
        return []
    current_index = -1   # record the index of list "actions"
    actions = list()
    explored = set()
    state2data = dict()  # each key-value pair of state: (parent_index, cost)
    fringe = util.PriorityQueue()
    for successor in problem.getSuccessors(current_state):
        fringe.push((successor[0], successor[1]), successor[2])
        state2data[successor[0]] = (current_index, successor[2])
    # implement UCS
    while not fringe.isEmpty():
        current_state, action = fringe.pop()
        explored.add(current_state)
        actions.append((current_state, action))
        current_index += 1
        if problem.isGoalState(current_state):
            solution = []
            while current_index >= 0:
                state, action = actions[current_index]
                solution.insert(0, action)
                current_index = state2data[state][0]
            return solution
        for successor in problem.getSuccessors(current_state):
            if successor[0] not in explored:
                cost = state2data[current_state][1] + successor[2]
                if successor[0] in state2data.keys() and cost < state2data[current_state][1] or successor[0] not in state2data.keys():
                    state2data[successor[0]] = (current_index, cost)
                fringe.update((successor[0], successor[1]), cost)

    print 'Cannot find a solution!'
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # initialization
    current_state = problem.getStartState()
    if problem.isGoalState(current_state):
        return []
    current_index = -1  # record the index of list "actions"
    actions = list()
    explored = set()
    state2data = dict()  # each key-value pair of state: (parent_index, cost)
    fringe = util.PriorityQueue()
    for successor in problem.getSuccessors(current_state):
        fringe.push((successor[0], successor[1]), successor[2]+heuristic(successor[0], problem))
        state2data[successor[0]] = (current_index, successor[2])
    # implement A*
    while not fringe.isEmpty():
        current_state, action = fringe.pop()
        explored.add(current_state)
        actions.append((current_state, action))
        current_index += 1
        if problem.isGoalState(current_state):
            solution = []
            while current_index >= 0:
                state, action = actions[current_index]
                solution.insert(0, action)
                current_index = state2data[state][0]
            return solution
        for successor in problem.getSuccessors(current_state):
            if successor[0] not in explored:
                cost = state2data[current_state][1] + successor[2] + heuristic(successor[0], problem)
                if successor[0] in state2data.keys() and cost < state2data[current_state][1] or successor[0] not in state2data.keys():
                    state2data[successor[0]] = (current_index, state2data[current_state][1] + successor[2])
                fringe.update((successor[0], successor[1]), cost)

    print 'Cannot find a solution!'
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
