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

class childNode:
    def __init__(self, cur, parent):
        self.state = cur[0]
        self.action = cur[1]
        self.cost = cur[2]
        self.parent = parent

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
    "*** YOUR CODE HERE ***"
    fringe = util.Stack()
    state = problem.getStartState()
    cur = childNode((state, None, None), None)
    fringe.push(cur)
    visited = set()
    while fringe.isEmpty() is False:
        cur = fringe.pop()
        # print("cur:", cur.state)
        visited.add(cur.state)
        if problem.isGoalState(cur.state):
            break
        successors = problem.getSuccessors(cur.state)
        # print("successors:", successors)
        for successor in successors:
            if successor[0] not in visited:
                node = childNode(successor, cur)
                fringe.push(node)
    # path = []
    actions = []
    while cur != None:
        # path.append(cur.state)
        if cur.action != None:
            actions.append(cur.action)
        #print("path cur:", cur.state)
        cur = cur.parent

    # path.reverse()
    actions.reverse()
    # print("path", path)
    # print("actions", actions)
    return actions

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    fringe = util.Queue()
    state = problem.getStartState()
    cur = childNode((state, None, None), None)
    fringe.push(cur)
    visited = []
    while fringe.isEmpty() is False:
        cur = fringe.pop()
        # print("cur.state:", cur.state)
        if cur.state in visited:
            continue
        visited.append(cur.state)
        if problem.isGoalState(cur.state):
            break
        successors = problem.getSuccessors(cur.state)
        for successor in successors:
            # print("successor 2:", successor)
            if successor[0] not in visited:
                node = childNode(successor, cur)
                fringe.push(node)

    actions = []
    while cur != None:
        if cur.action != None:
            actions.append(cur.action)
        cur = cur.parent

    actions.reverse()
    # print("actions", actions)
    return actions

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    fringe = util.PriorityQueue()
    state = problem.getStartState()
    cur = childNode((state, None, None), None)
    cost = 0
    fringe.push(cur, cost)
    visited = set()
    while fringe.isEmpty() is False:
        cur = fringe.pop()
        if cur.state in visited:
            continue
        visited.add(cur.state)
        if problem.isGoalState(cur.state):
            break
        successors = problem.getSuccessors(cur.state)
        for successor in successors:
            if successor[0] not in visited:
                if cur.cost == None:
                    cost = successor[2]
                else:
                    cost = cur.cost + successor[2]
                newSuccessor = (successor[0], successor[1], cost)
                node = childNode(newSuccessor, cur)
                fringe.push(node, cost)

    actions = []
    while cur != None:
        if cur.action != None:
            actions.append(cur.action)
        cur = cur.parent

    actions.reverse()
    # print("actions", actions)
    return actions

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
"================== A* Search Here ==================="
""" A* takes a heuristic function as an argument. 
    Heuristics take two arguments: a state in the search problem (the main argument), and the problem itself (for reference information).
    You should see that A* finds the optimal solution slightly faster than uniform cost search (about 549 vs. 620 search nodes expanded in our implementation, but ties in priority may make your numbers differ slightly).
    """
    
def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState() # initialize the start
    explored_list = [(start_state, [], 0)] # initialize list of previously explored spots
    
    explored = []
    
    while explored_list:
        current_state, actions, cost_so_far = explored_list.pop(0)
        #print("current_state:", current_state, " cost_so_far:", cost_so_far)
        if problem.isGoalState(current_state):
            return actions
        
        if current_state not in explored:
            explored.append(current_state)
            successors = problem.getSuccessors(current_state)
            for successor, action, step_cost in successors:
                new_cost = cost_so_far + step_cost
                #heuristic_cost = new_cost + heuristic(successor, problem)
                new_entry = (successor, actions + [action], new_cost)
                explored_list.append(new_entry)
                explored_list.sort(key=lambda x: x[2] + heuristic(x[0], problem))

    return []  # No solution found

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
