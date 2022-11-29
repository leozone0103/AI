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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    deque = util.Stack()
    actions = {} # state -> list of actions of how we get to that state from the starting position
    actions[problem.getStartState()] = list()
    visited = set()
    deque.push(problem.getStartState())
    while (not deque.isEmpty()):
        current_state = deque.pop()
        if (current_state not in visited):
            visited.add(current_state)
            if (problem.isGoalState(current_state)):
                return actions[current_state]
            children = problem.getSuccessors(current_state)    
            for (child_state, action, cost) in children:
                # update children's paths
                if (child_state not in visited):
                    actions[child_state] = list(actions[current_state])
                    actions[child_state].append(action)
                    deque.push(child_state)
        
    return []

    

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # same logic as DFS besides changing a stack to a queue
    deque = util.Queue()
    actions = {}
    actions[problem.getStartState()] = list()
    visited = set()
    deque.push(problem.getStartState())
    while (not deque.isEmpty()):
        current_state = deque.pop()
        if (current_state not in visited):
            visited.add(current_state)
            if (problem.isGoalState(current_state)):
                return actions[current_state]
            children = problem.getSuccessors(current_state)
            for (child_state, action, cost) in children:
                if (child_state not in actions.keys()):
                    actions[child_state] = list(actions[current_state])
                    actions[child_state].append(action)
                    deque.push(child_state)

    return []               


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    deque = util.PriorityQueue()
    deque.push(problem.getStartState(), 0)

    actions = {}
    actions[problem.getStartState()] = [[],0]

    visited = set()
    while (not deque.isEmpty()):
        current_state = deque.pop()
        if (problem.isGoalState(current_state)):
            return actions[current_state][0]
        if (current_state not in visited):
            visited.add(current_state)
            children = problem.getSuccessors(current_state)
            for (child_state, action, cost) in children:
                if (child_state in actions.keys()):
                    if (actions[child_state][1] > cost + actions[current_state][1]):
                        actions[child_state][0] = list(actions[current_state][0])
                        actions[child_state][0].append(action)
                        actions[child_state][1] = actions[current_state][1] + cost
                        deque.update(child_state,actions[child_state][1])
                else:
                    actions[child_state]=[actions[current_state][0] + [action], actions[current_state][1] + cost]
                    deque.push(child_state, actions[child_state][1])
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

    deque = util.PriorityQueue()
    deque.push(problem.getStartState(),heuristic(problem.getStartState(),problem))

    actions={}
    actions[problem.getStartState()]=[[],0]

    visited=set()
    while (not deque.isEmpty()):
        current_state = deque.pop()
        if problem.isGoalState(current_state):
            return actions[current_state][0]
        if current_state in visited:
            continue
        visited.add(current_state)
        children = problem.getSuccessors(current_state)
        for (child_state, action, cost) in children:
            if (child_state in actions.keys()):
                if (actions[child_state][1] > cost + actions[current_state][1]):
                    actions[child_state][0] = list(actions[current_state][0])
                    actions[child_state][0].append(action)
                    actions[child_state][1] = actions[current_state][1] + cost
                    deque.update(child_state,actions[child_state][1])
            else:
                actions[child_state]=[actions[current_state][0] + [action], actions[current_state][1] + cost]
                deque.push(child_state, heuristic(child_state, problem) + actions[child_state][1])
    
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
