# valueIterationAgents.py
# -----------------------
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


# valueIterationAgents.py
# -----------------------
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


import mdp, util

from learningAgents import ValueEstimationAgent
import collections

class ValueIterationAgent(ValueEstimationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A ValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100):
        """
          Your value iteration agent should take an mdp on
          construction, run the indicated number of iterations
          and then act according to the resulting policy.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state, action, nextState)
              mdp.isTerminal(state)
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0
        self.runValueIteration()

    def runValueIteration(self):
        # Write value iteration code here
        "*** YOUR CODE HERE ***"
        # all states of game
        states = self.mdp.getStates()

        # In each iteration, for each state, update values.
        for iteration in range(0,self.iterations):
            iterVals = util.Counter()
            for state in states:
                action = self.getAction(state) # Select the best action
                if action is not None:
                    iterVals[state] = self.getQValue(state, action)
            self.values = iterVals

    def getValue(self, state):
        """
          Return the value of the state (computed in __init__).
        """
        return self.values[state]


    def computeQValueFromValues(self, state, action):
        """
          Compute the Q-value of action in state from the
          value function stored in self.values.
        """
        "*** YOUR CODE HERE ***"
        """
          Q(s,a) = sum_{s'}{T(s,a,s')[R(s,a,s')+a.V_k(s)]}
        """
        sum = 0
        transStatesAndProbs = self.mdp.getTransitionStatesAndProbs(state, action)
        for tsAndP in transStatesAndProbs:
            nextState = tsAndP[0]
            prob = tsAndP[1]
            nextState = tsAndP[0]
            reward = self.mdp.getReward(state, action, nextState)
            val = self.getValue(nextState)
            sum += prob * (reward + self.discount * val)
        return sum

    def computeActionFromValues(self, state):
        """
          The policy is the best action in the given state
          according to the values currently stored in self.values.

          You may break ties any way you see fit.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return None.
        """
        "*** YOUR CODE HERE ***"
        if self.mdp.isTerminal(state):
            return None

        actions = self.mdp.getPossibleActions(state)
        if len(actions) == 0:
            return None

        maxVal = float("-inf")
        maxValAction = None
        for action in actions:
            val = self.getQValue(state, action)
            if maxVal < val:
                maxVal = val
                maxValAction = action
        return maxValAction

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)

class AsynchronousValueIterationAgent(ValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        An AsynchronousValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs cyclic value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 1000):
        """
          Your cyclic value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy. Each iteration
          updates the value of only one state, which cycles through
          the states list. If the chosen state is terminal, nothing
          happens in that iteration.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state)
              mdp.isTerminal(state)
        """
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def runValueIteration(self):
        "*** YOUR CODE HERE ***"
        states = self.mdp.getStates()
        statesNum = len(states)
        iterationCnt = 0
        # In each iteration, for each state, update values.
        for iteration in range(0,self.iterations):
            state = states[iterationCnt % statesNum]
            action = self.getAction(state) # Select the best action
            if action is not None:
                self.values[state] = self.getQValue(state, action)
            iterationCnt += 1

class PrioritizedSweepingValueIterationAgent(AsynchronousValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A PrioritizedSweepingValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs prioritized sweeping value iteration
        for a given number of iterations using the supplied parameters.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100, theta = 1e-5):
        """
          Your prioritized sweeping value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy.
        """
        self.theta = theta
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def runValueIteration(self):
        "*** YOUR CODE HERE ***"
        # predecessors contain key value pair, key is the next state,
        # value is the previous states list to achieve the next state
        predecessors = {}
        states = self.mdp.getStates()
        for state in states:
            if self.mdp.isTerminal(state):
                continue
            for action in self.mdp.getPossibleActions(state):
                for nState, prob in self.mdp.getTransitionStatesAndProbs(state, action):
                    if nState in predecessors:
                        predecessors[nState].add(state)
                    else:
                        predecessors[nState] = {state}

        pq = util.PriorityQueue()
        for state in states:
            if self.mdp.isTerminal(state):
                continue
            maxQVal = float('-inf')
            for action in self.mdp.getPossibleActions(state):
                qVal = self.computeQValueFromValues(state, action)
                if maxQVal < qVal:
                    maxQVal = qVal
            diff = abs(maxQVal - self.values[state])
            pq.update(state, -diff)

        for iteration in range(self.iterations):
            if pq.isEmpty():
                break
            s = pq.pop()
            if self.mdp.isTerminal(s) is False:
                maxQVal = float('-inf')
                for action in self.mdp.getPossibleActions(s):
                    qVal = self.computeQValueFromValues(s, action)
                    if qVal > maxQVal:
                        maxQVal = qVal
                self.values[s] = maxQVal

            for p in predecessors[s]:
                if self.mdp.isTerminal(p):
                    continue
                maxQVal = float('-inf')
                for action in self.mdp.getPossibleActions(p):
                    qVal = self.computeQValueFromValues(p, action)
                    if qVal > maxQVal:
                        maxQVal = qVal
                diff = abs(maxQVal - self.values[p])
                if diff > self.theta:
                    pq.update(p, -diff)