# algorithm.py - Oliver Stugard Os

import math
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from my_bicycle import MyBicycle
import csv
import pickle
import random
from searchClasses import *


start_time = time.time()

print("Loading from Pickle files")

with open('optimal_cost.pickle', 'rb') as handle:
    optimal_cost = pickle.load(handle)

print("10 %")

with open('transition_table.pickle', 'rb') as handle:
    transition_table = pickle.load(handle)

print("50 %")

with open('state_table.pickle', 'rb') as handle:
    state_table = pickle.load(handle)

print("80 %")

with open('cost_table.pickle', 'rb') as handle:
    cost_table = pickle.load(handle)

print("Finished creating pickle files in ",time.time() - start_time, " seconds!")

raw_input("Press Enter to continue...")


debug = False

class Algorithm:

	def __init__(self,
		goal,
		start_pos,
		input_grid,
		state_grid,
		optimal_cost,
		transition_table,
		state_table,
		costs,
		obstacles = None):

		#Setting the parameters
		self.start_pos = start_pos
		self.input_grid = input_grid
		self.state_grid = state_grid
		self.J = optimal_cost
		self.transition_table = transition_table
		self.state_table = state_table
		self.cost_table = costs
		self.obstacles = obstacles
		
		#Defining the state grids
		self.x_grid = state_grid[0]
		self.y_grid = state_grid[1]
		self.th_grid = state_grid[2]

		#Defining the maximum and minimum x and y values
		self.x_min = self.x_grid[0]
		self.x_max = self.x_grid[-1]
		self.y_min = self.y_grid[0]
		self.y_max = self.y_grid[-1]

		#Defining the amount of states in the different grids
		self.num_x = len(self.x_grid)
		self.num_y = len(self.y_grid)
		self.num_th = len(self.th_grid)
		self.num_a = len(self.input_grid)

		#Defining the goal as a tuple, fit to the grid

		x_g,y_g,th_g = self.getIndexes(goal[0],goal[1],0)
		self.goal = self.state_table[(x_g,y_g,0)]	

		#Defining the indexes of the starting position
		x,y,th = self.getIndexes(start_pos[0],start_pos[1],start_pos[2])

		#Defining the cost to go for the starting posisition
		cost_to_go = self.J[(x,y,th)]

		#Defining the starting position as a root node of the three
		root = Node((start_pos[0],start_pos[1],start_pos[2]),0,cost_to_go,0)
		self.tree = Graph(self.goal,self.obstacles)

		#Adds the root node to the tree
		self.tree.add_node(root)


	#The complete algiorithm. Puts together all the utility functions
	def motionPlan(self,iters):

		#Check if there exist a path from the root to the goal
		if self.updateCostEstimates(self.tree.get_root()):

			#If it does, return the path, problem solved
			return self.checkGoalPath(self.tree.get_root().state)[1]

		#Used to calculate the time spent on the iterations
		start_time = time.time()

		#Otherwise, loop for a specified number of iterations
		for i in range(iters):
			if (i%50 == True):
				print("Iters:  ",i)
				t = time.time() - start_time
				print("Time :  ",t)

			#Expand the three with a random node. This node is added as a child of the node in the tree which has the shortes cost to go from itself to the randomly sapled tree. In addition, create a node in halfway in the trajectory added. This creates more feasible nodes in the tree, and gives more flexibility in the path planning.
			next_nodes = self.expandTree()

			if next_nodes is not False:

				#Loop through all the new added nodes in the tree
				for node in next_nodes:

					#Check if any of them can be connected to goal, and update the lower and upper bound of the total time to go from each node
					success = self.updateCostEstimates(node)
		self.time = time.time() - start_time

		#Return the final goal path
		return self.tree.getGoalPath()





	#This founction aims to expand the tree by adding a random node
	def expandTree(self):
		while (True):

			#Defining the indices in the grid of a random point
			x = random.randint(40,160)
			y = random.randint(40,160)
			th = 0.

			#Defining the values of the random point in our grid
			(xval,yval,thval) = self.state_table[(x,y,th)]

			#Looping through all the nodes in the tree. They are sorted in the way such that the node with the lowest cost-to-go to the random point is first in the list etc.
			for node in self.getRandomNodes([xval,yval]):

				#Defines the position of the node
				x,y,th = node.state
				#Defining the adjusted start state to get the optimal collitionfree controlsequence
				start_state = [x-xval, y - yval, th]
				control_sequence,state_sequence = self.getOptimalControlSequence(start_state)

				#Defining the control sequence in our coordinate system
				state_sequence = self.adjustStateSequence(state_sequence,[xval,yval])

				#Check if the path is collition free
				if self.checkAllCollition(state_sequence):

					#Split the sequence up in two, in order to add more feasible nodes to the tree. This makes the algorithm more efficient. 
					sequences = self.splitSequence(state_sequence,2.,0)
					control_sequences = self.splitSequence(control_sequence,2.,1)
					
					next_nodes = []

					#Loop through the sequences
					for seq in sequences:	

						last_state = seq[-1]

						if debug:
							print("Last state of state sequence: ")
							print(last_state)

						#Find the indexes of the last state in the sequence, which will be added to the tree
						x_i, y_i, th_i = self.getIndexes(last_state[0], last_state[1], last_state[2])

						#Find the indexes of the states, relative to the goal, in order to get a lower bound on the cost to go
						x_g,y_g,th_g = self.getIndexes(last_state[0] - self.goal[0], last_state[1] - self.goal[1], last_state[2])

						#Define the state coordinates, cost to go, the time and the amount of parents. 
						xval,yval,thval = self.state_table[(x_i,y_i,th_i)]
						cost_to_go = self.J[(x_g,y_g,th_g)]
						time = len(seq) - 1 + node.time
						num_parents = node.num_parents + 1

						#Create the node, and add it to the tree
						child = Node((xval,yval,thval),time,cost_to_go,num_parents,node)
						self.tree.add_edge(node,child,control_sequence,seq)
						next_nodes.append(child)
						node = child
					return next_nodes
		return False

	#Updates the cost estimates if a path to the goal is found. This is used in order to prune the tree, which significantly speeds up the algorithm
	def updateCostEstimates(self,node):

		#Checks if a path exist from the node to the goal
		sequence = self.checkGoalPath(node.state)
		if sequence is not False:

			#Define the goal node
			cost_to_go = 0.
			time = node.lower_bound
			num_parents = node.num_parents + 1
			last_state = sequence[-1][-1]
			goal = Node((self.goal[0],self.goal[1],last_state[2]),time,cost_to_go,num_parents,node)

			#Add the goal node to the tree
			self.tree.add_edge(node,goal,sequence[0],sequence[1])

			#If the goaltime is less than the upper bound of the tree, change the uppber bound
			if (goal.time < self.tree.upper_bound):
				self.tree.upper_bound = goal.time

			#Prune the tree from unnessesary nodes, by removing the all nodes that has a lower bound that is higher than the trees upper bound
			self.tree.prune()

			#If a path exists, the upper bound of cost to go is set to the same as the lower bound
			node.upper_bound = node.lower_bound

			#updates the upper bound of the parent nodes recursively
			while (node.parent != None):
				parent = node.parent
				if (parent.upper_bound < node.upper_bound + node.incoming_edge.cost):
					break
				parent.upper_bound = node.upper_bound + node.incoming_edge.cost
				node = parent
			return True

		#If no path is found to the goal, the upper bound of cost to go is set to infinity, and false is returned
		return False



	#Checks if it exists a direct path from the a given state to the goal
	def checkGoalPath(self,state):

		if debug:
			print("Checking goal path!")
			print("State: ",state)
			print("Goal:  ",goal)

		#Defining the position and angle of the vehicle relative to goal
		x = state[0] - self.goal[0] 
		y = state[1] - self.goal[1] 
		th = self.wrapTheta(state[2])

		#Tries to find a control sequence to the goal
		if self.getOptimalControlSequence([x,y,th]) is not None:


			#Define both the control and state sequence to the goal
			control_sequence,state_sequence = self.getOptimalControlSequence([x,y,th]
				)

			#Adjust the state sequence to fit our coordinate frame
			state_sequence = self.adjustStateSequence(state_sequence,[self.goal[0],self.goal[1]])

			#If a control sequence is found, return it, and check all the states for collition
			if (self.checkAllCollition(state_sequence)):
				if debug:
					print("Collision test was successful! ")
				#If no states collides, return the control sequence
				return control_sequence, state_sequence

		#Otherwise, no path is found between the current state and the goal
		return False


	#Uses the optimal cost to go in order to find the optimal control and state sequence from a given position to origo.
	def getOptimalControlSequence(self,start):
		state = self.getIndexes(start[0],start[1],start[2])
		control_sequence = []
		state_sequence = []
		state_sequence.append([start[0],start[1],start[2]])

		while not(self.cost_table[(state[0],state[1],state[2])] == 0):
			state,policy,value = self.getOptimalPolicy(state)
			state_real = self.state_table[(state[0],state[1],state[2])]
			control_sequence.append(policy)
			state_sequence.append(state_real)

		return control_sequence, state_sequence


	#This gives the optimal policy in a given state
	def getOptimalPolicy(self,state):
		value = 1000000
		policy = 1

		#loop over all actions
		for a in range(self.num_a):

			#Define the coordinates of the next state
			x,y,th = self.transition_table[(state[0],state[1],state[2],a)]

			#Define the cost of the next state
			cost = self.cost_table[(x,y,th)]

			#Define the cost to go of the next state
			J_value = self.J[(x,y,th)]

			#Adds a bias towards choosing go straight forward(corresponding to action 2)
			if J_value + cost < value or (J_value + cost == value and a == 2):
				value = J_value + cost
				next_state = [x,y,th]
				policy = a
		return next_state,policy,value


	#This function checks all states in a state sequence. If any of them collides, return false, if all states are collition free, return true
	def checkAllCollition(self,states):
		for state in states:
			for ob in self.obstacles:
			#If collitioncheck returns true, a collition has occured. Return false
				if self.checkCollition(state,ob):
					return False 
		return True


	#Checks if a state collides with an obstacle
	def checkCollition(self,state,ob):
		x = state[0]
		y = state[1]
		if (ob[0] <= x <= ob[1]) and (ob[2] <= y <= ob[3]):
			return True
		if (x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max):
			return True
		return False

	#Key function for efficitnecy of the algorithm. It sorts all the nodes in the tree using insertion sort, and returns a list with the nodes that has the lowest cost to go from itself to the next state first. This can be interpreted as an heuristic of what nodes to try to connect to first.
	def getSortedNodes(self,state):

		sorted_nodes = []
		values = []

		for node in self.tree.nodes:

			if debug:
				print("Nodestate: ",node.state)

			x_i,y_i,th_i = self.getIndexes(node.state[0] - state[0], node.state[1] - state[1], node.state[2])

			value = self.J[(x_i,y_i,th_i)]

			if len(sorted_nodes) == 0:
				sorted_nodes.append(node)
				values.append(value)

			else:
				for i in range(len(sorted_nodes)):
					if value < values[i]:
						values.insert(i,value)
						sorted_nodes.insert(i,node)
						break
		return sorted_nodes


	#This returns all the nodes in the tree, in a random ordering
	def getRandomNodes(self,state):
		if len(self.tree.nodes) > 0 and self.tree.nodes is not None:
			random.shuffle(self.tree.nodes)
		return self.tree.nodes


	#Utility functions

	#Returns the grid indices of a given point
	def getIndexes(self,x,y,th):
		x_index = np.abs(np.array(self.x_grid) - x).argmin()
		y_index = np.abs(np.array(self.y_grid) - y).argmin()
		th_index = np.abs(np.array(self.th_grid) - th).argmin()
		return x_index,y_index,th_index


	#Transforms a sequence of states by a certain amount change
	def adjustStateSequence(self,state_sequence,change):

		out_seq = []
		for state in state_sequence:
			theta = self.wrapTheta(state[2])
			curr_seq = [state[0] + change[0],state[1] + change[1],theta] 
			out_seq.append(curr_seq)
		return out_seq
 
  #Wrap theta between 0 and 2 pi
	def wrapTheta(self,theta):
		if theta > 2*np.pi:
			theta -= 2*np.pi
		if theta < 0:
			theta += 2*np.pi
		return theta

	#Plots a sequence of states
	def drawSequence(self,state_sequence):
		x_points = []
		y_points = []
		th_points = []

		for point in state_sequence:
			x_points.append(point[0])
			y_points.append(point[1])
			th_pointsappend(point[2])

		plt.plot(x_points,y_points)
		plt.show()


	#Splits a sequence in a given amount num different sequences. Make the sequences overlap, such that the last state of any sequence is the first state of the next sequence
	def splitSequence(self,seq,num,control):
		if len(seq) == 1:
			return [seq]
		else:
			avg = len(seq)/num
			out = []
			last = 0.0
			while last < len(seq):
				if last > 0 and control is 0:
					out.append(seq[int(last)-1:int(last + avg)])
				else:
					out.append(seq[int(last):int(last + avg)])
				last += avg
		return out


#Testing
goal = [4.,5.]
start_state_1 = [-3.,-3.,np.pi]
start_state_2 = [-3.,4.,0.]
x_grid = np.linspace(-10,10,201).tolist()
y_grid = np.linspace(-10,10,201).tolist()
th_grid = np.linspace(0.,2*np.pi,61).tolist()
state_grid = [x_grid,y_grid,th_grid]
input_grid = np.linspace(-np.pi/20.,np.pi/20,5).tolist()
obstacles_1 = [[-3.5,-2,-2,-2.5],[-3.5,-2,-4.5,-4],[-2.5,-2,-4,-2],[2.5,3,-1,10],[2.5,3,-10,-4]]
obstacles_2 = [[2,2.3,0,5]]


alg = Algorithm(goal,start_state_1,input_grid,state_grid,optimal_cost,transition_table,state_table,cost_table,obstacles_1)

iters = 5000

trajectory = alg.motionPlan(iters)

alg.tree.drawGoalPath(iters,alg.time)
print("Drawing tree!")
alg.tree.drawTree(iters)











