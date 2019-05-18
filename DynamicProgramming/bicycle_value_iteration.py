import math
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from my_bicycle import MyBicycle
import csv


class DynamicProgramming:

	def __init__(self,goal,input_grid,state_grid,delta_t,system,obstacles = None):
		#The input parameters
		self.goal = goal
		self.input_grid = input_grid
		self.state_grid = state_grid
		self.delta_t = delta_t
		self.system = system
		self.obstacles = obstacles

		#Amount of states the different grids
		self.num_states = len(state_grid)
		self.x_grid = state_grid[0]
		self.y_grid = state_grid[1]
		self.th_grid = state_grid[2]

		#Defining the amount of states in each coordinate, and the number of actions
		self.num_x = len(x_grid)
		self.num_y = len(y_grid)
		self.num_th = len(th_grid)
		self.num_a = len(input_grid) + 1

		#Maximum and minimum values for the states
		self.max_x = self.x_grid[-1]
		self.min_x = self.x_grid[0]
		self.max_y = self.y_grid[-1]
		self.min_y = self.y_grid[0]
		self.max_th = self.th_grid[-1]
		self.min_th = self.th_grid[0]

		#Initializes the cost table with minimum time cost. All states that is not the goal has cost 1, goal has cost 0
		self.cost_table = self.initializeTimeCost()

		#Writes to csv
		w = csv.writer(open("bicycle_cost_table.csv","w"))
		for key,val in self.cost_table.items():
			w.writerow([key,val])

		print("Cost table done!")

		#Initializes the lookup table for cost to go with 0 in all states
		self.J = {}
		for i in range(self.num_x):
			for j in range(self.num_y):
				for k in range(self.num_th):
					self.J[(i,j,k)] = 0.
		print("J done")

		#This table transforms from grid indices to actual coordinates. Useful utility function
		self.state_table = {}
		for i in range(self.num_x):
			x = self.x_grid[i]

			for j in range(self.num_y):
				y = self.y_grid[j]

				for k in range(self.num_th):
					th = self.th_grid[k]
					if (i == 50 and j == 50 and k == 0):
						print("State: ",x,y,th)
					self.state_table[(i,j,k)] = (x,y,th)

		print("States done")

		#Transition table. If you are in a given state and take action a, it returns the next state. Uses the system derivatives, and assumes them to be constant for delta t in the computation. Also rounds to fit the grid.
		self.transition_table = {}
		for i in range(self.num_x):
			print(i)
			x = self.x_grid[i]

			for j in range(self.num_y):
				y = self.y_grid[j]

				for k in range(self.num_th):
					th = self.th_grid[k]

					for a in range(self.num_a):
						if (a == self.num_a -1):
							self.transition_table[(i,j,k,a)] = (i,j,k)
						else:
							u = self.input_grid[a]
							vx,vy,vt = system.calcKinDerivatives([x,y,th],u)
							self.transition_table[(i,j,k,a)] = (self.getNextStateIndex([x,y,th],[vx,vy,vt]))
		print("Initialited")

	#The key function of the dynamic programming. Loop through for the amount of specified iterations, and use the Bellman equation to update the cost to go function.
	def computeOptimalCost(self,iters):

		for it in range(iters):
			J_curr = self.J.copy()
			if (it % 10 == 0):
				print("Iters: ",it)

			for i in range(self.num_x):
				for j in range(self.num_y):
					for k in range(self.num_th):
						update_val = 100000.
						for a in range(self.num_a):
							x,y,th = self.transition_table[(i,j,k,a)]
							cost = self.cost_table[(i,j,k)]
							J_value = J_curr[(x,y,th)]

							#This is the crutial step. Find the action that minimizes the cost at the current state + the cost to go at the next state, and update the cost function accordingly
							if (cost + J_value < update_val):
								update_val = cost + J_value
						self.J[(i,j,k)] = update_val
		print("Done!")
		return self.J


	#Used to calculate the transition table. Takes in a state and derivatives, and assumes them to be constant for delta t. Returns the next state
	def getNextStateIndex(self,state,derivs):
		next_x,next_y,next_th  = self.wrapStates(state[0] + self.delta_t*derivs[0], state[1] + self.delta_t*derivs[1],state[2] + self.delta_t*derivs[2])
		x_index,y_index,th_index  = self.getIndexes(next_x,next_y,next_th)
		return x_index,y_index,th_index


	#Utility function to wrap the states within maximum and minimum and wrap theta.
	def wrapStates(self,x,y,th):
		if x > self.max_x:
			x = self.max_x
		if x < self.min_x:
			x = self.min_x
		if y > self.max_y:
			y = self.max_y
		if y < self.min_y:
			y = self.min_y
		if th > self.max_th:
			th -= 2*np.pi
		if th < self.min_th:
			th += 2*np.pi
		return x,y,th

	#Utility function to get the grid indices of a given coordinate position
	def getIndexes(self,x,y,th):
		x_index = np.abs(np.array(self.x_grid) - x).argmin()
		y_index = np.abs(np.array(self.y_grid) - y).argmin()
		th_index = np.abs(np.array(self.th_grid) - th).argmin()
		return x_index,y_index,th_index

	def getOptimalPolicy(self,state):
		value = 1000000
		policy = 1
		for a in range(self.num_a):
			x,y,th = self.transition_table[(state[0],state[1],state[2],a)]
			cost = self.cost_table[(x,y,th)]
			J_value = self.J[(x,y,th)]
			if J_value + cost < value:
				value = J_value + cost
				next_state = [x,y,th]
				policy = a
		return next_state,policy,value

	def getOptimalConstrolSequence(self,init_state):
		state = self.getIndexes(init_state[0],init_state[1],init_state[2])
		control_sequence = []
		state_sequence = []
		counter = 0
		while not (self.cost_table[(state[0],state[1],state[2])] == 0):
			state,policy,value = self.getOptimalPolicy(state)
			state_real = self.state_table[(state[0],state[1],state[2])]
			print(state_real,policy,value)
			control_sequence.append(policy)
			state_sequence.append(state_real)
			counter += 1
			if (counter > 1000):
				return None
		return control_sequence, state_sequence


	def initializeTimeCost(self):
		self.costs = {}
		for i in range(len(self.x_grid)):
			x = self.x_grid[i]

			for j in range(len(self.y_grid)):
				y = self.y_grid[j]

				for k in range(len(self.th_grid)):
					th = self.th_grid[k]

					if self.goalCheck([x,y,th]):
						self.costs[(i,j,k)] = 0
					else:
						self.costs[(i,j,k)] = 1
		return self.costs

	#This function checks if a state is a goal state or not
	def goalCheck(self,state):
		x = state[0]
		y = state[1]
		if ((self.goal[0] <= x <= self.goal[1]) and (self.goal[2] <= y <= self.goal[3])):
			return True
		return False

	#Checks if a state is colliding with a obstacle or not
	def obstacleCheck(self,state):
		x = state[0]
		y = state[1]
		for ob in self.obstacles:
			if (ob[0] <= x <= ob[1] and ob[2] <= y <= ob[3]):
				return True
		return False

	#Getters
	def getCostFunction(self):
		return self.J

	def getCostTable(self):
		return self.cost_table

	def getTransitionTable(self):
		return self.transition_table

	def getStateTable(self):
		return self.state_table

#min x, max x, min y, max y


#The final calculation of the look-up tables, including the cost-to-go function.
goal = [-0.0,0.0,-0.0,0.0]
input_grid = np.linspace(-np.pi/20.,np.pi/20.,5).tolist()
x_grid = np.linspace(-10,10,201).tolist()
y_grid = np.linspace(-10,10,201).tolist()
th_grid = np.linspace(0.,2*np.pi,61).tolist()
state_grid = [x_grid,y_grid,th_grid]
delta_t = 0.5
system = MyBicycle()

bicycle = DynamicProgramming(goal,input_grid,state_grid,delta_t,system)
bicycle.computeOptimalCost(220)



#Write data to CSV
w = csv.writer(open("bicycle_optimal_cost.csv","w"))
for key,val in bicycle.getCostFunction().items():
	w.writerow([key,val])

w = csv.writer(open("bicycle_transition_table.csv","w"))
for key,val in bicycle.getTransitionTable().items():
	w.writerow([key,val])

w = csv.writer(open("bicycle_state_table.csv","w"))
for key,val in bicycle.getStateTable().items():
	w.writerow([key,val])

w = csv.writer(open("bicycle_cost_table.csv","w"))
for key,val in bicycle.getCostTable().items():
	w.writerow([key,val])

print("csv finished")
