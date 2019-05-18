import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from my_quadrotor import MyQuadrotor
from my_pendulum import MyPendulum

class DynamicProgramming:

    def __init__(self,goal,input_grid,state_grid,delta_t,system):
        self.goal = goal
        self.input_grid = input_grid
        self.state_grid = state_grid
        self.delta_t = delta_t
        self.system = system
        self.J = np.zeros(state_grid[0].shape)
        self.thetas = state_grid[0][0,:]
        self.thetas_d = state_grid[1][:,0]

        self.theta_min = self.thetas[0]
        self.theta_max = self.thetas[-1]
        self.theta_d_min = self.thetas_d[0]
        self.theta_d_max = self.thetas_d[-1]

        self.theta_values = 0
        self.theta_interval = state_grid[0][0][1] - state_grid[0][0][0]
        self.theta_d_interval = state_grid[1][1][0] - state_grid[1][0][0]
        self.u_interval = input_grid[1] - input_grid[0]



    def getTimeCost(self,state,u):
        vec = state - self.goal
        if vec.dot(vec) < 0.5:
            return 0
        return 1

    def getQuadraticCost(self,state,u):
        vec = state - self.goal
        return vec.dot(vec) + u**2

    def find_nearest(array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return array[idx]


    def computeOptimalCost(self,iters):
        n,m = self.state_grid[0].shape
        for num in range(iters):
            if (num % 10 == 0):
                print("Iters:  ",num)
            J_curr = np.copy(self.J)
            for i in range(n):
                for j in range(m):
                    update_value = 10000000
                    th = self.thetas[j]
                    th_d = self.thetas_d[i]
                    for a in range(self.input_grid.size):
                        u = self.input_grid[a]
                        J_value = self.getJValue(th,th_d,u,J_curr)
                        cost = self.getTimeCost(np.array([th,th_d]),u)
                        if J_value + cost < update_value:
                            update_value = J_value + cost
                    self.J[i,j] = update_value
        return self.J

    def getOptimalPolicy(self,state):
        min_val = 100000000
        control = 1
        for i in range(self.input_grid.size):
            u = self.input_grid[i]
            J_value = self.getJValue(state[0],state[1],u,self.J)
            cost = self.getTimeCost(state,u)
            if J_value + cost < min_val:
                min_val = J_value + cost
                control = u
        return control

    def getOptimalControlSequence(self,init_state):
        state = init_state
        controlSequence = []
        while (not self.goalCheck(state)):
            control = self.getOptimalPolicy(state)
            state = self.updateState(state,control)
            controlSequence.append(control)
        return controlSequence

    def goalCheck(self,state):
        value = (state - self.goal).dot(state - self.goal)
        print(value)
        if (value) <= 0.05:
            return True
        return False

    def getJValue(self,theta,theta_d,u,J_curr):

        derivs = self.system.calcDerivs(theta,theta_d,u)
        theta_next = self.wrapTheta(theta + theta_d*self.delta_t) 
        theta_d_next = self.wrapThetaD(theta_d + derivs[1]*self.delta_t) 
        #print("theta_d_next", theta_d_next)
        theta_index = int(round((theta_next - self.theta_min)/self.theta_interval))
        theta_d_index = int(round((theta_d_next - self.theta_d_min)/self.theta_d_interval))
        return J_curr[theta_d_index,theta_index]

    def updateState(self,state,u):
        derivs = self.system.calcDerivs(state[0],state[1],u)
        theta_next = self.wrapTheta(state[0] + state[1]*self.delta_t) 
        theta_d_next = self.wrapThetaD(state[1] + derivs[1]*self.delta_t) 
        theta_index = int(round((theta_next - self.theta_min)/self.theta_interval))
        theta_d_index = int(round((theta_d_next - self.theta_d_min)/self.theta_d_interval))
        return np.array([self.thetas[theta_index],self.thetas_d[theta_d_index]])

    def wrapTheta(self,theta):
        if theta > self.theta_max:
            theta = theta - self.theta_max
        if theta < self.theta_min:
            theta = theta + self.theta_max
        return theta

    def wrapThetaD(self,thetad):
        if thetad > self.theta_d_max:
            thetad = self.theta_d_max
        if thetad < self.theta_d_min:
            thetad = self.theta_d_min
        return thetad



pendulum = MyPendulum(0.1,1.,0.1,0.,0.)
input_grid = np.linspace(-1.,1.,9)
theta_bins = np.linspace(0.,2*np.pi,101)
theta_d_bins = np.linspace(-6.,6.,61)
grid = np.meshgrid(theta_bins,theta_d_bins)
goal = np.array([np.pi,0.])
delta_t = 0.04

dynProg = DynamicProgramming(goal,input_grid,grid,delta_t,pendulum)
J = dynProg.computeOptimalCost(200)
print(J[30,50])
print(J[30,:])
print(J[:,50])
sequence = dynProg.getOptimalControlSequence(np.array([0,0]))
print(sequence)




    #def backward():
"""
pendulum.printState()

print(grid[0][0,:])
print(grid[1][:,0])
print(grid[0][0][1] - grid[0][0][0])
print(grid[1][1][0] - grid[1][0][0])
cost_to_go = np.zeros((51,31))
"""






'''
COPTER STUFF, TO BE USED LATER
copter = MyQuadrotor(1.,1.,1.,0.,0.,0.)

input_1_grid = np.linspace(-copter.getMaxInputRate(),copter.getMaxInputRate(),5)
input_2_grid = np.linspace(-copter.getMaxInputRate(),copter.getMaxInputRate(),5)


theta_bins = np.linspace(copter.getMinTheta(),copter.getMaxTheta(),15)
theta_dot_bins = np.linspace(-copter.getMaxThetaVel(),copter.getMaxThetaVel(),21)

x_bins = np.linspace(copter.getMinX(),copter.getMaxX(),51)
x_dot_bins = np.linspace(-copter.getMaxXVel(), copter.getMaxXVel(), 31)
y_bins = np.linspace(copter.getMinY(),copter.getMaxY(),51)
y_dot_bins = np.linspace(-copter.getMaxYVel(), copter.getMaxYVel(), 31)
'''