import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

class MyQuadrotor:

#Constructor: Mass, intertia, radius, starting pos, timestep, max_input_limit, maximum_input_rate
	def __init__(self,mass,
							 inertia,
							 radius,
							 x_start,
							 y_start,
							 theta_start,
							 delta_t = 0.1,
							 max_u = 10.0,
							 input_rate = 0.5,
							 max_x = 12.,
							 min_x = -2.,
							 max_y = 8.,
							 min_y = -3.,
							 min_theta = -np.pi/4,
							 max_theta = np.pi/4,
							 max_velocity = 3.,
							 max_theta_velocity = 0.2
							 ):
		
		self.g = 9.81
		self.mass = mass
		self.radius = radius
		self.inertia = inertia
		self.pos = np.array([x_start,y_start,theta_start])
		self.velocities = np.array([0.,0.,0.])
		self.delta_t = delta_t
		self.u = np.array([self.mass*self.g/2, self.mass*self.g/2])
		self.input_limit = max_u
		self.input_rate = input_rate

		self.max_pos = np.array([max_x,max_y,max_theta])
		self.min_pos = np.array([min_x, min_y, min_theta])
		self.max_vel = np.array([max_velocity,max_velocity,max_theta_velocity])
		

	def calcDerivatives(self,u1, u2):
		xdd = -((u1 + u2)/self.mass)*np.sin(self.pos[2])
		ydd = ((u1 + u2)/self.mass)*np.cos(self.pos[2]) - self.g
		thetadd = (self.radius/self.inertia)*(u1 - u2)
		acceleration = np.array([xdd,ydd,thetadd])
		return np.hstack((self.velocities,acceleration))

	def updateStates(self,du1,du2):
		du1,du2 = self.boundInputRate(du1,du2)
		self.u[0],self.u[1] = self.boundMaxInput(du1,du2)

		derivatives = self.calcDerivatives(self.u[0],self.u[1])
		self.updatePos(derivatives[0:3]*self.delta_t)
		self.updateVel(derivatives[3:6]*self.delta_t)	
		#self.pos += derivatives[0:3]*self.delta_t
		#self.velocities += derivatives[3:6]*self.delta_t

	def updatePos(self,vel):
		self.pos = np.where(self.pos + vel > self.max_pos, self.max_pos, self.pos + vel)
		self.pos = np.where(self.pos + vel < self.min_pos, self.min_pos, self.pos + vel)

	def updateVel(self,acc):
		self.velocities = np.where(self.velocities + acc > self.max_vel, self.max_vel, self.velocities + acc)
		self.velocities = np.where(self.velocities + acc < -self.max_vel, -self.max_vel,self.velocities + acc)

	def boundInputRate(self,du1,du2):
		if du1 > self.input_rate:
			du1 = self.input_rate
		if du1 < - self.input_rate:
			du1 = -self.input_rate

		if du2 > self.input_rate:
			du2 = self.input_rate
		if du2 < -self.input_rate:
			du2 = -self.input_rate

		return du1, du2

	def boundMaxInput(self,du1,du2):
		if self.u[0] + du1 > self.input_limit:
			du1 = self.input_limit - self.u[0]
		if self.u[0] + du1 < -self.input_limit:
			du1 = - self.u[0] - self.input_limit

		if self.u[1] + du2 > self.input_limit:
			du2 = self.input_limit - self.u[1]
		if self.u[1] + du2 < -self.input_limit:
			du2 = - self.u[1] - self.input_limit

		return self.u[0] + du1, self.u[1] + du2


	#Getters
	def getState(self):
		return np.hstack((self.pos,self.velocities))

	def getPos(self):
		return self.pos

	def getInput(self):
		return self.u

	def getMaxX(self):
		return self.max_pos[0]

	def getMaxY(self):
		return self.max_pos[1]

	def getMaxTheta(self):
		return self.max_pos[2]

	def getMinX(self):
		return self.min_pos[0]

	def getMinY(self):
		return self.min_pos[1]

	def getMinTheta(self):
		return self.min_pos[2]

	def getMaxXVel(self):
		return self.max_vel[0]

	def getMaxYVel(self):
		return self.max_vel[1]

	def getMaxThetaVel(self):
		return self.max_vel[2]

	def getMaxInputRate(self):
		return self.input_rate

	def getMaxInput(self):
		return self.input_limit

'''
TODO: Fix these functions!
'''

print(copter.getState())
print(copter.getInput())