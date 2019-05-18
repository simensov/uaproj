import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


class MyPendulum:

	def __init__(self,
		mass,
		length,
		damping,
		start_theta,
		start_theta_d,
		delta_t = 0.0,
		input_limit = 1.0,
		g=9.81):
		self.m = mass
		self.l = length
		self.b = damping
		self.g = g
		self.theta = start_theta
		self.theta_d = start_theta_d
		self.input_limit = input_limit
		self.delta_t = delta_t
		self.u = 0.

	def calcDerivatives(self,u):
		theta_dd = (1/(self.m*self.l**2))*(u - self.m*self.g*self.l*np.sin(self.theta) - self.b*self.theta_d)
		return np.array([self.theta_d,theta_dd])

	def calcDerivs(self,theta,theta_d,u):
		theta_dd = (1/(self.m*self.l**2))*(u - self.m*self.g*self.l*np.sin(theta) -self.b*theta_d)
		return np.array([theta_d,theta_dd])

	def updateStates(self,u):
		self.u = boundInput(u)
		derivatives = calcDerivatives(self.u)
		updateTheta(self.theta + derivatives[0]*self.delta_t)
		self.theta_d += derivatives[1]*self.delta_t


	def boundInput(self,u):
		if u > self.input_limit:
			u = self.input_limit
		if u < -self.input_limit:
			u = - self.input_limit
		return u

	def getPos(self):
		return np.array([self.theta])

	def getThetaD(self):
		return np.array([self.theta_d])

	def getInput(self):
		return self.u

	def updateTheta(self,theta):
		if theta > 2*np.pi:
			theta -= 2*np.pi
		if theta < 0:
			theta += 2*np.pi
		self.theta = theta

	def printState(self):
		print("Theta: ",self.theta)
		print("Theta_d: ",self.theta_d)

