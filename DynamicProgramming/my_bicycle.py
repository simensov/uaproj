# my_bicycle.py - Oliver Stugard Os

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


class MyBicycle:

	def __init__(self,
		mass = 2.5,
		length = 0.25,
		inertia = 0.21666,
		vel = 0.5):
		self.v = vel
		self.m = mass
		self.l = length
		self.I = inertia
		self.r = self.l/3.

	def calcKinDerivatives(self,state,u):
		vx = self.v*np.cos(state[2])
		vy = self.v*np.sin(state[2])
		vt = (self.v/self.l)*np.tan(u)
		return vx,vy,vt

    
