import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import yaml
from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
from check_path import check_path

from support import *

class KinematicBicycle:
  def __init__(self,
               length   = 0.25,
               velocity = 0.5,
               state    = (0,0),
               theta    = 0,
               statedot = (0,0),
               thetadot = 0,
               delta    = 0,
               dt       = 0.5,
               max_turn = np.pi/4,
               ):
    self.L        = length
    self.v        = velocity
    self.state    = state
    self.theta    = theta
    self.statedot = statedot
    self.thetadot = thetadot
    self.delta    = delta
    self.dt       = dt
    self.max_turn = max_turn

    def __repr__():
      return 'Bike, (x=%s, y=%s, th=%s)' % (self.x, self.y, self.theta)


    def constrainSteeringAngle():
      '''
      Keeps steering angle within a certain max
      
      :param:     angle (float): angle to constrain
                  max _angle(float): maximum steering angle
      '''
      if self.delta   > self.max_turn:
        self.delta    = self.max_turn
      elif self.delta < -self.max_turn:
        self.delta    = -self.max_turn

    def setSteeringAngle(nextNode):

      frontWheel=(self.state[0] + self.L * np.cos(self.theta),\
                  self.state[1] + self.L * np.sin(self.theta))
      dx = nextNode[0] - frontWheel[0]
      dy = nextNode[1] - frontWheel[1]
      # arctan2 gives both positive and negative angle -> map to 0,2pi
      angle_to_next = np.mod(np.arctan2(dy,dx), 2 * np.pi) 
      rel_angle     = angle_to_next - th
      # enforce steering to within range RELATIVE to theta -> map to (-pi,pi)
      self.delta    = np.arctan2(np.sin(rel_angle), np.cos(rel_angle))
      constrainSteeringAngle() # max_steer is default

    def calcKinematicDerivs():
      # don't update theta here, but use new theta
      self.thetadot    = self.v / self.L * np.tan(self.delta)
      self.statedot[0] = self.v * np.cos(self.theta + self.thetadot * self.dt)
      self.statedot[1] = self.v * np.sin(self.theta + self.thetadot * self.dt)

    def updateStates():
      # Keep theta (0,2pi)
      self.theta    = np.mod(self.theta + self.thetadot * self.dt, 2*np.pi)
      self.state[0] = self.state[0] + self.statedot[0] * self.dt
      self.state[1] = self.state[1] + self.statedot[1] * self.dt

    def steer(nextNode):
      '''
      TODO: since delta is being stored, a more realistic implementation would be to constrain delta considering the steering angle already made!

      :params:    firstNode and nextNode is a tuple of x,y pos of back wheel
                  theta is a float of bicycle frame angle wrt. x axis

      '''
      # 1) Find the new delta parameter for steering wheel
      setSteeringAngle()

      # 2) Use that as input to kinematics. Keep theta (0,2pi)
      calcKinematicDerivs()
      updateStates()

      return self.state,self.theta