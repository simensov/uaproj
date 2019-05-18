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

class DynamicBicycle:
  def __init__(self,
               length   = 0.25,
               states   = [0,0,0,0,0], # x,y,th,vy,delta
               derivs   = [0,0,0,0,0], 
               inputs   = [0,0], # gas and turn torque
               dt       = 0.2,
               max_turn = np.pi/4,
               max_turn_rate = 0.25,
               ):

    self.L             = length
    self.q             = states
    self.qdot          = derivs
    self.dt            = dt
    self.max_turn      = max_turn # maximum turning angle
    self.max_turn_rate =  max_turn_rate

    def __repr__():
      return 'DynoBike, (x=%s, y=%s, th=%s)' % (self.x, self.y, self.theta)

    def constrainSteeringAngle():
      '''
      Keeps steering angle within a certain max
      
      :param:     angle (float): angle to constrain
                  max _angle(float): maximum steering angle
      '''
      if self.q[4]   > self.max_turn:
        self.q[4]    = self.max_turn
      elif self.q[4] < -self.max_turn:
        self.q[4]    = -self.max_turn

    def changeSteeringAngle(delta):

      # TODO: this max_turn_rate has to be 
      if delta > self.q[4]: # increase swing angle
        self.q[4] += self.max_turn_rate * self.dt

      elif delta < self.q[4]: # increase swing angle
        self.q[4] -= self.max_turn_rate * self.dt


    def setSteeringAngle(nextNode):

      frontWheel=(self.state[0] + self.L * np.cos(self.theta),\
                  self.state[1] + self.L * np.sin(self.theta))
      dx = nextNode[0] - frontWheel[0]
      dy = nextNode[1] - frontWheel[1]

      # arctan2 gives both positive and negative angle -> map to 0,2pi
      angle_to_next = np.mod(np.arctan2(dy,dx), 2 * np.pi) 
      rel_angle     = angle_to_next - th
      # enforce steering to within range RELATIVE to theta -> map to (-pi,pi)
      delta    = np.arctan2(np.sin(rel_angle), np.cos(rel_angle))

      changeSteeringAngle(delta) # change must be limited to the max_turn_angle
      constrainSteeringAngle() # max_steer is default

    def calcKinematicDerivs():

    def updateStates():
      # Keep theta (0,2pi)

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