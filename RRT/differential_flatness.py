# differential_flatness.py - Simen Sem Oevereng
# Builds on example written by Russ Tedrake! Using tools from pydrake (also by Russ Tedrake)

# This file will not run in this folder, but is added to give an example

import matplotlib.pyplot as plt
import math
import numpy as np
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.trajectories import PiecewisePolynomial
from quadrotor2d import Quadrotor2D, Quadrotor2DVisualizer

from scipy.interpolate import make_interp_spline, BSpline
from scipy import interpolate

from RRT_py2 import * # rrt() and plotGoalPath()
from support_quadcopter import * # mainly different print statements
import time

# practical booleans for printing when doing wuick checks
plotDrone = True
plotPath = True
plotObstacles = True

###
### Main code, building on 
### Combining it with my own RRT implementations atm
###

class PPTrajectory():
    def __init__(self, sample_times, num_vars, degree, continuity_degree):
        self.sample_times = sample_times
        self.n = num_vars
        self.degree = degree

        self.prog = MathematicalProgram()
        self.coeffs = []
        for i in range(len(sample_times)):
            self.coeffs.append(self.prog.NewContinuousVariables(
                num_vars, degree+1, "C"))
        self.result = None

        # Add continuity constraints
        for s in range(len(sample_times)-1):
            trel = sample_times[s+1]-sample_times[s]
            coeffs = self.coeffs[s]
            for var in range(self.n):
                for deg in range(continuity_degree+1):
                    # Don't use eval here, because I want left and right
                    # values of the same time
                    left_val = 0
                    for d in range(deg, self.degree+1):
                        left_val += coeffs[var, d]*np.power(trel, d-deg) * \
                               math.factorial(d)/math.factorial(d-deg)
                    right_val = self.coeffs[s+1][var, deg]*math.factorial(deg)
                    self.prog.AddLinearConstraint(left_val == right_val)

        # Add cost to minimize highest order terms
        for s in range(len(sample_times)-1):
            self.prog.AddQuadraticCost(np.eye(num_vars),
                                       np.zeros((num_vars, 1)),
                                       self.coeffs[s][:, -1])

    def eval(self, t, derivative_order=0):
        if derivative_order > self.degree:
            return 0

        s = 0
        while s < len(self.sample_times)-1 and t >= self.sample_times[s+1]:
            s += 1
        trel = t - self.sample_times[s]

        if self.result is None:
            coeffs = self.coeffs[s]
        else:
            coeffs = self.result.GetSolution(self.coeffs[s])

        deg = derivative_order
        val = 0*coeffs[:, 0]
        for var in range(self.n):
            for d in range(deg, self.degree+1):
                val[var] += coeffs[var, d]*np.power(trel, d-deg) * \
                       math.factorial(d)/math.factorial(d-deg)

        return val

    def add_constraint(self, t, derivative_order, lb, ub=None):
        '''
        Adds a constraint of the form d^deg lb <= x(t) / dt^deg <= ub
        '''
        if ub is None:
            ub = lb

        assert(derivative_order <= self.degree)
        val = self.eval(t, derivative_order)
        self.prog.AddLinearConstraint(val, lb, ub)

    def generate(self):
        self.result = Solve(self.prog)
        assert(self.result.is_success())


#########################################
#########################################
# MY CONCATENATION OF RRT AND DIFF FLAT #
#########################################
#########################################

environment = Environment('env_superbug.yaml')

radius = 0.6
bounds = (-2, -3, 12, 8) # this is (minx, miny, maxX,maxY)
start = (0, 0)
x = 10.5 # x pos of goal
y = 5.5  # y pos of goal
d = 0.5  # half side length of goal region square
goal_reg = [(x-d,y-d), (x-d,y+d), (x+d,y+d), (x+d,y-d)]
goal_region = Polygon(goal_reg)

print("Finding path with RRT...")
# checking with big radius atm. Not changing var radius itself due to the plotting further down!
stime = time.time()
path = rrt(bounds, environment, start, 2*radius, goal_region)
print "...Done after", '%.4f' % (time.time()-stime), "seconds!"
startpos = [start[0], start[1]]
endpos = [x,y]
given_path = [start] ## path given to quadcopter

drone = Quadrotor2D()

#################
################# Starting
#################

if(True):
    tf = len(path)
    dots = 5*tf
    zpp = PPTrajectory(sample_times=np.linspace(0, tf, dots), num_vars=2,
                       degree=5, continuity_degree=4)

    if True:
        k = 0
        t = zpp.prog.NewContinuousVariables(1, "t_%d" % k)
        t_over_time = t

        # add time variables from t1 to tf-1. t0 shall be zero and t_f should be constrained
        for k in range(1,tf):
            t = zpp.prog.NewContinuousVariables(1, "t_%d" % k)
            t_over_time = np.vstack((t_over_time, t))

        ### Add constraint on t_0 = 0. Or set t=0 in add_constraint?
        ### Add constraint on t_i-1 <= t_i. "<=" allows the path to be done early
        ### Constrain final time instance to be == tf
        # zpp.prog.AddLinearConstraint(t_over_time[-1][0] == tf)
        # --- OR ---
        # Constrain final time instance to be <= tf, and add cost on it
        zpp.prog.AddLinearConstraint(t_over_time[0][0] == 0)
        
        timeslack = 0.01
        for i in range(0,tf-1):
            zpp.prog.AddLinearConstraint(t_over_time[i][0] <=\
                                         t_over_time[i+1][0] + timeslack)

        zpp.prog.AddLinearConstraint(t_over_time[-1][0] <= tf)
        zpp.prog.AddQuadraticCost( (t_over_time[-1,0])**2 )


    ### Add constraints on initial, visiting and end states + derivatices
    zpp.add_constraint(t=0, derivative_order=0, lb=startpos)
    zpp.add_constraint(t=0, derivative_order=1, lb=[0, 0])
    zpp.add_constraint(t=0, derivative_order=2, lb=[0, 0])
    
    ## Add all spacial constraints along path
    # Start at 1 since t = 0 is set. Give every second point (to avoid super strict path)
    for i in range(1,tf,2): 
        if(i != tf):
            pos = path[i] # a tuple
            given_path.append(pos)
            zpp.add_constraint(t=i, derivative_order=0, lb=[pos[0], pos[1]])

    zpp.add_constraint(t=tf, derivative_order=0, lb=endpos)
    zpp.add_constraint(t=tf, derivative_order=1, lb=[0, 0])
    zpp.add_constraint(t=tf, derivative_order=2, lb=[0, 0])

    print "Solving differential flatness..."
    stime = time.time()
    zpp.generate()
    print "...Done after", '%.4f' % (time.time()-stime), "seconds!"

    # Plot quadcopter and trajectory
    num_visualize = int(dots/3)+1
    num_sample = int(dots)+1
    dt = tf / float(num_sample)

    ###
    ### Extract u
    ###
    # Method one: fully mathematical according to the one that Russ started explaining about. Fully deriving thetaddot
    # -- simplified thetadot**2 * tan(theta) to be zero (both velocity and angle is small, so it is almost (small number)***3, which disappears)
    g = drone.gravity
    I = drone.inertia
    r = drone.length 
    m = drone.mass 
    u = []
    ts = []
    u1s = []
    u2s = []
    xplot = []
    yplot = []
    for t in np.linspace(0, tf, (num_sample)):
        x       = zpp.eval(t)[0]
        y       = zpp.eval(t)[1]
        xdot    = zpp.eval(t, 1)[0]
        ydot    = zpp.eval(t, 1)[1]
        xddot   = zpp.eval(t, 2)[0]
        yddot   = zpp.eval(t, 2)[1]
        xdddot  = zpp.eval(t, 3)[0]
        ydddot  = zpp.eval(t, 3)[1]
        xddddot = zpp.eval(t, 4)[0]
        yddddot = zpp.eval(t, 4)[1]
        theta   = np.arctan2(-xddot, (yddot + g))
        xplot.append(x)
        yplot.append(y)  
        # parts calculated from chapter 3
        part0 = yddot + g
        part1 = -xddddot * ( part0 ) + xddot*yddddot
        part2 = 2 * ydddot * ( xddot*ydddot - xdddot*part0)
        ## Adding together, trying to avoid dividing by zero
        rhs = (part1 * part0 - part2) / (part0 + 10**-3)**3
        thetaddot = np.cos(theta)**2 * rhs # -2 * thetadot * tan(theta) ignored

        # using equation for m * yddot since the angles are gonna be far from cos(theta) = 0. alternative, and more flexible, could be to change the procedure depending on if theta is close or far from 0 or pi

        u1 = 0.5 * ( I / r * thetaddot + ((m * part0) / np.cos(theta)) )
        u2 = ((m * part0)/np.cos(theta)) - u1
        u.append(u1+u2)
        u1s.append(u1)
        u2s.append(u2)
        ts.append(t)
        # print printStates(x,y), "\t", printStates(xdot,ydot), "\t" , printInputs(u1,u2)

    print "Fuel:", calculateFuel(u,dt)

    ###
    ### Russ' way of plotting the Quadcopter
    ###
    fig, ax = plt.subplots()

    if plotDrone:
      for t in np.linspace(0, tf, (num_visualize)):
          x = zpp.eval(t)
          xddot = zpp.eval(t, 2)
          theta = np.arctan2(-xddot[0], (xddot[1] + 9.81))
          v = Quadrotor2DVisualizer(ax=ax)
          context = v.CreateDefaultContext()
          context.FixInputPort(0, [x[0], x[1], theta, 0, 0, 0])
          v.draw(context)

    # Draw the actual obstacles from given environment on quadcopter plot
    if plotObstacles:
      for pt in environment.obstPoints:
          a = []; b = []
          for lt in pt:
              a.append(lt[0])
              b.append(lt[1])
          ax.fill(a, b,facecolor='darkred', edgecolor='k') 

    # finish 
    ax.set_xlim([bounds[0], bounds[2]])
    ax.set_ylim([bounds[1], bounds[3]])
    ax.set_xlabel('x')
    ax.set_ylabel('z')
    ax.set_title('Trajectory found from RRT + differential flatness')  

######################################### Finish up with plotting etc

# This is a method for normalizing distances between the points that was delegated to the trajectory. Remember that ish half of the points are currently given due to the fact that time step is 1. If the entire goal path could be given, but with more clever distribution of timestep constraints, then a pretty poor RRT could work well with differential flatness!
dist = np.array([np.sqrt( (given_path[i+1][0]-given_path[i][0])**2) + (given_path[i+1][1]-given_path[i][1])**2 for i in range(len(given_path) - 1)])

normalizedDist = (dist / np.linalg.norm(dist)).reshape(dist.shape[0],1)

fig, ax1 = plt.subplots()
ax1.plot(ts,u1s)
ax1.plot(ts,u2s)

if plotPath:
    given_path = zip(xplot,yplot) # alternative for smoother plotting!
    plotGoalPath(given_path,radius*0.8,ax,goal_region)
    ax.plot(xplot,yplot)

fullscreen = False
if fullscreen:
    # this scales a bit weird, but it is nice to have fullscreen popping up
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())


if plotDrone or plotPath or plotObstacles:
    plt.show()
    # plt.savefig('trajectory.png', dpi=fig.dpi) # just stores a white picture
