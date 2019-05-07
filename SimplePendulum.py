import numpy as np
from numpy import sin,cos
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from support import * # SearchNode class for example
import time

import yaml
from shapely.geometry import Point, Polygon, LineString, box
from colorama import Fore, Style

#print(f'This is {Fore.GREEN} color {Style.RESET_ALL}!')

TIMESTEP = 0.05
GRAVITY  = 9.81
MAX_ITER = 5000
PLOTS = True

class SimplePendulum(SearchNode):

  def __init__(self,
    state       = (0,0),
    parent_node = None,
    cost        = 0.0,
    u           = 0,
    length      = 1,
    mass        = 1,
    max_torque  = 1,
    iteration   = 0,
    dt          = TIMESTEP,
    gravity     = GRAVITY,
    damping     = 1):

    self.iteration = iteration
    self.max_torque = max_torque
    self.dt = dt
    self.g = gravity
    self.b = damping

    super(SimplePendulum, self).__init__(state,parent_node,u,length,mass)

  ###
  ###
  ###
  def __repr__(self):
    return 'Pendulum (th=%s, thdot=%s)' % (self.state[0], self.state[1])
  
  ###
  ###
  ###
  def __hash__(self):
    return hash(self.state)

  ###
  ###
  ###
  def __gt__(self, other):
    return self._cost > other._cost
   
  ###
  ###
  ###
  def calcF(self,u):
    self.u = u # commented out since node is initialized anyways
    vel=self.state[1]
    # From [Branicky, 2002: 'Nonlinear and Hybrid Control via RRTs']:
    #second=-3*self.g*sin(self.state[0])/(2*self.length) - 3*self.u/(self.m*self.length**2)
    # From Underactuated notes: gives much better plots
    accel=-self.g*sin(self.state[0])/(self.length) + self.u/(self.m*self.length**2)

    # secondDamped = accel - self.b*self.state[1]

    return (vel,accel)

  ###
  ###
  ###
  def getStates(self):
    return self.state

  ###
  ###
  ###
  def updateStates(self,u):
    dq0, dq1 = calcF(u)
    self.state = (self.state[0]+dq1*self.dt,self.state[1] + dq1 * self.dt)

  #######################################
  # SimplePendulum ######################
  #######################################
 

#######################################
#######################################
#######################################
def plotListOfTuples(ax,tupleList):
  for i in range(len(tupleList)-1):
    x = tupleList[i][0], tupleList[i+1][0]
    y = [tupleList[i][1], tupleList[i+1][1]]
    ax.plot(x,y)

###
###
###
def pendulumHasReachedGoal(node,radius,end_regions):
  '''
  :params:  Node: a tuple (x,y), radius: "tolerance" 
            end_regions is a list of shapely polygons with end region area 
  :return:  if the node has entered one of the regions
  '''
  for region in end_regions:
    if goalReached(node,radius,region):
      return True

  return False

###
###
###
def steerPendulum(node_nearest, node_rand):
  '''
  :params:  node_nearest is SimplePendulum object
            node_rand is th,thdot values of random sampled point in state space
  '''
  tmax = node_nearest.max_torque
  if not node_nearest.iteration % 4:
    u = random.choice([-tmax,0,tmax]) # Random int. TODO this seems stupid. Must be based on node_rand? But works
    # node_nearest.iteration += 1
  else:
    u = node_nearest.u
    # node_nearest.iteration += 1

  # u = random.choice([-tmax,0,tmax]) # Random int. TODO this seems stupid. Must be based on node_rand
  th, thdot = node_nearest.getStates()
  _, thddot = node_nearest.calcF(u)

  # TODO: RK4
  thNew = th + thdot * node_nearest.dt
  thdotNew = thdot + thddot * node_nearest.dt
  #thNew = np.arctan2(np.sin(thNew), np.cos(thNew))

  return (thNew,thdotNew), u

###
###
###
def steerPendulumRK4(node_nearest, node_rand):
  '''
  :params:  node_nearest is SimplePendulum object
            node_rand is th,thdot values of random sampled point in state space
  '''
  tmax = node_nearest.max_torque
  if not node_nearest.iteration % 4:
    u = random.choice([-tmax,0,tmax]) # Random int. TODO this seems stupid. Must be based on node_rand? But works
  else:
    u = node_nearest.u

  # u = random.choice([-tmax,0,tmax]) # Random int. TODO this seems stupid. Must be based on node_rand
  x1, x2 = node_nearest.getStates()
  thdot, thddot = node_nearest.calcF(u)
  n_n = node_nearest

  k11 = x2
  k12 = u/(n_n.m*n_n.length**2)-n_n.g/(n_n.length)*sin(x1)

  k21 = x2 + k12/2
  k22 = u / (n_n.m*n_n.length**2) - n_n.g/(n_n.length)*sin(x1 + k11/2)

  k31 = x2 + k22/2
  k32 = u / (n_n.m*n_n.length**2) - n_n.g/(n_n.length)*sin(x1 + k21/2)

  k41 = x2 + k32
  k42 = u / (n_n.m*n_n.length**2) - n_n.g/(n_n.length)*sin(x1 + k31)

  x1 = x1 + (k11+2*(k21+k31)+k41)*n_n.dt/6
  x2 = x2 + (k12+2*(k22+k32)+k42)*n_n.dt/6

  return (x1,x2), u

###
###
###
def plotPhaseplotGoal(ax,path,bounds,iterations):
  '''
  :params:  ax: matplotlib Axes-object
            path: list of tuples (th,thdot)
            bounds: tuple of (minx,miny,maxx,maxy)
            iterations: int of number of iterations RRT used

  '''
  plotListOfTuples(ax,path)
  ax.grid(True)
  ax.set_xlim(bounds[0],bounds[2])
  ax.set_ylim(bounds[1],bounds[3])
  ax.set_xlabel("Theta (rad)")
  ax.set_ylabel("Thetadot (rad/s)")
  ax.set_title("Phaseplot of goal path found after %s iterations" % iterations)
  ax.set_aspect('equal', 'datalim')

###
###
###
def plotPhaseplot(ax,transitions,bounds,portion=1):
    '''
  :params:  ax: a matplotlib Axes object
            transitions: a list of tuples of two lists:
            [ ([x1,x2],[y1,y2]), ([x3,x4],[y3,y4]), ... ]
            bounds: tuple of (minx,miny,maxx,maxy)
            portion: a float between (0,1] that gives portion to plot

            Takes some time to execute when iterations gets high!
  '''
    print("Plotting %0.1f"%(portion*100),"% of the RRT paths...")
    stime =time.time()
    iters = len(transitions)

    ax.grid(True)
    ax.set_xlim(bounds[0],bounds[2])
    ax.set_ylim(bounds[1],bounds[3])
    ax.set_xlabel("Theta (rad)")
    ax.set_ylabel("Thetadot (rad/s)")
    ax.set_title("Phaseplot of pendulum after %s iterations" % iters)   
    
    assert portion != 0.
    everyIt = int(1 / portion)
    for i in range(iters):
      if not i % (everyIt):
        ax.plot(transitions[i][0],transitions[i][1],lw=1)

    ax.set_aspect('equal', 'datalim')
    print("... Done after",time.time() - stime , "seconds")

###
###
###
def plotGoalPathParameters(axarr,goalPath):
  axarr[0].set_title('Pendulum parameters')
  thetas = np.array([state[0] for state in goalPath.path])
  dthetas = np.array([state[1] for state in goalPath.path])
  us = np.array(goalPath.inputs)
  ts = np.linspace(0,len(thetas)*TIMESTEP,len(thetas))
  axarr[0].plot(ts,thetas)
  axarr[0].set_title("Theta")
  axarr[1].plot(ts,dthetas)
  axarr[1].set_title("Thetadot")
  axarr[2].plot(ts,us)
  axarr[2].set_title("Inputs")
  axarr[2].set_xlabel("Seconds")

###
###
###
def animatePendulum(goalPath):
  '''
  Creates an animation of pendulum

  :params:  goalPath: a Path object (from support.py) 
  '''
  print("Animating pendulum...")
  stime = time.time()
  fig = plt.figure(figsize=(16, 9), facecolor='w')
  ax = fig.add_subplot(1, 1, 1)
  plt.rcParams['font.size'] = 15

  lns = []
  sol = goalPath.path
  ts = np.linspace(0,len(sol)*TIMESTEP,len(sol))
  for i in range(len(sol)):
    # TODO: multiply the trig functions with pendulum length
    ln, = ax.plot([0, sin(sol[i][0])],[0, -cos(sol[i][0])],
                  color='r', lw=2)

    ln1, = ax.plot([sin(sol[i][0])],[-cos(sol[i][0])], marker='o', markersize=3, color="blue")

    tm = ax.text(-1, 0.9, 'time = %.1fs' % ts[i])
    lns.append([ln, tm])
    #lns.append([ln1,tm])

  ax.set_aspect('equal', 'datalim')
  ax.grid()
  ani = animation.ArtistAnimation(fig, lns, interval=200) 
  ani.save('pendulum_swingup_from_RRT.mp4', fps=15)
  print("... Done after", time.time() - stime, "seconds")


###
###
###
def animatePhaseplot(transitions,bounds):
  '''
  :params:  transitions: a list of tuples of two lists:
            [ ([x1,x2],[y1,y2]), ([x3,x4],[y3,y4]), ... ]
            bounds: 
  Note:     Takes long time!
  '''
  print("Animating phaseplot...")
  stime = time.time()
  fig = plt.figure(figsize=(bounds[2], bounds[3]), facecolor='w')
  ax = fig.add_subplot(1, 1, 1)
  plt.rcParams['font.size'] = 15

  lns = []
  iters = len(transitions)

  for i in range(iters):
    ln, = ax.plot(transitions[i][0],transitions[i][1],lw=1)
    tm = ax.text(bounds[0]*0.9, bounds[3]*0.9, 'Iteration no %.0f' % i)
    lns.append([ln, tm])

  ax.set_aspect('equal', 'datalim')
  ax.grid()
  ani = animation.ArtistAnimation(fig, lns, interval=200)
  ani.save('pendulum_swingup_from_RRT_phaseplot.mp4', fps=15, dpi=100)
  print("... Done after", time.time() - stime, "seconds")


###
###
###
def printRRT():
  print(Fore.WHITE + Style.BRIGHT)
  print('     ____________________________')
  print('    /                           /\\ ')
  print('   / '+Fore.RED+'RRT'+Fore.WHITE+' in'+'              /*    / /\\')
  print('  /  '+Fore.BLUE+'Pyt'+Fore.YELLOW+'hon'+Fore.WHITE+'     _     ___|    / /\\')
  print(' /           o_/ \___/       / /\\')
  print('/___________________________/ /\\')
  print('\___________________________\/\\')
  print(' \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\'
        + Style.RESET_ALL + Style.BRIGHT)
  print(Fore.WHITE+ Style.RESET_ALL)

def rrtPendulum(bounds,start_pos,radius,end_regions):

  '''
  - bounds: (minx, miny, maxx, maxy) tuple over region
  - radius: radius of pendulum head
  - end_region: end_region is a shapely Polygon that describes the region that the robot needs to reach
  '''

  # Using set for now to see unique nodes sampled
  printRRT()
  stime = time.time()
  nodes = set(start_pos)
  transitions = []

  # Creating graph of SearchNodes / Simple Pendulum nodes
  graph = Graph()
  graph.add_node(SimplePendulum(start_pos))
  goalPath = Path(SimplePendulum(start_pos)) # in case of no path found

  for i in range(MAX_ITER):
  
    if not (i%1000):
      n, d = nearestEuclSNode(graph, end_regions[0].centroid.coords[0])
      print("Iteration", i,"\t ### Nearest +pi: ( %0.2f , %0.2f" % (n.state[0], n.state[1]),")")

    # TODO: UNNECESSARY?
    rand_th = random.uniform(bounds[0],bounds[2])
    rand_thdot = random.uniform(bounds[1],bounds[3])
    node_rand = (rand_th,rand_thdot)

    sampling_rate = MAX_ITER # TODO: unused for now 
    if not(i % sampling_rate):
      node_rand = end_regions[0].centroid.coords[0]

    node_nearest, node_dist = nearestEuclSNode(graph, node_rand) 
    steered_state, steered_u = steerPendulum(node_nearest, node_rand)

    if not (bounds[0] < steered_state[0] < bounds[2]) or not (bounds[1] < steered_state[1] < bounds[3]):
      continue

    node_steered = SimplePendulum(steered_state,node_nearest,node_nearest.cost+node_dist, u=steered_u)

    nodes.add(node_steered.state)
    graph.add_edge(node_nearest,node_steered,node_dist)
    transitions.append( ([node_nearest.state[0],node_steered.state[0]],
      [node_nearest.state[1], node_steered.state[1]] ))

    if pendulumHasReachedGoal(node_steered.state,radius,end_regions):  
      goalPath = Path(node_steered)
      break # break the while loop when solution is found!

  print("No paths found") if len(goalPath.path) == 1 else print("Path found after",i,"iterations and ",time.time()-stime,"seconds\n------> It takes the pendulum %0.2f"%(len(goalPath.path)*TIMESTEP) + "s to reach the top")

  return goalPath, nodes, i, transitions


###
### Initializing 
###

radius = 0.1
bounds = (-2*np.pi, -8, 2*np.pi, 8) # interesting plot if bounds are doubled!
start = (0, 0)
upright1=[(np.pi*0.9,-0.1), (np.pi*0.9,0.1),(np.pi*1.1,0.1), (np.pi*1.1,-0.1)]
end1 = Polygon(upright1)
up2=[(-np.pi*0.9,-0.1), (-np.pi*0.9,0.1),(-np.pi*1.1,0.1), (-np.pi*1.1,-0.1)]
end2 = Polygon(up2)
# TODO: Decrease both TIMESTEP and the tuning of these endzones

ends = [end1, end2]
goalPath, nodes, iters, trans = rrtPendulum(bounds,start,radius,ends)

if PLOTS:
  fig, axarr = plt.subplots(1,2,sharey=True)
  plotPhaseplotGoal(axarr[0],goalPath.path,bounds,iters)
  
  plotPhaseplot(axarr[1],trans,bounds,portion=0.7) 

  f, axarr1 = plt.subplots(3)
  plotGoalPathParameters(axarr1,goalPath)

plt.show() if PLOTS else print("Finished. Chosen to show no plots")

if False:
  # animatePendulum(goalPath)
  animatePhaseplot(trans,bounds)