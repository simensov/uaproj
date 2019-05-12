# SimplePendulum.py
# Implements the swing up procedure for a simple pendulum with damping by using RRT

# Should be good to go unless I want to clean it somewhat

from support import * 
from numpy import sin,cos
import matplotlib.animation as animation
import time

TIMESTEP = 0.05
GRAVITY  = 9.8
MAX_ITER = 10001
PLOTS    = True
ANIMATE  = False
DAMPING  = True
PLOTPORTION = 1.0

np.random.seed()

###
### SimplePendulum class
###
class SimplePendulum(SearchNode):

  def __init__(self,
    state       = (0,0),
    parent_node = None,
    cost        = 0.0,
    u           = 0.0,
    length      = 0.5,
    mass        = 1.0,
    max_torque  = 0.5,
    iteration   = 0,
    dt          = TIMESTEP,
    gravity     = GRAVITY,
    damping     = 0.1):

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
    '''
    Purpose:  calculate xdot = f(x,u)

    '''
    self.u = u 
    vel=self.state[1]

    # From Underactuated notes: gives much better plots
    accel=-self.g*sin(self.state[0])/(self.length) + u/(self.m*self.length**2)

    if DAMPING:
      accel = accel - self.b*self.state[1]

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
  Purpose   Steers pendulum towars node_rand. Updates parameters wit forward 
            euler integration
  :params:  node_nearest is SimplePendulum object
            node_rand is th,thdot values of random sampled point in state space
  '''
  tmax = node_nearest.max_torque
  th, thdot = node_nearest.getStates()
  diffTheta =  node_rand[0] - th
  diffVel = node_rand[1] - thdot

  if diffTheta > 0: # sampled angle is further anti-clockwise
    if diffVel > 0: # sampled velocity is larger than current
      u = tmax      # we want to continue this way - add positive torque
    else: 
      u = -tmax     # sampled velocity is smaller - we must reduce u
  else:             # sampled angle is further clockwise
    if diffVel < 0: # sampled angle has larger velocity anti clockwise
      u = -tmax     # inflict torque anti clockwise
    else:
      u = tmax

  ''' Old method that made the node_rand samping stupid
  if not (node_nearest.iteration % 3):
    u = random.choice( [-tmax,0,tmax]) 
    node_nearest.iteration += 1
  else:
    u = node_nearest.u
    node_nearest.iteration += 1
  '''

  _, thddot = node_nearest.calcF(u)

  thNew = th + thdot * node_nearest.dt
  thdotNew = thdot + thddot * node_nearest.dt

  return (thNew,thdotNew), u

###
###
###
def steerPendulumRK4(node_nearest, node_rand):
  '''
  Purpose:  Use RK4 for accurate integration technique
            IMPLEMENTED AT 3AM BUT DONT WORK LONGER - NO IDEA WHAT I DID
  :params:  node_nearest is SimplePendulum object
            node_rand is th,thdot values of random sampled point in state space
  '''
  tmax = node_nearest.max_torque
  x1, x2 = node_nearest.getStates()

  diffTheta =  node_rand[0] - x1
  diffVel = node_rand[1] - x2

  if diffTheta > 0: # sampled angle is further anti-clockwise
    if diffVel > 0: # sampled velocity is larger than current
      u = tmax      # we want to continue this way - add positive torque
    else: 
      u = -tmax     # sampled velocity is smaller - we must reduce u
  else:             # sampled angle is further clockwise
    if diffVel < 0: # sampled angle has larger velocity anti clockwise
      u = -tmax     # inflict torque anti clockwise
    else:
      u = tmax

  thdot, thddot = node_nearest.calcF(u)
  n_n = node_nearest

  k11 = x2
  k12 = u / (n_n.m*n_n.length**2) - n_n.g/(n_n.length)*sin(x1)

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
  plotListOfTuples(ax,path,width = 4, color = "green")
  # ax.grid(True)
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

    # ax.grid(True)
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
  '''
  Purpose:  Creates plot of theta, angular velocity and inputs

  '''
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

    # ln1, = ax.plot([sin(sol[i][0])],[-cos(sol[i][0])], marker='o', markersize=3, color="blue")

    tm = ax.text(-1, 0.9, 'time = %.1fs' % ts[i])
    lns.append([ln, tm])
    #lns.append([ln1,tm])

  ax.set_aspect('equal', 'datalim')
  ax.grid()
  ani = animation.ArtistAnimation(fig, lns, interval=20,blit=True) 
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
  ani = animation.ArtistAnimation(fig, lns, interval=20, blit=True)
  ani.save('pendulum_swingup_from_RRT_phaseplot.mp4', fps=15, dpi=100)
  print("... Done after", time.time() - stime, "seconds")


##############################################
##############################################
##############################################
##############################################
# ----          Main function           ---- #
##############################################
##############################################
##############################################
##############################################
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
  graph.add_node(SimplePendulum(start_pos,u=0.0))
  goalPath = Path(SimplePendulum(start_pos,u=0.0)) # in case of no path found

  for i in range(MAX_ITER):
  
    if not (i%1000):
      n, d = nearestEuclSNode(graph, end_regions[0].centroid.coords[0])
      print("Iteration", i,"\t ### Nearest +pi: ( %0.2f , %0.2f" % (n.state[0], n.state[1]),")")

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
### Initializing. Angle is zero in bottom and follows x,y.plane convenstion by increasing anti clockwise
###

radius = 0.1
bounds = (-2*np.pi, -8, 2*np.pi, 8) # interesting plot if bounds are doubled!
start = (0, 0)
upright1=[(np.pi*0.95,-0.05), (np.pi*0.95,0.05),(np.pi*1.05,0.05), (np.pi*1.05,-0.05)]
end1 = Polygon(upright1)
up2=[(-np.pi*0.95,-0.05), (-np.pi*0.95,0.05),(-np.pi*1.05,0.05), (-np.pi*1.05,-0.05)]
end2 = Polygon(up2)

# TODO: Decrease both TIMESTEP and the tuning of these endzones

ends = [end1, end2]
goalPath, nodes, iters, trans = rrtPendulum(bounds,start,radius,ends)

if PLOTS:
  # fig, axarr = plt.subplots(1,2,sharey=True) # give axarr[0], [1] to plotting
  fig, ax = plt.subplots()
  plotPhaseplotGoal(ax,goalPath.path,bounds,iters)
  plotPhaseplot(ax,trans,bounds,portion=PLOTPORTION)

  plot_poly(ax,Point( (np.pi, 0)).buffer(0.2,resolution=5),'green',alpha=.3)
  plot_poly(ax,Point( (-np.pi, 0)).buffer(0.2,resolution=5),'green',alpha=.3)

  f, axarr1 = plt.subplots(3)
  plotGoalPathParameters(axarr1,goalPath)

if ANIMATE:
  # animatePendulum(goalPath)
  animatePhaseplot(trans,bounds)

plt.show() if PLOTS else print("Finished. Chosen to show no plots")
