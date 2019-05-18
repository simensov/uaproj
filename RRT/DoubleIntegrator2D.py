# TODO: Suddently acts very weird. 30 min debugging in the steering function didn't help me. One problem seems to be that the several nodes are being sampled in the goal region initially since u only increase acceleration and not velocity momentarily, so the first nodes are sampled to be in the start position. when the nearest neighbor was asmpled to be the start node, everything just stayed zero. But giving start velocity fixed that. Adjusting the max vel etc perfectly made the node find the goal sometimes, but if it didn't, it never expanded the tree. Very strange, need to figure out why the tree only seemed to grow in the direction of the first node

from support import *
import time

TIMESTEP = 0.1
GRAVITY  = 9.8
MAX_ITER = 4000
PLOTS = True
REALTIME = False
ANIMATE = False
GOAL_SAMPLING_RATE = 30

class DoubleIntegrator2D(SearchNode):

  def __init__(self,
    state       = (0,0,0,0),
    parent_node = None,
    cost        = 0.0,
    u           = (0,0),
    mass        = 10.,
    max_thrust  = 0.5,
    max_vel     = 1,
    iteration   = 0,
    dt          = TIMESTEP,
    gravity     = GRAVITY):

    self.iteration = iteration
    self.max_thrust = max_thrust
    self.max_vel = max_vel
    self.dt = dt
    self.g = gravity

    super(DoubleIntegrator2D, self).__init__(state,parent_node,u,mass)

  ###
  ###
  ###
  def __repr__(self):
    return 'DblInteg2D (x=%s, y=%s)' % (self.state[0], self.state[1])
  
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
    u is tuple of (ux,uy)
    '''
    self.u = u
    x1dot = self.state[2] # vx
    x2dot = self.state[3] # vy
    x3dot = u[0] / self.m # ax
    x4dot = u[1] / self.m # ay
    return (x1dot,x2dot,x3dot,x4dot)

  ###
  ###
  ###
  def getStates(self):
    return self.state

  def getPos(self):
    return (self.state[0], self.state[1])

  ###
  ###
  ###

  #######################################
  # DoubleIntegrator2D ##################
  #######################################


def steerDoubleIntegrator2D(node_nearest, node_rand):

  umax = node_nearest.max_thrust
  x,y,vx,vy = node_nearest.state

  dx = node_rand[0] - x
  dy = node_rand[1] - y

  theta = np.arctan2(dy,dx)
  u1 = np.cos(theta) * node_nearest.max_thrust
  u2 = np.sin(theta) * node_nearest.max_thrust

  dXdt,dYdt,dVxdt,dVydt = node_nearest.calcF((u1,u2))

  vxnew = vx + dVxdt * TIMESTEP
  vynew = vy + dVydt * TIMESTEP
  xnew = x + dXdt * TIMESTEP
  ynew = y + dYdt * TIMESTEP

  if np.fabs(vxnew) > node_nearest.max_vel:
    u1 = -u1*0.2

  if np.fabs(vynew) > node_nearest.max_vel:
    u2 = -u2*0.2

  dXdt,dYdt,dVxdt,dVydt = node_nearest.calcF((u1,u2))
  vxnew = vx + dVxdt * TIMESTEP
  vynew = vy + dVydt * TIMESTEP
  xnew = x + dXdt * TIMESTEP
  ynew = y + dYdt * TIMESTEP

  stateout = (xnew,ynew,vxnew,vynew)

  return stateout, (u1,u2)

def nearestEuclIntegratorNode(graph,newNode):
    # returning tuple in nodeList that is closest to newNode
    # nearestNodeInGraph = SearchNode((1,1)) # initialize for return purpose
    dist = eucl_dist_noSqrt((-10000,-10000),(10000,10000)) # huge distance
    for i in graph._nodes: # iterating through a set. i becomes a SEARCH NODE
        pos = i.getPos()
        newdist = eucl_dist_noSqrt(pos, newNode)
        if newdist < dist:
           nearestNodeInGraph = i # Node
           dist = newdist
           
    newdistout = np.sqrt(dist)
    return nearestNodeInGraph, newdistout


def rrtDoubleIntegrator2D(bounds,environment,start_state,radius,end_regions):

  '''
  - bounds: (minx, miny, maxx, maxy) tuple over region
  - radius: radius of pendulum head
  - end_region: end_region is a shapely Polygon that describes the region that the robot needs to reach
  '''

  # Using set for now to see unique nodes sampled
  printRRT()
  stime = time.time()
  nodes = list(start_state)
  transitions = []
  goalPos = end_region.centroid.coords[0]

  # Creating graph of SearchNodes / Simple Pendulum nodes

  graph = Graph()
  graph.add_node(DoubleIntegrator2D(start_state))
  goalPath = Path(DoubleIntegrator2D(start_state))

  ax = plot_environment(environment,bounds)
  plot_poly(ax,Point((start_state[0],start_state[1])).buffer(radius,resolution=5),'blue',alpha=.2)
  plot_poly(ax, end_region,'red', alpha=0.2)

  for i in range(MAX_ITER):
    if not (i%100):
      _,dd = nearestEuclIntegratorNode(graph, goalPos)
      print("Iteration",i," - Closest to goal:", dd)

    rand_x = random.uniform(bounds[0],bounds[2])
    rand_y = random.uniform(bounds[1],bounds[3])
    node_rand = (rand_x,rand_y)

    if not(i % GOAL_SAMPLING_RATE):
      node_rand = goalPos

    node_nearest, node_dist = nearestEuclIntegratorNode(graph, node_rand) 
    #print("Rand", node_rand)
    #print("Near", node_nearest.getStates())
    steered_state, steered_u = steerDoubleIntegrator2D(node_nearest, node_rand)
    #print("Steer", steered_state, steered_u)
    
    if not (bounds[0] < steered_state[0] < bounds[2]) or not (bounds[1] < steered_state[1] < bounds[3]):
      continue

    node_dist = eucl_dist(node_nearest.state,steered_state)
    node_steered = DoubleIntegrator2D(steered_state,node_nearest,node_nearest.cost+node_dist, u=steered_u)

    # Seems like it is performing better if allowed to not check for existence
    if node_steered.state not in nodes:
      if not obstacleIsInPath(node_nearest.getPos(), node_steered.getPos(), environment,radius):                

        nodes.append(node_steered.state)
        graph.add_edge(node_nearest,node_steered,node_dist)
        transitions.append( ([node_nearest.state[0],node_steered.state[0]],[node_nearest.state[1], node_steered.state[1]]))

        line = LineString([node_nearest.getPos(),node_steered.getPos()])
        plot_line(ax, line)
        if REALTIME:
          plt.draw()
          plt.pause(0.000001)

      else:
        continue

    else:
      continue

    if goalReached(node_steered.getPos(),radius,end_region):  
      goalPath = Path(node_steered)
      break # break the while loop when solution is found!


  print("No paths found") if len(goalPath.path) == 1 else print("Path found after",i,"iterations and ",time.time()-stime,"seconds\n... It takes %0.2f"%(len(goalPath.path)*TIMESTEP) + "s to reach goal")

  return goalPath, nodes, i, transitions


##########################################
##########################################
##########################################
##########################################
##########################################
##########################################

environment = Environment('env_empty.yaml')
radius = 0.3
bounds = (-2, -3, 12, 8)
end_region = Polygon([(10,4), (10,5), (11,5), (11,4)])
start = (-1,4,0.01,0)


'''
ax = plot_environment(environment,bounds)
plot_poly(ax,Point(start).buffer(radius,resolution=5),'blue',alpha=.2)
plot_poly(ax, end_region,'red', alpha=0.2)
'''

goalPath, nodes, iterations, transitions = rrtDoubleIntegrator2D(bounds, environment,start, radius, end_region)


for j in range(len(goalPath.path)):
  print(goalPath.path[j] , "---" , goalPath.inputs[j])

plt.show()
