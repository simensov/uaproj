from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
import numpy as np
from sklearn.neighbors import NearestNeighbors
import scipy.interpolate as si
import random
from colorama import Fore, Style

class SearchNode(object):
    def __init__(self,
                 state,
                 parent_node = None,
                 cost        = 0.0,
                 u           = (0,0),
                 theta       = 0.0,
                 velocity    = (0.5,0.0,0.0),
                 length      = 1,
                 mass        = 0.5,
                 inertia     = 0.01,
                 ):

        self._parent    = parent_node # pointing to parent SearchNode object
        self._state     = state # made to be tuple of x,y
        self._u         = u # could be tuple of (u1,u2). alternative could be to treat u1,u2 as total thrust, since theta is going to be sampled anyway.
        self._cost      = cost
        ## TODO
        self._theta     = theta ## Positive orientation according to x,y-plane
        self._velocity  = velocity # Holds (xdot,ydot,thetadot)
        self._length    = length
        self._m         = mass
        self._inertia   = inertia

    def __repr__(self):
        return "<SearchNode (id: %s)| state: (%0.2f,%0.2f), cost: %s, parent_id: %s>" % (id(self), self.state[0],self.state[1], self.cost, id(self.parent))

    @property
    def state(self):
        """Get the state represented by this SearchNode"""
        return self._state

    @state.setter
    def state(self,value):
        """Get the state represented by this SearchNode"""
        self._state = value

    @property
    def parent(self):
        """Get the parent search node that we are coming from."""
        return self._parent

    @parent.setter
    def parent(self,value):
        self._parent = value

    @property
    def cost(self):
        """Get the cost to this search state"""
        return self._cost


    @cost.setter
    def cost(self,value):
        self._cost = value

    @property
    def u(self):
        """Get the u that was taken to get from parent to the state represented by this node."""
        return self._u

    @u.setter
    def u(self,value):
        self._u = value

    @property
    def theta(self):
        """Get the theta that was taken to get from parent to the state represented by this node."""
        return self._theta

    @property
    def velocity(self):
        """Get the velocity that was taken to get from parent to the state represented by this node."""
        return self._velocity

    @property
    def velocity(self):
        """Get the velocity that was taken to get from parent to the state represented by this node."""
        return self._velocity

    @property
    def length(self):
        """Get the velocity that was taken to get from parent to the state represented by this node."""
        return self._length
    
    @property
    def m(self):
        """Get the velocity that was taken to get from parent to the state represented by this node."""
        return self._m

    @property
    def inertia(self):
        """Get the velocity that was taken to get from parent to the state represented by this node."""
        return self._inertia

    def __eq__(self, other):
        return isinstance(other, SearchNode) and self._state == other._state

    def __hash__(self):
        return hash(self._state)

    def __gt__(self, other):
        return self._cost > other._cost


class Path(object):
    """This class computes the path from the starting state until the state specified by the search_node
    parameter by iterating backwards."""

    def __init__(self, search_node):
        self.path = []
        self.thetas = []
        self.inputs = []
        node = search_node
        while node is not None:
            self.path.append(node.state)
            self.thetas.append(node.theta)
            self.inputs.append(node.u)
            node = node.parent
        self.path.reverse()
        self.cost = search_node.cost


    def __repr__(self):
        return "Path of length %d, cost: %.3f: %s" % (len(self.path), self.cost, self.path)

    def edges(self):
        return zip(self.path[0:-1], self.path[1:])

# import pydot_ng as pydot
# import networkx as nx
import matplotlib.pyplot as plt

class NodeNotInGraph(Exception):
    def __init__(self, node):
        self.node = node

    def __str__(self):
        return "Node %s not in graph." % str(self.node)

class Edge(object):
    def __init__(self, source, target, weight=1.0):
        self.source = source
        self.target = target
        self.weight = weight

    def __hash__(self):
        return hash("%s_%s_%f" % (self.source, self.target, self.weight))

    def __eq__(self, other):
        return self.source == other.source and self.target == other.target \
               and self.weight == other.weight

    def __repr__(self):
        return "Edge(%r,%r,%r)" % (self.source, self.target, self.weight)

class Graph(object):
    def __init__(self, node_label_fn=None):
        self._nodes = list() # NB: CHANGED THIS TO LIST FROM set()
        self._edges = dict()
        self.node_label_fn = node_label_fn if node_label_fn else lambda x: x
        self.node_positions = dict()

    def __contains__(self, node):
        return node in self._nodes

    def add_node(self, node):
        """Adds a node to the graph."""
        # the function gets called when add_edge is called, so just check that we do not add several nodes 
        if not node in self._nodes:
            self._nodes.append(node) # NB: CHANGED THIS FROM .add(node)
        
        ##else:
            #print("Node",node,"already in node list")

    def add_edge(self, node1, node2, weight=1.0, bidirectional=False):
        """Adds an edge between node1 and node2. Adds the nodes to the graph first
        if they don't exist."""
        self.add_node(node1)
        self.add_node(node2)
        node1_edges = self._edges.get(node1, set())
        node1_edges.add(Edge(node1, node2, weight))
        self._edges[node1] = node1_edges
        if bidirectional:
            node2_edges = self._edges.get(node2, set())
            node2_edges.add(Edge(node2, node1, weight))
            self._edges[node2] = node2_edges

    def remove_edge(self, node1, node2): # maybe add bidirectional
        if node1 in self._edges:
            edgeset = self._edges[node1]
            # TODO: check if parent must be changed here
            for edge in edgeset:
                if edge.target == node2:
                    self._edges[node1].remove(edge)
                    break

        if node2 in self._edges:
            edgeset = self._edges[node2]
            for edge in edgeset:
                if edge.target == node1:
                    self._edges[node2].remove(edge)
                    break


    def set_node_positions(self, positions):
        self.node_positions = positions

    def set_node_pos(self, node, pos):
        """Sets the (x,y) pos of the node, if it exists in the graph."""
        if not node in self:
            raise NodeNotInGraph(node)
        self.node_positions[node] = pos

    def get_node_pos(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self.node_positions[node]

    def node_edges(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self._edges.get(node, set())




##############################################
##############################################
##############################################
##############################################
# ----   Support functions made by me   ---- #
##############################################
##############################################
##############################################
##############################################

def eucl_dist(a, b):
    """Returns the euclidean distance between a and b."""
    # a is a tuple of (x,y) values of first point
    # b is a tuple of (x,y) values of second point
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def eucl_dist_noSqrt(a,b):
    '''
    Returns the non-square-rooted distance for more efficient metric
    Since the sqrt(x) is strictly increasing for all x >0, which also x is ofcourse, then we can sort based on x instead of calculating sqrt(x)
    '''
    return (a[0]-b[0])**2 + (a[1] - b[1])**2


def nearestEuclSNode(graph, newNode):
    # TODO: improve the metric here? Norm seems very naive

    # returning tuple in nodeList that is closest to newNode
    # nearestNodeInGraph = SearchNode((1,1)) # initialize for return purpose
    dist = eucl_dist((-10000,-10000),(10000,10000)) # huge distance
    for i in graph._nodes: # iterating through a set. i becomes a SEARCH NODE
        newdist = eucl_dist_noSqrt(i.state, newNode)
        if newdist < dist: #SEARCH NODE.state is a tuple
           nearestNodeInGraph = i # a search
           dist = newdist
           newdistout = eucl_dist(i.state, newNode)

    return nearestNodeInGraph, newdistout


# improved version for RRT_experimental
def nearestEuclNeighbor(graph, newNode, k):
    # returning tuple in nodeList that is closest to newNode
    '''
    graph is Graph object
    newNode is a SearchNode
    '''
    states = []
    it = 0
    loc_pos = 0
    for node in graph._nodes:
        states.append([node.state[0],node.state[1]])
        if node.state == newNode.state:
            loc_pos = it
        it += 1

    X = np.array(states)
    nbrs = NearestNeighbors(n_neighbors=k, algorithm='ball_tree').fit(X)
    distances, indices = nbrs.kneighbors(X)

    # pick relevant ones
    dist = distances[loc_pos]
    indi = indices[loc_pos]
    SN_list = []
    for ind in indi:
        SN_list.append(graph._nodes[ind])

    return dist, SN_list

# improved version for RRT_experimental
# 
def steerPath(firstNode,nextNode,dist):
    # tuples as input - distance between them gives ish distance to move

    # avoiding errors when sampling node is too close
    if dist == 0:
        dist = 100000

    hori_dist = nextNode[0] - firstNode[0] #signed
    verti_dist = nextNode[1] - firstNode[1] #signed
    dist = dist*5
    # a new node that are closer to the next node - always smaller than the boundary-checked nextNode parameter
    # I chose to implement a slow but working solution -> normalize the distance to move. This could easily be changed
    return (firstNode[0] + hori_dist/dist, firstNode[1] + verti_dist/dist)



###
### # for expansion of RRT
### 

def constrainedSteeringAngle(angle,max_angle=3.14/10):
    '''
    Keeps steering angle within a certain max
    
    :param:     angle (float): angle to constrain
                max _angle(float): maximum steering angle
    '''
    if angle > max_angle:
        return max_angle
    elif angle < -max_angle:
        return -max_angle

    return angle


def calculateSteeringAngle(firstNode,theta,nextNode,L,maxx=np.pi / 10.):
    fWheel=(firstNode[0]+L*np.cos(theta),firstNode[1]+L*np.sin(theta))
    # arctan2 gives both positive and negative angle -> map to 0,2pi
    angle = np.mod(np.arctan2(nextNode[1]-fWheel[1],nextNode[0]-fWheel[0]),2*np.pi) 
    rel_angle = angle - theta
    # enforce steering to within range RELATIVE to theta -> map to (-pi,pi)
    rel_angle = np.arctan2(np.sin(rel_angle), np.cos(rel_angle))

    return constrainedSteeringAngle(rel_angle,maxx)#max_steer is default

def bicycleKinematics(theta,v,L,delta,dt):
    dth  = v/L*np.tan(delta)
    dx   = v * np.cos(theta)
    dy   = v * np.sin(theta)
    return (dx,dy,dth) 

def bicycleKinematicsRK4(initState,theta,v,L,delta,dt):
    x0,y0 = initState

    k1t = v/L*np.tan(delta)
    k1x = v * np.cos(theta)
    k1y = v * np.sin(theta)
    
    # k2t = v/L * np.tan(delta)
    k2x = v * np.cos(theta + k1t/2)
    k2y = v * np.sin(theta + k1t/2)

    # k3t = v/L * np.tan(delta)
    k3x = v * np.cos(theta + k1t/2)
    k3y = v * np.sin(theta + k1t/2)

    k4x = v * np.cos(theta + k1t)
    k4y = v * np.sin(theta + k1t)

    '''
    t = theta + k1t * dt # no effect since it is constant
    x = x0 + (k1x + 2*(k2x + k3x) + k4x)*dt/6
    y = y0 + (k1y + 2*(k2y + k3y) + k4y)*dt/6
    return (x,y,t) 
    '''
    dx = (k1x + 2*(k2x + k3x) + k4x) / 6
    dy = (k1y + 2*(k2y + k3y) + k4y) / 6
    dth = k1t

    return (dx,dy,dth)


def steerBicycleWithKinematics(firstNode,theta,nextNode,dt):
    '''
    TODO: The sexiest thing to do here would be to make a bicycle class
    Purpose:    Tests implementation of kinematic constraints by using a simple bicycle model

    TODO: if delta could be stored, a more realistic implementation would be to constrain delta considering the steering angle already made!

    :params:    firstNode and nextNode is a tuple of x,y pos of back wheel
                theta is a float of bicycle frame angle wrt. x axis

    '''
    L,v = (0.25,0.5) # frame length L, velocity v
    #vlist = [v/2.,v]
    #v = random.choice(vlist)
    # 1) Find the new delta parameter for steering wheel
    delta = calculateSteeringAngle(firstNode,theta,nextNode,L)

    # 2) Use that as input to kinematics
    # Old Euler Method vs more precice Runge-Kutta fourth order
    # dx, dy, dth = bicycleKinematics(theta,v,L,delta,dt)
    dx,dy,dth = bicycleKinematicsRK4(firstNode,theta,v,L,delta,dt)

    thetanew    = theta        + dth * dt
    xnew        = firstNode[0] + dx * dt
    ynew        = firstNode[1] + dy * dt
    
    # map thetanew to (0,2pi)
    return (xnew,ynew), np.mod(thetanew, 2 * np.pi)


def steerBicycleWithDynamics(firstNode,theta,nextNode,dt,velocity):
    '''
    Incorporates dynamic constraints on simple bicycle model
    Note that (x,y)-position is now calculated from CG instead of back wheel as in the kinematic version
    
    :params:    firstNode (tuple(floats)): x,y pos of back wheel before
                nextNode  (tuple(floats)): x,y pos of back wheel next
                theta (float): bicycle frame angle wrt. x axis (0,2pi)
                dt (float): 

    General note: ddx and ddelta would be inputs
    '''
    L,vx = (0.25 , 0.5) # frame length L, velocity v
    Lr = Lf = L/2. # setting distance from CG to front/back wheel to be equal
    
    # Standard bike dimensions
    Lstd = 1
    mstd = 10
    Istd = (1**2 + 0.2**2) * mstd / 12
    tire_radius_std = 0.33 * Lstd

    # Scale to wanted length (not necessarily linear scaling of mass etc IRL)
    scale = L / Lstd
    m = scale * mstd
    I = scale * Istd
    tire_radius = scale * tire_radius_std


    # TODO: this should absolutely be in relation to the previous angle to avoid. What needs to be done is to set a max on the rate of change of delta. Should be a very easy fix 
    delta = calculateSteeringAngle(firstNode,theta,nextNode,L,maxx=np.pi/20)

    # vx is velocity ALONG STRAIGHT FORWARD AXIS
    # vy is lateral velocity of cg (can move sideways if vx > 0 and delta != 0)
    # dth is rate of change of orientation vs x-axis
    # velocity at firstNode
    (vx,vy,dth) = velocity
    r = dth

    # values used from 
    Cf = Cr = 1 # usually quite large, but mass is going to be very small
    
    a = b = tire_radius
    cf = (vy + Lr * a)
    #cf = np.arctan2(np.sin(cf), np.cos(cf))
    cr = (vy - Lr * b)
    # cr = np.arctan2(np.sin(cr), np.cos(cr))

    # TODO: There angles should be really small
    # Plan: set forces to be very small, but sign in direction according to delta
    alphaf = np.arctan(cf/vx) - delta # before: this entire set was in arctan
    alphar = np.arctan(cr/vx)

    # print(alphar, alphaf) # adding minus sign infront of forces 
    Ff = -(Cf) * alphaf
    Fr = -(Cr) * alphar


    # euler integration
    dvx = 0 # just for clarity
    #dr = Lr * (Ff * np.cos(delta) - Fr) / I
    dr = (m*a*np.tan(delta)*(dvx-r*vy) + a * Ff / np.cos(delta) - b * Fr ) / I
    #dvy = (Ff * np.cos(delta) + Fr) / m - vx * r
    dvy = np.tan(delta)*(dvx-r*vy) + (Ff/np.cos(delta) + Fr)/m - vx*r
    
    rnew = dth + dr * dt

    thetanew = theta + rnew * dt # before or after dx,dy calculation?
    
    dx = vx * np.cos(thetanew) - vy * np.sin(thetanew)
    dy = vx * np.sin(thetanew) + vy * np.cos(thetanew)
    
    vynew = vy + dvy * dt 
    vxnew = vx + dvx * dt
    xnew = firstNode[0] + dx * dt
    ynew = firstNode[1] + dy * dt
    
    #print(dx,dy)
    newstate = (xnew,ynew)
    # TODO (actually solved): The problem is that the derivatives are usually very small or very big, so either it samples a point very far away, or it samples a point very close. It only add the ones that are very close, and according to the dynamics, they end up in a close path

    # map thetanew to (0,2pi)
    return newstate, np.mod(thetanew, 2 * np.pi), (vxnew,vynew,rnew)


def sampleQuadcopterInputs(searchNode,nextnode,L,m,I,g,u):
    '''
    
    Notes:  Differ between points that are above and below?
            This has to depend on the previous inputs and velocities

            Needs to have the ability to reduce both gains to decend
            - yddot will always be maintained if (u1+u2)*cos(th) == mg,
              so this is the parameters to adjust according to wanted altitude
            
            Make sure that th stays pretty small, indep of vertical movement
            - Extra: so that the dynamics might be linearizable?

    
    '''

    pos = searchNode.state
    theta = searchNode.theta
    dx = nextNode[0]-pos[0]
    dy = nextNode[1]-pos[1]
    # arctan2 gives both positive and negative angle -> map to 0,2pi
    angle = np.mod(np.arctan2(dy,dx),2*np.pi) 

    # This is the angle between the CG and the next point. Remember that setting theta towards this angle makes the quadcopter take of +90 deg in relation to that! -> Find the relative angle between the direction the quadcopter should point towards (which is +90 deg of theta)
    rel_angle = angle - (theta + np.pi)

    # Map to (-pi,pi). This range is directly applicable to theta and the change we need to enforce on it
    rel_angle = np.arctan2(np.sin(rel_angle), np.cos(rel_angle))

    # Now to how we change u1,u2. Remember that u1 is the rightmost input, so if it is larger than u2, theta increases and the quadcopter tips leftward.
    #current_accel = ( (u1+u2)*np.sin(theta)/m, (u1+u2)*np.cos(theta)/m - g ) 

    # first, rotate the drone according to rel_angle
    # adjust in such a way that yddot = 0 initially?

    # find the magnitude of this angle to decide when to do nothing, or just be happy with a drone that flies through the end region
    # then, find out if the magnitude of the vertical thrust needs to be increased, reduced (or unchanged if yddot wasn't enforced earlier)

    ### insert code for finding u1,u2 here

    # VOILA, we should have our new u1,u2's. As seen from differential flatness, they should be very similar, always

    ## TODO: increase and decrease u1,u2 in relation to previous ones and some kind of maximum rate of change (not realistic to suddenly change u1,u2 to be totally previous)

    return (u1,u2)

def steerWithQuadcopterDynamics(searchNode, nextNode, dt):
    '''
    searchNode  is supposed to be nodeNearest as SearchNode object


    samples new point reachable during dt from 

    '''
    # TODO: make a function that samples u1,u2 relative to what searchNode had before!!!

    L = 0.25     # length of rotor arm
    m = 0.486    # mass of quadrotor
    I = 0.00383  # moment of inertia
    g = 9.81     # gravity

    x,y = searchNode.state
    th  = searchNode.theta
    xdot,ydot,thdot = searchNode.velocity
    u1,u2 = searchNode.u

    xddot = -1/m * np.sin(theta) * (u1 + u2)
    yddot =  1/m * np.cos(theta) * (u1 + u2)
    thddot =  L/I * (u1 - u2)

    xdotnew = xdot + xddot * dt
    ydotnew = ydot + yddot * dt
    thdotnew = thdot + thddot * dt

    xnew = x + xdotnew * dt
    ynew = y + ydotnew * dt
    thnew = th + thdotnew * dt

    return (xnew,ynew),thnew,(u1,u2)

def obstacleIsInPath(firstNode,nextNode,env,radius):
    '''
    :returns:   a boolean for collision or not
    '''

    # Point from shapely
    start_pose = Point(firstNode).buffer(radius, resolution=3)
    end_pose = Point(nextNode).buffer(radius, resolution=3)

    # LineString from Shapely
    line = LineString([firstNode, nextNode])
    expanded_line = line.buffer(radius, resolution=3)

    if env.obstacles is not None:
        for i, obs in enumerate(env.obstacles):
            # Check collisions between the expanded line and each obstacle
            if (expanded_line.intersects(obs)):
                return True

    return False

def goalReached(node,radius,end_region):
    '''
    :params:    node is a tuple (xpos,ypos)
                radius is a scalar - size of the robot
                end_region is a polygon of four tuples drawn clockwise: lower left, upper left, upper right, lower right
    '''
    # returns a boolean for node tuple + radius inside the region
    return end_region.contains(Point(node))

def plot_line_mine(ax, line,width=2):
    # wanted to draw lines in path more clearly. gathered format from environment.py
    x, y = line.xy
    ax.plot(x, y, color='black', linewidth=width, solid_capstyle='round', zorder=1)

def random_environment(bounds, start, radius, goal, n, size_limits=(0.5, 1.5)):
    minx, miny, maxx, maxy = bounds
    # print(bounds)
    edges = 4
    minl, maxl = size_limits
    env = Environment(None)
    obs = []
    start_pose = Point(start).buffer(radius, resolution=3)
    obi = 0
    while obi < n:
        r = np.random.uniform(low=0.0, high=1.0, size=2)
        xy = np.array([minx + (maxx-minx)*r[0], miny + (maxy-miny)*r[1]])
        
        angles = np.random.rand(edges)
        angles = angles*2*np.pi / np.sum(angles)
        for i in range(1,len(angles)):
            angles[i] = angles[i-1] + angles[i]
        angles = 2*np.pi * angles / angles[-1] 
        angles = angles + 2*np.pi*np.random.rand()
        lengths = 0.5*minl + (maxl-minl) * 0.5 * np.random.rand(edges)
        xx = xy[0] + np.array([l*np.cos(a) for a,l in zip(angles,lengths)])
        yy = xy[1] + np.array([l*np.sin(a) for a,l in zip(angles,lengths)])
        p = Polygon([(x,y) for x,y in zip(xx,yy)])
        if p.intersects(start_pose) or p.intersects(goal):
            continue
        else:
            obi = obi + 1
            obs.append(p)
#         coords = xy + [l*np.cos(a),l*np.sin(a) for a,l in zip(angles,lengths)]
    env.add_obstacles(obs)
    return env

def bspline_planning(x, y, sn):

    N = 3
    t = range(len(x))
    x_tup = si.splrep(t, x, k=N)
    y_tup = si.splrep(t, y, k=N)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = si.splev(ipl_t, x_list)
    ry = si.splev(ipl_t, y_list)

    return rx, ry

def plot_bspline(ax,x,y,bounds,sn=100):
    '''
    ax is axis object
    x anf y are lists
    sn is number of samples (resolution on spline)
    '''
    xmin,ymin,xmax,ymax = bounds

    x = np.array(x)
    y = np.array(y)

    rx, ry = bspline_planning(x, y, sn)

    # show results
    ax.plot(rx, ry, '-r', color='green', linewidth=2, solid_capstyle='round', zorder=1)
    ax.grid(False)
    #ax.set_label("Goal path")
    #ax.legend()
    # ax.axis("equal")    
    ax.set_xlim(xmin,xmax)
    ax.set_ylim(ymin,ymax)

def plotListOfTuples(ax,tupleList):
  for i in range(len(tupleList)-1):
    x = tupleList[i][0], tupleList[i+1][0]
    y = [tupleList[i][1], tupleList[i+1][1]]
    ax.plot(x,y)

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


def plotEdges(ax,graph,bounds):
    x = []; y = []
    for key in graph._edges:
        edgething = graph._edges[key]
        for edge in edgething:
            x1,y1 = edge.source.state
            x2,y2 = edge.target.state
            ax.plot( [x1,x2] , [y1,y2] , color='grey', linewidth=0.5)