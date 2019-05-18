# support.py - Simen Sem Oevereng
# this file contains class definitions and calculation functions for the motion planning problems

# Contains general tools for distance calculation and plotting
from utils import * # Contains SearchNode, Graph, Edge, Path classes
from searchClasses import *
import math

###
###
###
def withinBounds(steered_node,bounds):
    return (bounds[0] < steered_node[0] < bounds[2]) and (bounds[1] < steered_node[1] < bounds[3])

###
###
###
def getRandomNode(bounds,iteration,goal_bias,bias_rate,end_region,informed=False,ellipse=None):
    '''
    Purpose:    samples random node from within given bounds

    :params:    ellipse is an Ellipse object
    '''

    if not(iteration % bias_rate) and (not iteration==0) and (goal_bias):
        return end_region.centroid.coords[0]
    else:
      if not ellipse is None and informed:
        return ellipse.randomPoint()
      else:
        rand_x = random.uniform(bounds[0],bounds[2])
        rand_y = random.uniform(bounds[1],bounds[3])
        return (rand_x,rand_y)
            
###
###
###
def steerPath(firstNode,nextNode,dist,eta=0.5):
    '''
    Purpose:    A super naive way of steering towards a sampled point

    :params:    tuples as input, dist between them gives ish distance to move
                eta is a float bounding:
                    eta <= epsilon, where |x1-x2| < eta for all steering |x1 - steer(x1,x2)| < epsilon
    '''

    # avoiding errors when sampling node is too close
    if dist == 0:
        dist = 100000

    hori_dist = nextNode[0] - firstNode[0] #signed
    vert_dist = nextNode[1] - firstNode[1] #signed
    dist = dist/2 # originally used to divide hori_dist and vert_dist on

    if True:
        if math.sqrt(hori_dist**2 + vert_dist**2) > eta:
            angle = np.arctan2(vert_dist,hori_dist)
            hori_dist = eta * np.cos(angle)
            vert_dist = eta * np.sin(angle)

        return (firstNode[0] + hori_dist, firstNode[1] + vert_dist)

    else:
        return (firstNode[0] + hori_dist/dist, firstNode[1] + vert_dist/dist)

###
### 
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

###
### 
### 
def calculateSteeringAngle(firstNode,theta,nextNode,L,maxx=np.pi / 10.):
    fWheel=(firstNode[0]+L*np.cos(theta),firstNode[1]+L*np.sin(theta))
    # arctan2 gives both positive and negative angle -> map to 0,2pi
    angle = np.mod(np.arctan2(nextNode[1]-fWheel[1],nextNode[0]-fWheel[0]),2*np.pi) 
    rel_angle = angle - theta
    # enforce steering to within range RELATIVE to theta -> map to (-pi,pi)
    rel_angle = np.arctan2(np.sin(rel_angle), np.cos(rel_angle))

    return constrainedSteeringAngle(rel_angle,maxx)#max_steer is default

###
### 
### 
def bicycleKinematics(theta,v,L,delta,dt):
    dth  = v/L*np.tan(delta)
    dx   = v * np.cos(theta)
    dy   = v * np.sin(theta)
    return (dx,dy,dth) 

###
### 
### 
def bicycleKinematicsRK4(initState,theta,v,L,delta):
    x0,y0 = initState

    k1t = v/L*np.tan(delta)
    k1x = v * np.cos(theta)
    k1y = v * np.sin(theta)
    
    k2x = v * np.cos(theta + k1t/2)
    k2y = v * np.sin(theta + k1t/2)

    k3x = v * np.cos(theta + k1t/2)
    k3y = v * np.sin(theta + k1t/2)

    k4x = v * np.cos(theta + k1t)
    k4y = v * np.sin(theta + k1t)

    dx = (k1x + 2*(k2x + k3x) + k4x) / 6
    dy = (k1y + 2*(k2y + k3y) + k4y) / 6
    dth = k1t # has no effect throughout; kjt is equal to k1t for all j

    return (dx,dy,dth)

###
### 
### 
def steerBicycleWithKinematics(firstNode,theta,nextNode,dt):
    '''
    Purpose:  Steer the kinematic bicycle model from firstNode towards nextNode

    If delta could be stored, a more realistic implementation would be to constrain delta considering the steering angle already made!

    :params:    firstNode and nextNode is a tuple of x,y pos of back wheel
                theta is a float of bicycle frame angle wrt. x axis

    '''
    L,v = (0.25,0.5) # frame length L, velocity v

    # 1) Find the new delta parameter for steering wheel
    delta = calculateSteeringAngle(firstNode,theta,nextNode,L)

    # 2) Use that as input to kinematics
    # Old Euler Method vs more precice Runge-Kutta fourth order
    # dx, dy, dth = bicycleKinematics(theta,v,L,delta,dt)
    dx,dy,dth = bicycleKinematicsRK4(firstNode,theta,v,L,delta)

    thetanew    = theta        + dth * dt
    xnew        = firstNode[0] + dx * dt
    ynew        = firstNode[1] + dy * dt

    # TODO: limit distance travelled to eta
    if False:
        if math.sqrt(hori_dist**2 + vert_dist**2) > eta:
            angle = np.arctan2(vert_dist,hori_dist)
            hori_dist = eta * np.cos(angle)
            vert_dist = eta * np.sin(angle)

        return (firstNode[0] + hori_dist, firstNode[1] + vert_dist)
    
    # map thetanew to (0,2pi)
    return (xnew,ynew), np.mod(thetanew, 2 * np.pi)

###
### 
### 
def steerBicycleWithDynamics(firstNode,theta,nextNode,dt,velocity):
    '''
    Purpose:  Incorporates dynamic constraints on simple bicycle model
              Note that (x,y)-position is now calculated from CG instead of back wheel as in the kinematic version
              Model used is from [Pepy, Lambert, Mounier: 2006]
    
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

    # Scale to wanted length (course estimation to fit the test environment)
    scale = L / Lstd
    m = scale * mstd
    I = scale * Istd
    tire_radius = scale * tire_radius_std

    delta = calculateSteeringAngle(firstNode,theta,nextNode,L,maxx=np.pi/20)

    # vx is velocity in direction of theta
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
    thetanew = theta + rnew * dt
    dx = vx * np.cos(thetanew) - vy * np.sin(thetanew)
    dy = vx * np.sin(thetanew) + vy * np.cos(thetanew)
    
    vynew = vy + dvy * dt 
    vxnew = vx + dvx * dt
    xnew = firstNode[0] + dx * dt
    ynew = firstNode[1] + dy * dt
    
    newstate = (xnew,ynew)

    # map thetanew to (0,2pi)
    return newstate, np.mod(thetanew, 2 * np.pi), (vxnew,vynew,rnew)

###
### 
### 
def steeringFunction(steer_f, node_nearest,node_rand,node_dist,dt,eta=0.5):
    steered_velocity = (0.5,0.0,0.0)
    steered_theta = 0

    if steer_f == None:
      steered_node = steerPath(node_nearest.state, node_rand,node_dist,eta)

    if steer_f == False:
      # kinematic model
      steered_node, steered_theta = steerBicycleWithKinematics(node_nearest.state, node_nearest.theta, node_rand, dt)
      steered_velocity = (0.5,0.0,0.0)
    
    if steer_f == True:
      # dynamic model
      steered_node, steered_theta, steered_velocity = steerBicycleWithDynamics(node_nearest.state, node_nearest.theta, node_rand, dt, velocity=node_nearest.velocity)

    return steered_node,steered_theta,steered_velocity

###
### 
### 
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

    ## Increase and decrease u1,u2 in relation to previous ones and some kind of maximum rate of change (not realistic to suddenly change u1,u2 to be totally previous)

    return (u1,u2)

###
### 
### 
def steerWithQuadcopterDynamics(searchNode, nextNode, dt):
    '''
    searchNode  is supposed to be nodeNearest as SearchNode object

    '''
    # TODO: Make a function that samples u1,u2 relative to what searchNode had before

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

###
### 
### 
def obstacleIsInPath(firstNode,nextNode,env,radius):
    '''
    Purpose:  Checks for an obstacle in line between firstNode and nextNode
    :returns: A boolean for collision or not
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

###
### 
### 
def goalReached(node,radius,end_region):
    '''
    :params:    node is a tuple (xpos,ypos)
                radius is a scalar - size of the robot
                end_region is a polygon of four tuples drawn clockwise: lower left, upper left, upper right, lower right
    '''
    # returns a boolean for node tuple + radius inside the region
    return end_region.contains(Point(node))

###
###
###
def getMaximumNeighborRadius(ETA,no_nodes):
  '''
  Purpose:  Implements the maximum radius from [Karaman and Frazzoli, 2013]
            Note that (log(x) / x)**(0.5) is always lower than 0.61

  '''
  return ETA * math.sqrt(math.log(no_nodes) / no_nodes)
###
###
###
def nearestNeighbors(s_f, graph, newNode, k, ETA, no_nodes, radius=1):
    '''
    Purpose:  find tuple in graph that is closest to newNode
    
    :params:  s_f is steering function choice TODO: how to use it?
              graph is Graph object
              newNode is a SearchNode
    '''

    # Get all nearest neighbors according to number to look for, and use the computationally inefficient way of keeping calculations down from Karaman, 2013, RRT star for nonholonomic constraints by maximuzing radii to look within a ball 

    states = []
    it = 0
    loc_pos = 0
    for node in graph._nodes:
        states.append([node.state[0],node.state[1]])
        if node.state == newNode.state:
            loc_pos = it
        it += 1

    X = np.array(states)
    nbrs=NearestNeighbors(n_neighbors=k,radius=radius,algorithm='ball_tree').fit(X)

    distances, indices = nbrs.kneighbors(X)

    SN_list = [graph._nodes[ind] for ind in indices[loc_pos]]

    return SN_list

###
### 
### 
def minCostPath(s_f,k,SN_list,node_min,node_steered,env,r, max_rad=1):
  '''
  Purpose:  Finds the node that contributes to the cheapest path out of the 
            k-1 nearest neighbors

  '''
  if s_f == None:
    # For all neighbors, add the steered node at the spot where it contributes to the lowest cost. Start counting at 1 since pos 0 is the node itself
    for j in range(1,k):
      node_near = SN_list[j]
      if not obstacleIsInPath(node_near.state, node_steered.state, env,r):
        cost_near=node_near.cost+eucl_dist(node_near.state,node_steered.state)
        if cost_near < node_steered.cost:
          node_min = node_near

    # Update parent and cost accordingly
    node_steered.parent = node_min
    relative_distance = eucl_dist(node_min.state,node_steered.state)
    newcost = node_min.cost + relative_distance
    node_steered.cost = newcost
    return node_min, relative_distance

  elif s_f == False:
    anyReachable = False

    for j in range(1,k):
      node_near = SN_list[j]
      if not obstacleIsInPath(node_near.state, node_steered.state,env,r) and nodeIsReachable(node_near,node_steered):

        anyReachable = True

        cost_near=node_near.cost+kin_dist(node_near,node_steered)
        if cost_near < node_steered.cost:
          node_min = node_near
    
    node_steered.parent = node_min
    relative_distance = kin_dist(node_min,node_steered)
    newcost = node_min.cost + relative_distance
    node_steered.cost = newcost

    return node_min, relative_distance, anyReachable

###
###
###
def nodeIsReachable(node_steered,node_near,max_steer=math.pi/10):
  '''
  Purpose:  Tests to see if node_near is within the range of node_steered
  '''
  x1,y1 = node_steered.state
  x2,y2 = node_near.state
  dx = x2-x1 ; dy = y2 - y1
  rel_angle = np.arctan2( dy , dx ) # between -pi and pi
  if rel_angle > max_steer or rel_angle < - max_steer:
    return False

  return True

###
###
###
def rewire(ax,bounds,s_f,graph,node_min,node_steered,env,radius,SN_list,k,max_steer):
  '''
  Purpose:  Rewires graph to remove sub-optimal cost paths for k nearest 
            neighbors
  '''

  # SN_list gives all neighbors within max radius. Manipulate the list to only contain reachable nodes according to current body frame angle and max steering angle

  for j in range(1,k):
    node_near = SN_list[j] # gives all neighbors within max radius
    if True or node_near is not node_min:

      if s_f == False: # kinematic behavior
        if not nodeIsReachable(node_steered,node_near,max_steer):
          continue

      if not obstacleIsInPath(node_near.state, node_steered.state,env,radius):

        if s_f == False: # kinematic behavior
          newcost = node_steered.cost+kin_dist(node_steered,node_near)
        else: # holonomic behavior
          newcost = node_steered.cost+eucl_dist(node_steered.state,node_near.state)

        if newcost < node_near.cost:
          node_parent = node_near.parent
          graph.remove_edge(node_parent,node_near)
          node_near.parent = node_steered
          node_near.cost = newcost

          if s_f == False:
            dist = kin_dist(node_steered,node_near)
          else:
            dist = eucl_dist(node_steered.state,node_near.state)

          graph.add_edge(node_steered,node_near,dist)
          graph.updateEdges(node_near) # the node cost has changed 
