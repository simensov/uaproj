# support.py
# this file contains class definitions and calculation functions for the motion planning problems

# Contains general tools for distance calculation and plotting
from utils import *
# Contains SearchNode, Graph, Edge, Path classes
from searchClasses import *

##############################################
##############################################
##############################################
##############################################
# ----         Implementations          ---- #
##############################################
##############################################
##############################################
##############################################


def steerPath(firstNode,nextNode,dist):
    # tuples as input - distance between them gives ish distance to move

    # avoiding errors when sampling node is too close
    if dist == 0:
        dist = 100000

    hori_dist = nextNode[0] - firstNode[0] #signed
    verti_dist = nextNode[1] - firstNode[1] #signed
    dist = dist
    # a new node that are closer to the next node - always smaller than the boundary-checked nextNode parameter
    # I chose to implement a slow but working solution -> normalize the distance to move. This could easily be changed
    return (firstNode[0] + hori_dist/dist, firstNode[1] + verti_dist/dist)



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

###
### 
### 
def steerBicycleWithKinematics(firstNode,theta,nextNode,dt):
    '''
    TODO: The sexiest thing to do here would be to make a bicycle class
    Purpose:    Tests implementation of kinematic constraints by using a simple bicycle model

    TODO: if delta could be stored, a more realistic implementation would be to constrain delta considering the steering angle already made!

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
    
    # map thetanew to (0,2pi)
    return (xnew,ynew), np.mod(thetanew, 2 * np.pi)

###
### 
### 
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

###
### 
### 
def steeringFunction(steer_f, node_nearest,node_rand,node_dist,dt):
    steered_velocity = (0.5,0.0,0.0)
    steered_theta = 0

    if steer_f == None:
      steered_node = steerPath(node_nearest.state, node_rand,node_dist)

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

    ## TODO: increase and decrease u1,u2 in relation to previous ones and some kind of maximum rate of change (not realistic to suddenly change u1,u2 to be totally previous)

    return (u1,u2)

###
### 
### 
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

###
### 
### 
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
def minimumCostPathFromNeighbors(k,SN_list,node_min,node_steered,env,radius):

   # for all neighbors, add the steered node at the spot where it contributes to the lowest cost
    # start counting at 1 since pos 0 is the node itself
    for j in range(1,k):
      node_near = SN_list[j]

      if not obstacleIsInPath(node_near.state, node_steered.state, env,radius):

        cost_near=node_near.cost+eucl_dist(node_near.state,node_steered.state)

        if cost_near < node_steered.cost:
          node_min = node_near

    # update parent and cost accordingly. Has been tested with prints and plots and should be working fine!
    node_steered.parent = node_min
    rel_dist = eucl_dist(node_min.state,node_steered.state)
    newcost = node_min.cost + rel_dist
    node_steered.cost = newcost

    return node_min, rel_dist