from __future__ import division
from matplotlib import pyplot as plt
import numpy as np
import yaml
from shapely.geometry import Point, Polygon, LineString, box
from environment import *
import time

from support import *
import dubins

plotAll = False
if plotAll:
  realTimePlotting = True

onePath = False
plotAllFeasible = False
noFeas = 100
RRTSTAR = True


def rrt(bounds, environment, start_pose, radius, end_region, start_theta=3.14):
    '''
    - bounds: (minx, miny, maxx, maxy) tuple over region
    - environment: instance of the Environment class that describes the placement of the obstacles in the scene
    - start_pose: start_pose = (x,y) is a tuple indicating the starting position of the robot
    - radius: radius is the radius of the robot (used for collision checking)
    - end_region: end_region is a shapely Polygon that describes the region that the robot needs to reach
    '''

    # Adding tuples nodes-list -> represent ALL nodes expanded by tree
    nodes = [start_pose]

    # TODO: better to add theta in state than have it as extra parameter all the time    
    # Searchgraph used
    graph = Graph()
    graph.add_node(SearchNode(start_pose,theta=start_theta))
    goalPath = Path(SearchNode(start_pose,theta=start_theta))
    bestPath = goalPath
    feasible_paths = []
    
    # Draw the environment 
    ax = plot_environment(environment,bounds)
    plot_poly(ax,Point(start_pose).buffer(radius,resolution=5),'blue',alpha=.2)
    plot_poly(ax, end_region,'red', alpha=0.2)
    
    GOAL_BIAS = True
    max_iter = 3000
    tenth = max_iter / 10
    sampling_rate = 30
    for i in range(max_iter):  # random high number
        if i%(1000) == 0:
            print("Iteration no. ", i)

        # Random sampling
        # TODO: also sample theta? -> maybe for dubins path
        rand_xval = random.uniform(bounds[0],bounds[2])
        rand_yval = random.uniform(bounds[1],bounds[3])
        node_rand = (rand_xval,rand_yval)

        if i==0: # force first node to look directly towards starting theta
          node_rand = (start_pose[0]+np.cos(start_theta),start_pose[1]+np.sin(start_theta))
        
        dt = 0.4
        # Goal bias
        
        if not(i % sampling_rate) and (not i==0) and (GOAL_BIAS):
            node_rand = end_region.centroid.coords[0]

        node_nearest, node_dist = nearestEuclSNode(graph, node_rand) 
        # node_nearest is a SearchNode() with both .state: (x,y) and .theta

        steer_f = None
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
      
        # boundary check  
        if not (bounds[0] < steered_node[0] < bounds[2]) or not (bounds[1] < steered_node[1] < bounds[3]):
            continue # sometimes not checking for this made the path go out of bounds

        node_steered = SearchNode(steered_node,node_nearest,node_nearest.cost+node_dist,theta=steered_theta,velocity=steered_velocity)
        
        if node_steered.state not in nodes:
            if not obstacleIsInPath(node_nearest.state, node_steered.state, environment,radius):

                nodes.append(node_steered.state)
                ## or for RRT* to work:
                graph.add_node(node_steered)
                node_min = node_nearest

                if RRTSTAR:
                  k = 10 # this has to be updated according to the log shit
                  if(len(graph._nodes) > k):
                    dist, SN_list = nearestEuclNeighbor(graph,node_steered,k);

                    for j in range(1,k):
                      node_near = SN_list[j]
                      if not obstacleIsInPath(node_near.state, node_steered.state, environment,radius):

                        cost_near = node_near.cost + eucl_dist(node_near.state,node_steered.state)

                        if cost_near < node_steered.cost:
                          node_min = node_near

                    node_steered.parent = node_min
                    rel_dist = eucl_dist(node_min.state,node_steered.state)
                    newcost = node_min.cost + rel_dist
                    node_steered.cost = newcost

                    graph.add_edge(node_min,node_steered,rel_dist)

                    # rewiring the remaining neighbors
                    for j in range(1,k-1):
                      node_near = SN_list[j]
                      if node_near is not node_min:
                        if not obstacleIsInPath(node_near.state, node_steered.state,environment,radius):

                          newcost = node_steered.cost + eucl_dist(node_steered.state,node_near.state)

                          if (node_near.cost > newcost):
                            # TODO: think about what is going on here
                            node_parent = node_near.parent
                            # remove edge between parent and near
                            # TODO: removes edges both ways
                            graph.remove_edge(node_parent,node_near)
                            graph.add_edge(node_steered, node_near)

                
                else:
                  # NO RRT START - NO CONSIDERING NEAREST OR REWIRING
                  graph.add_edge(node_nearest,node_steered,node_dist)

                if(plotAll):
                    #line = LineString([node_min.state,node_steered.state])
                    #plot_line(ax, line)
                    if realTimePlotting:
                      if not (len(graph._nodes) % 1000):
                        ax.cla()
                        plot_environment_on_axes(ax,environment,bounds)
                        plot_poly(ax,Point(start_pose).buffer(radius,resolution=5),'blue',alpha=.2)
                        plot_poly(ax, end_region,'red', alpha=0.2)
                        plotEdges(ax,graph,bounds)
                        
                        if GOAL_BIAS == False:
                        # means that goal path has been set
                          x = []; y = []
                          for tup in bestPath.path:
                            x.append(tup[0]); y.append(tup[1])

                          plot_bspline(ax,x,y,bounds,100)

                        plt.draw()
                        plt.pause(0.0001)
                    
            else:
                continue # Avoid goal check if collision is found

        else: # the node has already been sampled
          continue

        # Check last addition for goal state
        if goalReached(node_steered.state,radius,end_region):  
            goalPath = Path(node_steered)
            feasible_paths.append(goalPath)
            bestPath = goalPath
            print("Path",len(feasible_paths),", cost:",goalPath.cost,", it:",i)
            


            if onePath: # used if I just want to check something realquick
              break
            else:

              # GOAL_BIAS = False
              sampling_rate = int(sampling_rate / 2.)

              if len(feasible_paths) > noFeas:
                  break

              costs = [path.cost for path in feasible_paths] # list of costs: sort paths
              if len(costs):
                  idx =  costs.index(min(costs))
                  bestPath = feasible_paths[idx]

              if plotAll:
                ax.cla()
                plot_environment_on_axes(ax,environment,bounds)
                plot_poly(ax,Point(start_pose).buffer(radius,resolution=5),'blue',alpha=.2)
                plot_poly(ax, end_region,'red', alpha=0.2)
                plotEdges(ax,graph,bounds)
                x = []; y = []
                for tup in bestPath.path:
                  x.append(tup[0]); y.append(tup[1])

                plot_bspline(ax,x,y,bounds,100)
                ax.set_title("Best cost:" , min(costs))
                plt.draw()
                plt.pause(0.0001)


              # restart search
              #nodes = [start_pose]
              #graph = Graph()
              #graph.add_node(SearchNode(start_pose,theta=start_theta))
              #goalPath = Path(SearchNode(start_pose,theta=start_theta))
    
    #####
    ##### AFTER MAX ITER OR X PATHS FOUND
    #####

    # No. of nodes in total - No. of nodes in sol path - Sol path length
    noOfTotalNodes = len(nodes); 
    noOfNodesInSol = len(goalPath.path); 
    pathLength = goalPath.cost

    ax.cla()
    plot_environment_on_axes(ax,environment,bounds)
    plot_poly(ax,Point(start_pose).buffer(radius,resolution=5),'blue',alpha=.2)
    plot_poly(ax, end_region,'red', alpha=0.2)
    plotEdges(ax,graph,bounds)
    
    if True:
      for i in range(noOfNodesInSol-1):
          # Draw best goal path
          line = LineString([goalPath.path[i], goalPath.path[i+1]])
          plot_line_mine(ax, line, width=2.2)    
          #expanded_line = line.buffer(radius/2, resolution=3) #Draw expanded line
          # plot_poly(ax, expanded_line, 'green', alpha=0.1)
    
    else:
      # Approx 360 times faster (checked) 
      if len(goalPath.path) > 1:
        x = []; y = []
        for tup in goalPath.path:
          x.append(tup[0]); y.append(tup[1])

        plot_bspline(ax,x,y,bounds,100)

    # plotting last node in goalPath and setting title to format in task
    l_pos = goalPath.path[-1]
    plot_poly(ax, Point(l_pos).buffer(radius, resolution=5),'blue',alpha=.2)
    titleString = "Nodes total / in solution path: %s/ %s \nPath length: %0.3f"
    ax.set_title(titleString % (noOfTotalNodes,noOfNodesInSol,pathLength))

    return goalPath.path

################################
################################
################################
################################ Tests
################################
################################
################################

plots = True
if(plots):
  radius = 0.3
  bounds = (-2, -3, 12, 8)
  goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])

  if(False):
      # SUPERSIMPLE
      environment = Environment('supersimple.yaml')
      start = (0, 0)
      path = rrt(bounds, environment,\
        start, radius, goal_region,start_theta=0)

  if(False):
      # COMPARABLE TO QUADWORLD
      environment = Environment('env_superbug.yaml')
      start = (0, 0)
      path = rrt(bounds, environment,start, radius, goal_region,start_theta=np.pi * 0.8)

  if(False):
      # BUGTRAP
      environment = Environment('bugtrap.yaml')
      start = (2, 2.5)
      path = rrt(bounds, environment, start, radius, goal_region,start_theta=np.pi)

  if(True):
      environment = Environment('env_slit.yaml')
      start = (-1, 3)
      goal_region = Polygon([(11,7), (11,8), (12,8), (12,7)])
      path = rrt(bounds, environment, start, radius, goal_region,start_theta=0)

  start = (-1 , -1); radius = 0.1; bounds = (-5, -4, 15, 5)
  goal_region = Polygon([(12,3), (12,4), (13,4),(13,3)])
  envMed=random_environment(bounds, start, radius, goal_region, 40, (0.2,0.4))
  envHard=random_environment(bounds, start, radius, goal_region,400,(0.2,0.4))

  # Test in different environments
  if(False):
      print("Test medium environment")
      path = rrt(bounds, envMed, start, radius, goal_region,start_theta=0)

  if(False):
      print("Test hard environment")
      path = rrt(bounds, envHard, start, radius, goal_region)


plt.show() if plots else print("Finished. Chosen to show no plots")


####################
#################### Maybe use later?
####################

if False:
    import scipy.interpolate as interp

    def interpolate_polyline(polyline, num_points):
        duplicates = []
        for i in range(1, len(polyline)):
            if np.allclose(polyline[i], polyline[i-1]):
                duplicates.append(i)
        if duplicates:
            polyline = np.delete(polyline, duplicates, axis=0)
        tck, u = interp.splprep(polyline.T, s=0)
        u = np.linspace(0.0, 1.0, num_points)
        return np.column_stack(interp.splev(u, tck))

        B = interpolate_polyline(A, 100)

    # for dubins. was hard to use
    start = goalPath.path[0]
    # dubins_paths = np.array([start])
    turning_radius = 0.25
    step_size = 0.3
    p = goalPath.path
    dubins_paths = []
    for i in range(len(p)-1):
      q0 = (p[i][0], p[i][1], goalPath.thetas[i])
      q1 = (p[i+1][0], p[i+1][1], goalPath.thetas[i+1])
      path = dubins.shortest_path(q0, q1, turning_radius)
      configurations, _ = path.sample_many(step_size) # list of three tuples x,y,theta
      dubins_paths.append(configurations)

    print(p)
    for path in dubins_paths:
      print(path)
      for i in range(len(path) - 1):
        p1 = (path[i][0], path[i][1])
        p2 = (path[i+1][0], path[i+1][1])
        ax.plot(p1,p2)
