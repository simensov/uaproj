# Useful imports

from __future__ import division
from matplotlib import pyplot as plt
import numpy as np
import yaml
from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
from check_path import check_path

# NB! My own implementations and some of the classes used from earlier assignments are found in support_pset4.py
from support_pset4 import *

def rrt(bounds, environment, start_pose, radius, end_region):
    '''
    - bounds: (minx, miny, maxx, maxy) tuple over region
    - environment: instance of the Environment class that describes the placement of the obstacles in the scene
    - start_pose: start_pose = (x,y) is a tuple indicating the starting position of the robot
    - radius: radius is the radius of the robot (used for collision checking)
    - end_region: end_region is a shapely Polygon that describes the region that the robot needs to reach
    '''

    # Adding tuples to nodes -> represent all nodes expanded by tree
    nodes = [start_pose]

    # Creating graph of SearchNodes from the tuples to represent the tree with parents. Used to order and
    graph = Graph()
    graph.add_node(SearchNode(start_pose))
    goalPath = Path(SearchNode(start_pose)) # for initialization in case of no path found
    
    # Draw the environment (with its obstacles) and with the start node + end region
    ax = plot_environment(environment,bounds)
    plot_poly(ax, Point(start_pose).buffer(radius, resolution=3),'blue')
    plot_poly(ax, end_region,'red', alpha=0.2)
    
    for i in range(10000):  # random high number
        # this range must be evaluated according to size of problem. RRT do not ensure any solution paths
        
        # sample a random node inside bounds (random number between xmin/max and ymin/max). all bound-"checking" is donw here: makes sure we don't have to do ant checks later
        rand_xval = random.uniform(bounds[0],bounds[2])
        rand_yval = random.uniform(bounds[1],bounds[3])
        node_rand = (rand_xval,rand_yval)
        
        # for every x'th iteration - aim towards goal. a good value for this parameter varies
        # I tried to vary between 5 and 20 for the simple environment, and found no pattern due to the random sampling
        # a good choice also depends on the step length that I decided in STEERPATH
        # bu one thing is certain: not having this makes total number of nodes blow up
        # I have kept choices of when to goal bias and the travel distance in steering function to perform well in large environments
        
        if not(i % 15):
            node_rand = end_region.centroid.coords[0]

        # find out which node in our list that is the nearest to the random node
        node_nearest, node_dist = nearestSNode(graph, node_rand) # returning a searchNode and a float distance

        # steer towards the new node -> CORRECT PARENTS SO PATH CAN BE EXTRACTED DIRECTLY AND ADD COST FOR PATH LENGTH

        ## TODO: this steering function should somehow consider dynamics
        steered_node = steerPath(node_nearest.state, node_rand)
        
        if not (bounds[0] < steered_node[0] < bounds[2]) or not (bounds[1] < steered_node[1] < bounds[3]):
            continue # sometimes not checking for this made the path go out of bounds

        node_steered = SearchNode(steered_node,node_nearest, node_nearest.cost + node_dist)
        

        # check for nodes already in tree: we don't want a cycle
        if node_steered.state not in nodes:
            if not obstacleInPath(node_nearest.state, node_steered.state, environment,radius):

                # Add to list of tuples to be able to keep track of total number of nodes in tree
                nodes.append(node_steered.state)

                # ALL EDGES MUST HAVE WEIGHT SO WE CAN FIND LENGTH! This function also adds new nodes to graph
                graph.add_edge(node_nearest,node_steered,node_dist)
                
                # plot non-colliding edge to show all searched nodes
                line = LineString([node_nearest.state,node_steered.state])
                plot_line_mine(ax, line)

            else:
                continue #no point in checking the same node for goal all the time

        # check goal
        if goalReached(node_steered.state,radius,end_region):  # checking last addition
            goalPath = Path(node_steered)
            break # break the while loop when solution is found!

    # - the number of nodes in the tree - the number of nodes in the solution path - the path length
    numberOfTotalNodes = len(nodes); numberofNodesInSolution = len(goalPath.path); pathLength = goalPath.cost
    
    for i in range(numberofNodesInSolution-1):
        # Draw the original line
        line = LineString([goalPath.path[i], goalPath.path[i+1]])
        plot_line_mine(ax, line)
        
        # Draw the expanded line
        expanded_line = line.buffer(radius, resolution=3)
        plot_poly(ax, expanded_line, 'green', alpha=0.2)
        
    # plotting last node in goalPath and setting title to format in task
    plot_poly(ax, Point(goalPath.path[-1]).buffer(radius, resolution=3),'blue') 
    titleString = "Total number of nodes: %s - Number of nodes in solution path: %s \nPath length: %0.3f"
    ax.set_title( titleString % (numberOfTotalNodes,numberofNodesInSolution,pathLength))
    
    return goalPath.path


# Test your code in the simple environment
# don't modify this cell. It will be overwritten by the autograder.
# if you want to test things, please create additional cells at the end
environment = Environment('simple.yaml')
radius = 0.3
bounds = (-2, -3, 12, 8)
start = (0, 0)
goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])
path = rrt(bounds, environment, start, radius, goal_region)
# The check path function checks that the path is a list,
# its elements are tuples and the start and end are correct
# It doesn't check collisions.
check_path(path, bounds, environment, start, radius, goal_region)


# Test your code in the bugrap environment
# don't modify this cell. It will be overwritten by the autograder.
# if you want to test things, please create additional cells at the end
environment = Environment('bugtrap.yaml')
radius = 0.3
bounds = (-2, -3, 12, 8)
start = (2, 2.5)
goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])
path = rrt(bounds, environment, start, radius, goal_region)
# The check path function checks that the path is a list,
# its elements are tuples and the start and end are correct
# It doesn't check collisions.
check_path(path, bounds, environment, start, radius, goal_region)


# Test in moderate and hard environment
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


start = (-1 , -1)
radius = 0.1
goal_region = Polygon([(12,3), (12,4), (13,4),(13,3)])
bounds = (-5, -4, 15, 5)
envMed = random_environment(bounds, start, radius, goal_region, 40, (0.2, 0.4))
envHard = random_environment(bounds, start, radius, goal_region, 400, (0.2, 0.4))

# TEST MY SOLVER:
path = rrt(bounds, envMed, start, radius, goal_region)
    
# Test your code in the challenging environment that you made
path = rrt(bounds, envHard, start, radius, goal_region)
check_path(path, bounds, envHard, start, radius, goal_region)