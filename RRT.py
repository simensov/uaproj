from __future__ import division
import time
from support import * # also importing 'utils' and 'searchClasses'

################################
################################
################################
########### Globals ############
################################
################################
################################

### Plotting params
plotAll           = False
realTimePlotting  = False
stepThrough       = False
onePath           = False
plotAllFeasible   = False
noFeas            = 100
pauseDuration     = 0.001
rewireDuration    = 0.001

### Config params
RRTSTAR           = True
INFORMED          = False 
# TODO: reduce sample region once goal has been found
# Create line between start and end to get angle
# Then, use a "margin"-definition to find the points furthest away in the local coordinate system! Create ellipse based on this

STEERING_FUNCTION = False # None is none, False is kinematic, True is dynamic
GOAL_BIAS         = True
GOAL_BIAS_RATE    = 30
MAX_ITER          = 5000
MIN_NEIGH         = 3
MAX_NEIGH         = 50
ETA               = 0.5
DT                = 0.1

################################
################################
################################
############# RRT ##############
################################
################################
################################

printRRT()

def rrt(ax, bounds, env, start_pos, radius, end_region, start_theta=3.14):

    # Adding tuples nodes-list -> represent ALL nodes expanded by tree
    nodes = [start_pos]

    graph = Graph()
    start_node = SearchNode(start_pos,theta=start_theta)
    graph.add_node(start_node)
    goalPath = Path(start_node)
    bestPath = goalPath
    bestCost = 1000000
    feasible_paths = []
    goalNodes = []

    for i in range(MAX_ITER):  # random high number
        print("Iteration no. ", i) if i%(100) == 0 else None

        node_rand = getRandomNode(bounds,GOAL_BIAS_RATE,i,GOAL_BIAS,end_region)
        node_nearest, node_dist = nearestEuclSNode(graph, node_rand) 
        steered_state,steered_theta,steered_velocity = steeringFunction(STEERING_FUNCTION,node_nearest,node_rand,node_dist,DT,ETA)

        if not withinBounds(steered_state,bounds):
          continue
      
        # update dist after steering
        node_dist = eucl_dist(node_nearest.state,steered_state) 
        node_steered = SearchNode(steered_state,node_nearest,node_nearest.cost+node_dist,theta=steered_theta,velocity=steered_velocity)
        
        if node_steered.state not in nodes:
            if not obstacleIsInPath(node_nearest.state, node_steered.state, env,radius):

                nodes.append(node_steered.state)
                graph.add_node(node_steered)
                node_min = node_nearest

                no_nodes = len(graph._nodes)
                print("Nodes:", no_nodes) if not no_nodes % 100 else None

                if RRTSTAR:
                  # limit nearest neighbor search to be 10% of the entire graph
                  k = no_nodes-1 if no_nodes < MAX_NEIGH else int(no_nodes*0.1)
                  if (no_nodes > k and k > 0): 

                    # Find node that connects to node_steered through the cheapest collision-free path
                    max_n_radius = getMaximumNeighborRadius(ETA,no_nodes)

                    SN_list = nearestNeighbors(STEERING_FUNCTION,graph,node_steered,k, ETA, no_nodes, max_n_radius)

                    node_min,rel_dist = minCostPath(STEERING_FUNCTION,k,SN_list,node_min,node_steered,env,radius)

                    graph.add_edge(node_min,node_steered,rel_dist)

                    # Rewiring the remaining neighbors
                    rewire(ax,graph,node_min,node_steered,env,radius,SN_list,k,max_n_radius)


                  else:
                    # Not enough points to begin nearest neighbor yet
                    graph.add_edge(node_nearest,node_steered,node_dist)
       
                                      
                else:
                  # No RRT Star - Don't consider nearest or rewiring
                  graph.add_edge(node_nearest,node_steered,node_dist)

                  if plotAll and realTimePlotting:
                    ax.plot( [node_min.state[0], node_steered.state[0]], [node_min.state[1], node_steered.state[1]])
                    plt.draw()
                    plt.pause(0.01)


                    
            else:
                # Avoid goal check if collision is found
                continue

        else: 
          # The node has already been sampled
          continue

        # Check last addition for goal state
        if goalReached(node_steered.state,radius,end_region):  
            goalPath = Path(node_steered)
            goalNodes.append(node_steered)
            
            if onePath:
              return goalPath

            else:

              if not RRTSTAR:
                if len(feasible_paths) > noFeas:
                    break

              ## Important that there is a new init of Path object each time 
              feasible_paths = [Path(node) for node in goalNodes]
              costs = [pathObj.cost for pathObj in feasible_paths]
              idx =  costs.index(min(costs))
              bestPath = feasible_paths[idx]

              if min(costs) < bestCost:
                bestCost = min(costs)
                print("New best cost: %0.3f" % bestCost)

              if not RRTSTAR and not onePath:
                # Restart search and continue as long as the iterations allows
                nodes = [graphstart_pos]
                graph = Graph()
                graph.add_node(SearchNode(start_pos,theta=start_theta))
                goalPath = Path(SearchNode(start_pos,theta=start_theta))

    print("Plotting all edges")
    if plotAll:
      plotEdges(ax,graph)
    return bestPath

################################
################################
################################
############ Tests #############
################################
################################
################################

plots = True
if(plots):
  radius = 0.3
  xmin = -2; ymin = -3; xmax = 12; ymax = 8
  bounds = (xmin,ymin,xmax,ymax)
  goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])

  #environment = Environment('env_slit.yaml')
  environment = Environment('env_empty.yaml')
  start = (-1, 3)
  goal_region = Polygon([(11,7), (11,8), (12,8), (12,7)])
  st = time.time()

  ax = plot_environment(environment,bounds)
  ax.set_xlim(xmin,xmax)
  ax.set_ylim(ymin,ymax)

  goalPath=rrt(ax,bounds,environment,start,radius,goal_region,start_theta=0)

print("Took", time.time() - st , "seconds. Plotting...")

plot_poly(ax,Point(start).buffer(radius,resolution=5),'blue',alpha=.2)
plot_poly(ax, goal_region,'red', alpha=0.2)
plotListOfTuples(ax,goalPath.path)
ax.set_title("Best cost: "+ str(goalPath.cost))

q = input("Goal path should be found with cost " + str(goalPath.cost) +"\n Enter 'q' for anything else then plotting goalpath")

plt.close() if q == 'q' else plt.show()
plt.close()

################################
########## TODO's ##############

# In the rewiring step, we search for nearest neighbors once. that search output a LIST of objects that has their cost. But if there are more than one object in the list that could get their paths updated, and they are both connected, then the second 

# A weird example that might not actually be feasible. x is new node. the lowest right one is the one in 

# (1) The two 2-nearest neighbors of x is the two to the right. Their distances are stored. 
# 
# o -- x    o   
#  \       /
#   \     o 
#    \   /
#     \ /
#      o

# (2) The 2-nearest neighbor give the middle one as closest, and rewires that to give it a new cost. But notice that the cost of the rightmost will also have changed here, to the better. 
#
# o -- x     o
#  \    \   /
#   \     o 
#    \   
#     \ 
#      o

# (3) The rewiring will happen anyways, but the rightmost node was evaluated by its old cost! In this case it was obviously shorter to rewire, but could this not be the case in other cases?
#
# o -- x -- o   
#  \    \ 
#   \    o 
#    \   
#     \ 
#      o
