# RRT.py - Simen Sem Oevereng

from __future__ import division
import time
from support import * # imports utils and searchClasses as well

################################
################################
########### Globals ############
################################
################################

### Plotting params
plotAll           = False
realTimePlotting  = False
stepThrough       = False
onePath           = True
restartRRT        = False
plotAllFeasible   = False
noFeas            = 100
pauseDuration     = 0.001
rewireDuration    = 0.001

### Config params
MAX_ITER          = 5000
RRTSTAR           = False
INFORMED          = False
PATH_BIAS         = False
PATH_BIAS_RATE    = MAX_ITER # Not used for now
STEERING_FUNCTION = None # None is none, False is kinematic, True is dynamic
GOAL_BIAS         = True
GOAL_BIAS_RATE    = 30
MIN_NEIGH         = 3
MAX_NEIGH         = 50
MAX_STEER         = math.pi / 10
ETA               = 0.5 # 0.5 standard, only relevant for holonomic
DT                = 0.1

################################
################################
############# RRT ##############
################################
################################

def rrt(ax, bounds, env, start_pos, radius, end_region, start_theta=3.14):

    # Adding tuples nodes-list -> represent ALL nodes expanded by tree
    nodes = [start_pos]
    graph = Graph()
    start_node = SearchNode(start_pos,theta=start_theta)
    graph.add_node(start_node)
    goalPath = Path(start_node)
    bestPath = goalPath # path object
    bestCost = 1000000
    feasible_paths = []
    goalNodes = []
    info_region = None
    sampling_rate = GOAL_BIAS_RATE

    printRRTstar() if RRTSTAR else printRRT()
    print("Looking for path through",MAX_ITER,"iterations")
    for i in range(MAX_ITER):  # random high number
        printProgressBar(i,MAX_ITER,"Progress",suffix=("/ iteration "+str(i)))
        
        node_rand = getRandomNode(bounds,i,GOAL_BIAS,GOAL_BIAS_RATE,end_region,INFORMED,info_region)

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

                if RRTSTAR:
                  # limit nearest neighbor search to be 10% of the entire graph
                  k = no_nodes-1 if no_nodes < MAX_NEIGH else int(no_nodes*0.1)
                  if (no_nodes > k and k > 0): 

                    # Find node that connects to node_steered through the cheapest collision-free path
                    max_n_radius = getMaximumNeighborRadius(ETA,no_nodes)

                    SN_list = nearestNeighbors(STEERING_FUNCTION,graph,node_steered,k, ETA, no_nodes, max_n_radius)

                    node_min,rel_dist,ok = minCostPath(STEERING_FUNCTION,k,SN_list,node_min,node_steered,env,radius)

                    if not ok: # no reachable nodes
                      continue

                    graph.add_edge(node_min,node_steered,rel_dist)

                    # Rewiring the remaining neighbors
                    rewire(ax,bounds,STEERING_FUNCTION,graph,node_min,node_steered,env,radius,SN_list,k,MAX_STEER)

                    if False and plotAll and not i%500:
                      plotEdges(ax,graph)
                      if realTimePlotting:
                        plt.draw()
                        plt.pause(0.01)

                  else:
                    # Not enough points to begin nearest neighbor yet
                    graph.add_edge(node_nearest,node_steered,node_dist)
       
                                      
                else:
                  # No RRT Star - Don't consider nearest or rewiring
                  graph.add_edge(node_nearest,node_steered,node_dist)

                  if plotAll and realTimePlotting:

                    plot_poly(ax,Point(node_steered.state).buffer(radius/3,resolution=5),color='blue',alpha=.6)

                    plt.draw()
                    plt.pause(0.01)
                    input("Enter to continue")

                    plot_poly(ax,Point(node_min.state).buffer(radius/3,resolution=5),color="red",alpha=.8)
                    plotNodes(ax,graph)
                    plotEdges(ax,graph)
                    plt.draw()
                    plt.pause(0.01)
                    input("Enter to continue")

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
              if INFORMED and plotAll:
                # just to see what the ellipse is doing - testing plot etc.
                info_region = Ellipse()
                info_region.generateEllipseParams(goalPath.path)
                info_region.plot(ax)

              return goalPath

            else: # we allow looking for more paths

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
                bestNode = goalNodes[idx]

                if INFORMED:
                  info_region = Ellipse()
                  info_region.generateEllipseParams(bestPath.path)
                  sampling_rate = 1.1 * sampling_rate # bias less and less

                if plotAll:
                  plotListOfTuples(ax,bestPath.path)

                  if INFORMED:
                    # note that the end plot might show the goal path outside of the ellipse. This is just because of the rewiring
                    info_region.plot(ax)

                  if realTimePlotting:
                    plt.draw()
                    plt.pause(0.01)

                print("\nNew best cost: %0.3f" % bestCost + " ")

              if not RRTSTAR and not onePath and restartRRT:
                # Restart search and continue as long as the iterations allows
                nodes = [start_pos]
                graph = Graph()
                graph.add_node(SearchNode(start_pos,theta=start_theta))
                goalPath = Path(SearchNode(start_pos,theta=start_theta))

    if goalNodes != []:
      feasible_paths = [Path(node) for node in goalNodes]
      costs = [pathObj.cost for pathObj in feasible_paths]
      idx =  costs.index(min(costs))
      bestPath = feasible_paths[idx]

    if False and plotAll and not restartRRT:
      plotEdges(ax,graph)

    return bestPath

################################
################################
############ Tests #############
################################
################################

plots = True
if(plots):
  radius = 0.3
  xmin = -2; ymin = -3; xmax = 12; ymax = 8
  bounds = (xmin,ymin,xmax,ymax)
  goal_region = Polygon([(11,7), (11,8), (12,8), (12,7)])

  # cost of 16.35 is perfect lines
  environment = Environment('env_superbug.yaml'); start=(0,0) ; s_th=3*np.pi/4
  #environment = Environment('env_slit.yaml');     start=(-1,1); s_th=0
  #environment = Environment('env_empty.yaml'); start=(-1,1); s_th=0

  
  ax = plot_environment(environment,bounds)
  ax.set_xlim(xmin,xmax) 
  ax.set_ylim(ymin,ymax)

  st = time.time()
  goalPath=rrt(ax,bounds,environment,start,radius,goal_region,start_theta=s_th)

print(("\nTook %0.3f" % (time.time()-st)) + "seconds. \nPlotting...")
plot_poly(ax,Point(start).buffer(radius,resolution=5),'blue',alpha=.3)
plot_poly(ax, goal_region,'red', alpha=0.3)

plotListOfTuples(ax,goalPath.path,width=1.5)
#plotBsplineFromList(ax,goalPath.path,bounds,sn=200) # for faster plotting

time_used = DT * len(goalPath.path)
ax.set_title("Best cost, time: %0.3f units in %0.1f s" % (goalPath.cost, time_used) )

q = input("Goal path should be found with cost " + str(goalPath.cost) +"\n Enter 'q' for anything else then plotting goalpath")

plt.close() if q == 'q' else plt.show()
plt.close()
