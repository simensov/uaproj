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

plotAll = True
realTimePlotting = False
onePath = False
plotAllFeasible = True
noFeas = 100
RRTSTAR = True
STEERING_FUNCTION = None # None is none, False is kinematic, True is dynamic
GOAL_BIAS = True
MAX_ITER = 3000
NEIGHBORS = 9
PAUSETIME = 0.1

################################
################################
################################
############# RRT ##############
################################
################################
################################

def rrt(bounds, env, start_pose, radius, end_region, start_theta=3.14):

    # Adding tuples nodes-list -> represent ALL nodes expanded by tree
    nodes = [start_pose]

    # TODO: better to add theta in state than have it as extra parameter all the time    
    # Searchgraph used
    graph = Graph()
    start_node = SearchNode(start_pose,theta=start_theta)
    graph.add_node(start_node)
    goalPath = Path(start_node)
    bestPath = goalPath
    bestCost = 1000000
    feasible_paths = []
    goalNodes = []
    
    dt = 0.4
    
    # Draw the environment 
    ax = plot_environment(env,bounds)
    plot_poly(ax,Point(start_pose).buffer(radius,resolution=5),'blue',alpha=.2)
    plot_poly(ax, end_region,'red', alpha=0.2)
    
    
    sampling_rate = 30
    for i in range(MAX_ITER):  # random high number
        print("Iteration no. ", i) if i%(1000) == 0 else None

        node_rand = getRandomNode(bounds,sampling_rate,i,GOAL_BIAS,end_region)

        node_nearest, node_dist = nearestEuclSNode(graph, node_rand) 

        steered_state,steered_theta,steered_velocity = steeringFunction(STEERING_FUNCTION, node_nearest,node_rand,node_dist,dt)

        if not withinBounds(steered_state,bounds):
          continue
      
        node_dist = eucl_dist(node_nearest.state,steered_state)

        node_steered = SearchNode(steered_state,node_nearest,node_nearest.cost+node_dist,theta=steered_theta,velocity=steered_velocity)
        
        if node_steered.state not in nodes:
            if not obstacleIsInPath(node_nearest.state, node_steered.state, env,radius):

                nodes.append(node_steered.state)
                graph.add_node(node_steered)
                node_min = node_nearest
                
                if realTimePlotting:
                  if plotAll:
                    plotNodes(ax,graph)
                    plt.draw()
                    plt.pause(PAUSETIME)
                    #input("Drawing nodes")

                if RRTSTAR:
                  k = NEIGHBORS # keep odd to avoid ties
                  # this parameter to be updated according to the log shit
                  # look for neighbors when we have enough nodes
                  if(len(graph._nodes) > k): 
                    dist, SN_list = nearestEuclNeighbor(graph,node_steered,k);

                    node_min, rel_dist = minimumCostPathFromNeighbors(k, SN_list, node_min, node_steered,env,radius)

                    graph.add_edge(node_min,node_steered,rel_dist)

                    if realTimePlotting:
                      drawEdgesLive(ax,env,bounds,start_node.state,end_region,radius,node_steered,node_min,graph,PAUSETIME,'red')
                      #input("Drawing cheapest edge, iteration" + str(i))


                    # rewiring the remaining neighbors
                    for j in range(1,k):
                      node_near = SN_list[j]
                      if node_near is not node_min:
                        if not obstacleIsInPath(node_near.state, node_steered.state,env,radius):

                          newcost = node_steered.cost + eucl_dist(node_steered.state,node_near.state)

                          if (node_near.cost > newcost):

                            # remove parenthood and edge with old parent
                            node_parent = node_near.parent

                            if realTimePlotting:
                              # TODO: this is used for debugging
                              print("")
                              print("###########")
                              print("###########    REWIRING at iteration", i)
                              print("###########")
                              print("")
                              input("Watch the removal and adding process")
                              print("Remove node :", node_parent)
                              print("as parent of:", node_near)
                              print("Replace with:", node_steered)
                              print("")
                              #print("\nEdges from near before replacement:")
                              #print(graph.node_edges(node_near))

                              graph.remove_edge(node_parent,node_near)
                              drawEdgesLive(ax,env,bounds,start_node.state,end_region,radius,node_steered,node_near,graph,0.01,"green")

                              node_near.parent = node_steered
                              node_near.cost = newcost
                              dist=eucl_dist(node_steered.state,node_near.state)
                              graph.add_edge(node_steered,node_near,dist)
                              # update the edges since node has been changed
                              graph.updateEdges(node_near)
                              print("\nEdges from near after replacement")
                              print(graph.node_edges(node_near))
                              input("Drawing rewire: should have removed edge")
                            else:
                              # just remove the edge
                              graph.remove_edge(node_parent,node_near)
                              node_near.parent = node_steered
                              node_near.cost = newcost
                              dist=eucl_dist(node_steered.state,node_near.state)
                              graph.add_edge(node_steered,node_near,dist)
                              # update the edges since the node has changed
                              graph.updateEdges(node_near)

                            '''
                            node_near.parent = node_steered
                            node_near.cost = newcost
                            dist=eucl_dist(node_steered.state,node_near.state)
                            graph.add_edge(node_steered, node_near,dist)
                            '''

                            if realTimePlotting:
                              drawEdgesLive(ax,env,bounds,start_node.state,end_region,radius,node_steered,node_near,graph,0.01,"green")
                              input("Drawing rewire: added edge")
                                               
                  else:
                    # not enough points to begin nearest neighbor yet
                    graph.add_edge(node_nearest,node_steered,node_dist)
                    
                    if realTimePlotting:
                      plotNodes(ax,graph)
                      plotEdges(ax,graph)
                      plt.draw()
                      plt.pause(PAUSETIME)
                      # input("Drawing edges")
                  
                else:
                  # No RRT Star - Don't consider nearest or rewiring
                  graph.add_edge(node_nearest,node_steered,node_dist)

                    
            else:
                 # Avoid goal check if collision is found
                continue

        else: # the node has already been sampled
          continue

        # Check last addition for goal state
        if goalReached(node_steered.state,radius,end_region):  
            goalPath = Path(node_steered)
            goalNodes.append(node_steered)
            bestPath = Path(node_steered)

            print("Path",len(feasible_paths),", cost:",goalPath.cost,", it:",i)

            if(False):
              plotNodes(ax,graph)
              plotEdges(ax,graph)
            
            if onePath:
              break
            else:

              sampling_rate = sampling_rate*2 # turn of goal sampling and focus on improving path

              if not RRTSTAR:
                if len(feasible_paths) > noFeas:
                    break

              ## Important that there is a new init of Path object each time 

              feasible_paths = [Path(node) for node in goalNodes]
              costs = [pathObj.cost for pathObj in feasible_paths]
              idx =  costs.index(min(costs))
              bestPath = feasible_paths[idx]

              if plotAllFeasible:
                if min(costs) < bestCost:
                  bestCost = min(costs)

                  ax.cla()
                  plotListOfTuples(ax,bestPath.path)
                  plot_environment_on_axes(ax,env,bounds)
                  plot_poly(ax,Point(start_pose).buffer(radius,resolution=5),'blue',alpha=.2)
                  plot_poly(ax, end_region,'red', alpha=0.2)
                  
                  # input("Found better path. Hit enter to draw")
                  print("Found better path")
                  ax.set_title("Best cost out of " + str(len(goalNodes)) + " goal nodes: " + str(bestCost))
                  plt.draw()
                  plt.pause(0.0001)

                  if realTimePlotting:
                    plt.draw()
                    plt.pause(0.0001)
                    input("Found path")

                else: # not best cost, but update no. of paths found
                  ax.set_title("Best cost out of " + str(len(goalNodes)) + " goal nodes: " + str(bestCost))
                  plt.draw()
                  plt.pause(0.0001)

              if not RRTSTAR and not onePath:
                #restart search
                nodes = [start_pose]
                graph = Graph()
                graph.add_node(SearchNode(start_pose,theta=start_theta))
                goalPath = Path(SearchNode(start_pose,theta=start_theta))
    

    return bestPath, ax

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
  bounds = (-2, -3, 12, 8)
  goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])

  if(False):
      # COMPARABLE TO QUADWORLD
      environment = Environment('env_superbug.yaml')
      start = (0, 0)
      path, ax = rrt(bounds, environment,start, radius, goal_region,start_theta=np.pi * 0.8)

  if(True):
      #environment = Environment('env_slit.yaml')
      environment = Environment('env_empty.yaml')
      start = (-1, 3)
      goal_region = Polygon([(11,7), (11,8), (12,8), (12,7)])
      goalPath, ax = rrt(bounds, environment, start, radius, goal_region,start_theta=0)

# important detail!!! don't use the cost of the last searchnode as final cost. That cost was the cost of the path the searchnode was a part of when it found the goal the first time, but the path might have changed after that, so the distance metric is used to readjust the path cost when a Path.cost is called!

plotInFunction = False
plotListOfTuples(ax,goalPath.path)
q = input("Goal path should be found with cost " + str(goalPath.cost) +"\n Enter 'q' for anything else then plotting goalpath")


ax.set_title("Best cost out of "+ str(goalPath.cost))

plt.close() if q == 'q' else plt.show()

plt.close()