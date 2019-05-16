# this file contains regular utilities for length measurement and plotting etc.
import yaml
from sklearn.neighbors import NearestNeighbors
import random
import numpy as np
import math
from environment import *
import scipy.interpolate as sinterpol
from colorama import Fore, Style
from shapely.geometry import Point, Polygon, LineString, box
import sys

###
###
###
def eucl_dist(a, b):
	"""Returns the euclidean distance between a and b."""
	# a is a tuple of (x,y) values of first point
	# b is a tuple of (x,y) values of second point
	return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

###
###
###
def eucl_dist_noSqrt(a,b):
	'''
	Purpose:    Save some computational power in nearestEuclSNode

	:params:    a and b is both (x,y) tuples with a and b being floats
	:returns:   The non-square-rooted distance for more efficient metric.
							Since the sqrt(x) is strictly increasing for all x >0, which also x is ofcourse, then we can sort based on x instead of calculating sqrt(x)
	'''
	return (a[0]-b[0])**2 + (a[1] - b[1])**2

###
###
###
def kin_dist(node1,node2):
	'''
	Purpose:    Attempts to achieve a better metric for finding node in graph 
							that is cheaper to navigate to instead of eucl_dist
	'''
	x1,y1 = node1.state
	x2,y2 = node2.state 
	dx = x2-x1
	dy = y2-y1

	# node2 is steered node after steering process, so it has an angle
	abs_angle = math.fabs(node1.theta - node2.theta)
	angle_cost = abs_angle**2 / (2*math.pi)

	return math.sqrt( dx**2 + dy**2 +  angle_cost)

###
###
###
def nearestEuclSNode(graph, newNode):
	'''
	Purpose:    Finds the nearest node in graph of newNode based on eucledian distance in 2D
	'''
	# TODO: improve the metric here? Norm seems very naive

	# returning tuple in nodeList that is closest to newNode
	# nearestNodeInGraph = SearchNode((1,1)) # initialize for return purpose
	dist = eucl_dist((-10000,-10000),(10000,10000)) # huge distance
	for i in graph._nodes: # iterating through a set. i becomes a SEARCH NODE
			newdist = eucl_dist_noSqrt(i.state, newNode)
			if newdist <= dist: #SEARCH NODE.state is a tuple
					# changed to <= because in some cases, this search only sampled the startnode, with no velocity obviously, so nothing would move
					nearestNodeInGraph = i # a search
					dist = newdist
					newdistout = eucl_dist(i.state, newNode)

	return nearestNodeInGraph, newdistout

###
###
###
def plot_line_mine(ax, line,width=2,color='black'):
	# wanted to draw lines in path more clearly. gathered format from environment.py
	x, y = line.xy
	ax.plot(x,y,color=color,linewidth=width,solid_capstyle='round',zorder=1)

###
###
###
def bspline_planning(x, y, sn):

	N = 3
	t = range(len(x))
	x_tup = sinterpol.splrep(t, x, k=N)
	y_tup = sinterpol.splrep(t, y, k=N)

	x_list = list(x_tup)
	xl = x.tolist()
	x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

	y_list = list(y_tup)
	yl = y.tolist()
	y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

	ipl_t = np.linspace(0.0, len(x) - 1, sn)
	rx = sinterpol.splev(ipl_t, x_list)
	ry = sinterpol.splev(ipl_t, y_list)

	return rx, ry

###
###
###
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

###
###
###
def plotBsplineFromList(ax,tupleList,bounds,sn=100):
	x = []; y = []
	for i in range(len(tupleList)-1):
		x.append(tupleList[i][0])
		y.append(tupleList[i][1])

	plot_bspline(ax,x,y,bounds,sn)


###
###
###
def plotListOfTuples(ax,tupleList,width=1, color="green"):
	for i in range(len(tupleList)-1):
		x = tupleList[i][0], tupleList[i+1][0]
		y = [tupleList[i][1], tupleList[i+1][1]]
		ax.plot(x, y, linewidth = width, color = color)

###
###
###
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

def printRRTstar():
	print(Fore.WHITE + Style.BRIGHT)
	print('     ____________________________')
	print('    /                           /\\ ')
	print('   / '+Fore.RED+'RRT*'+Fore.WHITE+' in'+'             /*    / /\\')
	print('  /  '+Fore.BLUE+'Pyt'+Fore.YELLOW+'hon'+Fore.WHITE+'     _     ___|    / /\\')
	print(' /           o_/ \___/       / /\\')
	print('/___________________________/ /\\')
	print('\___________________________\/\\')
	print(' \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\ \\'
				+ Style.RESET_ALL + Style.BRIGHT)
	print(Fore.WHITE+ Style.RESET_ALL)

###
###
###
def plotEdges(ax,graph,color='black',width=1):
	x = []; y = []
	for key in graph._edges:
			edgething = graph._edges[key]
			for edge in edgething:
					x1,y1 = edge.source.state
					x2,y2 = edge.target.state
					ax.plot( [x1,x2] , [y1,y2] , color=color, linewidth=width)

###
###
###
def plotNodes(ax,graph,color='red', size=5,radius=0.3):
	for node in graph._nodes:
		plot_poly(ax,Point((node.state[0], node.state[1])).buffer(radius/3,resolution=5),color="red",alpha=.8)
			#ax.plot([node.state[0]], [node.state[1]], marker='o', markersize=size, color=color,alpha=0.8)

###
###
###
def drawEdgesLive(ax,environment,radius,node_steered,new_node,graph,bounds = (0,0,1,1), seconds=1,colorOnOther="green",start_pos=(0,0),end_region=(10,10)):


	#ax.cla()
	
	plotNodes(ax,graph)
	plot_environment_on_axes(ax,environment,bounds)
	#plot_poly(ax,Point(start_pos).buffer(radius,resolution=5),'blue',alpha=.2)
	#plot_poly(ax, end_region,'red', alpha=0.2)
	plot_poly(ax,Point(node_steered.state).buffer(0.1,resolution=5),'blue',alpha=.6)
	plot_poly(ax,Point(new_node.state).buffer(0.1,resolution=5),colorOnOther,alpha=.8)

	plotEdges(ax,graph)
	plt.draw()
	plt.pause(seconds)

###
###
###
def drawEdgesLiveLite(ax,env,radius,node_steered,new_node,graph,color="green"):
	left, right = ax.get_xlim()  # return the current xlim
	bottom, top = ax.get_ylim()  # return the current xlim
	ax.cla()


	plotNodes(ax,graph)
	# plot_environment_on_axes(ax,env)
	plot_poly(ax,Point(node_steered.state).buffer(radius/3,resolution=5),'blue',alpha=.6)
	plot_poly(ax,Point(new_node.state).buffer(radius/3,resolution=5),color =color,alpha=.8)

	plotEdges(ax,graph)
	ax.set_xlim(left,right)
	ax.set_ylim(bottom,top)
	plt.draw()
	plt.pause(0.01)


# Print iterations progress
def printProgressBar(iteration, total, prefix='', suffix='', decimals=1, bar_length=50):
	"""
	# Made by Aubrey Taylor - username: aubricus
	# https://gist.github.com/aubricus/f91fb55dc6ba5557fbab06119420dd6a


	Call in a loop to create terminal progress bar
	@params:
			iteration   - Required  : current iteration (Int)
			total       - Required  : total iterations (Int)
			prefix      - Optional  : prefix string (Str)
			suffix      - Optional  : suffix string (Str)
			decimals    - Optional  : positive number of decimals in percent complete (Int)
			bar_length  - Optional  : character length of bar (Int)
	"""
	str_format = "{0:." + str(decimals) + "f}"
	percents = str_format.format(100 * (iteration / float(total)))
	filled_length = int(round(bar_length * iteration / float(total)))
	bar = '█' * filled_length + '-' * (bar_length - filled_length)

	sys.stdout.write('\r%s |%s| %s%s %s' % (prefix, bar, percents,'%',suffix)),

	if iteration == total:
		sys.stdout.write('\n')

	sys.stdout.flush()

###
###
###
def degToRad(degrees):
	return degrees * math.pi / 180

###
###
###
def dotProd(theta,x):
	'''
	Purpose:    Take dot product without having to convert into numpy arrays

	:params:    theta and x are list of tuples with same amount of elements
							Can be different data types as long as their are as big

	:returns:   Dot product. Can be interpreted as signed distance * ||theta||

	'''
	assert len(theta) == len(x)
	prod = 0
	for i in range(len(theta)):
			prod += theta[i] * x[i]
	return prod

###
###
###
def plotEllipse(ax,ellipse,color="#BC3E23",increment=1):
	'''
	Purpose:	Plots an Ellipse object onto Axes object

	:params:	ax is the Axes object from matplotlib
						ellipse is the Ellipse object
	'''
	xc, yc = ellipse.getCenterPos()
	th = ellipse.getOrientation()
	x = [] ; y = []
	for deg in range(0,360,increment):
		radian = degToRad(deg)
		radius = ellipse.getRadius(radian) 
		x.append( xc + radius * math.cos(radian + th) )
		y.append( yc + radius * math.sin(radian + th) )

	ax.plot(x,y,color=color, linewidth=3)
	#ax.scatter(x,y,color=color,markersize=1)
