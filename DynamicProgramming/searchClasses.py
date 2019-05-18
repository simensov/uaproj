# searchClasses.py - Oliver Stugard Os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# These classes are used for the tree that the algorithm holds. Nodes denote states that are added to the tree, edges are trajectories between them. Standard tree functions for removing nodes, edges, adding nodes and edges, in addition to functions that gives the best path to goal, draws the path to goal, and draws the entire tree

class Node:

    def __init__(self,
        state,
        time,
        cost_to_go,
        num_parents,
        parent = None,
        incoming_edge = None):

        self._state = state
        self._time = time
        self._lower_bound = time + cost_to_go
        self._upper_bound = 1000000
        self._parent = parent
        self._num_parents = num_parents
        self._incoming_edge = incoming_edge


    @property
    def parent(self):
        return self._parent
    @property
    def state(self):
        return self._state

    @property
    def time(self):
        return self._time

    @property
    def lower_bound(self):
        return self._lower_bound

    @property
    def upper_bound(self):
        return self._upper_bound

    @property
    def incoming_edge(self):
        return self._incoming_edge

    @property
    def num_parents(self):
        return self._num_parents

    @parent.setter
    def parent(self,parent):
        self._parent = parent

    @incoming_edge.setter
    def incoming_edge(self,edge):
        self._incoming_edge = edge

    @upper_bound.setter 
    def upper_bound(self,bound):
        self._upper_bound = bound


    def __eq__(self, other):
        return isinstance(other, Node) and self._state == other._state and self._upper_bound == other._upper_bound and self._lower_bound == other._lower_bound

    def __hash__(self):
        return hash(self._state)

    def __gt__(self, other):
        return self._time > other._cost

    def __repr__(self):
        return "<SearchNode (id: %s), state: (%0.2f,%0.2f,%0.2f), cost: %0.2f, parent: %s>" % (id(self), self._state[0],self._state[1],self._state[2], self._cost, id(self.parent))


class NodeNotInGraph(Exception):
    def __init__(self, node):
        self.node = node

    def __str__(self):
        return "Node %s not in graph." % str(self.node)

class Edge(object):
    def __init__(self, source, target, control_input, states):
        self._source = source
        self._target = target
        self._control_input = control_input
        self._states = states
        self._cost = len(states)

    
    @property
    def source(self):
        return self._source

    @property
    def target(self):
        return self._target

    @property
    def control_input(self):
        return self._control_input


    @property
    def states(self):
        return self._states

    @property
    def cost(self):
        return self._cost

    
    def __hash__(self):
        return hash("%s_%s_%f" % (self._source, self._target, self._cost))

    def __eq__(self, other):
        return self.source == other.source and self.target == other.target 
               #and self.weight == other.weight # TODO: OK??

    def __repr__(self):
        return "Edge(\n %r \n %r \n %r \n)" % (self.source, self.target, self._cost)


class Graph(object):
    def __init__(self,goal,obstacles):
        self._nodes = list()
        self._edges = dict()
        self._goal = goal
        self._obstacles = obstacles
        self._upper_bound = 100000000

    @property
    def nodes(self):
        return self._nodes

    @property
    def edges(self):
        return self._edges

    @property
    def goal(self):
        return self._goal

    @property
    def upper_bound(self):
        return self._upper_bound

    @upper_bound.setter
    def upper_bound(self,upper_bound):
        self._upper_bound = upper_bound
    
    
    def __contains__(self, node):
        return node in self._nodes


    def add_node(self, node):
        """Adds a node to the graph."""
        # the function gets called when add_edge is called, so just check that we do not add several nodes 
        if not node in self._nodes:
            self._nodes.append(node) # NB: CHANGED THIS FROM .add(node)


    def add_edge(self, node1, node2,control_input,states):
        """Adds an edge between node1 and node2. Adds the nodes to the graph first if they don't exist."""
        self.add_node(node1)
        self.add_node(node2)

        #Get all the nodes from the the parent node
        node1_edges = self._edges.get(node1, list())

        #Define the edge
        edge = Edge(node1, node2, control_input, states)

        #Add the edge to the tree
        node1_edges.append(edge)
        self._edges[node1] = node1_edges

        #Add the edge to the childnode as an incoming edge
        node2.incoming_edge = edge


    def remove_edge(self, node1, node2):
        removed = False
        if node1 in self._edges.keys():
            edgeset = self._edges[node1]

            for edge in edgeset:
                if edge.target == node2:
                    try:
                        self._edges[node1].remove(edge)
                    except:
                        break

                    removed = True
                    break
        return removed

    def remove_node(self,node):

        if node in self._edges.keys():

            edges = self._edges[node]

            for edge in edges:
                target = edge.target
                try: 
                    self._edges[node].remove(edge)
                except:
                    print("Couldn't remove edge!")
                self.remove_node(target)
        if node in self._nodes:
            self._nodes.remove(node)



    def prune(self):

        for node in self._nodes:
            if node.lower_bound > self._upper_bound:
                self.remove_node(node)


    def node_edges(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self._edges.get(node, set())

    def get_root(self):
        for node in self._nodes:
            if node.parent == None:
                return node
        return None

    #Returns the control sequence and the state sequence from the starting node to the goal, given that the goal node is in the tree
    def getGoalPath(self):

        #Tries to find the goal node in the three
        node = None
        cost = 10000000
        for n in self._nodes:
            if n.state[0] == self._goal[0] and n.state[1] == self._goal[1]:
                if n.time < cost:
                    node = n
                    cost = n.time

        node_sequence = []

        #If the goal node exists in the tree
        if node is not None:

            #Add all parents of the goal node to the tree
            while node is not None:
                node_sequence.append(node)
                node = node.parent

            #Reverse the list
            node_sequence.reverse()

            #Define the state and control sequencce
            state_sequence = []
            control_sequence = []

            #Loop through all nodes in the node sequence 
            for i in range(len(node_sequence) - 1):
                node = node_sequence[i]
                next_node = node_sequence[i+1]

                #Find the edge that connects the current and next node
                for edge in self._edges[node]: 
                    if edge.source.state == node.state and edge.target.state == next_node.state:

                        #If the edge is found, add the states of the edge to the state sequence and the control input of the given trajectory to the control sequence
                        state_sequence.append(edge.states)
                        control_sequence.append(edge.control_input)

            #Returns the total control sequence and the state sequence
            return control_sequence,state_sequence

        #Return false if the goal does not exist in the three
        return False


    def drawGoalPath(self,iters,time):

        node = None
        cost = 100000
        for n in self._nodes:
            if n.state[0] == self._goal[0] and n.state[1] == self._goal[1]:
                if n.time < cost:
                    node = n
                    cost = n.time

        node_sequence = []
        state_sequence = []
        control_sequence = []

        #If the goal node exists in the tree
        if node is not None:

            #Add all parents of the goal node to the tree
            while node is not None:
                node_sequence.append(node)
                node = node.parent

            #Reverse the list
            node_sequence.reverse()

            #Define the state and control sequencce

            #Loop through all nodes in the node sequence 
            for i in range(len(node_sequence) - 1):
                node = node_sequence[i]
                next_node = node_sequence[i+1]

                #Find the edge that connects the current and next node
                for edge in self._edges[node]: 
                    if edge.source.state == node.state and edge.target.state == next_node.state:

                        #If the edge is found, add the states of the edge to the state sequence and the control input of the given trajectory to the control sequence
                        state_sequence.append(edge.states)
                        control_sequence.append(edge.control_input)
                        break

            #Returns the total control sequence and the state sequence

        x_points = []
        y_points = []
        th_points = []

        for state_seq in state_sequence:
            test = False

            for point in state_seq:
                x_points.append(point[0])
                y_points.append(point[1])
                th_points.append(point[2])

        dist = 0

        for i in range(len(x_points) -1):
            x1 = x_points[i]
            x2 = x_points[i+1]
            y1 = y_points[i]
            y2 = y_points[i+1]
            dist += ((x2 - x1)**2 + (y2-y1)**2)**0.5


        fig,ax = plt.subplots()

        for node in node_sequence:
            ax.plot(node.state[0],node.state[1],'ro',color = "blue") 

        ax.plot(x_points,y_points)

        for ob in self._obstacles:
            xy = (ob[0],ob[2])  
            width = ob[1] - ob[0]
            height = ob[3] - ob[2]
            angle = 0
            rect = plt.Rectangle(xy,width,height)
            plt.gca().add_patch(rect)

        x = self.goal[0]
        y = self.goal[1]
        ax.scatter(x,y,marker = 'x',color = "green",s = 10.)
        ax.text(x,y - 0.5,s = "Goal",fontsize = 14)
        root = self.get_root()
        x = root.state[0]
        y = root.state[1]
        ax.scatter(x,y,marker = "x",color = "red",s = 10.)
        ax.text(x,y-0.5,s = "Source",fontsize = 14)



        textstr = '\n'.join((
        r'$\mathrm{iters}=%.1f$' % (iters, ),
        r'$\mathrm{steps}=%.1f$' % (self._upper_bound, ),
        r'$\mathrm{length}=%.1f$' % (dist, ),
        r'$\mathrm{time}=%.2f$' % (time, )))

        ax.hist(x, 50)
        # these are matplotlib.patch.Patch properties
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)

        # place a text box in upper left in axes coords
        ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14,
            verticalalignment='top', bbox=props)


        plt.show()

        #Return false if the goal does not exist in the three
        return True


    def drawTree(self,iters):

        nodeList = []

        fig,ax = plt.subplots()

        for node,edges in self._edges.items():
            nodeList.append(node)

        for node in self._nodes:

            if node in nodeList:

                x = node.state[0]
                y = node.state[1]

                plt.plot(x,y,'ro',markersize = 3.5,color = "red")

                edges = self._edges[node]

                for edge in edges:

                    x_points = []
                    y_points = []
                    th_points = []

                    for point in edge.states:
                        x_points.append(point[0])
                        y_points.append(point[1])
                        th_points.append(point[2])

                    ax.plot(x_points,y_points,linewidth = 0.5,color = "blue")

        for ob in self._obstacles:
            xy = (ob[0],ob[2])  
            width = ob[1] - ob[0]
            height = ob[3] - ob[2]
            angle = 0
            rect = plt.Rectangle(xy,width,height)
            plt.gca().add_patch(rect)

        x = self.goal[0]
        y = self.goal[1]
        ax.scatter(x,y,marker = 'x',color = "green",s = 10.)
        ax.text(x,y - 0.5,s = "Goal",fontsize = 14)
        root = self.get_root()
        x = root.state[0]
        y = root.state[1]
        ax.scatter(x,y,marker = "x",color = "red",s = 10.)
        ax.text(x,y-0.5,s = "Source",fontsize = 14)

        textstr = '\n'.join((
        r'$\mathrm{iters}=%.1f$' % (iters, ),))

        ax.hist(x, 50)
        # these are matplotlib.patch.Patch properties
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)

        # place a text box in upper left in axes coords
        ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14,
            verticalalignment='top', bbox=props)

        plt.show()




