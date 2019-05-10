# searchClasses.py
# this file contains the implementation of the search classes used in RRT

from utils import eucl_dist

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
        self._state     = state       # Standard version: tuple (x,y)
        self._u         = u
        self._cost      = cost
        self._theta     = theta       # Positive anti-clockwise, wrt. x-axis
        self._velocity  = velocity 
        self._length    = length
        self._m         = mass
        self._inertia   = inertia

    def __repr__(self):
        return "<SearchNode (id: %s), state: (%0.2f,%0.2f), cost: %0.2f, parent: %s>" % (id(self), self.state[0],self.state[1], self.cost, id(self.parent))

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
        # TODO: cost can be updated by eucleidan measurments after reversing!!
        self.cost = 0
        for i in range(len(self.path)-1):
            self.cost += eucl_dist(self.path[i], self.path[i+1])

        # TODO: old one was self.cost = search_node.cost

    def __repr__(self):
        return "<Path: %d elements, cost: %.3f: %s>" % (len(self.path), self.cost, self.path)

    def edges(self):
        return zip(self.path[0:-1], self.path[1:])


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
               #and self.weight == other.weight # TODO: OK??

    def __repr__(self):
        return "Edge(\n %r \n %r \n %r \n)" % (self.source, self.target, self.weight)

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
        print("Removing edge from",id(node1),"to",id(node2))
        removed = False
        if node1 in self._edges:
            edgeset = self._edges[node1]
            print("Edges from", node1)
            for edge in edgeset:
                print(edge)
                print("Target:",edge.target)

            for edge in edgeset:
                if edge.target == node2:
                    print("Found :", node2, "as target")
                    try:
                        self._edges[node1].remove(edge)
                    except:
                        print("Didn't find edge",id(node1),id(node2))
                        break

                    removed = True
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