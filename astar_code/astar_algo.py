import Map
from scipy.spatial import distance

class Node:
    def __init__(self, pos = None, best_Parent = None):
        """"
        set g, h, and f to zero. Child node's g, h and f will later 
        be updated. Set a parent for the node and an empty list of its
        'children to be'
        """
        self.g = self.h = self.f = 0
        self.pos = pos
        self.best_Parent = best_Parent
        self.children = []
    
    def __eq__(self, other):
        """ Method for comparing nodes """
        return self.pos == other.pos
    
    def __gt__(self, other):
        """ Method for comparing nodes"""
        return (self.g + self.h) > (other.g + other.h)

def draw(map_obj, path):
    """ Draws the yellow path from start position to the goal position on the map"""
    for node_pos in path:
        map_obj.set_cell_value(node_pos, "Anything", True)

def cost(map_obj,next_pos):
    """ return the cost of moving from a
    node to another node """
    return map_obj.get_cell_value(next_pos)

def attach_best_parent(map_obj, child, parent, cost, heuristic):
    """ Attach the best known parent node to the child node"""
    child.best_parent = parent
    child.g = parent.g + cost(map_obj, child.pos)
    child.h = heuristic(map_obj, child.pos)    

def prop_impr(map_obj, parent):
    """ Propagate improved paths through the grid of nodes"""
    for child in parent.child:
        if parent.g + 1 < child.g:
            child.best_Parent = parent
            child.g = parent.g + cost(map_obj, parent.pos)
            child.f = child.g + child.h
            prop_impr(map_obj, child.pos)

def euclidean_distance(map_obj, pos):
    """" Calculate and return the Euclidean distance
    from this node to the goal node"""
    goal = map_obj.get_goal_pos()
    return abs(distance.euclidean(pos, goal))

def best_first_search(map_obj, heuristic, cost):
    """ Perform the A* best-first search"""
    # initiate lists to keep track of open and closed nodes
    open = []
    closed = []
    # Initiate start and end nodes
    start_node = Node(map_obj.get_start_pos())
    goal_node = Node(map_obj.get_goal_pos())

    # Put the start node in the open list, and start 
    # working through the nodes via the agenda loop
    open.append(start_node)

    # The agenda loop
    while open:
        temp = open.pop(0)
        closed.append(temp)
        # If the goal node is reached, return it.
        # Builds the path by working "backwards" 
        # from current node, and all parents all the
        # the way back to the start node
        if temp == goal_node:
            path = []
            current_node = temp
            while current_node is not None:
                path.append(current_node.pos)
                current_node = current_node.best_Parent
            return path
        # If we have not reached the goal node, start expanding the 
        # children of temp, and appending them to the open list 
        # of "nodes to explore"
        children = []

        for neighbour in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            child_pos = [temp.pos[0] + neighbour[0], temp.pos[1] + neighbour[1]]
            if cost(map_obj, child_pos) == -1:
                continue
        
            new_child = Node(child_pos, temp)
            children.append(new_child)

        # If the node is open, then it is the same node, but it is 
        # a child/parent of several nodes 
        # If the node is already closed, we are done with it. 
        for child in children:
            for node in open:
                if node == child:
                    child = node
            for node in closed:
                if node == child:
                    child = node
        
            temp.children.append(child)

            # if the child is not in open or closed, then the node temp 
            # is the best (known) node for this child, as it is the first
            # time we encounter it 
            if child not in open and child not in closed:
                attach_best_parent(map_obj, child, temp, cost, heuristic)
                open.append(child)
                open.sort()
            
            # if the cost of moving to the child plus the heuristic estimate to the 
            # goal is less than the cost of getting to the node
            # the child node attaches this as its best parent node
            elif temp.g + cost(map_obj, child.pos) < child.g:
                attach_best_parent(map_obj, child, temp, cost, heuristic)
                if child in closed:
                    prop_impr(map_obj, temp)
    return False