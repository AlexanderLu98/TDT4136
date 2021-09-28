import Map

class search_Node:
    def __init__(self, position = None, best_Parent = None):
        
        self.g = 0 #cost of getting to this node
        self.h = 0 #estimated cost to goal
        self.f = self.g + self.h #estimated total cost of a solution path going through this node
        self.position = position
        self.best_Parent = best_Parent
        self.kids = []
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __gt__(self, other):
        return (self.g + self.h) > (other.g + other.h)

def manhattan_distance(map_obj, position):
    """ Makes an admissible estimate cost goal """

    goal = map_obj.get_goal_pos()
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])
    
def euclidean_distance(map_obj, position):
    pass

def cost_function(map_obj,next_pos):
    """ Determines cost of a cell so that different cell costs are taken
        account of in the total cost """

    return map_obj.get_cell_value(next_pos)



def attach_and_eval(map_obj,child, parent, cost_function, heuristic_func):
    """ Attaches a child node to a node that is now considered its best parent """
    
    child.best_parent = parent
    child.g = parent.g + cost_function(map_obj, child.position)
    child.h = heuristic_func(map_obj, child.position)

def propagate_path_improvements(map_obj, parent):
    """ Propagation of path improvements through children and possible
        many more descendants """

    for kid in parent.kid:
        if parent.g + 1 < kid.g:
            kid.best_Parent = parent
            kid.g = parent.g + cost_function(map_obj, parent.position)
            kid.f = kid.g + kid.h
            propagate_path_improvements(map_obj, kid.position)


def best_first_search(map_obj, heuristic_func, cost_func):
    """ Implementation of best first search based on handed out pseudocode """
    
    closed = []
    open_ = []
    #Generate initial node
    start_node = search_Node(map_obj.get_start_pos())
    goal_node = search_Node(map_obj.get_goal_pos())

    start_node.g = 0
    start_node.h = heuristic_func(map_obj, start_node.position)
    start_node.h = start_node.g + start_node.h
    open_.append(start_node)

    #Agenda loop
    while open_:
        X = open_.pop(0)
        closed.append(X)
        if X == goal_node: #returns path when goal node is found
            path = []
            current_node = X
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.best_Parent
            return path

        kids = []
        directions = {
            "N": [0,1],
            "S": [0,-1],
            "W": [-1,0],
            "E": [1,0]
            }

        for direction in directions: #generating the nodes kids
            kid_pos = [X.position[0] + directions[direction][0], X.position[1] + directions[direction][1]]
            if cost_function(map_obj, kid_pos) == -1:
            #if map_obj.int_map[kid_pos[0]][kid_pos[1]] == -1:
                continue
        
            new_sn = search_Node(kid_pos, X)
            kids.append(new_sn)

        for kid in kids:
            for node in open_:
                if node == kid:
                    kid = node
            for node in closed:
                if node == kid:
                    kid = node
        
            X.kids.append(kid)

            if kid not in open_ and kid not in closed:
                attach_and_eval(map_obj, kid, X, cost_function, heuristic_func)
                open_.append(kid)
                open_.sort()
            
            elif X.g + cost_function(map_obj, kid.position) < kid.g:
                attach_and_eval(map_obj, kid, X, cost_function, heuristic_func)
                if kid in closed:
                    propagate_path_improvements(map_obj, X)
    return False

def draw_path(map_obj, path):
    for node_pos in path:
        map_obj.set_cell_value(node_pos, "Yellow", True)
        

def main():
    map_obj = Map.Map_Obj(task = 1)
    start_position = map_obj.get_start_pos()
    int_map, str_map = map_obj.get_maps()
    goal_position = map_obj.get_goal_pos()

    map_obj.show_map()
    
    path = best_first_search(map_obj, manhattan_distance, cost_function)
    print("Starting position: ", start_position)
    print("Goal position: ", goal_position)


    try:
        draw_path(map_obj, path)
        print("Drawing path...")
    except TypeError:
        print("Path does not exist :( ")

    map_obj.show_map()


if __name__ == "__main__":
    main()