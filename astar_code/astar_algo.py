from Map import *

class Node():

    def __init__(self, parent=None, pos=None):
        self.h = 0
        self.g = 0
        self.f = 0
        self.parent = parent
        self.pos = pos
        self.moves = []
        
    def getMovesPos():
        x = Node.pos[0]
        y = Node.pos[1]
        Node.moves.append([x+1,y])
        Node.moves.append([x-1,y])
        Node.moves.append([x,y+1])
        Node.moves.append([x,y-1])
        
class Astar():
    def __init__(self, task):
        self.map = Map_Obj(task)
        

    def algorithm(self):
        open = []
        closed = []
        start_node = Node(None, self.map.get_start_pos())
        start_node.h = start_node.g = start_node.f = 0
        open.append(start_node)
        end_node = Node(None, self.map.get_end_goal_pos())

        """Loop"""
        while len(open)>0:
            current_node = open[0]
            current_index = 0
            for index, item in enumerate(open):
                if item.f < current_node.f:
                    current_node=item
                    current_index=index
                
            open.pop(current_index)
            closed.append(current_node)

            """GoalChecker"""
            if current_node == end_node:
                path=[]
                current= current_node
                while current is not None:
                    path.append(current.pos)
                    current_node = current.parent
                return path[::-1]
            
            childrenList = current_node.getMovesPos()
            children = []
            for item in enumerate(childrenList):
                childrenNode = Node(current_node, item)
                children.append(childrenNode)

            for node in children:
                for item in closed:
                    if item == node:
                        continue
                node.g=current_node.g + 1
                """Pythogoras"""
                node.h= ((node.pos[0]-end_node.pos[0])**2 + (node.pos[1]-end_node.pos[1])**2)
                node.f=node.g+node.h
            
                for item in open:
                    if node == item and node.g > item.g:
                        continue
                open.append(node) 
        


def main():
    test=Astar(1)
    map = test.map.int_map
    start = test.map.get_start_pos()
    end = test.map.get_end_goal_pos()
    path = test.algorithm()
    print(path)

if __name__=="__main__":
    main()
