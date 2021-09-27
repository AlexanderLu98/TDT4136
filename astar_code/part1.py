import Map
import astar_algo as a

def main():
    """ Displays maps and solutions for task 1 and task 2 """
    tasks = [1, 2]

    for task in tasks:
        map_obj = Map.Map_Obj(task = task)
        start_position = map_obj.get_start_pos()
        goal_position = map_obj.get_goal_pos()
        int_map, str_map = map_obj.get_maps()

        map_obj.show_map() # Show map with start and goal

        path = a.best_first_search(map_obj, a.manhattan_distance, a.cost_function, False)

        print("Starting position: ", start_position)
        print("Goal position: ", goal_position)

        try:
            a.draw_path(map_obj, path)
            print("Drawing path...")
        except TypeError:
            print("Path does not exist :( ")

        map_obj.show_map() # Show map with path 

if __name__ == "__main__":
    main()