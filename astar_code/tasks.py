import astar_algo as aStar
import Map as m

def main():
    """ Solves the tasks with visual confirmation """
    tasks = [1, 2, 3, 4]

    #Iterates the tasks and solves them respectively
    for task in tasks:
        mappy = m.Map_Obj(task=task)
        # shows the map before using the A* algorithm as a staring point
        mappy.show_map()
        path = aStar.best_first_search(mappy, aStar.euclidean_distance, aStar.cost)

        # tries to draw the visual representation of the task,
        # if successfull it will display a PNG file with the correct path
        # if failure it will print out an error message
        try:
            aStar.draw(mappy, path)
        except TypeError:
            print("ERROR: no path")
        mappy.show_map()


if __name__ == "__main__":
    main()