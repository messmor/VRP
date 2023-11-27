

def plot_routes(routes, loads):
    import matplotlib.pyplot as plt
    import numpy as np
    """
    Plots a list of routes on a 2D plane. Each route is a list of (x, y) points.
    Each route is plotted in a unique color. The plot is centered at (0, 0).

    Args:
    routes (list of list of tuples): List of routes, where each route is represented
                                    as a list of (x, y) points.
    """
    # convert routes


    plt.figure()
    colors = plt.cm.jet(np.linspace(0, 1, len(routes)))  # Generates a color for each route

    for route, color in zip(routes, colors):
        # convert route to cartesian coords
        print("route", route)
        route = [loads[int(x)-1].convert_2_tuple()[i] for i in [0, 1] for x in route]
        route = [[0, 0]] + route + [[0, 0]]
        # Unzip the list of points into two lists of x and y coordinates
        x, y = zip(*route)

        # Plot points
        plt.scatter(x, y, color=color)

        # Plot directed lines between points
        for i in range(len(route) - 1):
            plt.arrow(route[i][0], route[i][1], route[i+1][0] - route[i][0], route[i+1][1] - route[i][1],
                      head_width=0.05, head_length=0.1, fc=color, ec=color)


    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.title('Routes Plot')
    plt.grid(True)
    plt.show()



if __name__ == "__main__":
    from Solutions.greedySolution import greedyBasicVRP
    from Optimization.destroyAndRepairOptimization import destroyAndRepairSolver
    from Utils.evaluateShared import loadProblemFromFile
    file_path = "../Training Problems/problem2.txt"
    loads = loadProblemFromFile(file_path).loads
    solution = greedyBasicVRP(file_path)
    plot_routes(solution, loads)
    # new_solution = destroyAndRepairSolver(file_path)
    # plot_routes(new_solution, loads)

