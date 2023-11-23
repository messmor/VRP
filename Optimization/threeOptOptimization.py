import itertools
import numpy as np
from pathlib import Path
from Utils.evaluateShared import loadProblemFromFile, distanceBetweenPoints, Load, Point


# predefined rules for the VRP
depot = Load(id='0', pickup=Point(0,0), dropoff=Point(0, 0)) # treat the depot as a load (location where drivers must start and end shift)
max_route_dst = 12 * 60 # drivers have a max shift of 12 hours, time in minutes == Euclidean dst


def compute_distance_matrix(loads):
    """create a matrix with dst_mat[i, j] = distance from load i to load j plus length of load j."""
    # add the depot as the first load
    loads.insert(0,depot)
    num_loads = len(loads)
    dst_mat = np.zeros((num_loads, num_loads))
    for start in range(num_loads):
        for end in range(num_loads):
            dst_mat[start, end] = distanceBetweenPoints(loads[start].dropoff, loads[end].pickup) + loads[end].load_dist()

    return dst_mat


def compute_route_length(route, dst_mat):
    """computes distance of a route using values stored in the distance matrix.
    inputs: -route a list of string route indices ex ['1', '4', '10']
            - dist_mat numpy array the distance matrix for the problem
    outputs: -route_dst a float the total distance of the route
    """
    # modify route so it starts and ends at depot
    if route[0] != '0':
        route.insert(0, '0')
    if route[-1] != '0':
        route.append('0')

    route_dst = 0
    for i in range(len(route) - 1):
        start_load = int(route[i])
        end_load = int(route[i+1])
        route_dst += dst_mat[start_load, end_load]

    return route_dst


def threeOptSwitch(route, i, j, k):

    assert '0' not in [i, j, k]
    new_route = route[:i] + route[i:j][::-1] + route[j:k][::-1] + route[k:]
    return route


def threeOptRoute(route, dst_mat):
    """Implements 3-opt optimization to improve a given route or schedule with in the
    larger solution.
    inputs: -route a list on str ids ex ['1', '4', '5']
    outputs: -best_route a new route with shorter distance.
    """
    # run optimization
    best_route = route
    best_distance = compute_route_length(route, dst_mat)
    decreasing = True
    while decreasing:
        decreasing = False
        # run through all switches that don't move first or last stop (these are the depot)
        for i, j, k in itertools.combinations(range(1, len(best_route)-1), 3):
            new_route = threeOptSwitch(best_route, i, j, k)
            new_distance = compute_route_length(new_route, dst_mat)
            if new_distance < best_distance:
                best_route = new_route
                best_distance = new_distance
                decreasing = True

    return best_route



def threeOptVRPSolution(file_path, solution):
    """Implements 3-opt optimization to improve a given VRP solution.
    inputs: -file_path a string path to the problem .txt file
            -solution a starting solution to the vrp problem. Format should be
            a schedules list [['1','3','5'], ...]
    outputs: -new_solution a schedules list with lower cost.
    """
    #  read in data about load locations
    if not Path(file_path).is_file():
        raise FileExistsError(f"file path {file_path} does not exist!")
    loads = loadProblemFromFile(file_path).loads
    dst_mat = compute_distance_matrix(loads)
    new_solution = []
    for route in solution:
        # modify route so it starts and ends at depot
        if route[0] != '0':
            route.insert(0, '0')
        if route[-1] != '0':
            route.append('0')
        best_route = threeOptRoute(route, dst_mat)
        #remove depot
        best_route = [x for x in best_route if x != '0']
        new_solution.append(best_route)

    return new_solution







if __name__ == "__main__":
    from Solutions.greedySolution import greedyBasicVRP
    file_path = "../Training Problems/problem1.txt"
    solution = greedyBasicVRP(file_path)
    new_solution = threeOptVRPSolution(file_path=file_path, solution=solution)

    print("old solution:", solution)
    print("new solution:", new_solution)





