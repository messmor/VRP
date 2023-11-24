from time import time
import itertools
import numpy as np
from pathlib import Path
from Utils.evaluateShared import loadProblemFromFile, distanceBetweenPoints, Load, Point


# predefined rules for the VRP
depot = Load(id='0', pickup=Point(0,0), dropoff=Point(0, 0)) # treat the depot as a load (location where drivers must start and end shift)
max_route_dst = 12 * 60 # drivers have a max shift of 12 hours, time in minutes == Euclidean dst


def convertSchedulesToTour(schedules, add_all_drivers=False):
    """Converts a list of schedules (i.e. a solution) into a single tour with multiple
    trips back to the depot. This allows us to apply k-opt optimization to the whole
    tour which is more effective than applying it to individual routes.
    """
    tour = []
    for route in schedules:
        # add depo
        tour.append('0')
        for load_id in route:
            tour.append(load_id)
    # add final depot
    tour.append('0')
    # we can have more drivers, as many as there are loads. Enter these as extra '0' at the end of
    # each route.
    if add_all_drivers:
        num_loads = max([int(x) for x in set(tour)])
        num_depot_visits = tour.count('0')
        curr_num_drivers = num_depot_visits - 1
        extra_depots = num_loads - curr_num_drivers

        while extra_depots > 0:
            for k in range(len(tour) - 1, -1, -1):
                if tour[k] == "0":
                    tour.insert(k + 1, '0')
                    extra_depots -= 1

    return tour


def convertTourToSchedule(tour):
    """Converts a tour (one long trip with multiple depot visits) into
    a list of individual routes.
    """
    tour = tour.copy()
    schedules = []
    route = []
    for load in tour:
        if load == "0" and len(route) > 0:
            schedules.append(route)
            route = []
        if load != '0':
            route.append(load)

    return schedules


def compute_distance_matrix(loads):
    """create a matrix with dst_mat[i, j] = distance from load i to load j plus length of load j."""
    # add the depot as the first load
    loads.insert(0, depot)
    num_loads = len(loads)
    dst_mat = np.zeros((num_loads, num_loads))
    for start in range(num_loads):
        for end in range(num_loads):
            dst_mat[start, end] = distanceBetweenPoints(loads[start].dropoff, loads[end].pickup) + loads[end].load_dist()

    return dst_mat


def compute_route_length(route, dst_mat, from_depot=False):
    """computes distance of a route using values stored in the distance matrix.
    inputs: -route a list of string route indices ex ['1', '4', '10']
            - dist_mat numpy array the distance matrix for the problem
    outputs: -route_dst a float the total distance of the route
    """
    if from_depot:
        route = route.copy()
        route.insert(0, '0')
        route.append('0')
    # note that this computes the route length neglecting distance from and to depot
    route_dst = sum([dst_mat[int(route[i]), int(route[i+1])] for i in range(len(route) - 1)])

    return route_dst


def is_valid_solution(tour, dist_mat):
    schedules = convertTourToSchedule(tour)
    value = True
    for route in schedules:
        route_dist = compute_route_length(route, dist_mat, from_depot=True)
        if route_dist > max_route_dst:
            value = False
            break
    return value


def twoOptSwitch(route, i, j):
    new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
    return new_route


def twoOptRoute(tour, dst_mat, improvement_threshold=0.01, time_cutoff=20):
    """Implements 2-opt optimization to improve a given route or schedule with in the
    larger solution.
    inputs: -route a list on str ids ex ['1', '4', '5']
    outputs: -best_route a new route with shorter distance.
    """
    # run optimization
    best_tour = tour
    # optimize for cost = num_routes * 500 + total_distance
    routes = convertTourToSchedule(tour)
    best_cost = 500 * len(routes) + compute_route_length(tour, dst_mat, from_depot=True)
    improvement_factor = 1
    #check time every 1000 iterations
    iter_count = 0
    start_time = time()
    while improvement_factor > improvement_threshold:

        previous_best_cost = best_cost
        # run through all switches that don't move first or last stop (these are the depot)
        for i, j in itertools.combinations(range(1, len(best_tour)-1), 2):
            iter_count += 1
            new_tour = twoOptSwitch(best_tour, i, j)
            # make sure route still meets max dist constraint
            if not is_valid_solution(new_tour, dst_mat):
                continue
            new_routes = convertTourToSchedule(new_tour)
            new_cost = 500 * len(new_routes) + compute_route_length(new_tour, dst_mat, from_depot=True)
            if new_cost < best_cost:
                best_tour = new_tour
                best_cost = new_cost
            # check time
            if iter_count % 1000 == 0:
                end_time = time()
                if end_time - start_time > time_cutoff:
                    break

        improvement_factor = 1 - best_cost / previous_best_cost



    return best_tour


def twoOptVRPSolution(file_path, schedules):
    """Implements 2-opt optimization to improve a given VRP solution.
    inputs: -file_path a string path to the problem .txt file
            -solution a starting solution to the vrp problem. Format should be
            in tour format.
    outputs: -new_solution a schedules list with lower cost.
    """
    #  read in data about load locations
    if not Path(file_path).is_file():
        raise FileExistsError(f"file path {file_path} does not exist!")
    loads = loadProblemFromFile(file_path).loads
    dst_mat = compute_distance_matrix(loads)
    tour = convertSchedulesToTour(schedules, add_all_drivers=True)
    best_tour = twoOptRoute(tour, dst_mat)
    best_schedules = convertTourToSchedule(best_tour)


    return best_schedules


def threeOptSwitch(route, i, j, k):
    new_route = route[:i] + route[i:j][::-1] + route[j:k][::-1] + route[k:]
    return new_route


def threeOptRoute(tour, dst_mat, improvement_threshold=0.001, time_cutoff=10):
    """Implements 3-opt optimization to improve a given route or schedule with in the
    larger solution.
    inputs: -route a list on str ids ex ['1', '4', '5']
    outputs: -best_route a new route with shorter distance.
    """
    # run optimization
    best_tour = tour
    # optimize for cost = num_routes * 500 + total_distance
    routes = convertTourToSchedule(tour)
    best_cost = 500 * len(routes) + compute_route_length(tour, dst_mat, from_depot=True)
    improvement_factor = 1
    # check time every 1000 iterations
    iter_count = 0
    improvement_iter = 0
    start_time = time()
    while improvement_factor > improvement_threshold:
        previous_best_cost = best_cost
        # run through all switches that don't move first or last stop (these are the depot)
        for i, j, k in itertools.combinations(range(1, len(best_tour)-1), 3):
            new_tour = threeOptSwitch(best_tour, i, j, k)
            # make sure route still meets max dist constraint
            if not is_valid_solution(new_tour, dst_mat):
                continue

            new_routes = convertTourToSchedule(new_tour)
            new_cost = 500 * len(new_routes) + compute_route_length(new_tour, dst_mat, from_depot=True)
            if new_cost < best_cost:
                best_tour = new_tour
                best_cost = new_cost
            # check time
            if iter_count % 1000 == 0:
                end_time = time()
                if end_time - start_time > time_cutoff:
                    break

        improvement_factor = 1 - best_cost / previous_best_cost
        improvement_iter += 1

    return best_tour


def threeOptVRPSolution(file_path, schedules):
    """Implements 3-opt optimization to improve a given VRP solution.
    inputs: -file_path a string path to the problem .txt file
            -solution a starting solution to the vrp problem. Format should be
            in tour format.
    outputs: -new_solution a schedules list with lower cost.
    """
    #  read in data about load locations
    if not Path(file_path).is_file():
        raise FileExistsError(f"file path {file_path} does not exist!")
    loads = loadProblemFromFile(file_path).loads
    dst_mat = compute_distance_matrix(loads)
    tour = convertSchedulesToTour(schedules, add_all_drivers=True)
    best_tour = threeOptRoute(tour, dst_mat)
    best_schedules = convertTourToSchedule(best_tour)


    return best_schedules












