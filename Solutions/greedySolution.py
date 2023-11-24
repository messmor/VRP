import random
from pathlib import Path
from copy import copy
from Utils.evaluateShared import loadProblemFromFile, Load, Point, distanceBetweenPoints, getSolutionCostWithError
from Optimization.threeOptOptimization import threeOptVRPSolution, twoOptVRPSolution
# from Optimization.twoOptOptimization import twoOptVRPSolution
# predefined rules for the VRP
depot = Load(id='0', pickup=Point(0,0), dropoff=Point(0, 0)) # treat the depot as a load (location where drivers must start and end shift)
max_route_dst = 12 * 60 # drivers have a max shift of 12 hours, time in minutes == Euclidean dst

def computeSolutionCost(schedules, file_path):
    problem = loadProblemFromFile(file_path)
    total_number_of_driven_minutes, _ = getSolutionCostWithError(problem, schedules)
    number_of_drivers = len(schedules)
    total_cost = 500 * number_of_drivers + total_number_of_driven_minutes

    return total_cost


def findNearestLoad(current_load, loads):
    """finds the non-depot load that is the shortest distance from the current load.
    inputs: -current_load a Load type
            -loads a list of Loads that need to be fulfilled
    outputs: -nearest_load a Load type
    """

    nearestLoad = None
    min_dist = float('inf')
    # find the nearest load
    for load in loads:
        dist_2_load = distanceBetweenPoints(load.pickup, current_load.dropoff)
        if dist_2_load < min_dist:
            nearestLoad = load
            min_dist = dist_2_load

    return nearestLoad


def findFurthestLoad(current_load, loads):
    """finds the non-depot load that is the Furthest distance from the current load.
    inputs: -current_load a Load type
            -loads a list of Loads that need to be fulfilled
    outputs: -furthest_load a Load type
    """
    furthestLoad = None
    max_dist = 0
    # find the nearest load
    for load in loads:
        dist_2_load = distanceBetweenPoints(load.pickup, current_load.dropoff)
        if dist_2_load > max_dist:
            furthestLoad = load
            max_dist = dist_2_load

    return furthestLoad


def greedyBasicVRP(file_path, mode="random"):
    """A method for solving the Vehicle Routing Problem using a nearest neighbor greedy algorithm
    inputs: -file_path a string file to the input txt file containing load information
            -starting_load None or int, tells nearest neighbor algorithm which node to start with
            if None it defaults to the one nearest the depot
    outputs: schedules a list of schedules refering the the load ids [[1, 3, 4], [2, 5], ... etc
    """
    assert mode in ["nearest", "furthest", "random"]
    #  read in data about load locations
    if not Path(file_path).is_file():
        raise FileExistsError(f"file path {file_path} does not exist!")
    loads = loadProblemFromFile(file_path).loads
    remaining_loads = copy(loads)
    schedules = []

    while remaining_loads:
        curr_sched = []
        curr_dst = 0
        curr_load = depot

        while remaining_loads:
            # when creating a new route choose either closest, furthest, or random
            if len(curr_sched) == 0:
                if mode == "nearest":
                    next_load = findNearestLoad(curr_load, remaining_loads)
                elif mode == "furthest":
                    next_load = findFurthestLoad(curr_load, remaining_loads)
                else:
                    next_load = random.choice(remaining_loads)
            else:
                next_load = findNearestLoad(curr_load, remaining_loads)
            # add to list if load does not break constraints
            if next_load is None:
                # no nearest load. End route
                break

            # check to see if driver can make it back to depot
            dist_2_nl = distanceBetweenPoints(curr_load.dropoff, next_load.pickup)
            dist_2_depot = distanceBetweenPoints(next_load.dropoff, depot.pickup)
            closing_dist = curr_dst + dist_2_nl + dist_2_depot + next_load.load_dist()

            # if they can't make it, send back to depot
            if closing_dist >= max_route_dst:
                break
            # if they can, continue the route
            else:
                curr_sched.append(next_load.id)
                curr_load = next_load
                curr_dst += (dist_2_nl + next_load.load_dist())
                remaining_loads.remove(next_load)

        schedules.append(curr_sched)

    return schedules


def greedyWith2OptVRP(file_path, mode):
    """A method for solving the Vehicle Routing Problem using a nearest neighbor greedy algorithm,
    followed by the 2-opt optimization technique.
    inputs: file_path a string file to the input txt file containing load information
    outputs: schedules a list of schedules refering the the load ids [[1, 3, 4], [2, 5], ... etc
    """
    schedules = greedyBasicVRP(file_path, mode)
    optimized_schedules = twoOptVRPSolution(file_path, schedules)

    return optimized_schedules


def greedyWith3OptVRP(file_path, mode):
    """A method for solving the Vehicle Routing Problem using a nearest neighbor greedy algorithm,
    followed by the 3-opt optimization technique.
    inputs: file_path a string file to the input txt file containing load information
    outputs: schedules a list of schedules refering the the load ids [[1, 3, 4], [2, 5], ... etc
    """
    schedules = greedyBasicVRP(file_path, mode)
    optimized_schedules = threeOptVRPSolution(file_path, schedules)

    return optimized_schedules


def greedyEnsembleVRP(file_path):
    """Runs greedy basic vrp in each mode (except furthest as further provides bad results).
     Then optimizes. Takes the solution with the lowest score.
    """
    loads = loadProblemFromFile(file_path).loads
    number_of_loads = len(loads)
    number_of_random_trials = min(10, int(0.1 * number_of_loads))
    lowest_cost = float("inf")
    optimal_solution = None
    for mode in ["nearest"] + ["random" for i in range(number_of_random_trials)]:
        schedules = greedyBasicVRP(file_path, mode)
        cost = computeSolutionCost(schedules, file_path)
        if cost < lowest_cost:
            lowest_cost = cost
            optimal_solution = schedules
    # run 3-opt
    optimized_schedules = twoOptVRPSolution(file_path, optimal_solution)

    return optimized_schedules


if __name__ == "__main__":
    import functools
    from Utils.testingUtils import testSolution
    file_path = "../Training Problems/problem3.txt"
    nearest = functools.partial(greedyBasicVRP, mode="nearest")
    nearest_3Opt = functools.partial(greedyWith3OptVRP, mode="nearest")
    nearest_2Opt = functools.partial(greedyWith2OptVRP, mode="nearest")
    print("### Greedy Baseline ###")
    testSolution(file_path, nearest, print_out_cost=True)
    print("### Greedy 2-Opt ###")
    testSolution(file_path, nearest_2Opt, print_out_cost=True)
    print("### Greedy 3-Opt ###")
    testSolution(file_path, nearest_3Opt, print_out_cost=True)
    # print("### Greedy Ensemble with 3-opt ###")
    # testSolution(file_path, greedyEnsembleVRP, print_out=True)













