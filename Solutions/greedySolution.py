from time import time
import random
from pathlib import Path
from copy import copy
from Utils.evaluateShared import loadProblemFromFile, Load, Point, distanceBetweenPoints, getSolutionCostWithError
from Optimization.lambdaOptOptimization import twoOptVRPSolution, threeOptVRPSolution

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


def greedyBasicVRP(file_path, mode="nearest"):
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
    number_of_random_trials = 10
    lowest_cost = float("inf")
    optimal_schedules = None
    for mode in ["nearest"] + ["random" for i in range(number_of_random_trials)]:
        schedules = greedyWith2OptVRP(file_path, mode)
        cost = computeSolutionCost(schedules, file_path)
        if cost < lowest_cost:
            lowest_cost = cost
            optimal_schedules = schedules

    return optimal_schedules

def greedyBestStartVRP(file_path):
    """Runs all possible start positions for nearest neighbors. Run 2-opt optimization
    on the top few. Times allowing."""
    canidate_schedules = []
    # first get 2-opt starting positions
    time_cutoff = 20
    start_time = time()
    loads = loadProblemFromFile(file_path).loads
    num_loads = len(loads)
    starting_states = [greedyBasicVRP(file_path, "nearest")]
    for i in range(len(loads)):
        starting_states.append(greedyBasicVRP(file_path, "random"))
    end_time1 = time()
    time_cutoff -= (end_time1 - start_time)
    # run 2-opt to get estimated time
    st_near = time()
    nearest_schedule = twoOptVRPSolution(file_path, starting_states[0])
    et_near = time()
    two_opt_rt = et_near - st_near
    # compute how many runs we can do
    time_cutoff -= two_opt_rt
    num_runs = min(int(time_cutoff // two_opt_rt), num_loads)
    # if time allows run more two opt with random starts
    if num_runs < 1:
        return nearest_schedule
    else:
        best_starts = sorted(starting_states[1::], key=lambda x: computeSolutionCost(x, file_path))[0: num_runs]
    # run random starts
    canidate_schedules.append(nearest_schedule)
    for start in best_starts:
        canidate_schedules.append(twoOptVRPSolution(file_path, start))

    best_schedule = sorted(canidate_schedules, key=lambda x: computeSolutionCost(x, file_path))[0]
    return best_schedule








if __name__ == "__main__":
    from Solutions.sweepSolution import sweepWith2OptVRP
    import functools
    from Utils.testingUtils import testSolution
    file_path = "../Training Problems/problem8.txt"
    print("greedy basic")
    testSolution(file_path, functools.partial(greedyBasicVRP, mode="nearest"), print_out_cost=True)
    print("greedy 2-opt")
    testSolution(file_path, functools.partial(greedyWith2OptVRP, mode="nearest"), print_out_route=False, print_out_cost=True)
    print("greedy best start")
    testSolution(file_path, greedyBestStartVRP, print_out_route=False,
                 print_out_cost=True)
    # print("greedy ensemble")
    # testSolution(file_path, greedyEnsembleVRP, print_out_cost=True)
    print("sweep 2-opt")
    testSolution(file_path, sweepWith2OptVRP, print_out_cost=True)












