import numpy as np
from pathlib import Path
from copy import copy
from Utils.evaluateShared import loadProblemFromFile, Load, Point, distanceBetweenPoints
from Optimization.threeOptOptimization import threeOptVRPSolution
from Optimization.twoOptOptimization import twoOptVRPSolution
# predefined rules for the VRP
 # treat the depot as a load (location where drivers must start and end shift)
max_route_dst = 12 * 60 # drivers have a max shift of 12 hours, time in minutes == Euclidean dst


def compute_angular_dst(starting_load, ending_load):
    return abs(starting_load.dropoff.theta - ending_load.pickup.theta)

def findNearestLoad(curr_load, remaining_loads):
    """Finds the load that is nearest in terms of polar angle theta."""
    nearest_load = sorted(remaining_loads, key=lambda x: compute_angular_dst(curr_load, x))[0]
    return nearest_load


def sweepVRP(file_path):
    #  read in data about load locations
    if not Path(file_path).is_file():
        raise FileExistsError(f"file path {file_path} does not exist!")
    loads = loadProblemFromFile(file_path).loads
    remaining_loads = copy(loads)
    schedules = []
    # get random starting direction.
    starting_dir = np.random.uniform(0, 2 * np.pi, 1)
    depot = Load(id='0', pickup=Point(0, 0, theta=starting_dir), dropoff=Point(0, 0, theta=starting_dir))

    while remaining_loads:
        curr_sched = []
        curr_dst = 0
        curr_load = depot
        while remaining_loads:
            next_load = findNearestLoad(curr_load, remaining_loads)
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

def sweepWith2OptVRP(file_path):
    """A method for solving the Vehicle Routing Problem using a sweep algorithm,
    followed by the 2-opt optimization technique.
    inputs: file_path a string file to the input txt file containing load information
    outputs: schedules a list of schedules refering the the load ids [[1, 3, 4], [2, 5], ... etc
    """
    schedules = sweepVRP(file_path)
    optimized_schedules = twoOptVRPSolution(file_path, schedules)

    return optimized_schedules

if __name__ == "__main__":
    file_path = "../Training Problems/problem1.txt"
    solutions = sweepVRP(file_path)
    print("solutions", solutions)