from time import time
from pathlib import Path
from copy import copy
from evaluateShared import loadProblemFromFile, Load, Point, distanceBetweenPoints, getSolutionCostWithError, getDistanceOfScheduleWithReturnHome

# predefined rules for the VRP
depot = Load(id='0', pickup=Point(0,0), dropoff=Point(0, 0)) # treat the depot as a load (location where drivers must start and end shift)
max_route_dst = 12 * 60 # drivers have a max shift of 12 hours, time in minutes == Euclidean dst


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


def solveVRP(file_path):
    """A method for solving the Vehicle Routing Problem using a nearest neighbor greedy algorithm
    inputs: file_path a string file to the input txt file containing load information
    outputs: schedules a list of schedules refering the the load ids [[1, 3, 4], [2, 5], ... etc
    """
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









