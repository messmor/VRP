import math

import numpy as np
from pathlib import Path
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from evaluateShared import loadProblemFromFile, distanceBetweenPoints

depot = (0, 0)
max_route_dst = 12 * 60 # drivers have a max shift of 12 hours, time in minutes == Euclidean dst

def tupleDistance(tup1, tup2):
    xDiff = tup1[0] - tup2[0]
    yDiff = tup1[1] - tup2[1]
    return math.sqrt(xDiff * xDiff + yDiff * yDiff)


def create_data_model(loads):
    """Stores the data for the problem."""
    data = {}

    #create list of loads to and from depot
    depot_loads_start = [(depot, tuple(load.pickup.toList())) for load in loads]
    depot_loads_finish = [(tuple(load.dropoff.toList()), depot) for load in loads]
    original_loads = [load.convert_2_tuple() for load in loads]
    all_loads = depot_loads_start + depot_loads_finish + original_loads
    data['loads'] = all_loads
    # create distance matrix using
    num_loads = len(all_loads)
    dst_matrix = np.zeros((num_loads, num_loads))
    for i in range(num_loads):
        for j in range(num_loads):
            # distance keeps track of distance between drop off and pick up and the length of next load
            dist_between_loads = tupleDistance(all_loads[i][1], all_loads[j][0])
            dist_of_load = tupleDistance(all_loads[j][0], all_loads[j][1])
            dst_matrix[i, j] = dist_between_loads + dist_of_load
    data['num_vehicles'] = num_loads
    data['depot'] = 0
    data['max_route_length'] = max_route_dst
    return data


def print_solution(manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        total_distance += route_distance
    print('Total Distance of all routes: {}m'.format(total_distance))


def ortoolsSolver(file_path):
    #  read in data about load locations
    if not Path(file_path).is_file():
        raise FileExistsError(f"file path {file_path} does not exist!")
    loads = loadProblemFromFile(file_path).loads
    # create data input dict
    data = create_data_model(loads)
    # create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['loads']),
                                           data['num_vehicles'], data['depot'])
    # create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    # create and register a transit callback.
    def load_distance_callback(from_index, to_index):
        """Returns the distance of the load."""
        # Convert from routing variable Index to load NodeIndex.
        from_load = data['loads'][manager.IndexToNode(from_index)]
        to_load = data['loads'][manager.IndexToNode(to_index)]
        # Compute distance for the load here
        return data['distance_matrix'][from_load][to_load]

    transit_callback_index = routing.RegisterTransitCallback(load_distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        data['max_route_length'],  # maximum distance per vehicle
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)


if __name__ == '__main__':
    file_path = "./Training Problems/problem1.txt"
    ortoolsSolver(file_path)
