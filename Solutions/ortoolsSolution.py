import numpy as np
from pathlib import Path
from copy import copy
from Utils.evaluateShared import loadProblemFromFile, Load, Point, distanceBetweenPoints
from Optimization.threeOptOptimization import threeOptVRPSolution
from Optimization.twoOptOptimization import twoOptVRPSolution
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from Utils.evaluateShared import loadProblemFromFile

depot = Load(id='0', pickup=Point(0, 0), dropoff=Point(0, 0))
# In the case of or-tools solution, distance input is taken as integer.
# To get around this we round to the 3 spot and multiple by 1000
# There is still small rounding error, so  we reduce max shift from 720 dst to 719 dst
max_route_dst = int(719*1000) # drivers have a max shift of 12 hours, time in minutes == Euclidean dst


def create_data_model(loads):
    data = {}
    loads.insert(0, depot)
    num_loads = len(loads)
    # when creating the data matrix, treat a load like a single node
    # distance from one node to another is the distance from dropoff of
    # starting loading to pick up of ending load plus the distance of the ending load.
    dist_mat = np.zeros((num_loads, num_loads), dtype=int)
    for start in range(num_loads):
        for end in range(num_loads):
            if start == end:
                dist_mat[start, end] = 0
            else:
                dist = 1000 * np.round((distanceBetweenPoints(loads[start].dropoff, loads[end].pickup)
                                        + distanceBetweenPoints(loads[end].pickup, loads[end].dropoff)), 3)
                dist_mat[start, end] = int(np.round(dist))

    data["distance_matrix"] = dist_mat.tolist()
    data["depot"] = 0
    data["num_vehicles"] = num_loads

    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        if route_distance > 0:
            print(plan_output)
        total_distance += route_distance
    print(f"Total Distance of all routes: {total_distance}m")


def reformat_solution(data, manager, routing, solution):
    """Takes the OR tools solution class and reformats all routes into
    a nested list conforming to our standard schedules format.
    inputs: -data the data input dict to the ortools solver
            -manager or tools route manager
            -routing or tools routing model used to create solution
            -solution an or tools solution class
    output: -schedules a list of schedules, each schedule a route ['1', '4', '5'] etc
    """
    schedules = []
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        curr_schedule = []
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            # convert index to a load id to be recognizable in load
            schedule_value = str(manager.IndexToNode(index))
            # skip depot loads
            if schedule_value == '0':
                continue
            curr_schedule.append(schedule_value)
        # don't include empty routes
        if curr_schedule:
            schedules.append(curr_schedule)

    return schedules


def ortoolsSolver(file_path, print_solutions=False):
    #  read in data about load locations
    if not Path(file_path).is_file():
        raise FileExistsError(f"file path {file_path} does not exist!")
    loads = loadProblemFromFile(file_path).loads
    # create data input dict
    data = create_data_model(loads)
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"])
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    # Define cost of each arc.

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_route_dst,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    # Print solution on console.
    if solution and print_solutions:
        print_solution(data, manager, routing, solution)
    # reformat solution into schedule format
    schedules = reformat_solution(data, manager, routing, solution)

    return schedules



if __name__ == '__main__':
    from time import time
    for i in range(1, 21, 1):
        print(f"### problem {i} ###")
        file_path = f"Training Problems/problem{i}.txt"
        loads = loadProblemFromFile(file_path).loads
        data = create_data_model(loads)
        st = time()
        schedules = ortoolsSolver(file_path, print_solutions=True)
        et = time()
        print("runtime", et-st)
        # print("#### ortools schedules ####")
        # for i, route in enumerate(schedules):
        #     print(f"route {i}: ", route)

