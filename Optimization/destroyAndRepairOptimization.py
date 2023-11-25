import random
import numpy as np
from time import time
from copy import copy, deepcopy
from Utils.evaluateShared import loadProblemFromFile, Load, Point, distanceBetweenPoints
from Solutions.greedySolution import greedyBasicVRP


def single_routes_intialization(loads):
    solution = []
    for load in loads:
        if load.id == 0:
            continue
        solution.append([str(load.id)])
    return solution


class largeNeighborhoodSearch(object):
    """A class to implement large neighborhood search using the neighborhood relocate
    heuristic.
    inputs: -file_path a string path to the VRP input problem .txt file
            - is_mode a string describing the intial solution genertation method
    """

    def __init__(self, file_path, is_mode="nearest_neighbor"):
        self.file_path = file_path
        self.loads = loadProblemFromFile(file_path).loads
        self.num_loads = len(self.loads)
        assert is_mode in ["nearest_neighbor", "single_routes"]
        self.is_mode = is_mode  # initial solution choices: nearest_neighbor, single_loads,...
        self.initial_solution = None
        # treat the depot as a load (location where drivers must start and end shift)
        self.depot = Load(id=0, pickup=Point(0, 0), dropoff=Point(0, 0))
        self.loads.insert(0, self.depot)
        # drivers have a max shift of 12 hours, time in minutes == Euclidean dst
        self.max_route_dst = 12 * 60
        # set up distance matrix in advance to reduce compute
        self.distance_matrix = self.get_distance_matrix()
        # destroy values
        self.number_removals = max(3, np.round(0.15*self.num_loads).astype(int))
        self.max_alg_iters = int(10000 / self.num_loads)
        self.improvement_threshold = 0.01
        self.max_run_time = 20


    def get_distance_matrix(self):
        """create a matrix with dst_mat[i, j] = distance from load i to load j plus length of load j."""
        # add the depot as the first load
        num_loads = len(self.loads)
        distance_matrix = np.zeros((num_loads, num_loads))
        for start in range(num_loads):
            for end in range(num_loads):
                distance_matrix[start, end] = (distanceBetweenPoints(self.loads[start].dropoff, self.loads[end].pickup)
                                               + self.loads[end].load_dist())

        return distance_matrix

    def compute_route_distance(self, route):
        # compute distance from depot to start
        route_dst = self.distance_matrix[0, int(route[0])]
        # add the distance along interior of route (all but depots)
        route_dst += sum([self.distance_matrix[int(route[i]), int(route[i + 1])] for i in range(len(route) - 1)])
        # add the distance to return depot
        route_dst += self.distance_matrix[int(route[-1]), 0]

        return route_dst

    def compute_solution_cost(self, solution):
        """Compute cost based on the equation cost = 500*number of driver + number of distance driven"""
        number_routes = len(solution)
        total_mileage = 0
        for route in solution:
            total_mileage += self.compute_route_distance(route)
        cost = 500 * number_routes + total_mileage

        return cost

    def is_valid_route(self, route):
        value = True
        if self.compute_route_distance(route) > self.max_route_dst:
            value = False
        return value

    def is_valid_tour(self, tour):
        routes = self.convert_tour_to_routes(tour)
        value = True
        for route in routes:
            if self.compute_route_distance(route) > self.max_route_dst:
                value = False
                break


        return value

    @staticmethod
    def convert_routes_to_tour(routes):
        """Converts a list of routes (i.e. a solution) into a single tour with multiple
        trips back to the depot. This allows us to apply k-opt optimization to the whole
        tour which is more effective than applying it to individual routes.
        """
        tour = []
        for route in routes:
            # add depo
            tour.append('0')
            for load_id in route:
                tour.append(load_id)
        # add final depot
        tour.append('0')

        return tour

    @staticmethod
    def convert_tour_to_routes(tour):
        """Converts a tour (one long trip with multiple depot visits) into
        a list of individual routes.
        """
        schedules = []
        route = []
        for load in tour:
            if load == "0" and len(route) > 0:
                schedules.append(route)
                route = []
            if load != '0':
                route.append(load)

        return schedules

    def get_initial_solution(self):
        """Creates a starting solution to optimize."""
        if self.is_mode == "nearest_neighbor":
            self.initial_solution = greedyBasicVRP(file_path=self.file_path, mode="nearest")
        elif self.is_mode == "single_routes":
            self.initial_solution = single_routes_intialization(self.loads)

    def destroy_solution(self, solution):
        """randomly removes loads from routes at random."""
        solution = solution.copy()
        broken_solution = []
        loads_to_remove = [x for x in
                           random.sample([str(load.id) for load in self.loads if load.id != 0], k=self.number_removals)]
        for route in solution:
            new_route = list(set(route) - set(loads_to_remove))
            if len(new_route) > 0:
                broken_solution.append(new_route)

        return broken_solution, loads_to_remove

    def repair_solution(self, broken_solution, loads_removed):
        """Uses greedy cost optimization to repair the destroyed or broken solution."""
        tour = self.convert_routes_to_tour(broken_solution)
        # add loads back in using greedy shortest dst method
        for load_id in loads_removed:
            best_distance_insert = float("inf")
            best_index = None
            for index in range(len(tour)):
                new_tour = tour.copy()
                new_tour.insert(index, load_id)
                if self.is_valid_tour(new_tour):
                    distance = self.compute_route_distance(new_tour)
                    if distance < best_distance_insert:
                        best_distance_insert = distance
                        best_index = index
            # add index to optimal spot
            if best_index is not None:
                tour.insert(best_index, load_id)
            # if no spot exists, add to end of tour as a new route
            else:
                tour.append(load_id)
                tour.append('0')

        routes = self.convert_tour_to_routes(tour)

        return routes

    def apply_destroy_and_repair(self, initial_solution):
        """Applies the relocate neighborhood heuristic to optimize the starting solution."""
        def accept(x_t, x):
            if self.compute_solution_cost(x_t) < self.compute_solution_cost(x):
                return True
            else:
                return False

        start_sol = initial_solution.copy()
        stop_criterion = False
        iters = 0
        while not stop_criterion:
            broken_sol, loads_removed = self.destroy_solution(start_sol)
            temp_sol = self.repair_solution(broken_sol, loads_removed)
            if accept(temp_sol, start_sol):
                start_sol = temp_sol.copy()

            iters += 1

            if iters > self.max_alg_iters:
                stop_criterion = True



        return start_sol

    def solver_VRP(self):
        """Runs search and destroy algorithm until improvement stops or runtime is exceeded"""
        if self.initial_solution is None:
            self.get_initial_solution()
        improvement_factor = 1
        # check time every after every app
        improvement_iter = 0
        start_time = time()
        best_sol = self.initial_solution.copy()
        best_cost = self.compute_solution_cost(best_sol)

        while improvement_factor > self.improvement_threshold:
            if time() - start_time > self.max_run_time:
                break

            previous_best_cost = copy(best_cost)
            best_sol = self.apply_destroy_and_repair(best_sol)
            best_cost = self.compute_solution_cost(best_sol)

            improvement_factor = 1 - (best_cost / previous_best_cost)
            improvement_iter += 1
            if improvement_factor <= self.improvement_threshold and improvement_iter < 3:
                self.number_removals += 1
                if self.number_removals > self.num_loads:
                    break
                improvement_factor = 1
                improvement_iter = 0
        return best_sol


def destroyAndRepairSolver(file_path):
    solver = largeNeighborhoodSearch(file_path, is_mode="nearest_neighbor")
    solution = solver.solver_VRP()

    return solution


if __name__ == "__main__":
    file_path = "../Training Problems/problem2.txt"
    lnhs = largeNeighborhoodSearch(file_path, is_mode="nearest_neighbor")
    st = time()
    sol = lnhs.solver_VRP()
    et = time()
    print("runtime", et - st)
    print("solutions", sol)
