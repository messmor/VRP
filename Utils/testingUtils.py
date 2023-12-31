import re
from time import time
from Utils.evaluateShared import loadProblemFromFile, getSolutionCostWithError, getDistanceOfScheduleWithReturnHome
from pathlib import Path

def testSolution(file_path, solution_func, print_out_route=False, print_out_cost=False):
    """Tests solution on examples and displays costs"""
    # run solution
    start_time = time()
    schedules = solution_func(file_path)
    end_time = time()
    # print results
    problem = loadProblemFromFile(file_path)
    # set up load by id
    loadByID ={}
    for load in problem.loads:
        loadByID[load.id] = load

    if print_out_route:
        print("#### schedules ####")
        for s_i, sched in enumerate(schedules):
            print(f"schedule {s_i}: ", sched)
            print(f"schedule drive time:", getDistanceOfScheduleWithReturnHome(sched, loadByID))
    # print solution metrics
    total_number_of_driven_minutes, _ = getSolutionCostWithError(problem, schedules)
    number_of_drivers = len(schedules)
    total_cost = 500 * number_of_drivers + total_number_of_driven_minutes
    run_time = (end_time - start_time)
    if print_out_cost:
        print("solution total cost:", total_cost)
        print(f"solution run time: {run_time} seconds!" )

    return total_cost, run_time


def get_numbers(string):
    numbers = re.findall(r'\d', string)
    return int(''.join(numbers))


def getAvgMetricsOnTraining(training_dir, solution_func):
    """Runs the VRP solver on all test examples and provides average metrics for the whole dataset."""
    training_dir = Path(training_dir)
    if not training_dir.is_dir():
        raise NotADirectoryError(f"directory {training_dir.as_posix()} does not exist!")

    total_time = 0
    total_cost = 0
    number_problems = 0
    max_runtime = 0
    file_paths = sorted(training_dir.iterdir(), key=lambda x: get_numbers(x.name))
    for file_path in file_paths:
        cost, runtime = testSolution(file_path, solution_func)
        print(f"problem {file_path.name} cost: {cost}, runtime {runtime}")
        total_time += runtime
        total_cost += cost
        number_problems += 1
        if runtime > max_runtime:
            max_runtime = runtime

    avg_time = total_time / number_problems
    avg_cost = total_cost / number_problems

    print("avg_time seconds", avg_time)
    print("avg_cost", avg_cost)
    print("max runtime", max_runtime)


if __name__ == "__main__":
    import functools
    from Solutions.greedySolution import greedyBasicVRP, greedyWith2OptVRP, greedyBestStartVRP
    from Optimization.destroyAndRepairOptimization import destroyAndRepairSolver

    training_dir = "Training Problems"
    print("### metrics for greedySolver nearest ###")
    getAvgMetricsOnTraining(training_dir, functools.partial(greedyBasicVRP, mode="nearest"))
    print("### destroy and repair solver ###")
    getAvgMetricsOnTraining(training_dir, destroyAndRepairSolver)
    # print("### metrics for greedySolver nearest 2-opt###")
    # getAvgMetricsOnTraining(training_dir, functools.partial(greedyWith2OptVRP, mode="nearest"))
    # print("### metrics for greedyEnsembleSolver ###")
    # getAvgMetricsOnTraining(training_dir, greedyBestStartVRP)



