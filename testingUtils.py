from time import time
from evaluateShared import loadProblemFromFile, getSolutionCostWithError, getDistanceOfScheduleWithReturnHome
from pathlib import Path

def testSolution(file_path, solution_func, print_out=False):
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

    if print_out:
        print("#### schedules ####")
        for s_i, sched in enumerate(schedules):
            print(f"schedule {s_i}: ", sched)
            print(f"schedule drive time:", getDistanceOfScheduleWithReturnHome(sched, loadByID))
    # print solution metrics
    total_number_of_driven_minutes, _ = getSolutionCostWithError(problem, schedules)

    number_of_drivers = len(schedules)
    total_cost = 500 * number_of_drivers + total_number_of_driven_minutes
    run_time = (end_time - start_time) * 1e6
    if print_out:
        print("solution total cost:", total_cost)
        print(f"solution run time: {run_time} ms!" )

    return total_cost, run_time


def getAvgMetricsOnTraining(training_dir, solution_func):
    """Runs the VRP solver on all test examples and provides average metrics for the whole dataset."""
    training_dir = Path(training_dir)
    if not training_dir.is_dir():
        raise NotADirectoryError(f"directory {training_dir.as_posix()} does not exist!")

    total_time = 0
    total_cost = 0
    number_problems = 0

    for file_path in training_dir.iterdir():
        cost, runtime = testSolution(file_path, solution_func)
        total_time += runtime
        total_cost += cost
        number_problems += 1

    avg_time = total_time / number_problems
    avg_cost = total_cost / number_problems

    print("avg_time ms", avg_time, "avg_time seconds", avg_time / 1e6)
    print("avg_cost", avg_cost)


if __name__ == "__main__":
    from greedySolution import solveVRP as greedySolver
    training_dir = "./Training Problems"
    print("### metrics for greedySolver ###")
    getAvgMetricsOnTraining(training_dir, greedySolver)