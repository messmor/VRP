import argparse
from Solutions.sweepSolution import sweepWith2OptVRP


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='reads in the file_path to the VRP problem.')
    parser.add_argument("file_path", type=str, help="The file path to the VRP problem .txt file.")
    args = parser.parse_args()
    # run VRP solution
    schedules = sweepWith2OptVRP(args.file_path)
    # print out routes to match submission guidelines
    for route in schedules:
        print([int(x) for x in route])

