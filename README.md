# VRP
The VRP repo is my first attempt at solving the Vehicle Routing Problem with Loads. The solution assigns routes to drivers so that each driver does not exceed a max time driving, while attempting to minimize costs associated with the number of routes and total mileage driven. The nearest neighbors greedy search algorithm is a good starting point for an initial solution. After finding a starting solution, I tried optimizing the solution using various classical heuristic as described [here ]((https://epubs.siam.org/doi/book/10.1137/1.9780898718515)https://epubs.siam.org/doi/book/10.1137/1.9780898718515). The 2-opt optimization worked best given the time constraint of solving each sample problem in under 30 seconds. Further research shows that googls OR tools is one of the state of the art open source VRP solvers. I also implemented a OR tools solution using the python api for OR tools. 

My algorithm (mySubmission.py) achieved an average cost of ~50,000 on the training problem set while the OR tools solution achieve an average cost of ~44,000. In the future, I plan to implment some meta heuristic methods to further improve my results (like GA, specifically ant colony optimization). Unfortunately, I'm short on time to do this now.

# Running the Solvers
To run the solver:
1. Clone this repo
2. Change directories to VRP (this repo)
3. run the command ```python mySubmission.py "file_path"``` where file_path is the path to the input problem .txt file describing the loads in the VPR problem.
4. to run the OR tools solver, run the command ```python mySubmissionOrTools.py "file_path"```
5. The python environment requirements are minimal. python >= 3.7 and numpy are all that are needed to run mySubmission.py. To run mySubmissionOrTools.py you must install ortools (```pip install ortools```).
