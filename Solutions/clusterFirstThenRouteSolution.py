import numpy as np
from sklearn.cluster import KMeans
from itertools import permutations
from pathlib import Path
from copy import copy
from Utils.evaluateShared import loadProblemFromFile, Load, Point, distanceBetweenPoints
from Optimization.threeOptOptimization import threeOptVRPSolution
from Optimization.twoOptOptimization import twoOptVRPSolution
# predefined rules for the VRP

max_route_dst = 12 * 60 # drivers have a max shift of 12 hours, time in minutes == Euclidean dst