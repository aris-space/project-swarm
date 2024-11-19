import numpy as np

#frequencies of different modules
PLANNER_FREQ= 0.1
LOC_FREQ = 5
PRES_FREQ = 250
IMU_FREQ = 250
LLC_FREQ = 500
SIM_FREQ = 1000

#sampling time for different modules
T_PLANNER = 1 / PLANNER_FREQ
T_LOC = 1 / LOC_FREQ
T_IMU = 1 / IMU_FREQ
T_LLC = 1 / LLC_FREQ
    
#number of planner updates/waypoints
NUM_PLANNER_UPDATES = 3

#for forward controller: how much angular error is allowed
DEG_MARGIN = 1

#how long will our program run
RUNTIME = NUM_PLANNER_UPDATES / PLANNER_FREQ






