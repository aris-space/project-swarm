import numpy as np

#frequencies of different modules
PLANNER_FREQ= 0.1
LOC_FREQ = 5
PRES_FREQ = 100
IMU_FREQ = 100
LLC_FREQ = 50
SIM_FREQ = 200

# angle rate 2-5x schneller als angle

#sampling time for different modules
T_PLANNER = 1 / PLANNER_FREQ
T_LOC = 1 / LOC_FREQ
T_IMU = 1 / IMU_FREQ
T_LLC = 1 / LLC_FREQ
    
#number of planner updates/waypoints
NUM_PLANNER_UPDATES = 4

#for forward controller: how much angular error is allowed
ANG_MARGIN = 0.05
ANG_RATE_MARGIN = 0.05

#how long will our program run
RUNTIME = NUM_PLANNER_UPDATES / PLANNER_FREQ

MASS = 5 #in kg






