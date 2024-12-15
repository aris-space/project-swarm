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
ANG_MARGIN = 0.1
ANG_RATE_MARGIN = 0.1
LANDING_ZONE_RADIUS = 0.5

#how long will our program run
RUNTIME = NUM_PLANNER_UPDATES / PLANNER_FREQ

#SUB PROPERTIES => TO BE COMPLETED WITH SIM PARAMS
MASS = 5 #in kg
DRAG_COEFFICIENT = 0.6


#Controll allocation constants
C_ROLL = 0.2
C_PITCH = 0.2
C_YAW = 0.2

PIN_ZFL = 18
PIN_ZFR = 19
PIN_ZBL = 23
PIN_ZBR = 24
PIN_XYFL = 99
PIN_XYFR = 99
PIN_XYBL = 99
PIN_XYBR = 99



REV_ZFL = False
REV_ZFR = False
REV_ZBL = False
REV_ZBR = False
REV_XYFL = False
REV_XYFR = False
REV_XYBL = False
REV_XYBR = False



MOTOR_SATURATION = 100

#PWM constants
FREQUENCY = 200              # PWM frequency in Hz
ESC_MIN = 1100               # Minimum pulse width (1.1 ms) for ESC (idle/stop)
ESC_MAX = 1900               # Maximum pulse width (1.9 ms) for ESC (full throttle)
ESC_NEUTRAL = 1500           # Neutral pulse width (1.5 ms)



#controller test
CONTROLLER_TEST_DURATION = 10