# Frequencies

# should not be used anymore if new main exists
from sac_.utils.helpers import *

planner_freq = 0.1
loc_freq = 5
imu_freq = 200
llc_freq = 400
    
num_planner_updates = 1

deg_margin = 1

# Initialize initial state


# Initialize Time Steps
runtime = num_planner_updates // planner_freq
t_planner, t_loc, t_imu, t_llc = initialize_time_steps(planner_freq, loc_freq, imu_freq, llc_freq)