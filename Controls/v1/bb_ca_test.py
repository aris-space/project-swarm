from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from single_agent_controller.controllers.ca_system import SCA

from utils.waypoints import *
from utils import CONSTANTS
from utils.constants2 import *


import time


if __name__ == "__main__":

    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'])
    sca = SCA(llc)

    sca.update_motor_thrusts_manual(100,0,0)
    
    #wait 3 sec
    time.sleep(3)

    sca.update_motor_thrusts_manual(0,0,0)

    #wait 3 sec
    time.sleep(3)

    #blablabla

    sca.update_motor_thrusts_zero()





