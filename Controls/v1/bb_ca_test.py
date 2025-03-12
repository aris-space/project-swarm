from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from single_agent_controller.controllers.ca_system import SCA

from utils.waypoints import *
from utils import CONSTANTS
from utils.constants2 import *


import time


if __name__ == "__main__":

    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'])
    sca = SCA(llc)

    sca.update_motor_thrusts_manual(40, 0, 0)

    """
    what should happen: only z motors spin, left motors push air downward, right motors push air upwards
    works
    """
    time.sleep(20)


    sca.update_motor_thrusts_manual(0, 40, 0)

    """
    what should happen: only z motors spin, front motors push air upwards, back motors push air downwards
    works
    """
    time.sleep(20)


    sca.update_motor_thrusts_manual(0, 0, 40)
    """
    what should happen: every motor pushes air in clockwise direction
    """


    #wait 3 sec
    time.sleep(20)

    sca.update_motor_thrusts_manual(0,0,0)


    sca.update_motor_thrusts_nonzero(10)

    #wait 3 sec
    time.sleep(3)

    sca.update_motor_thrusts_zero()





