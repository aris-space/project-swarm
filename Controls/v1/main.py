from sac_.controllers.llc_v1 import LLC
from sac_.utils.helpers import *
from sac_.config.constants import *
from sac_.visualisation.waypoints import *
from sac_.utils.states import *
import os
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":

    pid_params = load_yaml('main/Controls/v1/sac_/config/pid_params.yaml')
    init_params = load_yaml('main/Controls/v1/01config/constants.yaml')['initial_orientation']

    llc = LLC(pid_params, init_params, llc_freq)