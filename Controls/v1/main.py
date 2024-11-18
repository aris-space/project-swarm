from sac_.controllers.llc_v1 import LLC
from sac_.utils.helpers import *
from sac_.config.constants import *
from sac_.visualisation.waypoints import *
from sac_.utils.states import *
import os
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":

    # Initialize paths & load config files
    base_dir = os.path.dirname(os.path.abspath(__file__))
    pid_params_path, llc_config_path, log_file_path = initialize_paths(base_dir)

    pid_params = load_config(pid_params_path)
    llc_config = load_config(llc_config_path)


    llc = LLC(pid_params, llc_freq)