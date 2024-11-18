import yaml
import numpy as np

def load_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def initialize_vehicle():
    config = load_config("sim_config.yaml")
    height = config["dimensions"]["height"] / 1000  #mm -> m
    width = config["dimensions"]["width"] / 1000    #mm -> m
    depth = config["dimensions"]["depth"] / 1000    #mm -> m
    m = config["mass"]
    Ixx = 1/12*m*(height**2 + width**2)
    Iyy = 1/12*m*(height**2 + depth**2)
    Izz = 1/12*m*(width**2 + depth**2)

    vehicle_model = {
    "weight": config["mass"],
    "MoI Tensor" : np.array([[Ixx,0,0],
                            [0,Iyy,0],
                            [0,0,Izz]])
    }

    return vehicle_model

def initialize_states():
    config = load_config("sim_config.yaml")
    states = np.array(list(config["inital_states"].values()))

    return states

def initialize_time_state_matrix():
    config = load_config("sim_config.yaml")
    t0_s = 0
    tf_s = config["runtime"]
    h_s = 1/config["frequency"]
    states = initialize_states()

    #time array
    t_s = np.arange(t0_s, tf_s, h_s)

    #position matrix
    x = np.zeros([np.size(states), np.size(t_s)])
    x[:, 0] = states
    
    return t_s, x

def initialize():
    vehicle_model = initialize_vehicle()
    t_s, x = initialize_time_state_matrix()

    config = load_config("sim_config.yaml")
    h_s = 1/config["frequency"]

    return vehicle_model, t_s, h_s, x