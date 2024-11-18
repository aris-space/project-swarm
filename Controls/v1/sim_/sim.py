import numpy as np
import matplotlib.pyplot as plt
from six_dof_model import state_equations
from iterate import rk4
import helpers

vehicle_model, t_s, h_s, x = helpers.initialize()

#simulate i.e. integrate
t_s, x = rk4(t_s, h_s, x, state_equations, vehicle_model)

#plot solutions

helpers.plot(t_s, x, pos_ang)
