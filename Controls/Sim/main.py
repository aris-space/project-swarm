import numpy as np
import math
import matplotlib.pyplot as plt
from six_dof_model import state_equations
from iterate import rk4

#initialization
vehicle_model = {
    "weight": 5,
    "height": 0,
    "length": 0,
    "width": 0,
    "MoI Tensor" : np.array([[1,0,0],
                            [0,1,0],
                            [0,0,1]])
}

print(vehicle_model)

#initial conditions
u0_b_mps = 1
v0_b_mps = 0
w0_b_mps = 0
p0_b_rps = 0
q0_b_rps = 0
r0_b_rps = 0

x0 = np.array([
    u0_b_mps,
    v0_b_mps,
    w0_b_mps,
    p0_b_rps,
    q0_b_rps,
    r0_b_rps

])

print(x0)

#initial time
t0_s = 0
h_s = 0.005
tf_s = 10

#simulate by integrating

#time array
t_s = np.arange(t0_s, tf_s+0.001, h_s)
#position matrix
x = np.zeros([np.size(x0), np.size(t_s)])
print(t_s)

x[:, 0] = x0
print(x)

#simulate i.e. integrate
t_s, x = rk4(t_s, h_s, x, state_equations, vehicle_model)

#plot solutions
fig, axes = plt.subplots(2,3, figsize=(10,6))

axes[0,0].plot(t_s,x[0,:])
axes[0,0].set_title("Forward Velocity")
axes[0,0].set_xlabel("Time [s]")
axes[0,0].set_ylabel("u [m/s]")

axes[0,1].plot(t_s,x[1,:])
axes[0,1].set_title("Sideways Velocity")
axes[0,1].set_xlabel("Time [s]")
axes[0,1].set_ylabel("v [m/s]")

axes[0,2].plot(t_s,x[2,:])
axes[0,2].set_title("Vertical Velocity")
axes[0,2].set_xlabel("Time [s]")
axes[0,2].set_ylabel("w [m/s]")

axes[1,0].plot(t_s,x[3,:])
axes[1,0].set_title("Roll Rate")
axes[1,0].set_xlabel("Time [s]")
axes[1,0].set_ylabel("p [m/s]")

axes[1,1].plot(t_s,x[4,:])
axes[1,1].set_title("Pitch Rate")
axes[1,1].set_xlabel("Time [s]")
axes[1,1].set_ylabel("q [m/s]")

axes[1,2].plot(t_s,x[5,:])
axes[1,2].set_title("Yaw Rate")
axes[1,2].set_xlabel("Time [s]")
axes[1,2].set_ylabel("r [m/s]")

plt.tight_layout()
plt.savefig("simplots/sim4.png")
plt.show()