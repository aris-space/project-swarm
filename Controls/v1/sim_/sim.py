import numpy as np
import matplotlib.pyplot as plt
from six_dof_model import state_equations
from iterate import rk4
import helpers

vehicle_model, t_s, h_s, x = helpers.initialize()

#simulate i.e. integrate
t_s, x = rk4(t_s, h_s, x, state_equations, vehicle_model)

#plot solutions
fig, axes = plt.subplots(2,3, figsize=(10,6))

axes[0,0].plot(t_s,x[6,:])
axes[0,0].set_title("Forward Velocity")
axes[0,0].set_xlabel("Time [s]")
axes[0,0].set_ylabel("u [m/s]")

axes[0,1].plot(t_s,x[7,:])
axes[0,1].set_title("Sideways Velocity")
axes[0,1].set_xlabel("Time [s]")
axes[0,1].set_ylabel("v [m/s]")

axes[0,2].plot(t_s,x[8,:])
axes[0,2].set_title("Vertical Velocity")
axes[0,2].set_xlabel("Time [s]")
axes[0,2].set_ylabel("w [m/s]")

axes[1,0].plot(t_s,x[9,:])
axes[1,0].set_title("Roll Rate")
axes[1,0].set_xlabel("Time [s]")
axes[1,0].set_ylabel("p [r/s]")

axes[1,1].plot(t_s,x[10,:])
axes[1,1].set_title("Pitch Rate")
axes[1,1].set_xlabel("Time [s]")
axes[1,1].set_ylabel("q [r/s]")

axes[1,2].plot(t_s,x[11,:])
axes[1,2].set_title("Yaw Rate")
axes[1,2].set_xlabel("Time [s]")
axes[1,2].set_ylabel("r [r/s]")

plt.tight_layout()
plt.savefig("simplots/sim4.png")
plt.show()

"""
plt.plot(t_s, pos_ang[0])
plt.show()
"""

"""

#positions
ax = plt.figure().add_subplot(projection='3d')
ax.plot(x[0,:], x[1,:], x[2,:], label='parametric curve')
ax.set_xlabel('X[m]')
ax.set_ylabel('Y[m]')
ax.set_zlabel('Z[m]')
ax.set_title('3D Line Plot')

plt.show()

#angles
fig, axes = plt.subplots(1,3, figsize=(10,6))

axes[0].plot(t_s,x[3,:])
axes[0].set_title("Roll Angle")
axes[0].set_xlabel("Time [s]")
axes[0].set_ylabel("phi [rad]")

axes[1].plot(t_s,x[4,:])
axes[1].set_title("Pitch Angle")
axes[1].set_xlabel("Time [s]")
axes[1].set_ylabel("roh [rad]")

axes[2].plot(t_s,x[5,:])
axes[2].set_title("Yaw Angle")
axes[2].set_xlabel("Time [s]")
axes[2].set_ylabel("psi [m/s]")

plt.show()

"""