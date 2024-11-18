import numpy as np
import matplotlib.pyplot as plt
from six_dof_model import state_equations
from iterate import rk4
from pos_ang_integrator import posang


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

#initial conditions
x0_b_m = 0
y0_b_m = 0
z0_b_m = 0

phi0_b_rad = 0
theta0_b_rad = 0
psi0_b_rad = 0

u0_b_mps = 0
v0_b_mps = 0
w0_b_mps = 0

p0_b_rps = 0
q0_b_rps = 0
r0_b_rps = 0

Fx_b_kgmps = 0
Fy_b_kgmps = 0
Fz_b_kgmps = 0

L__b_kgm2ps2 = 0
M__b_kgm2ps2 = 0
N__b_kgm2ps2 = 0

pos_ang0 = np.array([
    x0_b_m,
    y0_b_m,
    z0_b_m,
    phi0_b_rad,
    theta0_b_rad,
    psi0_b_rad
])

x0 = np.array([
    u0_b_mps,
    v0_b_mps,
    w0_b_mps,
    p0_b_rps,
    q0_b_rps,
    r0_b_rps
])

forces = np.array([    
    Fx_b_kgmps,
    Fy_b_kgmps,
    Fz_b_kgmps,
    L__b_kgm2ps2,
    M__b_kgm2ps2,
    N__b_kgm2ps2
])

#initial time
t0_s = 0
h_s = 0.005
tf_s = 10

#simulate by integrating

#time array
t_s = np.arange(t0_s, tf_s+0.001, h_s)
#position matrix
x = np.zeros([np.size(x0), np.size(t_s)])
pos_ang = np.zeros([np.size(pos_ang0), np.size(t_s)])

x[:, 0] = x0
pos_ang[:, 0] = pos_ang0

#simulate i.e. integrate
t_s, x = rk4(t_s, h_s, x, state_equations, vehicle_model)
pos_ang = posang(t_s, h_s, x, pos_ang)

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

"""
plt.plot(t_s, pos_ang[0])
plt.show()
"""

#positions
ax = plt.figure().add_subplot(projection='3d')
ax.plot(pos_ang[0], pos_ang[1], pos_ang[2], label='parametric curve')
ax.set_xlabel('X[m]')
ax.set_ylabel('Y[m]')
ax.set_zlabel('Z[m]')
ax.set_title('3D Line Plot')

plt.show()

#angles
fig, axes = plt.subplots(1,3, figsize=(10,6))

axes[0].plot(t_s,pos_ang[3,:])
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