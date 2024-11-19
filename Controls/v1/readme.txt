planner gives a target dictionary to sac_
sac_ gives torch vec to sim_ with torques and thrust
sim_ computes vehicle response to thrust torques, plots and gives sensor data back at a slower freq
than torques and thrusts are given via a torch vec

To-Do's:
- implement more complex dynamics model
- implement 3d visualisation
- implement forward controller
- implement sensor noise
- implement scaling to [-1000,1000] and mavlink communication

Our naming scheme => the state vector also has the following sequence:
postitions:
    x, y, z
angles:
    roll, pitch, yaw
velocities:
    x_rate, y_rate, z_rate
angular rates:
    roll_rate, pitch_rate, yaw_rate
thrusts:
    x_thrust, y_thrust, z_thrust
torques:
    x_torque, y_torque, z_torque



