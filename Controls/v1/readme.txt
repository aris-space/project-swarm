planner gives a target dictionary to sac_
sac_ gives torch vec to sim_ with torques and thrust
sim_ computes vehicle response to thrust torques, plots and gives sensor data back at a slower freq
than torques and thrusts are given via a torch vec

To-Do's:
- implement more complex dynamics model (with gravity & boyancy) o
- implement forward controller /
- implement sim sensor noise /
- implement sensor noise filter o
- implement disturbances /
- implement scaling to [-1000,1000] and mavlink communication o

- implement 3d visualisation => more or less done
- implement saturation and anti-windup, set point weighting => done
- implement orientation assertion also for depth control to avoid gimbal lock => done



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



