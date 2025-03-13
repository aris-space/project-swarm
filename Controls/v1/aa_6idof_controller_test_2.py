#!/usr/bin/env python
#!/usr/bin/env python
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation

# Controller und Hilfsfunktionen
from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from single_agent_controller.controllers_utils.helpers import *

# Simulator-Module
from single_agent_simulator.x.system_dynamics import *
from single_agent_simulator.x.ode_solver import *
from single_agent_simulator.y.helpers import *
from single_agent_simulator.y.six_dof_model import *

# Visualiser
from single_agent_visualiser.vector_visualiser import *
from single_agent_visualiser.log_visualiser import *
from single_agent_visualiser.new_visualiser import Live3DVisualizer

# Utilities
from utils.waypoints import *
from utils import CONSTANTS
from utils.constants2 import *

#Localization
sys.path.insert(0, "/shared_folder/project-swarm")
#from MissionPlanner.swarm_storage import *

def plot_desired_velocities(t_s, desired_velocities):
    """
    Plottet die Desired Velocities (X, Y, Z) über die Simulationszeit.
    Es werden horizontale Linien bei +MAX_VEL und -MAX_VEL eingezeichnet.
    """
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    labels = ["Desired X Velocity", "Desired Y Velocity", "Desired Z Velocity"]
    for i in range(3):
        axes[i].plot(t_s, desired_velocities[i, :], label=labels[i])
        # Zeichne die Begrenzungslinien ein
        #axes[i].axhline(y=MAX_VEL, color='red', linestyle='--', label="MAX_VEL" if i == 0 else None)
        #axes[i].axhline(y=-MAX_VEL, color='red', linestyle='--', label="-MAX_VEL" if i == 0 else None)
        axes[i].set_xlabel("Time [s]")
        axes[i].set_ylabel("Velocity [m/s]")
        axes[i].legend()
    plt.tight_layout()
    plt.show()
    # + position plotten


if __name__ == "__main__":
    # Leere die Logdatei für Desired Velocities
    desired_log_filename = 'desired_velocities.log'
    open(desired_log_filename, 'w').close()

    # raw_data = DroneRawData()
    # filtered_data = DroneFilteredData()
    # swarm_data = SwarmSensorData()

    # Initialisiere den Low-Level Controller (LLC) mit den Konstanten aus CONSTANTS
    
    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'], MAX_VEL)
    # Setze den einzelnen Waypoint als globales Ziel
    llc.global_position_target = np.array([waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z']])
    llc.global_orientation_target_quat = llc.euler_zyx_to_quaternion(
        waypoints[0]['yaw'], waypoints[0]['pitch'], waypoints[0]['roll']
    )
    
    # 3x2 Array für Winkel (roll, pitch, yaw) und deren Raten aus den init_params
    angle_state = np.array([
        np.array([CONSTANTS['init_params']['roll_init'], CONSTANTS['init_params']['roll_rate_init']]),
        np.array([CONSTANTS['init_params']['pitch_init'], CONSTANTS['init_params']['pitch_rate_init']]),
        np.array([CONSTANTS['init_params']['yaw_init'], CONSTANTS['init_params']['yaw_rate_init']])
    ])
    
    # 3x2 Array für Position und deren Geschwindigkeiten
    position_state = np.array([
        np.array([CONSTANTS['init_params']['x_init'], CONSTANTS['init_params']['x_vel_init']]),
        np.array([CONSTANTS['init_params']['y_init'], CONSTANTS['init_params']['y_vel_init']]),
        np.array([CONSTANTS['init_params']['z_init'], CONSTANTS['init_params']['z_vel_init']])
    ])
    
    # Initialisiere den Simulator (Fahrzeugmodell, Zeitvektor, Zeitschritt, Zustandsvektor)
    vehicle_model, t_s, h_s, x = initialize()
    
    # Setze die Anfangszustände
    x[0, 0] = position_state[0, 0]
    x[1, 0] = position_state[1, 0]
    x[2, 0] = position_state[2, 0]
    x[3, 0] = angle_state[0, 0]
    x[4, 0] = angle_state[1, 0]
    x[5, 0] = angle_state[2, 0]
    
    num_steps = len(t_s)
    # Array zum Loggen der Desired Velocities (3 Geschwindigkeiten über die Zeit)
    desired_vel_log = np.zeros((3, num_steps))
    # position log? 
    
    with open('total_state.log', 'ab') as log:
        for i in range(1, num_steps):
            # Update im Modus 5; hier werden Desired Velocities und Torques sowie ein finished-Flag zurückgegeben
            desired_x_vel, desired_y_vel, desired_z_vel, torquex, torquey, torquez, finished = llc.update_w_mode5()
            
            # Kappe die Desired Velocities mit MAX_VEL
            desired_x_vel = np.clip(desired_x_vel, -MAX_VEL, MAX_VEL)
            desired_y_vel = np.clip(desired_y_vel, -MAX_VEL, MAX_VEL)
            desired_z_vel = np.clip(desired_z_vel, -MAX_VEL, MAX_VEL)
            
            # Logge die Desired Velocities in das Array und in die separate Logdatei
            desired_vel_log[:, i-1] = np.array([desired_x_vel, desired_y_vel, desired_z_vel])
            with open(desired_log_filename, 'ab') as dlog:
                np.savetxt(dlog, np.array([[desired_x_vel, desired_y_vel, desired_z_vel]]), delimiter=',')
            
            if finished:
                print("alle waypoints finished")
                break
            
            # Hier gehen wir davon aus, dass das Fahrzeugmodell die Desired Velocities als Steuergröße übernimmt.
            # Falls eine Umrechnung in Thrusts notwendig ist, muss hier ein entsprechender Block ergänzt werden.
            x[12:18, i - 1] = np.array([desired_x_vel, desired_y_vel, desired_z_vel, torquex, torquey, torquez])
            
            # Numerische Integration mittels Runge-Kutta 4 (RK4)
            k1 = state_equations(x[:, i - 1], vehicle_model)
            k2 = state_equations(x[:, i - 1] + h_s * k1 / 2, vehicle_model)
            k3 = state_equations(x[:, i - 1] + h_s * k2 / 2, vehicle_model)
            k4 = state_equations(x[:, i - 1] + h_s * k3, vehicle_model)
            x[:, i] = x[:, i - 1] + (h_s / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            # Aktualisiere die aktuellen Positionen und Geschwindigkeiten
            position_state[0, :] = np.array([x[0, i], x[6, i]])
            position_state[1, :] = np.array([x[1, i], x[7, i]])
            position_state[2, :] = np.array([x[2, i], x[8, i]])
            
            # Aktualisiere die aktuellen Winkel und Winkelraten
            angle_state[0, :] = np.array([x[3, i], x[9, i]])
            angle_state[1, :] = np.array([x[4, i], x[10, i]])
            angle_state[2, :] = np.array([x[5, i], x[11, i]])
            
            # Aktualisiere den Controller mit den neuen Zuständen:
            # Global orientieren (Reihenfolge: yaw, pitch, roll)
            llc.update_global_orientation_w_state(angle_state[2, 0], angle_state[1, 0], angle_state[0, 0])
            llc.update_global_position(position_state[0, 0], position_state[1, 0], position_state[2, 0])
            llc.update_actual_local_rates(angle_state[0, 1], angle_state[1, 1], angle_state[2, 1])
            llc.update_actual_local_velocities(position_state[0, 1], position_state[1, 1], position_state[2, 1])
            
            # Schreibe den Winkelzustand in die Logdatei
            np.savetxt(log, angle_state.reshape(1, -1), delimiter=',')
    
    # Nach der Simulation: Plotte die Desired Velocities
    plot_desired_velocities(t_s, desired_vel_log)
