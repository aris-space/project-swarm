#!/usr/bin/env python
import time
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

def plot(t_s, x, waypoints):
    """
    Plottet verschiedene Zustandsgrößen der Simulation.
    In den Subplots für Position (x, y, z) sowie Winkel (roll, pitch, yaw)
    werden alle gewünschten Sollwerte (Wegpunkte) als horizontale, grüne, gestrichelte Linien eingezeichnet.
    """
    # Filtere nur Dictionary-Einträge heraus (falls in der Liste auch Strings o.Ä. enthalten sind)
    filtered_waypoints = [wp for wp in waypoints if isinstance(wp, dict)]
    
    fig, axes = plt.subplots(4, 3, figsize=(10, 12))
    
    # Geschwindigkeiten
    axes[0,0].plot(t_s, x[6, :], label="Actual")
    axes[0,0].set_title("Forward Velocity")
    axes[0,0].set_xlabel("Time [s]")
    axes[0,0].set_ylabel("u [m/s]")
    
    axes[0,1].plot(t_s, x[7, :], label="Actual")
    axes[0,1].set_title("Sideways Velocity")
    axes[0,1].set_xlabel("Time [s]")
    axes[0,1].set_ylabel("v [m/s]")
    
    axes[0,2].plot(t_s, x[8, :], label="Actual")
    axes[0,2].set_title("Vertical Velocity")
    axes[0,2].set_xlabel("Time [s]")
    axes[0,2].set_ylabel("w [m/s]")
    
    # Winkelraten
    axes[1,0].plot(t_s, x[9, :], label="Actual")
    axes[1,0].set_title("Roll Rate")
    axes[1,0].set_xlabel("Time [s]")
    axes[1,0].set_ylabel("p [r/s]")
    
    axes[1,1].plot(t_s, x[10, :], label="Actual")
    axes[1,1].set_title("Pitch Rate")
    axes[1,1].set_xlabel("Time [s]")
    axes[1,1].set_ylabel("q [r/s]")
    
    axes[1,2].plot(t_s, x[11, :], label="Actual")
    axes[1,2].set_title("Yaw Rate")
    axes[1,2].set_xlabel("Time [s]")
    axes[1,2].set_ylabel("r [r/s]")
    
    # Positionen: X, Y, Z
    axes[2,0].plot(t_s, x[0, :], label="Actual")
    for wp in filtered_waypoints:
        if wp['x'] is not None:
            axes[2,0].axhline(y=wp['x'], color='green', linestyle='--', alpha=0.6)
    axes[2,0].set_title("X Position")
    axes[2,0].set_xlabel("Time [s]")
    axes[2,0].set_ylabel("x [m]")
    
    axes[2,1].plot(t_s, x[1, :], label="Actual")
    for wp in filtered_waypoints:
        if wp['y'] is not None:
            axes[2,1].axhline(y=wp['y'], color='green', linestyle='--', alpha=0.6)
    axes[2,1].set_title("Y Position")
    axes[2,1].set_xlabel("Time [s]")
    axes[2,1].set_ylabel("y [m]")
    
    axes[2,2].plot(t_s, x[2, :], label="Actual")
    for wp in filtered_waypoints:
        if wp['z'] is not None:
            axes[2,2].axhline(y=wp['z'], color='green', linestyle='--', alpha=0.6)
    axes[2,2].set_title("Z Position")
    axes[2,2].set_xlabel("Time [s]")
    axes[2,2].set_ylabel("z [m]")
    
    # Euler-Winkel: Roll, Pitch, Yaw
    axes[3,0].plot(t_s, x[3, :], label="Actual")
    for wp in filtered_waypoints:
        if wp['roll'] is not None:
            axes[3,0].axhline(y=wp['roll'], color='green', linestyle='--', alpha=0.6)
    axes[3,0].set_title("Roll Angle")
    axes[3,0].set_xlabel("Time [s]")
    axes[3,0].set_ylabel("phi [rad]")
    
    axes[3,1].plot(t_s, x[4, :], label="Actual")
    for wp in filtered_waypoints:
        if wp['pitch'] is not None:
            axes[3,1].axhline(y=wp['pitch'], color='green', linestyle='--', alpha=0.6)
    axes[3,1].set_title("Pitch Angle")
    axes[3,1].set_xlabel("Time [s]")
    axes[3,1].set_ylabel("theta [rad]")
    
    axes[3,2].plot(t_s, x[5, :], label="Actual")
    for wp in filtered_waypoints:
        if wp['yaw'] is not None:
            axes[3,2].axhline(y=wp['yaw'], color='green', linestyle='--', alpha=0.6)
    axes[3,2].set_title("Yaw Angle")
    axes[3,2].set_xlabel("Time [s]")
    axes[3,2].set_ylabel("psi [rad]")
    
    plt.subplots_adjust(hspace=0.7, wspace=0.3)
    for ax in axes.flatten():
        ax.legend()
    
    plt.show()



if __name__ == "__main__":
    # Leere Logdatei
    open('total_state.log', 'w').close()

    # Initialisiere den Low-Level Controller (LLC)
    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'], MAX_SAT)
    last_update = time.time()

    # 3x2 Array für roll, pitch, yaw und deren Raten aus den init_params
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

    with open('total_state.log', 'ab') as log:
        # Initialisiere den Simulator (Fahrzeugmodell, Zeitvektor, Zeitschritt, Zustandsvektor)
        vehicle_model, t_s, h_s, x = initialize()

        # Setze die Anfangszustände aus den position_state- und angle_state-Daten
        x[0, 0] = position_state[0, 0]
        x[1, 0] = position_state[1, 0]
        x[2, 0] = position_state[2, 0]
        x[3, 0] = angle_state[0, 0]
        x[4, 0] = angle_state[1, 0]
        x[5, 0] = angle_state[2, 0]

        for i in range(1, 100000):  # Planner-Updates
            # Update im Modus 5; hier wird zusätzlich ein Flag "finished" zurückgegeben,
            # sodass der Loop abgebrochen wird, wenn alle Wegpunkte abgefahren wurden
            thrustx, thrusty, thrustz, torquex, torquey, torquez, finished = llc.update_w_mode5()


            # Breche die Schleife ab, wenn der Controller signalisiert, dass alle Wegpunkte abgefahren wurden
            if finished:
                print("alle waypoints finished")
                break

            # Update der Steuerbefehle im Zustandsvektor (x[12:18] sind die Eingangsgrößen)
            x[12:18, i - 1] = np.array([thrustx, thrusty, thrustz, torquex, torquey, torquez])

            # Numerische Integration mittels Runge-Kutta 4 (RK4)
            k1 = state_equations(x[:, i - 1], vehicle_model)
            k2 = state_equations(x[:, i - 1] + h_s * k1 / 2, vehicle_model)
            k3 = state_equations(x[:, i - 1] + h_s * k2 / 2, vehicle_model)
            k4 = state_equations(x[:, i - 1] + h_s * k3, vehicle_model)
            x[:, i] = x[:, i - 1] + (h_s / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

            # Extrahiere die aktuellen Positionen und Geschwindigkeiten
            position_state[0, :] = np.array([x[0, i], x[6, i]])
            position_state[1, :] = np.array([x[1, i], x[7, i]])
            position_state[2, :] = np.array([x[2, i], x[8, i]])

            # Extrahiere die aktuellen Winkel und Winkelraten
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


    # Verwende alle Wegpunkte in der Plotfunktion
    plot(t_s, x, waypoints)
