#from single_agent_controller.controllers.low_level_ctrl import LLC
from single_agent_controller.controllers_utils.helpers import *
from single_agent_simulator.x.system_dynamics import *
from single_agent_simulator.x.ode_solver import *
from single_agent_visualiser.vector_visualiser import *
from single_agent_visualiser.log_visualiser import *
from single_agent_controller.controllers.low_level_ctrl_2 import LLC2

from utils.waypoints import *
from utils import CONSTANTS
from utils.constants2 import *

import matplotlib.pyplot as plt
import numpy as np
import csv
import os

"""
# needed for plotting
plt.ion()
fig = None
ax = None
"""

if __name__ == "__main__":

    #empty log file
    open('total_state.log', 'w').close()


    #initialize llc with initial state
    llc = LLC(CONSTANTS['pid_params'], CONSTANTS['init_params'], LLC_FREQ)
    last_update = time.time()
    #3x2 array for roll, pitch, yaw and their rates from the init_params
    angle_state = np.array([
        np.array([CONSTANTS['init_params']['roll_init'], CONSTANTS['init_params']['roll_rate_init']]), 
        np.array([CONSTANTS['init_params']['pitch_init'], CONSTANTS['init_params']['pitch_rate_init']]), 
        np.array([CONSTANTS['init_params']['yaw_init'], CONSTANTS['init_params']['yaw_rate_init']])
    ])
    # 1x2 array for depth and rate from the init_params
    depth_state = np.array([CONSTANTS['init_params']['depth_init'], CONSTANTS['init_params']['depth_rate_init']])

    #initialize short term thrust and torques memory (needed for RK4/Euler)
    prev_torques = np.zeros(3)
    prev_thrusts = np.zeros(3)

    skip_depth = True



    with open('total_state.log', 'ab') as f:

        start = time.time()

        for i in range(NUM_PLANNER_UPDATES): #planner updates

            #measure how long it takes for one iteration


            #update target state
            llc.update_target_state(waypoints[i])

            for _ in range(int(LOC_FREQ / PLANNER_FREQ)*5): #loc updates => note that there is still a factor of five in there to ensure app. steady state between waypoints

                #update yaw by global x and y and update x if orientation is allright

                for _ in range(PRES_FREQ // LOC_FREQ): #pres updates

                    #give pressure from pres to controller
                    #llc.update_from_pres_np_arr(depth_state[0], skip_depth)

                    for _ in range(IMU_FREQ // PRES_FREQ): #imu updates

                        #plot orientation => just here because fps is about right
                        #fig, ax = plot_orientation(fig, ax, 0, 0, depth_state[0], angle_state[0, 0], angle_state[1, 0], angle_state[2, 0])
                        if llc.check_orientation() == True:
                            skip_depth = False


                        #give angle rates from sim to controller
                        llc.update_from_IMU_np_arr(angle_state, depth_state, dt=(time.time()-last_update))
                        last_update = time.time()
                        print('IMU UPDATE!!!!!')
                        print(angle_state)
                        print(llc.roll_ctrl.current_detectable_angle)
                        print(llc.pitch_ctrl.current_detectable_angle)
                        print(llc.yaw_ctrl.current_detectable_angle)

                        for _ in range(LLC_FREQ // IMU_FREQ): #controller updates

                            #calculate desired rates from desired angles
                            llc.update_desired_arates()
                            llc.update_desired_drate(skip_depth)

                            #calculate torques from desired rates
                            torquex,torquey,torquez = llc.update_torques()
                            #print(torquex,torquey,torquez)

                            #calculate thrust from desired depth rate
                            thrustz = llc.update_thrust_z()

                            for _ in range(SIM_FREQ // LLC_FREQ): #sim updates
                                #update angles with RK4

                                angle_state[0,:] = rk4(state_equations, angle_state[0,:], prev_torques[0], torquex, SIM_FREQ)
                                angle_state[1,:] = rk4(state_equations, angle_state[1,:], prev_torques[1], torquey, SIM_FREQ)
                                angle_state[2,:] = rk4(state_equations, angle_state[2,:], prev_torques[2], torquez, SIM_FREQ)
                                #print(angle_state)
                                #print(llc.roll_ctrl.current_detectable_angle)
                                #print(llc.pitch_ctrl.current_detectable_angle)
                                #print(llc.yaw_ctrl.current_detectable_angle)
                                #depth_state = rk4(simple_system_dynamics, depth_state, prev_thrusts[2], thrustz, SIM_FREQ)
                                #measure angle rates

                                #log rate rates in a log file
                                #create a 4x2 array for roll, pitch, yaw and their rates and depth and rate
                                total_state = np.concatenate((angle_state, depth_state.reshape(1, -1)), axis=0)
                                
                                np.savetxt(f, total_state.reshape(1, -1), delimiter=',')

                                #if time >500ms, quit
                                if time.time() - start > 0.5:
                                    break
                                #time.sleep(1/SIM_FREQ/3)
                                

    #stop interactive plotting
    plt.ioff()

    end = time.time()
    print(f"Time taken for one iteration: {end - start}")
    print(RUNTIME)

    # Load the logged data
    data = np.loadtxt('total_state.log', delimiter=',')
    time_p = np.arange(len(data))

    # Plot the data
    log_visualiser(time_p, data)




    """


    llc2 = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'])

    
    euler_measured_x = np.zeros(5000)
    euler_measured_y = np.zeros(5000)
    euler_measured_z = np.zeros(5000)
    euler_actual_x = np.zeros(5000)
    euler_actual_y = np.zeros(5000)
    euler_actual_z = np.zeros(5000)
    
    # Open a CSV file in the v1 folder
    os.makedirs('v1', exist_ok=True)
    with open('v1/actual_states_log.csv', mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(['time_step', 'actual_roll', 'actual_pitch',  'actual_yaw',  'actual_depth',
                         'measured_roll', 'measured_pitch', 'measured_yaw',  'measured_depth',])

        for j in range(5):
            llc2.global_orientation_target_quat = llc2.euler_zyx_to_quaternion(waypoints[j]['yaw'], waypoints[j]['pitch'], waypoints[j]['roll'])
            for i in range(1000):
                llc2.update_angle_pids()
                tau_x, tau_y, tau_z = llc2.update_angle_rate_pids()

                # Simulation
                angle_state[0,:] = rk4(simple_system_dynamics, angle_state[0,:], prev_torques[0], tau_x, 1/LLC_FREQ)
                angle_state[1,:] = rk4(simple_system_dynamics, angle_state[1,:], prev_torques[1], tau_y, 1/LLC_FREQ)
                angle_state[2,:] = rk4(simple_system_dynamics, angle_state[2,:], prev_torques[2], tau_z, 1/LLC_FREQ)

                prev_torques = tau_x, tau_y, tau_z

                #euler_actual_x[j*1000+i] = angle_state[0,0]
                #euler_actual_y[j*1000+i] = angle_state[1,0]
                #euler_actual_z[j*1000+i] = angle_state[2,0]

                llc2.update_actual_local_rates(angle_state[0,1], angle_state[1,1], angle_state[2,1])
                llc2.update_global_orientation_w_dead_reckoning(angle_state[2,1], angle_state[1,1], angle_state[0,1], dt=1/LLC_FREQ)
                #if i % 100 == 0:
                #    llc2.update_global_orientation_w_state(angle_state[2,0], angle_state[1,0], angle_state[0,0], dt=1/LLC_FREQ)
                
                euler_measured_z[j*1000+i], euler_measured_y[j*1000+i], euler_measured_x[j*1000+i] = llc2.quaternion_to_euler_zyx(llc2.global_orientation_estimate_quat)

                # Log the state to the CSV file
                writer.writerow([j*1000+i, angle_state[0,0], angle_state[1,0], angle_state[2,0], depth_state[0]]             
                                    )


    #extract states from states_log.csv
    with open('actual_states_log.csv', mode='r') as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            euler_actual_x[int(row[0])] = float(row[1])
            euler_actual_y[int(row[0])] = float(row[3])
            euler_actual_z[int(row[0])] = float(row[5])
    #plot actual euler angles
    plt.plot(euler_actual_x)
    plt.plot(euler_actual_y)
    plt.plot(euler_actual_z)


    #include legend
    plt.legend(['perceived_euler_roll', 'perceived_euler_pitch', 'perceived_euler_yaw', 'actual_euler_roll', 'actual_euler_pitch', 'actual_euler_yaw'])
    plt.show()



    # Todo:
    # Integration dt to s and pass dt in 0.005 
    # converting the system to a class => Kris am MI
    # hand class of system to controller => Nino am MI
    # convert simulator to one 6DOF model => Kris am MI
    # decouple process of estimating angles => today
    # compare data from estimation with truth

    



    #initialisierung des globalen quaternions durch initial euler yaw pitch roll 'ZYX' => done
    #initialiserung des globalen ziel quaternions durch euler yaw pitch roll 'ZYX' => done
    #berechnung des error quaternions q_error = q_target * q_current.conj => done
    #konvertierung des q_error in local angular error = 2 * [q1,q2,q3] => done
    #q1 q2 q3 in local pid's einfügen, gibt w_x, w_y, w_z => done
    #w_x w_y w_z in local pid's einfügen, gibt tau_x, tau_y, tau_z => done
    #die tatsächlichen local angular rates zurückerhalten und abspeichern => done
    #das neue globale quaternion erhalten mit q_new = q_current + dt/2 * Omega ('angular vel. matr.) * q current => done
    #berechnung, der benötigten winkelgeschwindigkeiten im globalen frame   

    """