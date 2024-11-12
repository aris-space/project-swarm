import numpy as np
from scipy.integrate import odeint
from controllers.pid import PID

# Comment out all kd terms
kd_range = [0]  # Set kd range to only include 0

# Define the system to be controlled
def system(y, t, u):
    dydt = -y + u
    return dydt

# Define the performance metric (e.g., sum of squared errors)
def performance_metric(setpoint, y):
    return np.sum((setpoint - y) ** 2)

# Grid search algorithm
def grid_search(setpoint, kp_range, ki_range, kd_range, t, y0):
    best_kp, best_ki, best_kd = None, None, None
    best_performance = float('inf')

    for kp in kp_range:
        for ki in ki_range:
            for kd in kd_range:
                pid = PID(kp, ki, kd)
                y = np.zeros_like(t)
                y[0] = y0
                for i in range(1, len(t)):
                    dt = t[i] - t[i-1]
                    u = pid.update(y[i-1], setpoint, dt)
                    y[i] = odeint(system, y[i-1], [t[i-1], t[i]], args=(u,))[-1][0]

                performance = performance_metric(setpoint, y)
                if performance < best_performance:
                    best_performance = performance
                    best_kp, best_ki, best_kd = kp, ki, kd

    return best_kp, best_ki, best_kd

# Example usage
setpoint = 1.0
kp_range = np.linspace(0, 10, 10)
ki_range = np.linspace(0, 10, 10)
kd_range = np.linspace(0, 0, 1)
t = np.linspace(0, 10, 100)
y0 = 0

best_kp, best_ki, best_kd = grid_search(setpoint, kp_range, ki_range, kd_range, t, y0)
print(f"Best kp: {best_kp}, ki: {best_ki}, kd: {best_kd}")

# Define the PID controller for two consecutive systems
def grid_search_two_pid(setpoint, kp_range, ki_range, kd_range, t, y0):
    best_kp1, best_ki1, best_kd1 = None, None, None
    best_kp2, best_ki2, best_kd2 = None, None, None
    best_performance = float('inf')

    for kp1 in kp_range:
        for ki1 in ki_range:
            for kd1 in kd_range:
                for kp2 in kp_range:
                    for ki2 in ki_range:
                        for kd2 in kd_range:
                            pid1 = PID(kp1, ki1, kd1)
                            pid2 = PID(kp2, ki2, kd2)
                            y = np.zeros_like(t)
                            y[0] = y0
                            for i in range(1, len(t)):
                                dt = t[i] - t[i-1]
                                u1 = pid1.update(y[i-1], setpoint, dt)
                                y1 = odeint(system, y[i-1], [t[i-1], t[i]], args=(u1,))[-1]
                                u2 = pid2.update(y1, setpoint, dt)
                                y[i] = odeint(system, y1, [t[i-1], t[i]], args=(u2,))[-1][0]

                            performance = performance_metric(setpoint, y)
                            if performance < best_performance:
                                best_performance = performance
                                best_kp1, best_ki1, best_kd1 = kp1, ki1, kd1
                                best_kp2, best_ki2, best_kd2 = kp2, ki2, kd2

    return best_kp1, best_ki1, best_kd1, best_kp2, best_ki2, best_kd2

# Example usage for two consecutive PID controllers
best_kp1, best_ki1, best_kd1, best_kp2, best_ki2, best_kd2 = grid_search_two_pid(setpoint, kp_range, ki_range, kd_range, t, y0)
print(f"Best kp1: {best_kp1}, ki1: {best_ki1}, kd1: {best_kd1}")
print(f"Best kp2: {best_kp2}, ki2: {best_ki2}, kd2: {best_kd2}")