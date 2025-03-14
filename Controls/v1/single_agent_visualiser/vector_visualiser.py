import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from itertools import count
import pandas as pd
from matplotlib.animation import FuncAnimation



def plot_orientation(fig=None, ax=None, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, positions=None, waypoints=None):

    
    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = Rz @ Ry @ Rx
    
    # Basis vectors
    o = R @ np.array([10, 0, 0])

    #increase the size of the plot
    plt.rcParams['figure.figsize'] = [12, 9]
    
    if fig is None or ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    #delete everything on the axis
    ax.clear()

    plt.cla()
    
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([20, 0])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.quiver(x, y, z, o[0], o[1], o[2], color='r', length=0.5, normalize=False)
    ax.scatter(positions[0,:], positions[1,:], positions[2,:], color='black', marker='.')
    ax.scatter(positions[0,0], positions[1,0], positions[2,0], color='red', marker='x')
    ax.scatter(waypoints[0], waypoints[1], waypoints[2], color='green', marker='x')

    plt.draw()
    plt.pause(0.002) 

    return fig, ax