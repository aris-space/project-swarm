import sys
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl

class Live3DVisualizer:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Live 3D Visualization')
        self.window.setGeometry(0, 110, 800, 600)
        self.window.opts['distance'] = 40
        self.window.show()

        # Create axis
        axis = gl.GLAxisItem()
        axis.setSize(x=10, y=10, z=10)
        self.window.addItem(axis)

        # Create a grid
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.window.addItem(grid)

        # Create a cube to visualize
        mesh_data = gl.MeshData.sphere(rows=10, cols=20, radius=1)
        self.object = gl.GLMeshItem(meshdata=mesh_data, smooth=False, color=(1, 0, 0, 1), shader='shaded')
        self.object.translate(0, 0, 0)
        self.window.addItem(self.object)

       # Initial position and orientation
        self.position = np.array([0, 0, 0])
        self.euler_angles = np.array([0, 0, 0])  # [z_angle, y_angle, x_angle]

        # Timer for updating the visualization
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)  # Update rate in milliseconds


    def set_euler_angles(self, z_angle, y_angle, x_angle):
        # Update the Euler angles
        self.euler_angles = np.array([z_angle, y_angle, x_angle])

    def set_position(self, x, y, z):
        # Update the object's position
        self.position = np.array([x, y, z])

    def update(self):
        self.object.resetTransform()
        self.object.translate(*self.position)
        self.object.rotate(self.euler_angles[0], 0, 0, 1)  # Rotate around z-axis
        self.object.rotate(self.euler_angles[1], 0, 1, 0)  # Rotate around y-axis
        self.object.rotate(self.euler_angles[2], 1, 0, 0)  # Rotate around x-axis

    def start(self):
        self.app.exec_()
