import serial
import matplotlib.pyplot as plt
import matplotlib.animation as anim

class position:
    def __init__(self, draw:bool=False, memory:int=1):
        self.x = []
        self.y = []
        self.z = []
        self.draw = draw
        #self.line = None
        self.memory = memory
        self.ser = serial.Serial('/dev/serial10', 115200)
        self.fig = None
        self.ax = None

        if self.draw:
            # Create a new figure and axis
            self.fig, self.ax = plt.subplots()

            # Set the x and y axis limits
            self.ax.set_xlim(0, 4000)
            self.ax.set_ylim(0, 4000)

            self.ax.set_xlabel("x")
            self.ax.set_ylabel("y")
            self.ax.set_title("Absolute Position")

            anim.FuncAnimation(self.fig, self.update_visual, frames=range(100), interval=100, blit=True)

            plt.show()
        
    def update_visual(self):
        line, = self.ax.plot([], [], 'bo')
        line.set_data(self.x, self.y)
        return line
    
    def update(self):
        xByte = self.ser.read(1)
        yByte = self.ser.read(1)

        self.x.append(int.from_bytes(xByte, byteorder='big'))
        self.y.append(int.from_bytes(yByte, byteorder='big'))

        if len(self.x) > self.memory:
            self.x.pop(0)
            self.y.pop(0)
