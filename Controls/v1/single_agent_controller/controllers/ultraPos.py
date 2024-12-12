import serial
import matplotlib.pyplot as plt
import matplotlib.animation as anim

class Position:
    def __init__(self, draw: bool = False, memory: int = 1, port: str = '/dev/serial10', baudrate: int = 115200):
        self.x = []
        self.y = []
        self.z = []
        self.draw = draw
        self.memory = memory
        self.ser = None
        self.fig = None
        self.ax = None

        try:
            self.ser = serial.Serial(port, baudrate)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return

        if self.draw:
            self.fig, self.ax = plt.subplots()
            self.ax.set_xlim(0, 4000)
            self.ax.set_ylim(0, 4000)
            self.ax.set_xlabel("x")
            self.ax.set_ylabel("y")
            self.ax.set_title("Absolute Position")
            self.line, = self.ax.plot([], [], 'bo')
            self.ani = anim.FuncAnimation(self.fig, self.update_visual, frames=range(100), interval=100, blit=True)
            plt.show()

    def update_visual(self, frame):
        self.line.set_data(self.x, self.y)
        return self.line,

    def update(self):
        try:
            xByte = self.ser.read(1)
            yByte = self.ser.read(1)
        except serial.SerialException as e:
            print(f"Error reading from serial port: {e}")
            return

        self.x.append(int.from_bytes(xByte, byteorder='big'))
        self.y.append(int.from_bytes(yByte, byteorder='big'))

        if len(self.x) > self.memory:
            self.x.pop(0)
            self.y.pop(0)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

# Example usage:
# pos = Position(draw=True, memory=10)
# while True:
#     pos.update()
