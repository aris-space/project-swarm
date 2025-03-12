class Xsens_IMU:

    def __init__(self, swarm_storage):

        self.swarm_storage = swarm_storage

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser.flush()

        self.quat_offsets = np.array([])
        self.quat = np.array([])
        self.angular_rates = np.array([])
        self.acceleration = np.array([])


    def send_input():
        
        key = sys.stdin.read(1)  # Read one character at a time
        if key == '1':
            ser.write(b'\n1')
            print("Sent: 1")
        elif key == '0':
            ser.write(b'\n0')
            print("Sent: 0")


    def receive_input_xsens(ser, quat_offsets, quat, angular_rates, acceleration):

        #global j,k,l,m

        if ser.in_waiting > 0:
            #print(j, k, l, m)
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if "QR" in line:
                
                self.quat_offsets = np.array(line[3:].split(','), dtype = float)
                # MOVE TO SWARM STORAGE HERE

                # j+=1
                # #if (j%50 == 0): print("Quaternion relative to offsets:", quat_offsets)
            elif 'Q' in line:
                
                self.quat = np.array(line[2:].split(','), dtype = float)
                # MOVE TO SWARM STORAGE HERE
                
                #k+=1
                # #if (k%50 == 0): print("Quaternion:", quat)
            elif "R" in line and "Q" not in line:
                
                self.angular_rates = np.array(line[2:].split(','), dtype = float)
                # MOVE TO SWARM STORAGE HERE
                
                #l+=1
                # #if (l%50 == 0): print("Angular Rates (x,y,z):", angular_rates)
            elif "A" in line:
                
                self.acceleration = np.array(line[2:].split(','), dtype = float)
                # MOVE TO SWARM STORAGE HERE

                #m+=1
                # #if (m%50 == 0): print("Acceleration", acceleration)
            else:
                print("Unknown line:", line)
