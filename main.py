#Startbasis of code: swarm_final.py
#angepasste calc
import pigpio
import time
import struct

'''
PRE: data package; 2
POST: CRC8 value for data package
'''
def crc8(data):
   """
   Calculate an 8-bit CRC using the polynomial: x^8 + x^2 + x + 1
   (as described in the Keller documentation).
   """
   crc = 0
   for d in data:
       crc ^= d << 8
       for _ in range(8):
           if crc & 0x8000:
               crc ^= 0x8380
           crc <<= 1
   return (crc >> 8) & 0xFF
'''
PRE: regAddr: Register address (0x00 for pressure, 0x04 for Temperature).
POST: return float value for regAddr (Pressure in bar, Temperature in °C), NONE if failure
'''



#start communication
pi = pigpio.pi() # Connect to the pigpio daemon
handle = pi.i2c_open(1, 0x40) # Open I2C on bus 1: 7-bit I2C address for the 9LX sensor

def read_sensor_float(regAddr):
    
    #request
    # Build 3-byte request: [0x20, regAddr, CRC] (0x20 means "request 4 data bytes from Block 0" (float mode))
    request = [0x20, regAddr]
    request.append(crc8(request))
    # Write request -> sensor
    pi.i2c_write_device(handle, bytes(request))
    # Wait for the sensor to prepare data; (adjust if necessary)
    time.sleep(0.02)
    
    
    #read
    # Read 7 bytes from the sensor
    count, data = pi.i2c_read_device(handle, 7)
    if count < 7:
        print("\n\n\n>>>>>>>> ERROR: read 7 bytes\n\n\n")
        return None
    response = list(data)
    # Verify the CRC over the first 6 bytes
    computed_crc = crc8(response[:6])
    if computed_crc != response[6]:
        print("\n\n\n>>>>>>>> ERROR: CRC\n\n\n")
        return None
    # Check that the I2CDataReady flag (bit 4 in the state byte) is set.
    if (response[0] & 0x10) == 0:
        print("\n\n\n>>>>>>>> ERROR: ready flag\n\n\n")
        return None
    # Reassemble the 4 data bytes (big-endian) into a 32-bit unsigned integer.
    raw = (response[2] << 24) | (response[3] << 16) | (response[4] << 8) | response[5]
    # Convert the raw 32-bit value to a float using struct (network order is big-endian)
    value = struct.unpack("!f", struct.pack("!I", raw))[0]

    return value





zero_air_pressure = None #in bar
'''
#PRE: sensor correctly attached, lib: pigpio,time, struct
#POST: depth in meter, if error in reading -> Value = none
'''
def keller_depth_float():
    global zero_air_pressure
    g = 9.80600 # dübi [m/s^2]
    density = 997.0474 #freshwater [kg/m^3]
    # gets pressure
    pressure = read_sensor_float(0x00)#read_sensor_float(0x00) # [bar]
    
    
    # zero_air_pressure -> very first pressure reading
    if(zero_air_pressure == None):
        zero_air_pressure = pressure
    

    #print("P=", pressure, "bar")

    if pressure == None:
       return None
    #calculates depth
    depth = (pressure - zero_air_pressure)*100000 / (g * density) # meter below surface
    return round(depth, 3), round(pressure, 4)



'''
#PRE: sensor correctly attached, lib: pigpio,time, struct
#POST: temperature in °C, if error in reading -> Value = none
'''
def keller_temperature_float():
   temp = read_sensor_float(0x04)
   if temp == None:
       return None
   return round(temp,1) # °C


def main():
    print("START...")
    i = 0
    max_diff = 0
    max_h=0
    new_max = False
    while True:
        
            value = keller_depth_float()
            h = value[0]
            p = value[1]
            diff_p = abs(round(p - zero_air_pressure, 4))
            if diff_p > max_diff and i > 1:
                max_diff = diff_p
                new_max = True
                
            if abs(h) > max_h and i > 1:
                max_h = abs(h)
                new_max = True
            
            if new_max:
                print("\n  --->>>>>>>   max_diff_p=", max_diff, "bar / max_h=", max_h,"m")
            
            new_max = False

            if h > 0.05:
                print("====================================================== h > 0.05  ====================================================== ")


            print(i,"pressure=" ,p,"bar / diff_p=",diff_p,"bar / depth=",h,"m / max_diff_p=", max_diff, "bar / max_h=", max_h,"m")
            
 

            
            #print("T=", keller_temperature_float(), "°C")
            i +=1
            time.sleep(1)


    #end communication
    pi.i2c_close(handle)
    pi.stop()


   






if __name__ == "__main__":
   main()