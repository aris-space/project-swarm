import CA
import time


motor1 = CA.motor(23,False)



# motor1.set_thrust(50)
# print("set speed to 50")
# time.sleep(5)

# motor1.set_thrust(0)
# print("set speed to 0")
# time.sleep(5)

# motor1.set_thrust(-50)
# print("set speed to -50")
# time.sleep(5)

# motor1.set_thrust(0)
# print("set speed to 0")
# time.sleep(5)


for i in range(50):
    motor1.set_thrust(i)
    time.sleep(0.1)

for i in range(50):
    motor1.set_thrust(50-i)
    time.sleep(0.1)



# -------------------------------------- “
# Important always put this at the end!!!
motor1.set_thrust(0)