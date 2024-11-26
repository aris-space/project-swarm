import CA
import time

motor1 = CA.motor(1,23,False)



motor1.set_thrust(50)
print("set speed to 50")
time.sleep(5)

motor1.set_thrust(0)
print("set speed to 0")
time.sleep(5)

motor1.set_thrust(-50)
print("set speed to -50")
time.sleep(5)

motor1.set_thrust(0)
print("set speed to 0")
time.sleep(5)

