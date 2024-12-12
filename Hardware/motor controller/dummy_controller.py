import CA
import time
import dht11_driver

motor1 = CA.motor(6,True)
motor2 = CA.motor(7,True)
motor3 = CA.motor(8,True)
motor4 = CA.motor(9,True)

motor5 = CA.motor(5,True)
motor6 = CA.motor(3,True)
motor7 = CA.motor(4,False)
motor8 = CA.motor(2,False)



dhl11 = dht11_driver.Humidity_aSensor()

# for i in range(30):
#     dhl11.read_data()
#     time.sleep(1)

# call sudo pigpiod

def startup():
    motor1.set_thrust(0)
    motor2.set_thrust(0)
    motor3.set_thrust(0)
    motor4.set_thrust(0)
    motor5.set_thrust(0)
    motor6.set_thrust(0)
    motor7.set_thrust(0)
    motor8.set_thrust(0)
    time.sleep(10)

#startup()

time.sleep(2)

# motor1.set_thrust(30)
# motor2.set_thrust(30)
# motor3.set_thrust(30)
# motor4.set_thrust(30)

time.sleep(2)

# motor1.set_thrust(0)
# motor2.set_thrust(0)
# motor3.set_thrust(0)
# motor4.set_thrust(0)

def spin_cycle(motor):
    
    for i in range(20):
        motor.set_thrust(i)
        time.sleep(0.1)

    for i in range(20):
        motor.set_thrust(20-i)
        time.sleep(0.1)

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


spin_cycle(motor8)
spin_cycle(motor6)
spin_cycle(motor7)
spin_cycle(motor5)

# for i in range(50):
#     motor5.set_thrust(20)
#     motor6.set_thrust(20)
#     motor7.set_thrust(20)
#     motor8.set_thrust(20)

#     time.sleep(0.1)


# -------------------------------------- “
# Important always put this at the end!!!
motor5.set_thrust(0)
motor6.set_thrust(0)
motor7.set_thrust(0)
motor8.set_thrust(0)