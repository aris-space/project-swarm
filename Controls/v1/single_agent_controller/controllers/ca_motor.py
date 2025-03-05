import pigpio
import time
from scipy.interpolate import interp1d

# map PWM timing to range of -100 to 100
T = interp1d([-100,100],[1100,1900])


# Setup change 
#PWM_PINS = [18, 19, 23, 24]  # GPIO pins for PWM signals
FREQUENCY = 200              # PWM frequency in Hz
ESC_MIN = 1100               # Minimum pulse width (1.1 ms) for ESC (idle/stop)
ESC_MAX = 1900               # Maximum pulse width (1.9 ms) for ESC (full throttle)
ESC_NEUTRAL = 1500           # Neutral pulse width (1.5 ms)

# Initialize pigpio
pi = pigpio.pi()


class MCA:

    def __init__(self, pin_def: int, reversed_def: bool): # todo: add reversed

        #self.motor_nr = motor_nr_def # redundant atm

        self.pin = pin_def
        self.reversed = reversed_def

        # Startup ESC

        pi.set_PWM_frequency(self.pin, FREQUENCY) # set frequency
        pi.set_servo_pulsewidth(self.pin, ESC_NEUTRAL)  # Set neutral position
        time.sleep(0.5) # need to leave signal at neutral for a quick moment such that the ESC will start up

        return None

    def set_thrust(self, thrust):
        # Note: for start "speed" we assume linear relationship
        # However this function should handle non linear tuning later on
        # sanity check
        if int(thrust) not in range(-40,41): # warning. 100 is veeryy fast! If safety doesn't trigger. Run!
            raise ValueError("Speed must be in range [-40,40] and is currently " + str(thrust))
        
        #check if eversed
        if self.reversed:
            #print("is reversed: ", T(-thrust))
            pi.set_servo_pulsewidth(self.pin, T(-thrust -5)) # fine tuning for dead zone
        else:
            pi.set_servo_pulsewidth(self.pin, T(thrust))


        return None
