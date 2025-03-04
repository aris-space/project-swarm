import time

class PID_w_error:
    def __init__(self, kp:float, ki:float, kd:float, windup_max=None, saturation_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.windup_max = windup_max
        self.saturation_max = saturation_max
        # Initialize other variables like integral, previous_error, etc.


        self.previous_error = 0
        self.previous_integral = 0

        self.previous_time = time.time()

        #timing => maybe include own clock?
        
    def update(self, error, dt=None, skip=False):

        if dt == None:
            dt = (time.time() - self.previous_time)
        self.previous_time = time.time()

        if skip == False:
            
            integral = self.previous_integral + error * dt
            derivative = (error - self.previous_error) / dt

            
            #anti-windup
            if self.windup_max is not None:
                if integral > self.windup_max:
                    integral = self.windup_max
                elif integral < -self.windup_max:
                    integral = -self.windup_max

            #anti-derivative kick => Set Point Weighting
            #to be implemented

            output = self.kp * error + self.ki * integral + self.kd * derivative

            #saturation for actuator, anti-windup in case of max saturation
            if self.saturation_max is not None:
                if output > self.saturation_max:
                    output = self.saturation_max
                    integral = self.previous_integral
                elif output < -self.saturation_max:
                    output = -self.saturation_max
                    self.integral = self.previous_integral


            self.previous_integral = integral
            self.previous_error = error

            return output
        
        else:
            self.previous_time = time.time()
            return 0.0
        

