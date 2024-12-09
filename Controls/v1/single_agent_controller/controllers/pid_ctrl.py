import time

class PID:
    def __init__(self, kp:float, ki:float, kd:float, windup_max=None, saturation_max=None, set_point_weighting=False, weight_b=1, weight_c=1):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.windup_max = windup_max
        self.saturation_max = saturation_max
        self.set_point_weighting = set_point_weighting
        self.weight_b = weight_b
        self.weight_c = weight_c
        # Initialize other variables like integral, previous_error, etc.

        self.previous_target_value = 0
        self.previous_value = 0

        self.previous_error = 0
        self.previous_integral = 0

        self.previous_time = time.time()

        #timing => maybe include own clock?
        
    def update(self, target_value, current_value, dt=None, skip=False):

        if dt ==None:
            dt = (time.time() - self.previous_time) 
        self.previous_time = time.time()

        if skip == False:

            self.target_value = target_value
            
            error = target_value - current_value
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

            if self.set_point_weighting is False:
                output = self.kp * error + self.ki * integral + self.kd * derivative
            else:
                error2 = self.weight_b*target_value - current_value
                derivative2 = (target_value - self.previous_target_value)
                derivative3 = (current_value - self.previous_value)
                error3 = self.weight_c*derivative2 - derivative3
                output = self.kp * error2 + self.ki * integral + self.kd * error3

            #saturation for actuator, anti-windup in case of max saturation
            if self.saturation_max is not None:
                if output > self.saturation_max:
                    output = self.saturation_max
                    integral = self.previous_integral
                elif output < -self.saturation_max:
                    output = -self.saturation_max
                    self.integral = self.previous_integral

            self.previous_target_value = target_value
            self.previous_value = current_value
            self.previous_integral = integral
            self.previous_error = error

            return output
        
        else:
            self.previous_time = time.time()
            return 0.0
        

