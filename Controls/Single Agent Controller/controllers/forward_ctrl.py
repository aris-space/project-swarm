from controllers.pid_v1 import PID
class fwd_ctrl:
    def __init__(self, pid_params, controller_freq):
        self.fwd_controller = PID(**pid_params['abs'])
        self.fwd_rate_controller = PID(**pid_params['rate'])

        #self.current_fwd = 0
        self.desired_fwd = 0
        #self.current_fwd_rate = 0
        self.desired_fwd_rate = 0
        #self.current_thrust = 0
        self.desired_thrust = 0

        self.current_detectable_fwd = 0
        self.current_detectable_fwd_rate = 0
        self.current_detectable_thrust = 0

        self.dt = 1/controller_freq

    def update_dd(self, desired_fwd): #desired fwd, should be called when new command is given
        self.desired_fwd = desired_fwd
        self.fwd_controller.integral = 0
        self.fwd_rate_controller.integral = 0
        self.fwd_controller.last_error = 0
        self.fwd_rate_controller.last_error = 0
        return self.desired_fwd

    def update_cdd(self, imu_fwd): #current fwd, should be called if new fwd data is available
        self.current_detectable_fwd = imu_fwd
        return self.current_detectable_fwd

    def update_ddr(self): #desired fwd rate, should be called after new fwd data is given
        self.desired_fwd_rate = self.fwd_controller.update(self.desired_fwd, self.current_detectable_fwd, self.dt)
        print("desired fwd rate: ", self.desired_fwd_rate)
        return self.desired_fwd_rate
   
    def update_cddr(self, imu_fwd_rate): #current fwd rate, should be called if new fwd rate data is available
        self.current_detectable_fwd_rate = imu_fwd_rate
        return self.current_detectable_fwd_rate
    
    def update_dt(self): #desired thrust, should be called after desired fwd rate is calculated
        self.desired_thrust = self.fwd_rate_controller.update(self.desired_fwd_rate, self.current_detectable_fwd_rate, self.dt)
        self.current_detectable_thrust = self.desired_thrust
        print("desired thrust: ", self.desired_thrust)
        return self.desired_thrust

    #def update_fwd_and_rate(fwd, rate):
        self.current_fwd = fwd
        self.current_fwd_rate = rate

    def append_to_state(self, state):
        state['detectable fwd'] = self.current_detectable_fwd
        state['detectable rate'] = self.current_detectable_fwd_rate
        state['detectable thrust'] = self.current_detectable_thrust