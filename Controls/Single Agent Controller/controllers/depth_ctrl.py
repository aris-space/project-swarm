from controllers.pid_v1 import PID
class depth_ctrl:
    def __init__(self, pid_params, controller_freq):
        self.depth_controller = PID(**pid_params['abs'])
        self.depth_rate_controller = PID(**pid_params['rate'])

        #self.current_depth = 0
        self.desired_depth = 0
        #self.current_depth_rate = 0
        self.desired_depth_rate = 0
        #self.current_thrust = 0
        self.desired_thrust = 0

        self.current_detectable_depth = 0
        self.current_detectable_depth_rate = 0
        self.current_detectable_thrust = 0

        self.dt = 1/controller_freq

    def update_dd(self, desired_depth): #desired depth, should be called when new command is given
        self.desired_depth = desired_depth
        return self.desired_depth

    def update_cdd(self, imu_depth): #current depth, should be called if new depth data is available
        self.current_detectable_depth = imu_depth
        return self.current_detectable_depth

    def update_ddr(self): #desired depth rate, should be called after new depth data is given
        self.desired_depth_rate = self.depth_controller.update(self.desired_depth, self.current_detectable_depth, self.dt)
        print("desired depth rate: ", self.desired_depth_rate)
        return self.desired_depth_rate
   
    def update_cddr(self, imu_depth_rate): #current depth rate, should be called if new depth rate data is available
        self.current_detectable_depth_rate = imu_depth_rate
        return self.current_detectable_depth_rate
    
    def update_dt(self): #desired thrust, should be called after desired depth rate is calculated
        self.desired_thrust = self.depth_rate_controller.update(self.desired_depth_rate, self.current_detectable_depth_rate, self.dt)
        self.current_detectable_thrust = self.desired_thrust
        print("desired thrust: ", self.desired_thrust)
        return self.desired_thrust

    #def update_depth_and_rate(depth, rate):
        self.current_depth = depth
        self.current_depth_rate = rate

    def append_to_state(self, state):
        state['detectable depth'] = self.current_detectable_depth
        state['detectable rate'] = self.current_detectable_depth_rate
        state['detectable thrust'] = self.current_detectable_thrust

