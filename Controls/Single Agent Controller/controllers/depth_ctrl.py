from controllers.pid import PID
class depth_ctrl:
    def __init__(self, pid_params, controller_freq):
        self.depth_controller = PID(**pid_params['abs'])
        self.depth_rate_controller = PID(**pid_params['rate'])
        self.current_depth = 0
        self.desired_depth = 0
        self.current_depth_rate = 0
        self.desired_depth_rate = 0
        self.desired_thrust_z = 0
        self.dt = 1/controller_freq

        self.current_detectable_depth_state = {}
        self.depths = [] 
        self.detectable_depths = []

    def update_dd(self, desired_depth): #desired depth, should be called when new command is given
        self.desired_depth = desired_depth
        return self.desired_depth

    def update_cd(self, current_depth): #current depth, should be called if new depth data is available
        self.current_depth = current_depth
        return self.current_depth

    def update_ddr(self): #desired depth rate, should be called after new depth data is given
        self.desired_depth_rate = self.depth_controller.update(self.desired_depth, self.current_depth, self.dt)
        return self.desired_depth_rate
   
    def update_cdr(self, current_depth_rate): #current depth rate, should be called if new depth rate data is available
        self.current_depth_rate = current_depth_rate
        return self.current_depth_rate
    
    def update_dtz(self): #desired thrust z, should be called after desired depth rate is calculated
        self.desired_thrust_z = self.depth_rate_controller.update(self.desired_depth_rate, self.current_depth_rate, self.dt)
        return self.desired_thrust_z
        
    #def update_all(self, target_state, current_state): #should not be used in the end
        # Get current and target depths
        self.update_cd(current_state)
        self.update_dd(target_state)
        # Update depth rate
        self.update_ddr()
        self.update_cdr(current_state)
        # Update thrust_z
        thrust_z = self.update_dtz()

        return thrust_z