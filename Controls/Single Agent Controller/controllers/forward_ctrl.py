from controllers.pid import PID
class forward_ctrl:
    def __init__(self, pid_params, controller_freq):
        self.forward_controller = PID(**pid_params['abs'])
        self.forward_rate_controller = PID(**pid_params['rate'])
        self.current_forward = 0
        self.desired_forward = 0
        self.current_forward_rate = 0
        self.desired_forward_rate = 0
        self.desired_thrust_z = 0
        self.dt = 1/controller_freq

        self.current_detectable_forward_state = {}
        self.forwards = [] 
        self.detectable_forwards = []

    def update_dd(self, desired_forward): #desired forward, should be called when new command is given
        self.desired_forward = desired_forward
        return self.desired_forward

    def update_cd(self, current_forward): #current forward, should be called if new forward data is available
        self.current_forward = current_forward
        return self.current_forward

    def update_ddr(self): #desired forward rate, should be called after new forward data is given
        self.desired_forward_rate = self.forward_controller.update(self.desired_forward, self.current_forward, self.dt)
        return self.desired_forward_rate
   
    def update_cdr(self, current_forward_rate): #current forward rate, should be called if new forward rate data is available
        self.current_forward_rate = current_forward_rate
        return self.current_forward_rate
    
    def update_dtz(self): #desired thrust z, should be called after desired forward rate is calculated
        self.desired_thrust_z = self.forward_rate_controller.update(self.desired_forward_rate, self.current_forward_rate, self.dt)
        return self.desired_thrust_z
        
    #def update_all(self, target_state, current_state): #should not be used in the end
        # Get current and target forwards
        self.update_cd(current_state)
        self.update_dd(target_state)
        # Update forward rate
        self.update_ddr()
        self.update_cdr(current_state)
        # Update thrust_z
        thrust_z = self.update_dtz()

        return thrust_z