from controllers.pid import PID
class angle_ctrl:
    def __init__(self, pid_params, controller_freq):
        self.angle_controller = PID(**pid_params['abs'])
        self.angle_rate_controller = PID(**pid_params['rate'])
        self.current_angle = 0
        self.desired_angle = 0
        self.current_angle_rate = 0
        self.desired_angle_rate = 0
        self.desired_torque_y = 0
        self.dt = 1/controller_freq

        self.current_detectable_angle_state = {}
        self.angles = []
        self.detectable_angles = []

    def update_dd(self, desired_angle): #desired angle, should be called when new command is given
        self.desired_angle = desired_angle
        return self.desired_angle

    def update_cd(self, current_angle): #current angle, should be called if new angle data is available
        self.current_angle = current_angle
        return self.current_angle

    def update_ddr(self): #desired angle rate, should be called after new angle data is given
        self.desired_angle_rate = self.angle_controller.update(self.desired_angle, self.current_angle, self.dt)
        return self.desired_angle_rate
   
    def update_cdr(self, current_angle_rate): #current angle rate, should be called if new angle rate data is available
        self.current_angle_rate = current_angle_rate
        return self.current_angle_rate
    
    def update_dtau(self): #desired torque y, should be called after desired angle rate is calculated
        self.desired_torque = self.angle_rate_controller.update(self.desired_angle_rate, self.current_angle_rate, self.dt)
        return self.desired_torque
        
    #def update_all(self, target_state, current_state): #should not be used in the end
        # Get current and target angles
        self.update_cd(current_state)
        self.update_dd(target_state)
        # Update angle rate
        self.update_ddr()
        self.update_cdr(current_state)
        # Update_torque_y
        torque_y = self.update_dtz()

        return torque_y