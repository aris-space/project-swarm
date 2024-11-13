from controllers.pid import PID
class roll_ctrl:
    def __init__(self, pid_params):
        self.roll_controller = PID(**pid_params['roll_controller'])
        self.roll_rate_controller = PID(**pid_params['roll_controller'])
        self.current_roll = 0
        self.desired_roll = 0
        self.current_roll_rate = 0
        self.desired_roll_rate = 0
        self.desired_torque_y = 0
        self.dt = 0.001

    def update_dd(self, desired_roll): #desired roll, should be called when new command is given
        self.desired_roll = desired_roll
        return self.desired_roll

    def update_cd(self, current_roll): #current roll, should be called if new roll data is available
        self.current_roll = current_roll
        return self.current_roll

    def update_ddr(self): #desired roll rate, should be called after new roll data is given
        self.desired_roll_rate = self.roll_controller.update(self.desired_roll, self.current_roll, self.dt)
        return self.desired_roll_rate
   
    def update_cdr(self, current_roll_rate): #current roll rate, should be called if new roll rate data is available
        self.current_roll_rate = current_roll_rate
        return self.current_roll_rate
    
    def update_dtauy(self): #desired torque y, should be called after desired roll rate is calculated
        self.desired_torque_y = self.roll_rate_controller.update(self.desired_roll_rate, self.current_roll_rate, self.dt)
        return self.desired_torque_y
        
    #def update_all(self, target_state, current_state): #should not be used in the end
        # Get current and target rolls
        self.update_cd(current_state)
        self.update_dd(target_state)
        # Update roll rate
        self.update_ddr()
        self.update_cdr(current_state)
        # Update_torque_y
        torque_y = self.update_dtz()

        return torque_y