from single_agent_controller.controllers.pid_ctrl import PID
class angle_ctrl:
    def __init__(self, pid_params, controller_freq):
        self.angle_controller = PID(**pid_params['abs'])
        self.angle_rate_controller = PID(**pid_params['rate'])

        #self.current_angle = 0
        self.desired_angle = 0
        #self.current_angle_rate = 0
        self.desired_angle_rate = 0
        #self.current_torque = 0
        self.desired_torque = 0

        self.current_detectable_angle = 0
        self.current_detectable_angle_rate = 0
        self.current_detectable_torque = 0

        self.dt = 1/controller_freq

    def update_da(self, desired_angle): #desired angle, should be called when new command is given
        self.desired_angle = desired_angle
        self.angle_controller.integral = 0
        self.angle_rate_controller.integral = 0
        self.angle_controller.last_error = 0
        self.angle_rate_controller.last_error = 0
        return self.desired_angle

    def update_cda(self, imu_angle): #current angle, should be called if new angle data is available
        self.current_detectable_angle = imu_angle
        return self.current_detectable_angle

    def update_dar(self): #desired angle rate, should be called after new angle data is given
        self.desired_angle_rate = self.angle_controller.update(self.desired_angle, self.current_detectable_angle, self.dt)
        print("desired angle rate: ", self.desired_angle_rate)
        return self.desired_angle_rate
   
    def update_cdar(self, imu_angle_rate): #current angle rate, should be called if new angle rate data is available
        self.current_detectable_angle_rate = imu_angle_rate
        return self.current_detectable_angle_rate
    
    def update_dtau(self): #desired torque, should be called after desired angle rate is calculated
        self.desired_torque = self.angle_rate_controller.update(self.desired_angle_rate, self.current_detectable_angle_rate, self.dt)
        self.current_detectable_torque = self.desired_torque
        print("desired torque: ", self.desired_torque)
        return self.desired_torque

    def append_to_state(self, state):
        state['detectable angle'] = self.current_detectable_angle
        state['detectable rate'] = self.current_detectable_angle_rate
        state['detectable torque'] = self.current_detectable_torque

