from v1.single_agent_controller.controllers.ca_system import LLC2
from v1.single_agent_controller.controllers.ca_motor import MCA
from utils.constants2 import *
import numpy as np


class SCA(LLC2):
    def __init__(self, LLC):

        self.LLC = LLC()
        
        self.motor_tl_control_allocation = MCA(pin_def= PIN_TL, reversed_def=REV_TL)
        self.motor_tr_control_allocation = MCA(pin_def= PIN_TR, reversed_def=REV_TR)
        self.motor_bl_control_allocation = MCA(pin_def= PIN_BL, reversed_def=REV_BL)
        self.motor_br_control_allocation = MCA(pin_def= PIN_BR,  reversed_def=REV_BR)


        self.control_allocation_matrix = {
            #motors         pos direction   
            'motor_tl': {   'roll': -1,     'pitch': 1  },
            'motor_tr': {   'roll': 1,      'pitch': 1  },
            'motor_bl': {   'roll': -1,     'pitch': -1 },
            'motor_br': {   'roll': 1,      'pitch': -1 }
        }

        self.motor_signals = np.zeros(4)


    def update_motor_thrusts(self):

        # motors are numbered from left to right, from top to bottom
        #motor                  local_x_torque * C_ROLL * +/-1 + local_y_torque * C_PITCH * +/-1
        self.motor_signals[0] = self.LLC.local_x_torque * C_ROLL * self.control_allocation_matrix['motor_tl']['roll'] + self.LLC.local_y_torque * C_PITCH * self.control_allocation_matrix['motor_tl']['pitch']
        self.motor_signals[1] = self.LLC.local_x_torque * C_ROLL * self.control_allocation_matrix['motor_tr']['roll'] + self.LLC.local_y_torque * C_PITCH * self.control_allocation_matrix['motor_tr']['pitch']
        self.motor_signals[2] = self.LLC.local_x_torque * C_ROLL * self.control_allocation_matrix['motor_bl']['roll'] + self.LLC.local_y_torque * C_PITCH * self.control_allocation_matrix['motor_bl']['pitch']
        self.motor_signals[3] = self.LLC.local_x_torque * C_ROLL * self.control_allocation_matrix['motor_br']['roll'] + self.LLC.local_y_torque * C_PITCH * self.control_allocation_matrix['motor_br']['pitch']

        #saturation check
        for x in self.motor_signals:
            if x > MOTOR_SATURATION:
                x = MOTOR_SATURATION
                print("Pos. Motor saturation reached")
            elif x < -MOTOR_SATURATION:
                x = -MOTOR_SATURATION
                print("Neg. Motor saturation reached")
       
        print("motor signals: ", self.motor_signals)

        self.motor_tl_control_allocation.set_thrust(self.motor_signals[0])
        self.motor_tr_control_allocation.set_thrust(self.motor_signals[1])
        self.motor_bl_control_allocation.set_thrust(self.motor_signals[2])
        self.motor_br_control_allocation.set_thrust(self.motor_signals[3])
        

            
        
        