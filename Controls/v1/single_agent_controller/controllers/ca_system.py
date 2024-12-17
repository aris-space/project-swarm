from v1.single_agent_controller.controllers.ca_system import LLC2
from v1.single_agent_controller.controllers.ca_motor import MCA
from utils.constants2 import *
import numpy as np


class SCA(LLC2):
    def __init__(self, LLC):

        self.LLC = LLC()
        
        self.motor_zfl_control_allocation = MCA(pin_def= PIN_ZFL, reversed_def=REV_ZFL)
        self.motor_zfr_control_allocation = MCA(pin_def= PIN_ZFR, reversed_def=REV_ZFR)
        self.motor_zbl_control_allocation = MCA(pin_def= PIN_ZBL, reversed_def=REV_ZBL)
        self.motor_zbr_control_allocation = MCA(pin_def= PIN_ZBR,  reversed_def=REV_ZBR)
        self.motor_xyfl_control_allocation = MCA(pin_def= PIN_XYFL, reversed_def=REV_XYFL)
        self.motor_xyfr_control_allocation = MCA(pin_def= PIN_XYFR, reversed_def=REV_XYFR)
        self.motor_xybl_control_allocation = MCA(pin_def= PIN_XYBL, reversed_def=REV_XYBL)
        self.motor_xybr_control_allocation = MCA(pin_def= PIN_XYBR,  reversed_def=REV_XYBR)



        self.control_allocation_matrix = {
            #motors         pos direction   
            'motor_zfl':  {   'roll': -1 ,    'pitch': 1  ,   'yaw': 0},
            'motor_zfr':  {   'roll': 1  ,    'pitch': 1  ,   'yaw': 0},
            'motor_zbl':  {   'roll': -1 ,    'pitch': -1 ,   'yaw': 0},
            'motor_zbr':  {   'roll': 1  ,    'pitch': -1 ,   'yaw': 0},
            'motor_xyfl': {   'roll': 0  ,    'pitch': 0  ,   'yaw': 1},
            'motor_xyfr': {   'roll': 0  ,    'pitch': 0  ,   'yaw': 1},
            'motor_xybl': {   'roll': 0  ,    'pitch': 0  ,   'yaw': 1},
            'motor_xybr': {   'roll': 0  ,    'pitch': 0  ,   'yaw': 1}
            

        }

        """
            sub

                            front

                    xyfl                xyfr
            left            zfl zfr             right
                            zbl zbr
                    xybl                xybr
                    
                            back
            
            
            
        """




        self.motor_signals = np.zeros(4)


    def update_motor_thrusts(self):

        # motors are numbered from left to right, from top to bottom
        #motor                  local_x_torque * C_ROLL * +/-1 + local_y_torque * C_PITCH * +/-1
        self.motor_signals[0] = self.LLC.local_x_torque * C_ROLL * self.control_allocation_matrix['motor_zfl']['roll'] + self.LLC.local_y_torque * C_PITCH * self.control_allocation_matrix['motor_zfl']['pitch']
        self.motor_signals[1] = self.LLC.local_x_torque * C_ROLL * self.control_allocation_matrix['motor_zfr']['roll'] + self.LLC.local_y_torque * C_PITCH * self.control_allocation_matrix['motor_zfr']['pitch']
        self.motor_signals[2] = self.LLC.local_x_torque * C_ROLL * self.control_allocation_matrix['motor_zbl']['roll'] + self.LLC.local_y_torque * C_PITCH * self.control_allocation_matrix['motor_zbl']['pitch']
        self.motor_signals[3] = self.LLC.local_x_torque * C_ROLL * self.control_allocation_matrix['motor_zbr']['roll'] + self.LLC.local_y_torque * C_PITCH * self.control_allocation_matrix['motor_zbr']['pitch']
        self.motor_signals[4] = self.LLC.local_z_torque * C_YAW * self.control_allocation_matrix['motor_xyfl']['roll'] 
        self.motor_signals[5] = self.LLC.local_z_torque * C_YAW * self.control_allocation_matrix['motor_xyfr']['roll']
        self.motor_signals[6] = self.LLC.local_z_torque * C_YAW * self.control_allocation_matrix['motor_xybl']['roll']
        self.motor_signals[7] = self.LLC.local_z_torque * C_YAW * self.control_allocation_matrix['motor_xybr']['roll']

        #saturation check
        for x in self.motor_signals:
            if x > MOTOR_SATURATION:
                x = MOTOR_SATURATION
                print("Pos. Motor saturation reached")
            elif x < -MOTOR_SATURATION:
                x = -MOTOR_SATURATION
                print("Neg. Motor saturation reached")
       
        print("motor signals: ", self.motor_signals)

        self.motor_zfl_control_allocation.set_thrust(self.motor_signals[0])
        self.motor_zfr_control_allocation.set_thrust(self.motor_signals[1])
        self.motor_zbl_control_allocation.set_thrust(self.motor_signals[2])
        self.motor_zbr_control_allocation.set_thrust(self.motor_signals[3])
        self.motor_xyfl_control_allocation.set_thrust(self.motor_signals[4])
        self.motor_xyfr_control_allocation.set_thrust(self.motor_signals[5])
        self.motor_xybl_control_allocation.set_thrust(self.motor_signals[6])
        self.motor_xybr_control_allocation.set_thrust(self.motor_signals[7])

    def update_motor_thrusts_manual(self, local_x_torque_man, local_y_torque_man, local_z_torque_man):

        self.motor_signals[0] = local_x_torque_man * C_ROLL * self.control_allocation_matrix['motor_zfl']['roll'] + local_y_torque_man * C_PITCH * self.control_allocation_matrix['motor_zfl']['pitch']
        self.motor_signals[1] = local_x_torque_man * C_ROLL * self.control_allocation_matrix['motor_zfr']['roll'] + local_y_torque_man * C_PITCH * self.control_allocation_matrix['motor_zfr']['pitch']
        self.motor_signals[2] = local_x_torque_man * C_ROLL * self.control_allocation_matrix['motor_zbl']['roll'] + local_y_torque_man * C_PITCH * self.control_allocation_matrix['motor_zbl']['pitch']
        self.motor_signals[3] = local_x_torque_man * C_ROLL * self.control_allocation_matrix['motor_zbr']['roll'] + local_y_torque_man * C_PITCH * self.control_allocation_matrix['motor_zbr']['pitch']
        self.motor_signals[4] = local_z_torque_man * C_YAW * self.control_allocation_matrix['motor_xyfl']['roll'] 
        self.motor_signals[5] = local_z_torque_man * C_YAW * self.control_allocation_matrix['motor_xyfr']['roll']
        self.motor_signals[6] = local_z_torque_man * C_YAW * self.control_allocation_matrix['motor_xybl']['roll']
        self.motor_signals[7] = local_z_torque_man * C_YAW * self.control_allocation_matrix['motor_xybr']['roll']

        #saturation check
        for x in self.motor_signals:
            if x > MOTOR_SATURATION:
                x = MOTOR_SATURATION
                print("Pos. Motor saturation reached")
            elif x < -MOTOR_SATURATION:
                x = -MOTOR_SATURATION
                print("Neg. Motor saturation reached")
       
        print("motor signals: ", self.motor_signals)

        self.motor_zfl_control_allocation.set_thrust(self.motor_signals[0])
        self.motor_zfr_control_allocation.set_thrust(self.motor_signals[1])
        self.motor_zbl_control_allocation.set_thrust(self.motor_signals[2])
        self.motor_zbr_control_allocation.set_thrust(self.motor_signals[3])
        self.motor_xyfl_control_allocation.set_thrust(self.motor_signals[4])
        self.motor_xyfr_control_allocation.set_thrust(self.motor_signals[5])
        self.motor_xybl_control_allocation.set_thrust(self.motor_signals[6])
        self.motor_xybr_control_allocation.set_thrust(self.motor_signals[7])

    def update_motor_thrusts_zero(self):
        self.motor_zfl_control_allocation.set_thrust(0)
        self.motor_zfr_control_allocation.set_thrust(0)
        self.motor_zbl_control_allocation.set_thrust(0)
        self.motor_zbr_control_allocation.set_thrust(0)
        self.motor_xyfl_control_allocation.set_thrust(0)
        self.motor_xyfr_control_allocation.set_thrust(0)
        self.motor_xybl_control_allocation.set_thrust(0)
        self.motor_xybr_control_allocation.set_thrust(0)



        

            
        
        