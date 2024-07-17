from library import *
from globals import *
from servoController import *
from fuzzyControl import *
from legKinematics import *

class Gait:
    def __init__(self):
        self.step_x = 0
        self.step_y = 5
        self.step_z = 5
        self.fuzzy_control = FuzzyControl()
        self.servo_controller = ServoController()
        self.leg_kinematics = LegKinematics()
    
    def trot(self):
        self.step_1_trot(0, 0, 0, 0)
        self.step_2_trot(0, 0, 0, 0)
        self.step_3_trot(0, 0, 0, 0)
        self.step_4_trot(0, 0, 0, 0)
        self.step_5_trot(0, 0, 0, 0)
        self.step_6_trot(0, 0, 0, 0)
    
    def trot_fuzzy(self, yaw_value):
        adjustment = self.fuzzy_control.apply_adjustment(yaw_value)
        adjustment_trot = self.get_adjustment_trot(adjustment)
        
        self.step_1_trot(adjustment_trot[0][0], adjustment_trot[0][1], adjustment_trot[0][2], adjustment_trot[0][3])
        self.step_2_trot(adjustment_trot[1][0], adjustment_trot[1][1], adjustment_trot[1][2], adjustment_trot[1][3])
        self.step_3_trot(adjustment_trot[2][0], adjustment_trot[2][1], adjustment_trot[2][2], adjustment_trot[2][3])
        self.step_4_trot(adjustment_trot[3][0], adjustment_trot[3][1], adjustment_trot[3][2], adjustment_trot[3][3])
        self.step_5_trot(adjustment_trot[4][0], adjustment_trot[4][1], adjustment_trot[4][2], adjustment_trot[4][3])
        self.step_6_trot(adjustment_trot[5][0], adjustment_trot[5][1], adjustment_trot[5][2], adjustment_trot[5][3])

    def get_adjustment_trot(self, adjustment):
        if adjustment < 0:
            return [
                (0, 0, 0, 0),
                (0, adjustment, adjustment, 0),
                (0, adjustment, adjustment, 0),
                (0, 0, 0, 0),
                (adjustment, 0, 0, adjustment),
                (adjustment, 0, 0, adjustment)
            ]
        elif adjustment > 0:
            return [
                (0, 0, 0, 0),
                (adjustment, 0, 0, adjustment),
                (adjustment, 0, 0, adjustment),
                (0, 0, 0, 0),
                (0, adjustment, adjustment, 0),
                (0, adjustment, adjustment, 0)
            ]
        else:
            return [
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0)
            ]
        
    def step_1_trot(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        self.leg_kinematics.inverse_kinematics(0, 0, 0, self.step_z)
        self.leg_kinematics.inverse_kinematics(1, 0, 0, -self.step_z)
        self.leg_kinematics.inverse_kinematics(2, 0, 0, self.step_z)
        self.leg_kinematics.inverse_kinematics(3, 0, 0, -self.step_z)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_2_trot(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        self.leg_kinematics.inverse_kinematics(0, self.step_x, self.step_y, self.step_z)
        self.leg_kinematics.inverse_kinematics(1, -self.step_x, -self.step_y, -self.step_z)
        self.leg_kinematics.inverse_kinematics(2, self.step_x, self.step_y, self.step_z)
        self.leg_kinematics.inverse_kinematics(3, -self.step_x, -self.step_y, -self.step_z)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_3_trot(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        self.leg_kinematics.inverse_kinematics(0, self.step_x, self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(1, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(2, self.step_x, self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(3, -self.step_x, -self.step_y, 0)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_4_trot(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        self.leg_kinematics.inverse_kinematics(0, 0, 0, -self.step_z)
        self.leg_kinematics.inverse_kinematics(1, 0, 0, self.step_z)
        self.leg_kinematics.inverse_kinematics(2, 0, 0, -self.step_z)
        self.leg_kinematics.inverse_kinematics(3, 0, 0, self.step_z)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_5_trot(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):        
        self.leg_kinematics.inverse_kinematics(0, -self.step_x, -self.step_y, -self.step_z)
        self.leg_kinematics.inverse_kinematics(1, self.step_x, self.step_y, self.step_z)
        self.leg_kinematics.inverse_kinematics(2, -self.step_x, -self.step_y, -self.step_z)
        self.leg_kinematics.inverse_kinematics(3, self.step_x, self.step_y, self.step_z)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)
    
    def step_6_trot(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):        
        self.leg_kinematics.inverse_kinematics(0, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(1, self.step_x, self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(2, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(3, self.step_x, self.step_y, 0)
        self.servo_controller.transmit_pulsa(0.1, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def walk(self):
        self.step_1_walk(0, 0, 0, 0)
        self.step_2_walk(0, 0, 0, 0)
        self.step_3_walk(0, 0, 0, 0)
        self.step_4_walk(0, 0, 0, 0)
        self.step_5_walk(0, 0, 0, 0)
        self.step_6_walk(0, 0, 0, 0)
        self.step_7_walk(0, 0, 0, 0)
        self.step_8_walk(0, 0, 0, 0)
    
    def walk_fuzzy(self, yaw_value):
        adjustment = self.fuzzy_control.apply_adjustment(yaw_value)
        adjustment_walk = self.get_adjustment_walk(adjustment)

        self.step_1_walk(adjustment_walk[0][0], adjustment_walk[0][1], adjustment_walk[0][2], adjustment_walk[0][3])
        self.step_2_walk(adjustment_walk[1][0], adjustment_walk[1][1], adjustment_walk[1][2], adjustment_walk[1][3])
        self.step_3_walk(adjustment_walk[2][0], adjustment_walk[2][1], adjustment_walk[2][2], adjustment_walk[2][3])
        self.step_4_walk(adjustment_walk[3][0], adjustment_walk[3][1], adjustment_walk[3][2], adjustment_walk[3][3])
        self.step_5_walk(adjustment_walk[4][0], adjustment_walk[4][1], adjustment_walk[4][2], adjustment_walk[4][3])
        self.step_6_walk(adjustment_walk[5][0], adjustment_walk[5][1], adjustment_walk[5][2], adjustment_walk[5][3])
        self.step_7_walk(adjustment_walk[6][0], adjustment_walk[6][1], adjustment_walk[6][2], adjustment_walk[6][3])
        self.step_8_walk(adjustment_walk[7][0], adjustment_walk[7][1], adjustment_walk[7][2], adjustment_walk[7][3])
    
    def get_adjustment_walk(self, adjustment):
        if adjustment < 0:
            return [
                (adjustment, 0, 0, adjustment),
                (adjustment, 0, 0, adjustment),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, adjustment, adjustment, 0),
                (0, adjustment, adjustment, 0),
                (0, adjustment, 0, adjustment),
                (0, adjustment, 0, adjustment)
            ]
        elif adjustment > 0:
            return [
                (0, adjustment, adjustment, 0),
                (0, adjustment, adjustment, 0),
                (adjustment, 0, adjustment, 0),
                (adjustment, 0, adjustment, 0),
                (adjustment, 0, 0, adjustment),
                (adjustment, 0, 0, adjustment),
                (0, 0, 0, 0),
                (0, 0, 0, 0)
            ]
        else:
            return [
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                (0, 0, 0, 0)
            ]

    def step_1_walk(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        self.leg_kinematics.inverse_kinematics(0, -self.step_x, -self.step_y, -self.step_z)
        self.leg_kinematics.inverse_kinematics(1, self.step_x, self.step_y, self.step_z)
        self.leg_kinematics.inverse_kinematics(2, -self.step_x, -self.step_y, -self.step_z)
        self.leg_kinematics.inverse_kinematics(3, self.step_x, self.step_y, 0)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_2_walk(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        self.leg_kinematics.inverse_kinematics(0, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(1, self.step_x, self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(2, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(3, self.step_x, self.step_y, 0)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_3_walk(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        self.leg_kinematics.inverse_kinematics(0, self.step_x, self.step_y, self.step_z)
        self.leg_kinematics.inverse_kinematics(1, 0, 0, -self.step_z)
        self.leg_kinematics.inverse_kinematics(2, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(3, 0, 0, -self.step_z)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_4_walk(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        self.leg_kinematics.inverse_kinematics(0, self.step_x, self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(1, 0, 0, 0)
        self.leg_kinematics.inverse_kinematics(2, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(3, 0, 0, 0)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_5_walk(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):        
        self.leg_kinematics.inverse_kinematics(0, self.step_x, self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(1, -self.step_x, -self.step_y, -self.step_z)
        self.leg_kinematics.inverse_kinematics(2, self.step_x, self.step_y, self.step_z)
        self.leg_kinematics.inverse_kinematics(3, -self.step_x, -self.step_y, -self.step_z)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)
    
    def step_6_walk(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):        
        self.leg_kinematics.inverse_kinematics(0, self.step_x, self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(1, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(2, self.step_x, self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(3, -self.step_x, -self.step_y, 0)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_7_walk(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):        
        self.leg_kinematics.inverse_kinematics(0, 0, 0, -self.step_z)
        self.leg_kinematics.inverse_kinematics(1, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(2, 0, 0, -self.step_z)
        self.leg_kinematics.inverse_kinematics(3, self.step_x, self.step_y, self.step_z)
        self.servo_controller.transmit_pulsa(0.2, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

    def step_8_walk(self, adjust_rf, adjust_rh, adjust_lh, adjust_lf):        
        self.leg_kinematics.inverse_kinematics(0, 0, 0, 0)
        self.leg_kinematics.inverse_kinematics(1, -self.step_x, -self.step_y, 0)
        self.leg_kinematics.inverse_kinematics(2, 0, 0, 0)
        self.leg_kinematics.inverse_kinematics(3, self.step_x, self.step_y, 0)
        self.servo_controller.transmit_pulsa(0.1, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf)

