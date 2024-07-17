from library import *
from globals import *
from servoController import *

class LegKinematics:
    def __init__(self):
        self.L1 = 55
        self.L2 = 107.5
        self.L3 = 130
        self.servo_controller = ServoController()

    def limit_coxa(self, leg_num, lc):
        if lc == 0:
            max_coxa = [setpoint_kaki[0][2], setpoint_kaki[1][2], setpoint_kaki[2][2], setpoint_kaki[3][2]]
            if leg_pos[leg_num][2] != max_coxa[leg_num]:
                leg_pos[leg_num][2] = max_coxa[leg_num]
        else:
            max_coxa = [setpoint_kaki[0][2] + lc, setpoint_kaki[1][2] + lc, setpoint_kaki[2][2] + lc, setpoint_kaki[3][2] + lc]
            min_coxa = [setpoint_kaki[0][2] - lc, setpoint_kaki[1][2] - lc, setpoint_kaki[2][2] - lc, setpoint_kaki[3][2] - lc]
            if leg_pos[leg_num][2] > max_coxa[leg_num]:
                leg_pos[leg_num][2] = max_coxa[leg_num]
            elif leg_pos[leg_num][2] < min_coxa[leg_num]:
                leg_pos[leg_num][2] = min_coxa[leg_num]

    def limit_femur(self, leg_num, lf):
        if lf == 0:
            max_femur = [setpoint_kaki[0][1], setpoint_kaki[1][1], setpoint_kaki[2][1], setpoint_kaki[3][1]]
            if leg_pos[leg_num][1] != max_femur[leg_num]:
                leg_pos[leg_num][1] = max_femur[leg_num]
        else:
            max_femur = [setpoint_kaki[0][1] + lf, setpoint_kaki[1][1] + lf, setpoint_kaki[2][1] + lf, setpoint_kaki[3][1] + lf]
            min_femur = [setpoint_kaki[0][1] - lf, setpoint_kaki[1][1] - lf, setpoint_kaki[2][1] - lf, setpoint_kaki[3][1] - lf]
            if leg_pos[leg_num][1] > max_femur[leg_num]:
                leg_pos[leg_num][1] = max_femur[leg_num]
            elif leg_pos[leg_num][1] < min_femur[leg_num]:
                leg_pos[leg_num][1] = min_femur[leg_num]

    def limit_tibia(self, leg_num, lt):
        max_tibia = [setpoint_kaki[0][0] + lt, setpoint_kaki[1][0] + lt, setpoint_kaki[2][0] + lt, setpoint_kaki[3][0] + lt]
        min_tibia = [setpoint_kaki[0][0] - lt, setpoint_kaki[1][0] - lt, setpoint_kaki[2][0] - lt, setpoint_kaki[3][0] - lt]
        
        if leg_pos[leg_num][0] > max_tibia[leg_num]:
            leg_pos[leg_num][0] = max_tibia[leg_num]
        elif leg_pos[leg_num][0] < min_tibia[leg_num]:
            leg_pos[leg_num][0] = min_tibia[leg_num]

    def calculate_angles(self, x, y, z):
        theta1 = np.arcsin(z/self.L1) # untuk tibia
        theta1 = math.degrees(theta1)
        
        r = np.sqrt(x**2+y**2)
        if r == 0:
            theta2 = 0.0
        else:
            theta2 = np.arccos(math.radians((r**2 + self.L2**2 - self.L3**2) / (2 * r * self.L2)))
            theta2 = math.degrees(theta2) # untuk femur
            if y < 0:
                theta2 = -theta2
        
        theta3 = np.arctan2(y, x) # untuk coxa
        theta3 = math.degrees(theta3)
        return theta1, theta2, theta3

    def inverse_kinematics(self, leg_num, x, y, z):    
        theta1, theta2, theta3 = self.calculate_angles(x, y, z)
        # RF - Right Front
        if leg_num == 0:
            leg_pos[leg_num][0] = math.floor(setpoint_kaki[leg_num][0] + theta1)
            leg_pos[leg_num][1] = math.floor(setpoint_kaki[leg_num][1] + theta2)
            leg_pos[leg_num][2] = math.floor(setpoint_kaki[leg_num][2] + theta3)
        # RH - Right Hind
        elif leg_num == 1:
            leg_pos[leg_num][0] = math.floor(setpoint_kaki[leg_num][0] + theta1)
            leg_pos[leg_num][1] = math.floor(setpoint_kaki[leg_num][1] + theta2)
            leg_pos[leg_num][2] = math.floor(setpoint_kaki[leg_num][2] - theta3)
        # LH - Left Hind
        elif leg_num == 2:
            leg_pos[leg_num][0] = math.floor(setpoint_kaki[leg_num][0] - theta1)
            leg_pos[leg_num][1] = math.floor(setpoint_kaki[leg_num][1] - theta2)
            leg_pos[leg_num][2] = math.floor(setpoint_kaki[leg_num][2] + theta3)
            # LF - Left Front
        elif leg_num == 3:
            leg_pos[leg_num][0] = math.floor(setpoint_kaki[leg_num][0] - theta1)
            leg_pos[leg_num][1] = math.floor(setpoint_kaki[leg_num][1] - theta2)
            leg_pos[leg_num][2] = math.floor(setpoint_kaki[leg_num][2] - theta3)
        
        self.limit_coxa(leg_num, lc)
        self.limit_femur(leg_num, lf)
        self.limit_tibia(leg_num, lt)
        #self.servo_controller.leg_debug(leg_num)
            
