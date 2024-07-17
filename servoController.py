from library import *
from globals import *

class ServoController:
    def __init__(self, num_servo_channels=16, nbServos=12):
        self.nbServos = nbServos
        self.kit = ServoKit(channels=num_servo_channels)

    def init_servo(self, servo_num):
        print("init servo")
        self.kit.servo[servo_num].angle = 90
        sleep(1)

    def move_servo(self, servo_num, angle):
        self.kit.servo[servo_num].angle = angle

    def check_servo(self, servo_num):
        for angle in range(0, 181, 10):
            self.move_servo(servo_num, angle)
            sleep(1)
        for angle in range(180, -1, -10):
            self.move_servo(servo_num, angle)
            sleep(1)
    
    def nonaktif_servo(self):
        for i in range(self.nbServos):
            self.kit.servo[i].angle = None

    def center(self):
        print("\nKuro Center\n")
        # RF - Right Front
        self.move_servo(0, setpoint_kaki[0][0])
        self.move_servo(1, setpoint_kaki[0][1])
        self.move_servo(2, setpoint_kaki[0][2])

        # RH - Right Hind
        self.move_servo(3, setpoint_kaki[1][0])
        self.move_servo(4, setpoint_kaki[1][1])
        self.move_servo(5, setpoint_kaki[1][2])

        # LH - Left Hind
        self.move_servo(6, setpoint_kaki[2][0])
        self.move_servo(7, setpoint_kaki[2][1])
        self.move_servo(8, setpoint_kaki[2][2])
        
        # LF - Left Front
        self.move_servo(9, setpoint_kaki[3][0])
        self.move_servo(10, setpoint_kaki[3][1])
        self.move_servo(11, setpoint_kaki[3][2])

    def transmit_pulsa(self, delay, leg_pos, adjust_rf, adjust_rh, adjust_lh, adjust_lf):
        # Leg 1
        self.move_servo(0, leg_pos[0][0])
        self.move_servo(1, leg_pos[0][1] + adjust_rf)
        self.move_servo(2, leg_pos[0][2])
        
        # Leg 2
        self.move_servo(3, leg_pos[1][0])
        self.move_servo(4, leg_pos[1][1] + adjust_rh)
        self.move_servo(5, leg_pos[1][2])
        
        # Leg 3
        self.move_servo(6, leg_pos[2][0])
        self.move_servo(7, leg_pos[2][1] + adjust_lh)
        self.move_servo(8, leg_pos[2][2])
        
        # Leg 4
        self.move_servo(9, leg_pos[3][0])
        self.move_servo(10, leg_pos[3][1] + adjust_lf)
        self.move_servo(11, leg_pos[3][2])
        sleep(delay)
        
        print(f"[{leg_pos[0][1] + adjust_rf}, {leg_pos[1][1] + adjust_rh}, {leg_pos[2][1] + adjust_lh}, {leg_pos[3][1] + adjust_lf}]")
        
    def leg_debug(self, leg_num):
        print(f"[leg {leg_num+1}: {leg_pos[leg_num][0]}, {leg_pos[leg_num][1]}, {leg_pos[leg_num][2]}]")
        
