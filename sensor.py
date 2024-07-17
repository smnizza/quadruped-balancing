from library import *

class Sensor:
    def __init__(self):
        self.setpoint_imu = 999

    def get_yaw(self, tfd_data):
        data = float(tfd_data)

        if self.setpoint_imu == 999:
            self.setpoint_imu = data
                        
        if self.setpoint_imu > 0:
            data = round(data - self.setpoint_imu, 2)
        elif self.setpoint_imu < 0:
            self.setpoint_imu_pos = self.setpoint_imu * -1
            data = round(data + self.setpoint_imu_pos, 2)
        return data


