from library import *
from globals import *
from sensor import*
from gait import *
from servoController import *
from exportData import *

class MainProgram:
    def __init__(self):
        self.sensor = Sensor()
        self.servo_controller = ServoController()
        self.export_data = ExportData()
        self.gait = Gait()

    def run(self):
        ser = serial.Serial('/dev/ttyUSB0', 9600)
        imu_data = []
        total_time = 0
        total_data = 0
        finish_time = 0

        print("Starting...")
        file_name = input("Export data to ")
        sleep(3)
        self.servo_controller.center()
        sleep(2)
        
        program_start = time()
        start_time = time()

        while finish_time <= 30:
            tfd_data = ser.readline().decode('utf-8').strip()
            try:
                data = self.sensor.get_yaw(tfd_data)

                finish_time = time() - program_start
                elapsed_time = time() - start_time

                if elapsed_time >= 0.1:
                    total_time += elapsed_time
                    total_data += 1
                    #print(f"elapsed: {round(elapsed_time, 1)}s, total time: {round(total_time, 1)}s, data: {data}, total data: {total_data}, program: {round(finish_time, 1)}s")
                    print(f"total time: {round(finish_time, 1)}s, data: {data}, total data: {total_data}")
                    imu_data.append([data])
                    
                    #self.gait.trot_fuzzy(data)
                    self.gait.walk_fuzzy(data)
                    
                    #self.gait.trot()
                    #self.gait.walk()
                    
                    start_time = time()
            except (ValueError, IndexError):
                pass

        print("Time has been 30s -> Start exporting...")
        self.export_data.export_to_csv(imu_data, file_name)
        print("Done Exporting!")
        sleep(5)
        self.servo_controller.nonaktif_servo()

def on_key_press(keyboard_event):
    main_program = MainProgram()
    if keyboard_event.name == 'f':
        print("F is pressed...")
        main_program.run()
    elif keyboard_event.name == 'w':
        print("W is pressed")
        exit()

if __name__ == '__main__':
    on_press_key('f', on_key_press)
    on_press_key('w', on_key_press)

    try:
        print("Press F -> Full program\nPress W -> Exit without exporting data")
        wait('ctrl+c')          
    except KeyboardInterrupt:
        pass

