# Quadruped Robot Stabilization

This project focuses on stabilizing a quadruped robot using fuzzy control based on yaw angle readings from an IMU sensor. The robot has a mammal-like design with three degrees of freedom per leg, enabling movement through inverse kinematics and gait planning.  

## System Analysis  
- The quadruped robot has **three joints per leg**: hip, thigh, and knee.  
- Leg segments include **coxa, femur, and tibia**, each controlled by servo motors.  
- The robot moves using **walk and trot gaits**, generated through gait planning and inverse kinematics.  
- **Yaw stabilization** is achieved via fuzzy control, adjusting servo angles to maintain balance.  
- **Performance criteria** include:  
  1. The robot returns to the set point after deviation.  
  2. Movement remains within predefined limits.  
  3. Fuzzy-controlled movement shows lower peak and total error than uncontrolled movement.  

## Folder Structure  
- **`exportData.py`** → Exports data for analysis in CSV format.  
- **`fuzzyControl.py`** → Implements fuzzy logic for leg movement control.  
- **`gait.py`** → Defines walk and trot gait algorithms.  
- **`globals.py`** → Declares and initializes global variables.  
- **`legKinematics.py`** → Computes inverse kinematics for leg movement.  
- **`library.py`** → Includes necessary libraries.  
- **`main.py`** → Main program execution.  
- **`sensor.py`** → Reads data from the IMU sensor.  
- **`servoController.py`** → Controls servo motors for leg movement.
