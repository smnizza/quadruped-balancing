from time import time, sleep
from keyboard import on_press_key, wait, unhook_all, is_pressed
# initservo
from adafruit_servokit import ServoKit
# inversekinematics
import numpy as np
import math
# fuzzy
import skfuzzy as fuzz
from skfuzzy import control as ctrl
# export data
import csv
# sensor
import serial
