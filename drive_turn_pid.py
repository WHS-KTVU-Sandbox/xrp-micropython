"""
Original code created by Tin Pham (c/o 2024) in June 2024
Made using Open-STEM/WPI's XRP Libraries
https://github.com/Open-STEM/XRP_MicroPython
"""

import math
import time

from XRPLib.differential_drive import DifferentialDrive
from XRPLib.encoder import Encoder
from XRPLib.encoded_motor import EncodedMotor
from XRPLib.imu import IMU
from XRPLib.imu_defs import *   # LSM_ADDR_PRIMARY
from XRPLib.motor import Motor
from XRPLib.pid import PID

left_motor = Motor(6, 7, True)
left_enc = Encoder(0, 4, 5)
left_enc_mot = EncodedMotor(left_motor, left_enc)

right_motor = Motor(14, 15)
right_enc = Encoder(1, 12, 13)
right_enc_mot = EncodedMotor(right_motor, right_enc)

imu = IMU(19, 18, LSM_ADDR_PRIMARY) # IMU(clock_pin, data_pin, mem_address)
imu.calibrate()
imu.reset_yaw()

differential_drive = DifferentialDrive(left_enc_mot, right_enc_mot, imu)

straight_pid = PID(kp=0.1)
turn_pid = PID(kp=0.047, ki=0.002, kd=0.0005, tolerance=0.1)

wheel_diameter = 6

def move(distance):
    left_enc.reset_encoder_position()
    right_enc.reset_encoder_position()
    while abs(left_enc.get_position()) * math.pi * wheel_diameter < distance:
        left_motor.set_effort(0.5)
        right_motor.set_effort(0.5)
        
def left_turn(degrees):
    while imu.get_yaw() < degrees:
        left_motor.set_effort(-0.5)
        right_motor.set_effort(0.5)
        
def right_turn(degrees):
    while imu.get_yaw() > degrees:
        left_motor.set_effort(0.5)
        right_motor.set_effort(-0.5)
        
def pid_straight(distance):
    left_enc.reset_encoder_position()
    right_enc.reset_encoder_position()
    imu.reset_yaw()
    while ((abs(left_enc.get_position()) + abs(right_enc.get_position())) / 2) * (math.pi * wheel_diameter) < distance:
        print("distance " +  str(((abs(left_enc.get_position()) + abs(right_enc.get_position())) / 2) * (math.pi * wheel_diameter))) 
        time.sleep(0.01)
        error = 0 - imu.get_yaw()
        differential_drive.arcade(0.7, straight_pid.update(error))
    differential_drive.stop()
    imu.reset_yaw()
    
def pid_turn(degrees):
    time.sleep(0.5)
    while not turn_pid.is_done():
        error = degrees - imu.get_yaw()
        print(error)
        time.sleep(0.01)
        differential_drive.arcade(0, turn_pid.update(error))
    differential_drive.stop()
    left_enc.reset_encoder_position()
    imu.reset_yaw()
    turn_pid.clear_history()

pid_straight(60)
pid_turn(-90)
pid_straight(60)
pid_turn(90)
pid_straight(90)
pid_turn(90)
pid_straight(60)
pid_turn(-90)
pid_straight(30)
pid_turn(180)