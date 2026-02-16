from config import *
from control.pid import PID

class OuterLoop:

    def __init__(self):
        self.roll_pid = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD)
        self.pitch_pid = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD)
        self.yaw_pid = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD)

    def update(self, current_angles, desired_angles, dt):

        roll, pitch, yaw = current_angles
        d_roll, d_pitch, d_yaw = desired_angles

        roll_rate_des = self.roll_pid.compute(d_roll, roll, dt)
        pitch_rate_des = self.pitch_pid.compute(d_pitch, pitch, dt)
        yaw_rate_des = self.yaw_pid.compute(d_yaw, yaw, dt)

        return roll_rate_des, pitch_rate_des, yaw_rate_des
