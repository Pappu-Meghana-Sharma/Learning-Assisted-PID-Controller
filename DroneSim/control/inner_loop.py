from config import *
from control.pid import PID

class InnerLoop:

    def __init__(self):
        self.roll_pid = PID(RATE_KP, RATE_KI, RATE_KD)
        self.pitch_pid = PID(RATE_KP, RATE_KI, RATE_KD)
        self.yaw_pid = PID(RATE_KP, RATE_KI, RATE_KD)

    def update(self, current_rates, desired_rates, dt):

        p, q, r = current_rates
        d_p, d_q, d_r = desired_rates

        tau_roll = self.roll_pid.compute(d_p, p, dt)
        tau_pitch = self.pitch_pid.compute(d_q, q, dt)
        tau_yaw = self.yaw_pid.compute(d_r, r, dt)

        return tau_roll, tau_pitch, tau_yaw
