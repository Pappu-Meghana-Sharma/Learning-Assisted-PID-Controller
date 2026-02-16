from config import ARM_LENGTH

def mix(thrust_total, tau_roll, tau_pitch, tau_yaw):

    L = ARM_LENGTH

    F1 = thrust_total/4 - tau_roll/(2*L) - tau_yaw/4
    F2 = thrust_total/4 - tau_pitch/(2*L) + tau_yaw/4
    F3 = thrust_total/4 + tau_roll/(2*L) - tau_yaw/4
    F4 = thrust_total/4 + tau_pitch/(2*L) + tau_yaw/4

    return [F1, F2, F3, F4]
