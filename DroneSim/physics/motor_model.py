from config import B_THRUST, D_DRAG

def thrust(omega):
    return B_THRUST * omega**2

def drag_torque(omega):
    return D_DRAG * omega**2
