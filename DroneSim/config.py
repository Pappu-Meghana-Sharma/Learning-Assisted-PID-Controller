# Physical constants
MASS = 1.0
GRAVITY = 9.81
ARM_LENGTH = 0.25

# Aerodynamic
LIN_DRAG = 0.1
ANG_DRAG = 0.02

# Wind
WIND_FORCE = [0.0, 0.0, 0.0]  # constant wind in X

# Motor constants
B_THRUST = 1e-5
D_DRAG = 1e-6

# Simulation
DT = 1/240

# Angle PID (outer loop)
ANGLE_KP = 6
ANGLE_KI = 0
ANGLE_KD = 2

# Rate PID (inner loop)
RATE_KP = 0.04
RATE_KI = 0
RATE_KD = 0.001

# Altitude PID
ALT_KP = 3
ALT_KI = 0
ALT_KD = 1
