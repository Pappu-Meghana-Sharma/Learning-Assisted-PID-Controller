import time
import numpy as np
import pybullet as p

from config import *
from physics.world import init_world
from physics.drone_model import create_drone
from physics.motor_model import thrust, drag_torque
from sensors.imu import read_imu
from control.outer_loop import OuterLoop
from control.inner_loop import InnerLoop
from control.mixer import mix
from utils.logger import Logger
from estimator import SimpleKalman

# =========================
# INITIALIZATION
# =========================
init_world()
drone = create_drone()

outer_loop = OuterLoop()
inner_loop = InnerLoop()
logger = Logger()
kf = SimpleKalman()

# Desired setpoints
desired = {
    "z": 2.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
}

# Motor geometry (X configuration)
motor_positions = [
    [ ARM_LENGTH,  0, 0],   # Front
    [ 0,  ARM_LENGTH, 0],   # Right
    [-ARM_LENGTH,  0, 0],   # Back
    [ 0, -ARM_LENGTH, 0]    # Left
]

# =========================
# SIMULATION LOOP
# =========================
while True:

    dt = DT

    # -------- Read Sensors --------
    state = read_imu(drone)

    # -------- Kalman Filtering --------
    filtered_angles = kf.update(state["angles"])
    state["angles"] = filtered_angles

    # -------- Altitude Control --------
    z = state["pos"][2]
    alt_error = desired["z"] - z
    thrust_total = MASS * GRAVITY + ALT_KP * alt_error

    # -------- Outer Loop (Angle -> Rate) --------
    desired_rates = outer_loop.update(
        state["angles"],
        (desired["roll"], desired["pitch"], desired["yaw"]),
        dt
    )

    # -------- Inner Loop (Rate -> Torque) --------
    tau_roll, tau_pitch, tau_yaw = inner_loop.update(
        state["rates"],
        desired_rates,
        dt
    )

    # -------- Motor Mixing --------
    motor_forces = mix(thrust_total, tau_roll, tau_pitch, tau_yaw)

    # Convert Force -> Motor Speed
    omegas = [
        np.sqrt(max(f, 0) / B_THRUST)
        for f in motor_forces
    ]

    # =========================
    # APPLY MOTOR FORCES
    # =========================
    for i in range(4):

        F = thrust(omegas[i])

        p.applyExternalForce(
            drone,
            -1,
            [0, 0, F],
            motor_positions[i],
            p.LINK_FRAME
        )

    # =========================
    # APPLY YAW DRAG TORQUE
    # =========================
    yaw_drag = (
        drag_torque(omegas[0])
        - drag_torque(omegas[1])
        + drag_torque(omegas[2])
        - drag_torque(omegas[3])
    )

    p.applyExternalTorque(
        drone,
        -1,
        [0, 0, yaw_drag],
        p.LINK_FRAME
    )

    # =========================
    # AERODYNAMIC DRAG
    # =========================
    lin_vel = state["vel"]
    ang_vel = state["rates"]

    drag_force = [-LIN_DRAG*v for v in lin_vel]
    drag_torque_vec = [-ANG_DRAG*w for w in ang_vel]

    p.applyExternalForce(
        drone,
        -1,
        drag_force,
        [0,0,0],
        p.LINK_FRAME
    )

    p.applyExternalTorque(
        drone,
        -1,
        drag_torque_vec,
        p.LINK_FRAME
    )

    # =========================
    # WIND DISTURBANCE
    # =========================
    p.applyExternalForce(
        drone,
        -1,
        WIND_FORCE,
        [0,0,0],
        p.WORLD_FRAME
    )

    # =========================
    # CAMERA FOLLOW
    # =========================
    pos = state["pos"]
    p.resetDebugVisualizerCamera(
        cameraDistance=4,
        cameraYaw=60,
        cameraPitch=-25,
        cameraTargetPosition=pos
    )

    # =========================
    # LOG DATA
    # =========================
    logger.log(state, desired)

    print("Altitude:", round(pos[2], 2))

    p.stepSimulation()
    time.sleep(dt)
