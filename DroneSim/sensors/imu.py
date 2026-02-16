import numpy as np
import pybullet as p

def read_imu(drone):

    pos, orn = p.getBasePositionAndOrientation(drone)
    lin_vel, ang_vel = p.getBaseVelocity(drone)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)

    noise_angle = np.random.normal(0, 0.002, 3)
    noise_rate = np.random.normal(0, 0.01, 3)

    return {
        "pos": pos,
        "vel": lin_vel,
        "angles": (
            roll + noise_angle[0],
            pitch + noise_angle[1],
            yaw + noise_angle[2]
        ),
        "rates": (
            ang_vel[0] + noise_rate[0],
            ang_vel[1] + noise_rate[1],
            ang_vel[2] + noise_rate[2]
        )
    }
