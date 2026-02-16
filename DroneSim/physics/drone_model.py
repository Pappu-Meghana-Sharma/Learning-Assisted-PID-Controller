import pybullet as p
from config import MASS

def create_drone():
    collision = p.createCollisionShape(p.GEOM_BOX,
                                       halfExtents=[0.1,0.1,0.03])
    visual = p.createVisualShape(p.GEOM_BOX,
                                 halfExtents=[0.1,0.1,0.03],
                                 rgbaColor=[0,0,1,1])

    drone = p.createMultiBody(
        baseMass=MASS,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0,0,1]
    )
    return drone
