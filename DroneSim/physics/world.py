import pybullet as p
import pybullet_data
from config import GRAVITY

def init_world():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -GRAVITY)
    p.loadURDF("plane.urdf")
