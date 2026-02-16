import numpy as np

class SimpleKalman:

    def __init__(self):
        self.x = np.zeros(3)  # roll, pitch, yaw
        self.P = np.eye(3) * 0.1
        self.Q = np.eye(3) * 0.001
        self.R = np.eye(3) * 0.01

    def update(self, measurement):
        z = np.array(measurement)

        # Predict
        self.P += self.Q

        # Update
        K = self.P @ np.linalg.inv(self.P + self.R)
        self.x += K @ (z - self.x)
        self.P = (np.eye(3) - K) @ self.P

        return self.x
