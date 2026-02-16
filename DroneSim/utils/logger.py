class Logger:

    def __init__(self):
        self.data = []

    def log(self, state, desired):
        self.data.append({
            "z": state["pos"][2],
            "roll": state["angles"][0],
            "pitch": state["angles"][1],
            "yaw": state["angles"][2],
            "z_des": desired["z"]
        })

    def get_data(self):
        return self.data
