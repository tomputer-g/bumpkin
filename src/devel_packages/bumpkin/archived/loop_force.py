from frankapy import FrankaArm
import numpy as np

fa = FrankaArm(with_gripper=False)

while True:
    sensed_ft = fa.get_ee_force_torque()
    print("Sensed force: ", sensed_ft)
    print("Sensed norm: ", np.linalg.norm(sensed_ft))
    # if np.linalg.norm(sensed_ft) > 2.0:
    #     print("Bump detected")
    #     self.goal_msg = None
    # else:
    #     print("No bump detected")