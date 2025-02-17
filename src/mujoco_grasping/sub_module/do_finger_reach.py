from robot_env.instance import PandaArmEnv
from mujoco.viewer import Handle
import numpy as np
from print_color import print


def do_finger_reach(
        env        : PandaArmEnv,
        viewer     : Handle,
        qpos_finger: np.ndarray,
    ):
    # import ipdb; ipdb.set_trace()
    env.set_ctrl_finger(qpos_finger)
    env.reset_finger_reach_flag()
    while not env._finger_grasped:
        env.start_step()
        # ----
        env.check_finger_stop()
        # ----
        env.step()
        env.wait_step()
        viewer.sync()
    # ----
    print(f"grasp finish!",
            tag = "Grasp", color="c", tag_color="c")
