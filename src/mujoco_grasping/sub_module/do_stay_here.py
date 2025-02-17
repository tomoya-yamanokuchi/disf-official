import numpy as np
from robot_env.instance import PandaArmEnv
from robot_env.utils import MyViewerWrapper
from print_color import print


def do_stay_here(
        env      : PandaArmEnv,
        viewer   : MyViewerWrapper,
        stay_step: int,
    ):
    # while True:
    for i in range(stay_step):
        env.start_step()
        # env.solve_ik()
        env.step()
        env.wait_step()
        viewer.sync()
    # ----
    print(f"stay finish!",
            tag = "Grasp", color="c", tag_color="c")
