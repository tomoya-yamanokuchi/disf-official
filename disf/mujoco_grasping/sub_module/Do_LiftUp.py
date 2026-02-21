import os
import numpy as np
from print_color import print
from disf.domain_object.builder import DomainObject
from disf.robot_env.instance import PandaArmEnv
from disf.robot_env.utils import MyViewerWrapper
from .TimeWatch import TimeWatch


class Do_LiftUp:
    def __init__(self, domain_object: DomainObject):
        self.env: PandaArmEnv = domain_object.env
        self.time_watch = TimeWatch(domain_object)

    def execute(self,
                viewer: MyViewerWrapper,
                lift_up_position: np.ndarray,
                pre_grasp_quat: np.ndarray,
                ):
        self.env.initialize_ik_solver()
        self.env.ik_solver.update_target_pose(
            target_pos=lift_up_position,
            target_quat=pre_grasp_quat,
        )
        self.env.ik_solver.set_reaching_threshold_lift_up()
        self.env.ik_solver.reset_ik_reach_flag()
        # ---
        self.time_watch.start()
        # ---------
        while not self.env.ik_solver.reached:
            self.env.start_step()
            self.env.solve_ik()
            self.env.step()
            self.env.wait_step()
            viewer.sync()
            # -----
            if not self.time_watch.check_continue():
                break
            # import ipdb; ipdb.set_trace()
        # ---------
        if self.env.ik_solver.reached:
            print(f"reached at lift up pose!",
                  tag="Grasp", color="c", tag_color="c")
        # ---------
        # import ipdb; ipdb.set_trace()
