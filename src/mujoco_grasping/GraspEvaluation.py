import os
import mujoco
import numpy as np
from domain_object.builder import DomainObject
from robot_env.instance import PandaArmEnv
from service import is_within_orientation_threshold
from print_color import print


class GraspEvaluation:
    def __init__(self, domain_object: DomainObject):
        self.geom = domain_object.geom
        self.env : PandaArmEnv   = domain_object.env
        self.grasp_pos_threshold = domain_object.config_grasp_evaluation.pos_threshold
        self.grasp_ori_threshold = np.deg2rad(domain_object.config_grasp_evaluation.ori_threshold_deg)
        # ----
        self.offset_for_mujoco   = domain_object.offset_for_mujoco
        self.z_direction_world   = domain_object.z_direction_world
        self.lift_up_height      = domain_object.lift_up_height
        # ----
        self.results_save_dir    = domain_object.results_save_dir
        # ----
        self.set_target_pos()
        self.set_target_quat()
        # import ipdb; ipdb.set_trace()


    def set_target_pos(self):
        id_table_geom    = self.geom.name2id("table")
        table_geom_size  = self.env.model.geom_size[id_table_geom]
        table_geom_hight = (2 * table_geom_size[2])
        init_object_pos  = (self.z_direction_world * table_geom_hight) + self.offset_for_mujoco
        # ----
        add_lift_up_pos  = (self.lift_up_height * self.z_direction_world)
        self.target_pos  = init_object_pos + add_lift_up_pos

    def set_target_quat(self):
        self.target_quat = np.array([1, 0, 0, 0])

    def evaluate(self, save: bool=False):
        # ----
        current_pos  = self.env.data.body(self.env.id_body_object).xpos
        current_quat = self.env.data.body(self.env.id_body_object).xquat
        # -----
        error_pos   = (self.target_pos - current_pos)
        # ---------------- Orientation error --------------------
        current_quat_conj = np.zeros(4)
        error_quat        = np.zeros(4)
        mujoco.mju_negQuat(current_quat_conj, current_quat) # Conjugate quaternion (opposite same rotation)
        mujoco.mju_mulQuat(error_quat, self.target_quat, current_quat_conj)
        # --------------------------------------------------------------------
        grasped_position = (np.linalg.norm(error_pos)  < self.grasp_pos_threshold)
        grasped_quat     = is_within_orientation_threshold(error_quat, self.grasp_ori_threshold)
        grasped          = bool(grasped_position and grasped_quat)
        print("grasped =", grasped, tag="eval", color="yellow", tag_color="yellow")
        # import ipdb; ipdb.set_trace()
        # --------------
        if save:
            np.save(
                file = os.path.join(self.results_save_dir, "grasp_success.npy"),
                arr  = grasped
            )
        # --------------
        return grasped
