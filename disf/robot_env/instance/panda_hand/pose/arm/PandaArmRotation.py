from disf.domain_object.builder import DomainObject
from disf.service import ExtendedRotation
import numpy as np
import mujoco


class PandaArmRotation:
    """
        fingertip rotation depends on parent body (not geom itself in the pand hand)
        (off cource, it depends on each individual XML model)
    """

    def __init__(self, domain_object: DomainObject):
        self.geom = domain_object.geom
        self.body = domain_object.body
        self.model = domain_object.model
        self.data = domain_object.data
        # ----
        self.fingertip_geom_id = self.geom.name2id(geom_name="right_fingertip_center")
        self.fingertip_body_id = self.geom.parent_body_id(geom_id=self.fingertip_geom_id)
        self.palm_body_id = self.body.name2id(body_name="hand")
        # ----
        # self.canonical_hand_single_xpos = np.array(domain_object.config_env.hand_pose_offset_info.canonical_hand_single_xpos)
        # self.canonical_hand_single_xquat = np.array(domain_object.config_env.hand_pose_offset_info.canonical_hand_single_xquat)

    def set_rotation_palm2finger_static(self):  # doesn't depends on keyframe
        self.hand_with_arm_xpos = self.body.xpos(self.palm_body_id)
        self.hand_with_arm_xquat = self.body.xquat(self.palm_body_id)
        # ----
        # R_hand          = ExtendedRotation.from_quat(self.canonical_hand_single_xquat).as_matrix()
        self.R_hand_with_arm = ExtendedRotation.from_quat(self.hand_with_arm_xquat).as_matrix()
        # self.delta_R    = (R_hand_with_arm @ R_hand.T)

        # self.R_world_palm_bias = ExtendedRotation.from_quat(self.body.quat(self.palm_body_id)).as_matrix()
        # import ipdb; ipdb.set_trace()

        # self.R_world_palm_bias = ExtendedRotation.from_quat(self.data.body(self.palm_body_id).xquat).as_matrix()

        self.R_palm2finger = np.eye(3)  # 相対的な回転とは...??
        # import ipdb; ipdb.set_trace()

    def compute_rotation_finger2palm(self, rotation_finger_world: ExtendedRotation):
        R_finger_world = rotation_finger_world.as_rodrigues()
        R_palm_world = np.dot(R_finger_world, self.R_palm2finger.T)
        # R_palm_world   = R_finger_world @ (self.R_palm2finger.T @ self.R_hand_with_arm.T)
        # R_palm_world   = R_finger_world @ (self.R_palm2finger.T @ self.delta_R.T)

        # import ipdb ; ipdb.set_trace()
        return ExtendedRotation.from_matrix(R_palm_world)
