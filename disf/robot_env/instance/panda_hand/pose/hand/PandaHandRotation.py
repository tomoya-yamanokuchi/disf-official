from disf.domain_object.builder import DomainObject
from disf.service import ExtendedRotation
import numpy as np


class PandaHandRotation:
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

    def set_rotation_palm2finger_static(self):  # doesn't depends on keyframe
        # ---
        self.R_palm2finger = np.eye(3)  # 相対的な回転とは...??
        # ---
        self.canonical_hand_single_xpos = self.body.xpos(self.palm_body_id)
        self.canonical_hand_single_xquat = self.body.xquat(self.palm_body_id)
        # import ipdb; ipdb.set_trace()

    def compute_rotation_finger2palm(self, rotation_finger_world: ExtendedRotation):
        R_finger_world = rotation_finger_world.as_rodrigues()
        R_palm_world = np.dot(R_finger_world, self.R_palm2finger.T)
        # import ipdb ; ipdb.set_trace()
        return ExtendedRotation.from_matrix(R_palm_world)
