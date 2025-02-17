from domain_object.builder import DomainObject
from service import ExtendedRotation
import numpy as np
from service import transform_gripper


class PandaHandTranslation:
    def __init__(self, domain_object: DomainObject):
        self.env            = domain_object.env
        self.geom           = domain_object.geom
        self.body           = domain_object.body
        self.model          = domain_object.model
        self.gripper_normal = domain_object.v0
        self.d0             = domain_object.d0
        self.d_min          = domain_object.d_min
        # ---
        self.fingertip_geom_id             = self.geom.name2id(geom_name="right_fingertip_center")
        self.palm_body_id                  = self.body.name2id(body_name="hand")
        self.fingertip_geom_parent_body_id = self.geom.parent_body_id(geom_id=self.fingertip_geom_id)
        self.j = 1

    def _compute_translation_z(self):
        geom_local_pos              = self.geom.pos(geom_id=self.fingertip_geom_id)
        parent_body_local_pos       = self.body.pos(body_id=self.fingertip_geom_parent_body_id)
        translation_geom_local      = (parent_body_local_pos + geom_local_pos) # h
        tz                          = translation_geom_local[-1]
        # import ipdb; ipdb.set_trace()
        return np.array([tz])


    # def set_translation_palm2finger_static(self, d_opt: float):
    #     # -----------------------------
    #     tx = np.array([0.0])
    #     # ---
    #     trans_y_related = transform_gripper(
    #         source_points     = np.zeros(3),
    #         finger_indices    = np.array(self.j),
    #         gripper_normal    = np.array(self.gripper_normal),
    #         rotation_matrix   = np.eye(3),
    #         translation       = np.zeros(3),
    #         delta_d           = d_opt,
    #     )
    #     ty = trans_y_related.squeeze()[1]
    #     # ---
    #     tz = self._compute_translation_z()
    #     # -----------------------------
    #     self.t_palm2finger = np.hstack([tx, ty, tz])


    def set_translation_palm2finger_static(self, d_opt: float):
        """
            without T_rigid (use paired fingers)
        """
        # -----------------------------
        tx = np.array([0.0])
        # ---
        trans_y_related = transform_gripper(
            source_points     = np.zeros(3),
            finger_indices    = np.array(self.j),
            gripper_normal    = np.array(self.gripper_normal),
            rotation_matrix   = np.eye(3),
            translation       = np.zeros(3),
            delta_d           = 0.0,
        )
        ty = trans_y_related.squeeze()[1]
        # ---
        tz = self._compute_translation_z()
        # -----------------------------
        self.t_palm2finger = np.hstack([tx, ty, tz])


    def compute_translation_finger2palm(self,
            t_finger_world      : np.ndarray,
            rotation_palm_world : ExtendedRotation,
        ):
        R_palm_world = rotation_palm_world.as_rodrigues()
        t_palm_world = t_finger_world - np.dot(R_palm_world, self.t_palm2finger)
        # import ipdb ; ipdb.set_trace()
        return t_palm_world
