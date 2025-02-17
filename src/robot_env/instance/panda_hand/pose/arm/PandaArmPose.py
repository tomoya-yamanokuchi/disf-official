from domain_object.builder import DomainObject
from service import ExtendedRotation
from .PandaArmRotation import PandaArmRotation
from .PandaArmTranslation import PandaArmTranslation
from ...utils import compute_qpos_finger_for_antipodal_gripper
import numpy as np


class PandaArmPose:
    def __init__(self, domain_object: DomainObject):
        self.env         = domain_object.env
        self.d0          = domain_object.d0
        self.d_min       = domain_object.d_min
        self.d_bias      = domain_object.d_bias
        self.rotation    = PandaArmRotation(domain_object)
        self.translation = PandaArmTranslation(domain_object)
        self.j           = 1 # finger index of riht finger

    def set_relational_parameters(self, delta_d: float):
        d_opt = (self.d0 + delta_d)
        self.rotation.set_rotation_palm2finger_static()
        self.translation.set_translation_palm2finger_static(d_opt)

    def compute_qpos_palm_with_keyframe_initialization(self,
            rotation_fingertip_world   : ExtendedRotation,
            translation_fingertip_world: np.ndarray,
        ):
        assert (self.env.initialized_with_keyframe == True)
        # ---
        rotation_palm_world = self.rotation.compute_rotation_finger2palm(rotation_fingertip_world)
        t_palm_world        = self.translation.compute_translation_finger2palm(translation_fingertip_world, rotation_palm_world)
        quat_palm_world     = rotation_palm_world.as_quat_scalar_first()
        qpos_palm           = np.hstack([t_palm_world, quat_palm_world])
        return qpos_palm, rotation_palm_world

    def compute_qpos_finger(self, delta_d: float):
        return compute_qpos_finger_for_antipodal_gripper(
            d0      = self.d0,
            d_min   = self.d_min,
            delta_d = delta_d,
            d_bias  = self.d_bias,
        )
