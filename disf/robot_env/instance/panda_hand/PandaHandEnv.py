from ..DexterousHandEnv import DexterousHandEnv
from disf.domain_object.builder import DomainObject
from disf.value_object import GripperTransformationParams


class PandaHandEnv(DexterousHandEnv):
    def __init__(self, domain_object: DomainObject):
        super().__init__(domain_object)
        self.mocap = domain_object.mocap

    def set_params(self):
        self.__set_ids()

    def __set_ids(self):
        self.id_right_fingertip = self.geom.name2id("right_fingertip_center")
        self.id_left_fingertip = self.geom.name2id("left_fingertip_center")
        self.id_object = self.geom.name2id("object")

    def update_static_mocap(self):
        self.mocap.object_mocap.update()
        self.mocap.fingertip_base_mocap.update()

    def update_transformed_mocap_into_base(self, gripper_transform_params: GripperTransformationParams):
        self.mocap.fingertip_transformed_mocap.update_transform_into_base(gripper_transform_params)
        # self.mocap.object_correspondence_mocap.update() ### ????

    def update_transformed_mocap_into_self(self, gripper_transform_params: GripperTransformationParams):
        self.mocap.fingertip_transformed_mocap.update_transform_into_self(gripper_transform_params)
        self.mocap.object_correspondence_mocap.update_valid_points_and_normals()

    def update_correspondence_mocap(self):
        self.mocap.object_correspondence_mocap.update_correspondece()

    def get_right_fingertip_transformed_points(self):
        return self.mocap.fingertip_transformed_mocap.right.get_points_world()

    def get_left_fingertip_transformed_points(self):
        return self.mocap.fingertip_transformed_mocap.left.get_points_world()

    def get_object_correspondence_right_fingertip_points(self):
        return self.mocap.object_correspondence_mocap.right_corres.get_points_world()

    def get_object_correspondence_left_fingertip_points(self):
        return self.mocap.object_correspondence_mocap.left_corres.get_points_world()

    def get_object_correspondence_right_fingertip_normal(self):
        return self.mocap.object_correspondence_mocap.right_corres.normal_world

    def get_object_correspondence_left_fingertip_normal(self):
        return self.mocap.object_correspondence_mocap.left_corres.normal_world
