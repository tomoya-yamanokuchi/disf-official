from domain_object.builder import SelfContainedDomainObjectBuilder


class YCB_Object_ContactEstimation_Director:
    @staticmethod
    def construct(builder: SelfContainedDomainObjectBuilder):
        # ---
        builder.build_config_ipfo(ipfo_config_name="refined_ipfo")
        # =========== datatset generation ===========
        # --- finger ---
        builder.build_right_finger_source()
        # --- object ---
        builder.build_config_point_cloud_data()
        builder.build_ycb_data_point_cloud()
        builder.build_ycb_data_normal_estimation()
        builder.build_ycb_data_PointNormal_ValueObject()
        builder.build_config_contact_estimation()
        builder.build_ycb_target_by_contact_estimation_with_hand_plane()
        # builder.build_approach_direction()
        # --- dataset ---
        builder.build_point_cloud_dataset()
        # ---
        return builder.get_domain_object()

