from domain_object.builder import SelfContainedDomainObjectBuilder


class YCBObjectSurfaceDataDirector:
    @staticmethod
    def construct(
            builder    : SelfContainedDomainObjectBuilder,
            object_name: str,
        ):
        # ---
        builder.build_config_point_cloud_data(config_name=object_name)
        builder.build_ycb_xml_scene_file(object_name)
        builder.build_ycb_data_point_cloud()
        builder.build_ycb_data_normal_estimation()
        builder.build_ycb_data_PointNormal_ValueObject()

        # --- load dataset & initial transformation ----
        builder.shift_ycb_whole_object_center_to_origin()
        builder.build_ycb_target_by_contact_estimation_with_hand_plane()
        builder.shift_ycb_target_center_to_origin()
        builder.shift_ycb_whole_object_center_with_contact_info()
        builder.build_point_cloud_dataset()
        # ---
        return builder.get_domain_object()

