from domain_object.builder import SelfContainedDomainObjectBuilder


class PFOCommonDirector:
    @staticmethod
    def construct(
            builder            : SelfContainedDomainObjectBuilder,
            isf_config_name    : str,
            pc_data_config_name: str,
        ):
        # ---------
        builder.build_config_isf(config_name=isf_config_name)
        builder.build_ipfo_parameters()
        builder.build_ipfo_text_logger()
        builder.build_config_point_cloud_data(pc_data_config_name)
        builder.build_loop_criteria()
        # ---------
        return builder.get_domain_object()

