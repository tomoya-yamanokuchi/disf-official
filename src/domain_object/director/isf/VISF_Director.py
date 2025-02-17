from domain_object.builder import SelfContainedDomainObjectBuilder
from .PFOCommonDirector import PFOCommonDirector



class VISF_Director:
    @staticmethod
    def construct(builder: SelfContainedDomainObjectBuilder, config_name: str):
        # ---
        builder = PFOCommonDirector.construct(
            builder             = builder,
            isf_config_name     = "visf",
            pc_data_config_name = config_name
        )
        # ---
        builder.build_isf_visualizer()
        builder.build_set_error()
        builder.build_isf_loop_criteria()
        # ------------- Fingertip Dataset --------------
        builder.build_paired_finger_indices()
        # --------------- Least Square  ----------------
        builder.build_visf_palm_Rt_EpEnEa_least_square()
        builder.build_visf_fingertip_displacement_least_square()
        # --------------- Optimization ------------------
        builder.build_visf_palm_Rt_opt()
        builder.build_visf_finger_opt()
        # ------------------- IPFO ----------------------
        builder.build_vanilla_isf()
        # -----------------------------------------------
        return builder.get_domain_object()

