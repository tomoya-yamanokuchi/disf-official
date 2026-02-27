from disf.domain_object.builder import SelfContainedDomainObjectBuilder
from ..isf import DISF_Director
from ..isf import VISF_Director
from ..isf import CMA_Director
from ..icp import ICP
from ..isf import PFOCommonDirector
from ..data import GripperSurfaceDataDirector, YCBObjectSurfaceDataDirector


class ISF_Planning_With_YCB_Object_Director:
    @staticmethod
    def construct(
            builder: SelfContainedDomainObjectBuilder,
            object_name: str,
            isf_model: str,
    ):
        # ----------------------------------------

        # =========== datatset generation ===========
        if isf_model == "disf":
            builder = PFOCommonDirector.construct(builder, "disf", object_name)
        elif isf_model == "visf":
            builder = PFOCommonDirector.construct(builder, "visf", object_name)
        elif isf_model == "cma":
            builder = PFOCommonDirector.construct(builder, "cma", object_name)
        else:
            raise NotImplementedError()
        # ---------------------------------------------------
        builder = GripperSurfaceDataDirector.construct(builder)
        builder = YCBObjectSurfaceDataDirector.construct(builder, object_name)
        # ---
        builder = ICP.construct(builder)
        # ----
        builder.build_com_error()
        # =========== datatset generation ===========
        if isf_model == "disf":
            builder = DISF_Director.construct(builder, config_name=object_name)  # assumed to be same
        elif isf_model == "visf":
            builder = VISF_Director.construct(builder, config_name=object_name)  # assumed to be same
        elif isf_model == "cma":
            builder = CMA_Director.construct(builder, config_name=object_name)  # assumed to be same
        else:
            raise NotImplementedError()
        # =============== ISF object ===============
        builder.build_isf_loop_with_single_icp()
        # ---
        return builder.get_domain_object()
