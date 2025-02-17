from domain_object.builder import SelfContainedDomainObjectBuilder
from .MujocoArmEnvDirector import MujocoArmEnvDirector
from ..isf_planning import ISF_Planning_With_YCB_Object_Director

class MujocoGrasping_with_ISF_Planning:
    @staticmethod
    def construct(
            builder        : SelfContainedDomainObjectBuilder,
            env_config_name: str = "panda_simple",
            object_name    : str = None,
            isf_model      : str = None,
        ):
        builder = ISF_Planning_With_YCB_Object_Director.construct(builder, object_name, isf_model)
        builder = MujocoArmEnvDirector.construct(builder, env_config_name)
        builder.save_configs()
        # ---
        return builder.get_domain_object()

