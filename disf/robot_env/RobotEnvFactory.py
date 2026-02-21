from .instance import DexterousManipulatorEnv
from .instance import DexterousHandEnv
# from .instance import PandaHandEnv

from .instance import PandaArmEnv
from .instance import PandaHandSimpleEnv
from disf.domain_object.builder import DomainObject


class RobotEnvFactory:
    @staticmethod
    def create(env_name: str, domain_object: DomainObject):
        if "dexterous_manipulator" in env_name: return DexterousManipulatorEnv(domain_object)
        if "dexterous_hand" in env_name: return DexterousHandEnv(domain_object)
        # if "panda_hand"            in env_name : return PandaHandEnv(domain_object)

        if "panda_arm" in env_name: return PandaArmEnv(domain_object)
        if "panda_simple" in env_name: return PandaHandSimpleEnv(domain_object)
        raise NotImplementedError()
