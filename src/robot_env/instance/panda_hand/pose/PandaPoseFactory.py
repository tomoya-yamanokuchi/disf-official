from .arm import PandaArmPose
from .hand import PandaHandPose
from domain_object.builder import DomainObject


class PandaPoseFactory:
    @staticmethod
    def create(env_name: str,  domain_object: DomainObject):
        # import ipdb; ipdb.set_trace()
        if "arm" in env_name: return PandaArmPose(domain_object)
        else                : return PandaHandPose(domain_object)
