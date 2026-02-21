from typing import NamedTuple
from disf.value_object import TargetPointNormalIndexPairs
from disf.value_object import PointNormalIndexUnitPairs


class ICPResult(NamedTuple):
    source: PointNormalIndexUnitPairs
    target: TargetPointNormalIndexPairs
    num_correspondences: int
