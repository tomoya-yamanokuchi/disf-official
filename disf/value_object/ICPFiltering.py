from typing import TypedDict, NamedTuple
import numpy as np
from disf.value_object import PointNormalUnitPairs
from disf.value_object import PointNormalIndexUnitPairs


class ICPFiltering(NamedTuple):
    selected_indices: np.ndarray
    distances: np.ndarray
    # ---
    source: PointNormalIndexUnitPairs
    target: PointNormalUnitPairs
