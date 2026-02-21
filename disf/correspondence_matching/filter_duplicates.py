import numpy as np
from disf.value_object import PointNormalUnitPairs, PointNormalIndexUnitPairs
from disf.value_object import CorrespondenceFilteredResult
from disf.value_object import TargetPointNormalIndexPairs


def filter_duplicates(
        source: PointNormalIndexUnitPairs,
        target_correspondences: PointNormalUnitPairs,
        target_correspondences_indices: np.ndarray,
) -> CorrespondenceFilteredResult:
    """
    重複したインデックスを検出し、1つだけを残してフィルタリングを行う関数。
    対応点の元のインデックス空間における位置も返します。
    """
    # インデックスのユニークな値とその最初の出現位置を取得
    unique_corres_indices_val, unique_corres_indices_first_position = np.unique(
        target_correspondences_indices, return_index=True
    )

    # フィルタリング用のマスクを作成
    valid_mask = np.zeros_like(target_correspondences_indices, dtype=bool)
    valid_mask[unique_corres_indices_first_position] = True

    # filtering: target
    filtered_target = TargetPointNormalIndexPairs(
        points=target_correspondences.points[valid_mask],
        normals=target_correspondences.normals[valid_mask],
        indices=target_correspondences_indices[valid_mask],
    )
    # filtering: source
    filtered_source = PointNormalIndexUnitPairs(
        points=source.points[valid_mask],
        normals=source.normals[valid_mask],
        finger_indices=source.finger_indices[valid_mask],
    )

    # import ipdb; ipdb.set_trace()
    return CorrespondenceFilteredResult(
        filtered_target=filtered_target,
        filtered_source=filtered_source,
        # unique_corres_indices_first_position = unique_corres_indices_first_position
    )
