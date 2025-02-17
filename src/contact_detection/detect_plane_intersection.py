import numpy as np


def detect_plane_intersection(
        point_cloud,
        hand_plane_origin,
        hand_plane_normal,
        d1,
        d2,
    ) -> np.ndarray:
    """
    手平面と交差し、かつ手平面原点に近い点を検出
        :param point_cloud: np.array, 形状 (N, 3) のポイントクラウド
        :param hand_plane_origin: tuple, 手平面の原点 (x0, y0, z0)
        :param hand_plane_normal: tuple, 手平面の法線 (A, B, C)
        :param d1: float, 手平面からの許容距離
        :param d2: float, 手平面原点からの許容距離
        :return: np.array, 手平面と交差し、原点に近い点群とそのインデックス
    """
    # 手平面の法線
    A, B, C = hand_plane_normal
    x0, y0, z0 = hand_plane_origin

    # 手平面の方程式 D = -(A*x0 + B*y0 + C*z0)
    D = -(A * x0 + B * y0 + C * z0)

    # 各点の平面からの距離を計算
    distances = np.abs(A * point_cloud[:, 0] + B * point_cloud[:, 1] + C * point_cloud[:, 2] + D) / np.sqrt(A**2 + B**2 + C**2)

    # 許容距離内の点のインデックスを選択（平面との交差）
    intersecting_indices = np.where(distances <= d1)[0]

    # 該当する点を抽出
    intersecting_points = point_cloud[intersecting_indices]

    # 手平面原点からの距離を計算
    origin_distances = np.linalg.norm(intersecting_points - np.array(hand_plane_origin), axis=1)

    # import ipdb; ipdb.set_trace()
    # 原点からの許容距離内の点のインデックスを選択
    filtered_indices = intersecting_indices[origin_distances <= d2]

    return filtered_indices
