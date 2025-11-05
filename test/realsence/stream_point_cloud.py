import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt


# ====== RealSense パイプライン準備 ======
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# 深度をカラーに整列
align = rs.align(rs.stream.color)

# オートエクスポージャ等が落ち着くまで数フレーム捨てる
for _ in range(5):
    pipeline.wait_for_frames()

# 1フレーム取得
frames = pipeline.wait_for_frames()
aligned = align.process(frames)
depth = aligned.get_depth_frame()
color = aligned.get_color_frame()

assert depth and color

# ====== PointCloud を生成 ======
pc = rs.pointcloud()
pc.map_to(color)             # 点群にカラーを貼るためのマッピング
points = pc.calculate(depth) # 深度を3D点群へ投影

# 頂点（メートル単位）とテクスチャ座標を取り出し
v = np.asanyarray(points.get_vertices())   # (N,) の rs.vertex 配列
verts = np.vstack([ [p.x, p.y, p.z] for p in v ]).astype(np.float32)

# Z==0（無効深度）を除外
valid = verts[:,2] > 0
verts = verts[valid]

# カラー画像を取り出し（BGR→RGB）
cimg = np.asanyarray(color.get_data())[:, :, ::-1]  # to RGB

# 点群に色をのせる：テクスチャ座標から画素をサンプリング
# （points.get_texture_coordinates() は [0,1] のUV）
uv = np.asanyarray(points.get_texture_coordinates())
uv = np.vstack([ [t.u, t.v] for t in uv ])[valid]
h, w, _ = cimg.shape
uu = np.clip((uv[:,0] * w).astype(int), 0, w-1)
vv = np.clip((uv[:,1] * h).astype(int), 0, h-1)
cols = cimg[vv, uu] / 255.0  # 0-1

pipeline.stop()

# ====== 描画（matplotlib 3D scatter） ======
# 間引き（多すぎると重い）：例えば 100k 点→5k 点に
N = verts.shape[0]
max_points = 8000
if N > max_points:
    idx = np.random.choice(N, max_points, replace=False)
    verts_plot = verts[idx]
    cols_plot = cols[idx]
else:
    verts_plot = verts
    cols_plot = cols

# カメラ座標系：RealSenseはX右+、Y下+、Z前+（メートル）
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

x, y, z = verts_plot[:,0], -verts_plot[:,1], verts_plot[:,2]  # Y反転で見やすく
ax.scatter(x, y, z, s=0.1, c=cols_plot)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('RealSense D435 Point Cloud (matplotlib)')
ax.view_init(elev=20, azim=-60)
plt.tight_layout()
plt.show()
