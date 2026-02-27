import numpy as np
import mujoco
import time
from ..ik_solver import LevenbergMarquardt, DampedLeastSquares
from ..utils import Renderer, Camera, MatplotlibViewer, MujocoViewer, BodyManager, MocapManager, GeomManager


class DexterousHandEnv:
    def __init__(self, config):
        self.env_name = "dexterous_hand"
        self.config   = config
        self.dt       = self.config.timestep

    def reset(self, keyframe: int = None):
        self.model              = mujoco.MjModel.from_xml_path(self.config.xml)
        self.model.opt.timestep = self.config.timestep # Override the simulation timestep.
        self.data               = mujoco.MjData(self.model)
        self.body               = BodyManager(self.model, self.data)
        self.geom               = GeomManager(self.model, self.data)
        self.mocap              = MocapManager(self.model, self.data, self.body)
        self.camera             = Camera(self.config, model=self.model)
        self.renderer           = Renderer(self.config, self.model, self.data, self.camera.camera)
        self.matplotlib_viewer  = MatplotlibViewer(self.config)
        self.mujoco_viewer      = MujocoViewer(self.model, self.data, self.config)
        # ---
        self.hand_dof_ids     = np.array([self.model.joint(name).id    for name in self.config.hand_joint_names])
        # ----
        if keyframe is None:
            print("keyframe is None")
            mujoco.mj_resetData(self.model, self.data)
        else:
            print(f"keyframe = {keyframe}")
            mujoco.mj_resetDataKeyframe(self.model, self.data, keyframe)
        # ----
        self.forward()

    def initialize_mujoco_viewer(self, viewer):
        self.mujoco_viewer.initialize(viewer)

    def get_mujoco_viewer_params(self, show_ui: bool = None):
        return self.mujoco_viewer.get_viewer_params(show_ui)

    def forward(self):
        mujoco.mj_forward(self.model, self.data)

    def start_step(self):
        self.step_start_time = time.time()

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def wait_step(self):
        time_until_next_step = self.dt - (time.time() - self.step_start_time)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    def get_object_joint_id(self):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.object_joint_name)

    def get_robot_qpos(self):
        qpos_address = self.model.jnt_qposadr[self.get_object_joint_id()]
        qpos_value   = self.data.qpos[:qpos_address]
        return qpos_value

    def set_qpos(self, qpos: np.ndarray):
        self.data.qpos[:] = qpos

    def set_qpos_hand(self, qpos: np.ndarray):
        assert qpos.shape == (len(self.hand_dof_ids),)
        self.data.qpos[self.hand_dof_ids] = qpos

    def set_ctrl_hand(self, qpos: np.ndarray):
        assert qpos.shape == (len(self.hand_dof_ids),)
        self.data.ctrl[self.hand_dof_ids] = qpos

    def view_matplotlib(self):
        self.renderer.update_scene()
        rgb = self.renderer.render(bgr=False)
        self.matplotlib_viewer.show(rgb=rgb)

    def get_site_id(self, site_name: str):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)

    def set_site_pos(self, site_id: int, pos: np.ndarray):
        # import ipdb ; ipdb.set_trace()
        print(f"site_id = {site_id} : pos = {pos}")
        # self.model.site_pos[site_id][:] = pos
        self.data.site_xpos[site_id] = pos

    def get_site_xpos(self, site_id: int):
        return self.data.site_xpos[site_id]

    def get_site_id2name(self, site_id: int):
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, site_id)