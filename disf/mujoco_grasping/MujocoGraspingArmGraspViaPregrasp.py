import os
from disf.domain_object.builder import DomainObject
from disf.robot_env.instance import PandaArmEnv
from disf.robot_env.instance.panda_hand.pose.arm import PandaArmPose
from disf.robot_env.utils import save_video, save_captured_frame
from .sub_module import do_finger_reach, do_stay_here
from .sub_module import Do_OptimalGraspPoseReaching
from .sub_module import Do_PreGraspPoseReaching
from .sub_module import Do_LiftUp
from disf.value_object import ISFResult


class MujocoGraspingArmGraspViaPregrasp:
    def __init__(self, domain_object: DomainObject):
        self.geom = domain_object.geom
        self.viewer_wrapper = domain_object.viewer_wrapper
        self.env: PandaArmEnv = domain_object.env
        self.pose: PandaArmPose = domain_object.pose
        self.qpos_object = domain_object.qpos_object
        self.d0 = domain_object.d0
        # ----
        self.show_ui = domain_object.show_ui
        self.config_viewer = domain_object.config_env.viewer
        self.config_env = domain_object.config_env
        self.stay_step = domain_object.config_ik_solver.stay_step
        # ----
        self.t_shift_ycb_by_whole = domain_object.t_shift_ycb_by_whole
        self.t_shift_ycb_contact_by_contact = domain_object.t_shift_ycb_contact_by_contact
        self.offset_for_mujoco = domain_object.offset_for_mujoco
        self.n_z0 = domain_object.n_z0
        self.sphere_radius = domain_object.sphere_radius
        self.lift_up_height = domain_object.lift_up_height
        self.z_direction_world = domain_object.z_direction_world
        # ---
        self.model_name = domain_object.model_name
        # ----
        self.results_save_dir = domain_object.results_save_dir
        self.grasp_evaluator = domain_object.grasp_evaluator
        # ----
        self.do_pre_grasp = Do_PreGraspPoseReaching(domain_object)
        self.do_optimal_grasp = Do_OptimalGraspPoseReaching(domain_object)
        self.do_lift_up = Do_LiftUp(domain_object)

    def add_table_height(self, qpos_palm):
        id_table_geom = self.geom.name2id("table")
        table_geom_size = self.env.model.geom_size[id_table_geom]
        table_geom_hight = (2 * table_geom_size[2])
        trans_add_table_hight = (self.z_direction_world * table_geom_hight)
        # import ipdb; ipdb.set_trace()
        return (qpos_palm + trans_add_table_hight)

    def execute(self, isf_result: ISFResult):
        # ----------------------------------------
        rotation = isf_result.rotation
        translation = isf_result.translation
        delta_d = isf_result.delta_d
        # ----------------------------------------
        '''
            new version usecase with trans_center in domain_builder
        '''
        translation = (translation - self.t_shift_ycb_by_whole)
        translation = (translation - self.t_shift_ycb_contact_by_contact)
        translation = (translation + self.offset_for_mujoco)
        # -----------------------------------------
        qpos_finger = self.pose.compute_qpos_finger(delta_d if isinstance(delta_d, float) else delta_d[0])
        # -----------------------------------------
        self.env.reset(keyframe=0)
        self.env.set_params()

        # ------
        # self.grasp_evaluator.evaluate() # debug !!!!!!
        # -----------------------------------------
        self.pose.set_relational_parameters(delta_d)
        qpos_palm, rotation_palm_world = self.pose.compute_qpos_palm_with_keyframe_initialization(
            rotation_fingertip_world=rotation,
            translation_fingertip_world=translation,
        )

        # print(np.rad2deg(rotation_palm_world.as_rotvec()))
        # import ipdb; ipdb.set_trace()
        # ------------- pre-grasp ---------------
        R_nz = rotation_palm_world.apply(self.n_z0)
        pre_grasp_pos = qpos_palm[:3] + ((-1) * self.sphere_radius * R_nz)
        pre_grasp_pos = self.add_table_height(pre_grasp_pos)
        # -----
        pre_grasp_quat = qpos_palm[3:]
        # ------------ optimal grasp ------------
        optimal_grasp_pos = self.add_table_height(qpos_palm[:3])
        optimal_grasp_quat = pre_grasp_quat
        # ------------ lift up pose -------------
        grasp_pos = qpos_palm[:3]
        lift_up_position = grasp_pos + (self.lift_up_height * self.z_direction_world)
        lift_up_position = self.add_table_height(lift_up_position)
        # ----------------------------------------

        self.viewer_wrapper.launch()
        self.viewer_wrapper.initialize_for_env()
        # import ipdb; ipdb.set_trace()
        # 2) with ブロックで使う
        with self.viewer_wrapper as viewer:
            viewer.camera.set_overview()
            # -----
            self.do_pre_grasp.execute(viewer, pre_grasp_pos, pre_grasp_quat)
            do_stay_here(self.env, viewer, stay_step=self.stay_step.pre_grasp)
            # import ipdb; ipdb.set_trace()
            # -----
            self.do_optimal_grasp.execute(viewer, optimal_grasp_pos, optimal_grasp_quat)
            do_stay_here(self.env, viewer, stay_step=self.stay_step.optimal_reach)
            # import ipdb; ipdb.set_trace()
            # -----
            do_finger_reach(self.env, viewer, qpos_finger)
            do_stay_here(self.env, viewer, stay_step=self.stay_step.finger_close)
            # import ipdb; ipdb.set_trace()
            # -----
            self.do_lift_up.execute(viewer, lift_up_position, pre_grasp_quat)
            # do_stay_here(self.env, viewer, stay_step=200000) # self.stay_step.lift_up)
            do_stay_here(self.env, viewer, stay_step=self.stay_step.lift_up)
            # import ipdb; ipdb.set_trace()

            # ===================================================================================
            self.grasp_evaluator.evaluate(save=True)
            # ===================================================================================

            if not self.config_env.viewer.use_gui:
                # -------
                save_video(
                    frames=viewer.frames,
                    fps=self.config_viewer.save.fps,
                    skip=self.config_viewer.save.skip,
                    save_path=os.path.join(self.results_save_dir,
                                           self.config_viewer.save.filename + f"_{self.model_name}" + ".mp4"),
                )
                # -------
                save_captured_frame(
                    frame=viewer.sync(),
                    save_path=os.path.join(self.results_save_dir, "lift_up_overview" + f"_{self.model_name}" + ".png"),
                )
                viewer.camera.set_zoom_with_fingertip_center(fingertip_center=self.env.fingertip_center_xpos())
                save_captured_frame(
                    frame=viewer.sync(),
                    save_path=os.path.join(self.results_save_dir, "lift_up_zoom" + f"_{self.model_name}" + ".png"),
                )
        # -------
