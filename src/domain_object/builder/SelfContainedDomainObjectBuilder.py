import os
import numpy as np
from copy import deepcopy

class SelfContainedDomainObjectBuilder:
    """
        This object is a "Self-contained Domain Object Builder",
        where the object itself has both of the role of
        instantiation of objects and the role as a usual Domain Object.

        This means that this object looks like a
            (1) "bulder object" for director object
        while it looks like a
            (2) "domain object" for the external object
                    that receives this object in its constructor.
    """

    def get_domain_object(self):
        return self

    # ===================== Mujoco Env =====================
    def build_model(self):
        import mujoco
        self.model = mujoco.MjModel.from_xml_path(self.config_env.xml)
        self.model.opt.timestep = self.config_env.option.timestep # Override the simulation timestep.

    def buid_data(self):
        import mujoco
        self.data = mujoco.MjData(self.model)

    def buid_body(self):
        from robot_env.utils import BodyManager
        self.body = BodyManager(self.model, self.data)

    def buid_site(self):
        from robot_env.utils import SiteManager
        self.site = SiteManager(self.model, self.data)

    def build_geom(self):
        from robot_env.utils.GeomManager import GeomManager
        self.geom = GeomManager(self.model, self.data)

    def build_camera(self):
        from robot_env.utils.Camera import Camera
        self.camera = Camera(self.config_env, model=self.model)

    def build_option(self):
        from robot_env.utils.Option import Option
        self.option = Option(config_viewer=self.config_env.viewer)

    def build_scene(self):
        from robot_env.utils.Scene import Scene
        self.scene = Scene(self.config_env, model=self.model)

    def build_viewer(self):
        from robot_env.utils import MyViewerWrapper
        self.viewer_wrapper = MyViewerWrapper(
            model      = self.model,
            data       = self.data,
            camera     = self.camera,
            opt        = self.option.option,
            scn        = self.scene.scene,
            config_env = self.config_env
        )

    def build_config_env(self, config_name: str = "panda_hand"):
        from config_loader import ConfigLoader
        # :::::::::::: domain object ::::::::::::
        self.config_env = ConfigLoader.load_env(
            config_name = config_name,
        )
        # ----
        self.sphere_radius     = self.config_env.pre_grasp.sphere_radius
        self.lift_up_height    = self.config_env.pre_grasp.lift_up_height
        self.z_direction_world = np.array(self.config_env.pre_grasp.z_direction_world)
        self.show_ui           = self.config_env.viewer.show_ui
        self.d_bias            = self.config_env.d_bias

    def build_config_ik_solver(self, config_name: str = "default_ik_solver"):
        from config_loader import ConfigLoader
        self.config_ik_solver = ConfigLoader.load_ik_solver(
            config_name = config_name,
        )

    def build_config_cma(self, config_name: str = "default"):
        from config_loader import ConfigLoader
        self.config_cma = ConfigLoader.load_cma(
            config_name = config_name,
        )


    def build_config_grasp_evaluation(self, config_name: str = "default"):
        from config_loader import ConfigLoader
        self.config_grasp_evaluation = ConfigLoader.load_grasp_evaluation(
            config_name = config_name,
        )

    def build_config_icp(self, config_name: str = "default_icp"):
        from config_loader import ConfigLoader
        self.config_icp = ConfigLoader.load_icp(
            config_name = config_name,
        )

    def build_config_rotation_initialization(self, config_name: str = "default_rot_init"):
        from config_loader import ConfigLoader
        self.config_rot_init = ConfigLoader.load_rot_init(
            config_name = config_name,
        )

    def build_env(self):
        from robot_env import RobotEnvFactory
        self.env = RobotEnvFactory.create(
            env_name      = self.config_env.env_name,
            domain_object = self,
        )

    def build_pose(self):
        from robot_env.instance.panda_hand.pose import PandaPoseFactory
        self.pose = PandaPoseFactory.create(
            env_name      = self.config_env.env_name,
            domain_object = self,
        )

    def build_ik_solver(self):
        from robot_env.ik_solver import DampedLeastSquares
        self.ik_solver = DampedLeastSquares(
            model            = self.model,
            data             = self.data,
            config_env       = self.config_env,
            config_ik_solver = self.config_ik_solver,
        )

    def build_grasp_evaluator(self):
        from mujoco_grasping import GraspEvaluation
        self.grasp_evaluator = GraspEvaluation(self)

    # ======================== ISF config ========================
    def build_config_isf(self, config_name: str):
        from config_loader import ConfigLoader
        # :::::::::::: domain object ::::::::::::
        self.config_isf = ConfigLoader.load_isf(
            config_name = config_name,
        )


    def build_ipfo_parameters(self):
        from service import normalize_vector
        from service import compute_object_qpos
        # :::::::::::: domain object ::::::::::::
        # ----------------------------------------
        self.n_z0 = normalize_vector(self.config_isf.hand_plane_z)
        self.v0   = normalize_vector(self.config_isf.gripper_normal)

        self.alpha              = self.config_isf.alpha
        self.beta               = self.config_isf.beta
        self.gamma              = self.config_isf.gamma

        self.num_fingertip_surface_points = self.config_isf.num_fingertip_surface_points

        self.rotvec_rad_object  = np.deg2rad(self.config_isf.object_pose.rotvec_degree)
        self.translation_object = np.array(self.config_isf.object_pose.translation)

        self.d_min              = self.config_isf.d_min
        self.d_max              = self.config_isf.d_max
        self.d0                 = self.config_isf.d0

        self.delta_e            = self.config_isf.delta_e

        self.offset_for_mujoco  = np.array(self.config_isf.gripper_pose.offset_for_mujoco)
        self.delta_d_est_init   = self.config_isf.delta_d_est_init
        # self.delta_d_opt        = (self.d0 - self.object_width)
        self.epsilon            = self.config_isf.epsilon
        self.verbose            = self.config_isf.verbose
        self.method_name        = self.config_isf.method_name
        # -----------
        self.qpos_object = compute_object_qpos(**self.config_isf.object_pose)
        # -----------
        self.save_dir = self.config_isf.save.save_dir
        from service import create_directory_if_not_exists
        create_directory_if_not_exists(directory=self.save_dir)



    def build_ipfo_text_logger(self):
        from grasp_planning.utils import IPFOTextLogger
        self.text_logger = IPFOTextLogger(self)

    def build_surface_visualizer(self):
        from grasp_planning.disf.visualization import SurfaceVisualization
        self.surface_visualizer = SurfaceVisualization(self)

    def build_loop_criteria(self):
        from grasp_planning.utils import LoopCriteria
        self.loop_criteria = LoopCriteria(self)

    def build_isf_loop_criteria(self):
        from grasp_planning.utils import ISF_LoopCriteria
        self.isf_loop_criteria = ISF_LoopCriteria(self)

    def build_config_point_cloud_data(self, config_name: str):
        from config_loader import ConfigLoader
        self.config_pc_data = ConfigLoader.load_point_cloud_data(
            config_name = config_name,
        )
        # ---------
        self.contact_plane_origin       = np.array(self.config_pc_data.contact_plane.position_origin, dtype=float)
        self.contact_plane_normal       = np.array(self.config_pc_data.contact_plane.normal_direction_z, dtype=float)
        self.d1                         = self.config_pc_data.contact_plane.d1
        self.d2                         = self.config_pc_data.contact_plane.d2
        # ----
        self.gripper_translation        = np.array(self.config_pc_data.gripper_pose.translation)
        self.gripper_euler_angle_rad    = np.deg2rad(self.config_pc_data.gripper_pose.euler_angle_degree)
        # ----
        from service import normalize_vector
        n_app_unnorm = np.array(self.config_pc_data.n_approach)
        self.n_app   = normalize_vector(n_app_unnorm)
        # ------------------------------------------------------
        self.output_file  = self.config_pc_data.generate.output_file
        self.dir_path     = self.config_pc_data.load.dir_path
        self.suffix_fname = self.config_pc_data.load.suffix_fname
        self.model_name   = self.config_pc_data.model_name
        # -------------------------------------------------------
        from service import create_directory_if_not_exists
        self.results_save_dir = os.path.join(self.save_dir, self.method_name, self.model_name)
        create_directory_if_not_exists(self.results_save_dir)

    def build_ycb_xml_scene_file(self, object_name: str):
        from xml_generation import generate_mujoco_grasping_scene_with_ycb
        # ----
        generate_mujoco_grasping_scene_with_ycb(
            output_file = self.output_file,
            object_name = object_name, # <-- dynamic !!!!!
            base_path   = self.dir_path,
        )

    def build_ycb_data_point_cloud(self):
        import open3d as o3d
        # ---
        file_path = os.path.join(self.dir_path, self.model_name, self.suffix_fname)
        self.point_cloud = o3d.io.read_point_cloud(file_path)
        self.point_cloud = self.point_cloud.voxel_down_sample(
            voxel_size = self.config_pc_data.pre_prosessing.voxel_down_sample_size
        )

    def build_ycb_data_normal_estimation(self):
        import open3d as o3d
        # estimate normal vector
        self.point_cloud.estimate_normals(
            search_param = o3d.geometry.KDTreeSearchParamHybrid(
                radius = self.config_pc_data.pre_prosessing.normal_estimation.radius_neighbor_search,
                max_nn = self.config_pc_data.pre_prosessing.normal_estimation.max_neighbor_number,
            )
        )
        # adjust normal vector direction
        self.point_cloud.orient_normals_consistent_tangent_plane(
            k = self.config_pc_data.pre_prosessing.normal_estimation.k_normals_consistent,
        )
        # --- compute centroid of point cloud ---
        centroid = np.mean(np.asarray(self.point_cloud.points), axis=0)
        # --- adjust the direction of normal vector based on the centroid ---
        points   = np.asarray(self.point_cloud.points)
        normals  = np.asarray(self.point_cloud.normals)
        # --- adjust the normal vector such that it faces outward ---
        for i in range(len(normals)):
            direction = points[i] - centroid       # vectro from centroid to points
            if np.dot(normals[i], direction) < 0:  # if normal vector is pointing at outside
                normals[i] = -normals[i]           # set it opposite direction
        # --- apply the aligned normal ---
        self.point_cloud.normals = o3d.utility.Vector3dVector(normals)


    def build_ycb_data_PointNormal_ValueObject(self):
        from value_object import PointNormalUnitPairs
        points  = np.asarray(self.point_cloud.points)
        normals = np.asarray(self.point_cloud.normals)
        self.ycb_point_normal = PointNormalUnitPairs(points, normals)
        self.point_cloud = None


    def build_ycb_target_by_contact_estimation_with_hand_plane(self):
        from contact_detection import detect_plane_intersection
        from value_object import PointNormalUnitPairs
        filtered_indices = detect_plane_intersection(
            point_cloud       = self.centered_ycb_point_normal_by_whole.points,
            hand_plane_origin = self.contact_plane_origin,
            hand_plane_normal = self.contact_plane_normal,
            d1                = self.d1,
            d2                = self.d2,
        )
        # ---
        self.contact_indices = filtered_indices
        contact_points       = self.centered_ycb_point_normal_by_whole.points[filtered_indices]
        contact_normals      = self.centered_ycb_point_normal_by_whole.normals[filtered_indices]
        # ----
        self.centered_ycb_contact_point_normal_by_whole =  PointNormalUnitPairs(
            points  = contact_points,
            normals = contact_normals,
        )

    def shift_ycb_target_center_to_origin(self):
        from service import shift_object_center_to_origin
        self.centered_ycb_contact_point_normal, self.t_shift_ycb_contact_by_contact = shift_object_center_to_origin(
            target_point_normal = self.centered_ycb_contact_point_normal_by_whole,
        )
        self.target                 = self.centered_ycb_contact_point_normal
        self.object_contact_surface = self.centered_ycb_contact_point_normal

    def shift_ycb_whole_object_center_to_origin(self):
        from service import shift_object_center_to_origin
        self.centered_ycb_point_normal_by_whole, self.t_shift_ycb_by_whole = shift_object_center_to_origin(
            target_point_normal = self.ycb_point_normal,
        )

    def shift_ycb_whole_object_center_with_contact_info(self):
        from value_object import PointNormalUnitPairs
        self.centered_ycb_point_normal = PointNormalUnitPairs(
                points  = (self.centered_ycb_point_normal_by_whole.points + self.t_shift_ycb_contact_by_contact),
                normals = self.centered_ycb_point_normal_by_whole.normals,
        )
        self.object_whole_surface = self.centered_ycb_point_normal


    def build_single_fingertip_target_box(self, j: int):
        from value_object import PointNormalCorrespondencePairs, PointNormalUnitPairs
        from service import generate_surface_points_panda_hand
        # -----
        base_source_points   = generate_surface_points_panda_hand(
            num_fingertip_surface_points = self.num_fingertip_surface_points,
            d0                           = 0.0,
            finger_index                 = j,
        )
        # -----
        object_half_size = self.config_pc_data.object_half_size
        # -----
        base_target_points  = np.copy(base_source_points)
        target_points       = base_target_points + (-1)**(j)*(self.v0*object_half_size)
        target_normals      = np.zeros_like(target_points) + (-1)**(j)*(self.v0)
        # -----
        return PointNormalUnitPairs(
            points  = target_points,
            normals = target_normals,
        )

    def build_paired_finger_box_target(self):
        from value_object import PointNormalUnitPairs
        from copy import deepcopy
        # ---
        right_target : PointNormalUnitPairs = self.build_single_fingertip_target_box(j=1)
        left_target  : PointNormalUnitPairs = self.build_single_fingertip_target_box(j=2)
        # ---
        self.target = PointNormalUnitPairs(
            points  = np.concatenate([right_target.points, left_target.points]),
            normals = np.concatenate([right_target.normals, left_target.normals]),
        )
        # ----
        self.contact_indices        = np.arange(self.target.points.shape[0])
        self.object_contact_surface = deepcopy(self.target)
        self.object_whole_surface   = deepcopy(self.target)

        self.model_name = "Box"

    def build_point_cloud_dataset(self):
        from value_object import PointNormalCorrespondencePairs
        self.point_cloud_dataset = PointNormalCorrespondencePairs(
            source = self.source,
            target = self.target,
        )

    def build_point_cloud_dataset_from_ycb_whole_data(self):
        from value_object import PointNormalCorrespondencePairs
        self.point_cloud_dataset = PointNormalCorrespondencePairs(
            source = self.source,
            target = self.ycb_point_normal,
        )

    def build_right_finger_source(self):
        from value_object import PointNormalIndexUnitPairs
        from service import generate_surface_points_panda_hand
        # ----- right ------
        right_source_points = generate_surface_points_panda_hand(
            num_fingertip_surface_points = self.config_isf.num_fingertip_surface_points,
            d0                           = self.d0,
            finger_index                 = 1,
        )
        right_source_normals = np.zeros_like(right_source_points) + np.array([0, 1, 0])
        # -----
        j = 1
        self.source = PointNormalIndexUnitPairs(
            points         = right_source_points,
            normals        = right_source_normals,
            finger_indices = np.zeros(right_source_points.shape[0]) + j,
        )
        return deepcopy(self.source)

    def build_left_finger_source(self):
        from value_object import PointNormalIndexUnitPairs
        from service import generate_surface_points_panda_hand
        # ----- right ------
        left_source_points = generate_surface_points_panda_hand(
            num_fingertip_surface_points = self.config_isf.num_fingertip_surface_points,
            d0                           = self.d0,
            finger_index                 = 2,
        )
        left_source_normals = np.zeros_like(left_source_points) + np.array([0, -1, 0])
        # -----
        j = 2
        self.source = PointNormalIndexUnitPairs(
            points         = left_source_points,
            normals        = left_source_normals,
            finger_indices = np.zeros(left_source_points.shape[0]) + j,
        )
        return deepcopy(self.source)

    def build_paired_finger_source(self):
        from value_object import PointNormalIndexUnitPairs
        # ---
        right_source : PointNormalIndexUnitPairs = self.build_right_finger_source()
        left_source  : PointNormalIndexUnitPairs = self.build_left_finger_source()
        # ---
        self.source = PointNormalIndexUnitPairs(
            points         = np.concatenate([right_source.points, left_source.points]),
            normals        = np.concatenate([right_source.normals, left_source.normals]),
            finger_indices = np.concatenate([right_source.finger_indices, left_source.finger_indices]),
        )

    def build_hand_origin(self):
        from value_object import PointNormalUnitPairs
        self.hand_origin = PointNormalUnitPairs(
            points  = np.array(self.contact_plane_origin).reshape(1, -1),
            normals = np.array(self.contact_plane_normal).reshape(1, -1),
        )

    def build_single_finger_indices(self, j: int):
        from service import create_single_finger_indices
        self.finger_indices = create_single_finger_indices(
            num_fingertip_surface_points = self.config_isf.num_fingertip_surface_points,
            j = j,
        )
        return deepcopy(self.finger_indices)

    def build_paired_finger_indices(self):
        # ----
        right_index = self.build_single_finger_indices(j=1)
        left_index  = self.build_single_finger_indices(j=2)
        # ----
        self.finger_indices = np.hstack([right_index, left_index])

    def build_palm_least_square(self):
        from grasp_planning.least_square import PalmPoseLeastSquares
        self.palm_least_square = PalmPoseLeastSquares(self)

    def build_coupling_palm_least_square(self):
        from grasp_planning.least_square import CouplingPalmPoseLeastSquares
        self.coupling_palm_least_square = CouplingPalmPoseLeastSquares(self)

    def build_palm_approach_least_square(self):
        from grasp_planning.least_square import PalmPoseApproachRLestSquare
        self.palm_approach_least_square = PalmPoseApproachRLestSquare(self)


    def build_disf_palm_R_ls(self):
        from grasp_planning.disf.optimization.least_square import PalmRotationLeastSquare_EnEa
        self.disf_palm_R_ls = PalmRotationLeastSquare_EnEa(self)

    def build_disf_finger_ls_Ep(self):
        from grasp_planning.disf.optimization.least_square import Point2PlaneLeastSquareWithFingertipDisplacement
        self.disf_finger_ls_Ep = Point2PlaneLeastSquareWithFingertipDisplacement(self)

    def build_palm_point2plane_least_square(self):
        from grasp_planning.least_square import PalmPosePoint2PlaneLS
        self.palm_point2plane_least_square = PalmPosePoint2PlaneLS(self)

    def build_palm_normal_alignment_least_square(self):
        from grasp_planning.least_square import PalmPoseNormalAlignmentLS
        self.palm_normal_alignment_least_square = PalmPoseNormalAlignmentLS(self)


    def build_finger_least_square(self):
        from grasp_planning.least_square import FingertipDisplacementLeastSquare
        self.finger_least_square = FingertipDisplacementLeastSquare(self)

    def build_set_error(self):
        from grasp_planning.utils.error_computation import SetErrorComputation
        from grasp_planning.utils.error_computation import EpComputation
        from grasp_planning.utils.error_computation import EnComputation
        from grasp_planning.utils.error_computation import EaComputation
        # ----
        self.verbose_error = self.config_isf.verbose.textlog.error
        # ----
        self.Ep = EpComputation(self)
        self.En = EnComputation(self)
        self.Ea = EaComputation(self)
        # ----
        self.error = SetErrorComputation(self)

    def build_com_error(self):
        from grasp_planning.utils.error_computation import CoMError
        self.com_error = CoMError(self)

    '''
        VISF: Vanilla Iterative Surface Fitting
    '''
    def build_visf_palm_Rt_opt(self):
        from grasp_planning.visf.optimization import VISF_PalmOptimization
        self.visf_palm_Rt_opt = VISF_PalmOptimization(self)

    def build_visf_finger_opt(self):
        from grasp_planning.visf.optimization import IPFO_FingertipDisplacementOptimization
        self.visf_finger_opt = IPFO_FingertipDisplacementOptimization(self)

    def build_visf_palm_Rt_EpEnEa_least_square(self):
        from grasp_planning.visf.optimization.least_square import PalmLeastSquare_EpEnEa
        self.visf_palm_Rt_ls = PalmLeastSquare_EpEnEa(self)

    def build_visf_fingertip_displacement_least_square(self):
        from grasp_planning.visf.optimization.least_square import FingertipDisplacement
        self.visf_finger_ls = FingertipDisplacement(self)

    '''
        DISF: Disentangled Iterative Surface Fitting
    '''
    def build_disf_palm_R_opt(self):
        from grasp_planning.disf.optimization import DISF_PalmRotationOptimization
        self.disf_palm_R_opt = DISF_PalmRotationOptimization(self)

    def build_disf_trans_centroid(self):
        from grasp_planning.disf.optimization import DISF_TranslationCentroid
        self.disf_trans_centroid = DISF_TranslationCentroid(self)

    def build_disf_finger_opt(self):
        from grasp_planning.disf.optimization import DISF_FingertipDisplacementOptimization
        self.disf_finger_opt = DISF_FingertipDisplacementOptimization(self)


    def build_isf_visualizer(self):
        from grasp_planning.disf.visualization import DISFVisualization
        self.isf_visualizer = DISFVisualization(self)

    def build_ipfo_error_computation(self):
        from grasp_planning.visf import IPFO_ErrorCompute
        self.ipfo_error_computation = IPFO_ErrorCompute(self)

    def build_ipfo_alpha_vs_rotation_est(self):
        from grasp_planning.visf import IPFO_alpha_vs_rotation_est
        self.ipfo_alpha_vs_rotation_est = IPFO_alpha_vs_rotation_est(self)

    def build_vanilla_isf(self):
        from grasp_planning.visf import VISF
        self.visf = VISF(self)
        self.isf  = self.visf

    def build_disf(self):
        from grasp_planning.disf import DISF
        self.disf = DISF(self)
        self.isf  = self.disf

    def build_cmasf(self):
        from grasp_planning.cma import CMASF
        self.cmasf = CMASF(self)
        self.isf   = self.cmasf

    def build_isf_error_compute_static_pair(self):
        from grasp_planning.visf import ISF_ErrorCompute_StaticPair
        self.isf_error_compute_static_pair = ISF_ErrorCompute_StaticPair(self)

    def build_isf_alpha_vs_rotation_est_static_pair(self):
        from grasp_planning.visf import ISF_alpha_vs_rotation_est_StaticPair
        self.isf_alpha_vs_rotation_est_static_pair = ISF_alpha_vs_rotation_est_StaticPair(self)

    def build_isf_loop_with_single_icp(self):
        from grasp_planning.disf import ISF_Planning_With_Single_ICP
        self.isf_planning = ISF_Planning_With_Single_ICP(self)

    def build_icp_matcher(self):
        from correspondence_matching import ICPPointMatcherWithNormals
        self.icp_matcher = ICPPointMatcherWithNormals(self)

