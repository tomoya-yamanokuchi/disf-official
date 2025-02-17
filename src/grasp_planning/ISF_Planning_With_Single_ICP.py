import os
import numpy as np
from service import ExtendedRotation
from domain_object.builder import DomainObject
from value_object import ISFResult
from value_object import SourcePointSurfaceSet, TargetPointSurfaceSet
from print_color import print
from service import transform_source_set
from service import transform_fingertip
from service import update_R, update_t, update_delta_d
import pickle


class ISF_Planning_With_Single_ICP:
    def __init__(self, domain_object: DomainObject):
        self.isf                       = domain_object.isf # disf or visf
        self.isf_visualizer            = domain_object.isf_visualizer
        # ---
        self.icp_matcher               = domain_object.icp_matcher
        self.loop_criteria             = domain_object.loop_criteria
        self.text_logger               = domain_object.text_logger
        self.d0                        = domain_object.d0
        # ----
        self.source                    = domain_object.source
        self.gripper_euler_angle_rad   = domain_object.gripper_euler_angle_rad
        self.gripper_translation       = domain_object.gripper_translation
        # ----
        self.rotvec_rad_object         = domain_object.rotvec_rad_object
        self.v0                        = domain_object.v0
        self.n_z0                      = domain_object.n_z0
        self.model_name                = domain_object.model_name
        self.method_name               = domain_object.method_name
        # ---
        self.contact_indices = domain_object.contact_indices
        # ----
        self.object_contact_surface    = domain_object.centered_ycb_contact_point_normal
        self.centered_ycb_point_normal = domain_object.centered_ycb_point_normal
        self.results_save_dir          = domain_object.results_save_dir
        # ---
        self.call_level_inner = 2
        self.call_level_outer = 3
        # ----
        self.Ea = domain_object.Ea
        self.En = domain_object.En
        self.Ep = domain_object.Ep


    def run(self, rotvec0: np.ndarray = None, return_all: bool = False) -> ISFResult:
        if rotvec0 is None:
            R0 = ExtendedRotation.from_euler(self.gripper_euler_angle_rad).as_rodrigues()
        else:
            R0 = ExtendedRotation.from_rotvec(rotvec0).as_rodrigues()
        # ----------------------------
        t0 = self.gripper_translation
        # ----------------------------
        source_surface = transform_fingertip(self.source, R0, t0, delta_d=0, v=self.v0)
        target_surface = self.object_contact_surface
        n_z            = (R0 @ self.n_z0)
        # -------
        self.icp_matcher.set_target(target_surface)
        icp_result = self.icp_matcher.find_correspondences(source_surface)
        # -------
        source_set = SourcePointSurfaceSet(correspondence=icp_result.source, surface=source_surface)
        target_set = TargetPointSurfaceSet(
            correspondence  = icp_result.target,
            contact_surface = target_surface,
            whole_surface   = self.centered_ycb_point_normal
        )
        # -------
        self.isf_visualizer.set_target_information(
            object_whole_surface = self.centered_ycb_point_normal,
            contact_indices      = self.contact_indices,
        )
        self.isf_visualizer.visualize(
            source_set = source_set,
            target_set = target_set,
            n_z        = n_z,
            call_level = self.call_level_outer,
            title      = f"[ISF] Init",
            filename   = f"{self.method_name}_{self.model_name}_init.png",
        )
        # ---------------------------------------------------------
        result = self.isf.optimize(source_set, target_set, R0)
        # ---------------------------------------------------------
        self.isf_visualizer.visualize(
            source_set = result.aligned_source_set,
            target_set = target_set,
            n_z        = result.aligned_n_z,
            call_level = self.call_level_outer,
            title      = f"[ISF] Opt",
            filename   = f"{self.method_name}_{self.model_name}_opt.png",
        )
        # -------- compute optimal translation for palm of robot --------
        R_opt       = update_R(R=result.R_sum, Rt=R0)
        t_opt       = update_t(t=result.t_sum, R=result.R_sum, t_t=t0)
        delta_d_opt = result.delta_d_sum
        # --------- set optimal IPFO parameters ---------
        result_isf = ISFResult(
            rotation         = ExtendedRotation.from_matrix(R_opt),
            translation      = t_opt,
            delta_d          = delta_d_opt,
            error            = result.e_p_sum,
            Rt_hist          = None, # np.stack(Rt_hist),
            R_object         = None, # ExtendedRotation.from_rotvec(self.rotvec_rad_object).as_rodrigues(),
            pos_object       = None, # self.pos_object,
            pfo_error_hist   = None, # np.stack(pfo_error_hist),
            es_hist          = None, # np.stack(es_hist),
        )
        self.text_logger.isf_finished(result_isf)
        # ---------- save isf planning results ----------
        np.save(file=os.path.join(self.results_save_dir, "E_total_opt.npy"), arr=np.array([result.e_p_sum.total]))
        np.save(file=os.path.join(self.results_save_dir, "elapsed_time.npy"), arr=np.array([result.elapsed_time]))
        np.save(file=os.path.join(self.results_save_dir, "com_error.npy"), arr=np.array([result.e_com]))
        # -----------------------------------------------
        if not return_all: return result.e_p_sum, # for CMA
        else             : return result_isf




