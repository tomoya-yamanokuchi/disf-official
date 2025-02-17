import os

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import matplotlib.pyplot as plt
from matplotlib import ticker, cm
matplotlib.use('TkAgg')

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from typing import List
from domain_object.builder import DomainObject
from value_object import PointNormalUnitPairs
from value_object import SourcePointSurfaceSet, TargetPointSurfaceSet
# from value_object import PointNormalUnitPairs
from .axis_point_normal_plot import axis_point_normal_plot
from .axis_point_normal_plot_with_different_finger_color import axis_point_normal_plot_with_different_finger_color
from service import set_aspect_equal_3d
from service import set_grid_line_3d
from service import set_empty_ticks_3d


class DISFVisualization:
    def __init__(self, domain_object: DomainObject):
        self.figsize                   = domain_object.config_isf.visualize.figsize
        self.mode                      = domain_object.config_isf.visualize.mode
        self.elev                      = domain_object.config_isf.visualize.elev
        self.azim                      = domain_object.config_isf.visualize.azim
        # ----
        self.use_fixed_limit = domain_object.config_isf.visualize.use_fixed_limit
        #
        self.x_min = domain_object.config_isf.visualize.x_min
        self.x_max = domain_object.config_isf.visualize.x_max
        #
        self.y_min = domain_object.config_isf.visualize.y_min
        self.y_max = domain_object.config_isf.visualize.y_max
        #
        self.z_min = domain_object.config_isf.visualize.z_min
        self.z_max = domain_object.config_isf.visualize.z_max
        # ----
        self.hand_origin_point_size    = domain_object.config_isf.visualize.hand_origin_point_size
        self.hand_origin_normal_length = domain_object.config_isf.visualize.hand_origin_normal_length
        self.label_fontsize            = domain_object.config_isf.visualize.label_fontsize
        # -----
        self.results_save_dir = domain_object.results_save_dir
        self.axis_label       = domain_object.config_isf.visualize.axis_label
        self.empty_ticks      = domain_object.config_isf.visualize.empty_ticks
        self.use_title        = domain_object.config_isf.visualize.use_title
        # -----
        self.config_point_normal = domain_object.config_isf.visualize.point_normal
        # -----
        self.visualize_call = domain_object.config_isf.verbose.visualize_call
        self.n_app          = domain_object.n_app
        # self.call_level = 1


    def set_target_information(self,
            object_whole_surface: PointNormalUnitPairs,
            contact_indices     : np.ndarray,
        ):
        self.object_whole_surface = object_whole_surface
        self.contact_indices      = contact_indices


    def plot_source_correspondence_skew(self,
            correspondence: PointNormalUnitPairs,
        ):
        axis_point_normal_plot_with_different_finger_color(
            ax                = self.ax,
            point_normal      = correspondence,
            label             = "source: skew (correspondence)",
            finger_color_dict = self.finger_correspondence_skew_color_dict,
            point_size        = self.point_size*3,
            normal_length     = self.normal_length*3,
            point_alpha       = self.point_alpha,
            normal_alpha      = self.normal_alpha,
        )


    def plot_source_correspondence(self,
            correspondence: PointNormalUnitPairs,
        ):
        axis_point_normal_plot_with_different_finger_color(
            ax                = self.ax,
            point_normal      = correspondence,
            label             = "source (correspondence)",
            finger_color_dict = self.config_point_normal.color.source.correspondence,
            # ---
            point_size    = self.config_point_normal.point_size.source.correspondence,
            normal_length = self.config_point_normal.normal_length.source.correspondence,
            point_alpha   = self.config_point_normal.point_alpha.source.correspondence,
            normal_alpha  = self.config_point_normal.normal_alpha.source.correspondence,
        )


    def plot_target_correspondence(self,
            correspondence: PointNormalUnitPairs,
        ):
        axis_point_normal_plot(
            ax            = self.ax,
            point_normal  = correspondence,
            label         = f"target (correspondence)",
            color         = self.config_point_normal.color.target.correspondence,
            # ---
            point_size    = self.config_point_normal.point_size.target.correspondence,
            normal_length = self.config_point_normal.normal_length.target.correspondence,
            point_alpha   = self.config_point_normal.point_alpha.target.correspondence,
            normal_alpha  = self.config_point_normal.normal_alpha.target.correspondence,
        )


    def plot_n_z(self, n_z: np.ndarray):
        p0 = np.zeros(3)
        self.ax.quiver(
            *p0,
            *n_z,
            # ---
            color  = self.config_point_normal.color.n_z,
            length = self.config_point_normal.normal_length.n_z,
            alpha  = self.config_point_normal.normal_alpha.n_z,
            linestyles = self.config_point_normal.linestyles.n_z,
        )

    def plot_n_app(self):
        p0 = np.zeros(3)
        self.ax.quiver(
            *p0,
            *self.n_app,
            # ---
            color  = self.config_point_normal.color.n_app,
            length = self.config_point_normal.normal_length.n_app,
            alpha  = self.config_point_normal.normal_alpha.n_app,
            linestyles = self.config_point_normal.linestyles.n_app,
        )


    def plot_source_surface_skew(self,
            surface          : PointNormalUnitPairs,
        ):
        axis_point_normal_plot_with_different_finger_color(
            ax                = self.ax,
            point_normal      = surface,
            label             = f"source: skew (surface)",
            finger_color_dict = self.finger_surface_skew_color_dict,
            point_size        = self.point_size*0.6,
            normal_length     = self.normal_length*1,
            point_alpha       = self.point_alpha,
            normal_alpha      = self.normal_alpha,
        )


    def plot_source_contact(self,
            surface          : PointNormalUnitPairs,
        ):
        axis_point_normal_plot_with_different_finger_color(
            ax                = self.ax,
            point_normal      = surface,
            label             = f"source (surface)",
            finger_color_dict = self.config_point_normal.color.source.contact,
            # ---
            point_size    = self.config_point_normal.point_size.source.contact,
            normal_length = self.config_point_normal.normal_length.source.contact,
            point_alpha   = self.config_point_normal.point_alpha.source.contact,
            normal_alpha  = self.config_point_normal.normal_alpha.source.contact,
        )

    def plot_object_contact_surface(self,
            target_set: TargetPointSurfaceSet,
            data_type : str, # source or target
            color     : str,
        ):
        # -----
        contact_surface    = target_set.contact_surface
        all_indices        = np.arange(contact_surface.points.shape[0])
        complement_indices = np.setdiff1d(all_indices, target_set.correspondence.indices)
        # -----
        visualize_contact_surface = PointNormalUnitPairs(
            points  = contact_surface.points[complement_indices],
            normals = contact_surface.normals[complement_indices],
        )
        # import ipdb ; ipdb.set_trace()
        axis_point_normal_plot(
            ax            = self.ax,
            point_normal  = visualize_contact_surface,
            label         = f"{data_type} (surface)",
            color         = color,
            # ---
            point_size    = self.config_point_normal.point_size.target.contact,
            normal_length = self.config_point_normal.normal_length.target.contact,
            point_alpha   = self.config_point_normal.point_alpha.target.contact,
            normal_alpha  = self.config_point_normal.normal_alpha.target.contact,
        )


    def plot_whole_object(self, color : str):
        all_indices        = np.arange(self.object_whole_surface.points.shape[0])
        complement_indices = np.setdiff1d(all_indices, self.contact_indices)
        # -----
        visualize_object_surface = PointNormalUnitPairs(
            points  = self.object_whole_surface.points[complement_indices],
            normals = self.object_whole_surface.normals[complement_indices],
        )
        # ----
        if complement_indices.shape[0] == 0:
            return # for box case
        # -----
        axis_point_normal_plot(
            ax            = self.ax,
            point_normal  = visualize_object_surface,
            label         = "target (whole)",
            color         = color,
            # ----
            point_size    = self.config_point_normal.point_size.target.surface,
            normal_length = self.config_point_normal.normal_length.target.surface,
            point_alpha   = self.config_point_normal.point_alpha.target.surface,
            normal_alpha  = self.config_point_normal.normal_alpha.target.surface,
        )

    def set_parameters(self, title: str = None):
        if self.axis_label:
            self.ax.set_xlabel('X', fontsize=self.label_fontsize)
            self.ax.set_ylabel('Y', fontsize=self.label_fontsize)
            self.ax.set_zlabel('Z', fontsize=self.label_fontsize)
        # ---
        if self.use_title:
            plt.title(title, fontsize=self.label_fontsize)
        # ---
        set_aspect_equal_3d(self.ax)
        set_grid_line_3d(self.ax)
        if self.empty_ticks: set_empty_ticks_3d(self.ax)
        self.ax.view_init(elev=self.elev, azim=self.azim)

        if self.use_fixed_limit:
            self.ax.set_xlim([self.x_min, self.x_max])
            self.ax.set_ylim([self.y_min, self.y_max])
            self.ax.set_zlim([self.z_min, self.z_max])
        # ---


    def show_or_save(self, filename: str):
        if self.mode == 0:
            plt.show()
        elif self.mode == 1:
            # plt.tight_layout() # 3Dプロットと相性が悪いので使わない
            plt.savefig(
                os.path.join(self.results_save_dir, filename),
                dpi=500,
                bbox_inches='tight'
            )
            plt.close()


    def visualize(self,
            source_set : SourcePointSurfaceSet,
            target_set : TargetPointSurfaceSet,
            n_z        : np.ndarray,
            call_level : int,
            title      : str = None,
            filename   : str = "ipfo.png",
        ):
        if self.visualize_call == -1              : return
        if not (self.visualize_call <= call_level): return

        # ---- make plot object ----
        fig              = plt.figure(figsize=self.figsize)
        self.ax : Axes3D = fig.add_subplot(111, projection='3d')

        # -------- whole_object ---------
        self.plot_whole_object(color='gray')
        # -------- surface ---------
        self.plot_source_contact(source_set.surface)
        self.plot_object_contact_surface(target_set, data_type="target", color='skyblue')
        # ----- correspondence -----
        self.plot_source_correspondence(source_set.correspondence)
        self.plot_target_correspondence(target_set.correspondence)
        # --------- n_app ---------
        if n_z is not None:
            self.plot_n_z(n_z)
            self.plot_n_app()
        # --------- save -----------
        self.set_parameters(title=title)

        self.show_or_save(filename)





    def visualize_with_skew(self,
            source_set_skew: SourcePointSurfaceSet,
            source_set_rod : SourcePointSurfaceSet,
            target_set     : TargetPointSurfaceSet,
            n_z            : np.ndarray,
            call_level     : int,
            title          : str = None,
        ):
        if self.visualize_call == -1              : return
        if not (self.visualize_call <= call_level): return

        # ---- make plot object ----
        fig              = plt.figure(figsize=self.figsize)
        self.ax : Axes3D = fig.add_subplot(111, projection='3d')

        # -------- whole_object ---------
        self.plot_whole_object(color='gray')

        # -------- surface ---------
        self.plot_source_surface_skew(source_set_skew.surface)
        self.plot_source_surface(source_set_rod.surface)
        # ---
        self.plot_object_contact_surface(target_set, data_type="target", color='skyblue')
        # ----- correspondence -----
        self.plot_source_correspondence(source_set_skew.correspondence)
        self.plot_source_correspondence(source_set_rod.correspondence)
        # ---
        self.plot_target_correspondence(target_set.correspondence)
        # ------ normal vector -------
        self.plot_n_z(n_z)
        self.plot_n_app()
        # --------- save -----------
        self.set_parameters(title=title)
        self.show_or_save()
