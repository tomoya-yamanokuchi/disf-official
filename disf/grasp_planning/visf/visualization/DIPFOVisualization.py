import matplotlib

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import matplotlib.pyplot as plt
from matplotlib import ticker, cm

matplotlib.use('TkAgg')

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from typing import List
from disf.domain_object.builder import DomainObject
from disf.value_object import PointNormalUnitPairs
from disf.value_object import SourcePointSurfaceSet, TargetPointSurfaceSet
# from disf.value_object import PointNormalUnitPairs
from .axis_point_normal_plot import axis_point_normal_plot
from .axis_point_normal_plot_with_different_finger_color import axis_point_normal_plot_with_different_finger_color
from disf.service import set_aspect_equal_3d


class DIPFOVisualization:
    def __init__(self, domain_object: DomainObject):
        self.figsize = domain_object.config_isf.visualize.figsize
        self.save_path = domain_object.config_isf.visualize.save_path
        self.mode = domain_object.config_isf.visualize.mode
        self.elev = domain_object.config_isf.visualize.elev
        self.azim = domain_object.config_isf.visualize.azim
        self.point_size = domain_object.config_isf.visualize.point_size
        self.normal_length = domain_object.config_isf.visualize.normal_length
        self.point_alpha = domain_object.config_isf.visualize.point_alpha
        self.normal_alpha = domain_object.config_isf.visualize.normal_alpha
        self.hand_origin_point_size = domain_object.config_isf.visualize.hand_origin_point_size
        self.hand_origin_normal_length = domain_object.config_isf.visualize.hand_origin_normal_length
        self.label_fontsize = domain_object.config_isf.visualize.label_fontsize
        # -----
        self.visualize_call = domain_object.config_isf.verbose.visualize_call
        self.n_app = domain_object.n_app
        # self.call_level = 1
        # ------
        self.finger_surface_skew_color_dict = {
            "1": "tan",  # right
            "2": "lightblue",  # left
        }
        self.finger_correspondence_skew_color_dict = {
            "1": "darkgoldenrod",  # right
            "2": "teal",  # left
        }

        self.finger_surface_color_dict = {
            "1": "plum",  # right
            "2": "limegreen",  # left
        }
        self.finger_correspondence_color_dict = {
            "1": "red",  # right
            "2": "green",  # left
        }
        self.target_correspondence_color = "blue"

    def set_target_information(self,
                               object_whole_surface: PointNormalUnitPairs,
                               contact_indices: np.ndarray,
                               ):
        self.object_whole_surface = object_whole_surface
        self.contact_indices = contact_indices

    def plot_source_correspondence_skew(self,
                                        correspondence: PointNormalUnitPairs,
                                        ):
        axis_point_normal_plot_with_different_finger_color(
            ax=self.ax,
            point_normal=correspondence,
            label="source: skew (correspondence)",
            finger_color_dict=self.finger_correspondence_skew_color_dict,
            point_size=self.point_size * 3,
            normal_length=self.normal_length * 3,
            point_alpha=self.point_alpha,
            normal_alpha=self.normal_alpha,
        )

    def plot_source_correspondence(self,
                                   correspondence: PointNormalUnitPairs,
                                   ):
        axis_point_normal_plot_with_different_finger_color(
            ax=self.ax,
            point_normal=correspondence,
            label="source (correspondence)",
            finger_color_dict=self.finger_correspondence_color_dict,
            point_size=self.point_size * 3,
            normal_length=self.normal_length * 3,
            point_alpha=self.point_alpha,
            normal_alpha=self.normal_alpha,
        )

    def plot_target_correspondence(self,
                                   correspondence: PointNormalUnitPairs,
                                   ):
        axis_point_normal_plot(
            ax=self.ax,
            point_normal=correspondence,
            label=f"target (correspondence)",
            color=self.target_correspondence_color,
            point_size=self.point_size * 3,
            normal_length=self.normal_length * 3,
            point_alpha=self.point_alpha,
            normal_alpha=self.normal_alpha,
        )

    def plot_n_z(self, n_z: np.ndarray):
        p0 = np.zeros(3)
        self.ax.quiver(
            *p0,
            *n_z,
            # ---
            color="darkorange",
            length=self.normal_length * 10,
            alpha=self.normal_alpha,
        )

    def plot_n_app(self):
        p0 = np.zeros(3)
        self.ax.quiver(
            *p0,
            *self.n_app,
            # ---
            color="blue",
            length=self.normal_length * 10,
            alpha=self.normal_alpha,
        )

    def plot_source_surface_skew(self,
                                 surface: PointNormalUnitPairs,
                                 ):
        axis_point_normal_plot_with_different_finger_color(
            ax=self.ax,
            point_normal=surface,
            label=f"source: skew (surface)",
            finger_color_dict=self.finger_surface_skew_color_dict,
            point_size=self.point_size * 0.6,
            normal_length=self.normal_length * 1,
            point_alpha=self.point_alpha,
            normal_alpha=self.normal_alpha,
        )

    def plot_source_surface(self,
                            surface: PointNormalUnitPairs,
                            ):
        axis_point_normal_plot_with_different_finger_color(
            ax=self.ax,
            point_normal=surface,
            label=f"source (surface)",
            finger_color_dict=self.finger_surface_color_dict,
            point_size=self.point_size * 0.6,
            normal_length=self.normal_length * 1,
            point_alpha=self.point_alpha,
            normal_alpha=self.normal_alpha,
        )

    def plot_object_contact_surface(self,
                                    target_set: TargetPointSurfaceSet,
                                    data_type: str,  # source or target
                                    color: str,
                                    ):
        # -----
        contact_surface = target_set.contact_surface
        all_indices = np.arange(contact_surface.points.shape[0])
        complement_indices = np.setdiff1d(all_indices, target_set.correspondence.indices)
        # -----
        visualize_contact_surface = PointNormalUnitPairs(
            points=contact_surface.points[complement_indices],
            normals=contact_surface.normals[complement_indices],
        )
        # import ipdb ; ipdb.set_trace()
        axis_point_normal_plot(
            ax=self.ax,
            point_normal=visualize_contact_surface,
            label=f"{data_type} (surface)",
            color=color,
            point_size=self.point_size * 0.6,
            normal_length=self.normal_length * 1,
            point_alpha=self.point_alpha,
            normal_alpha=self.normal_alpha,
        )

    def plot_whole_object(self, color: str):
        all_indices = np.arange(self.object_whole_surface.points.shape[0])
        complement_indices = np.setdiff1d(all_indices, self.contact_indices)
        # -----
        visualize_object_surface = PointNormalUnitPairs(
            points=self.object_whole_surface.points[complement_indices],
            normals=self.object_whole_surface.normals[complement_indices],
        )
        # -----
        axis_point_normal_plot(
            ax=self.ax,
            point_normal=visualize_object_surface,
            label="target (whole)",
            color=color,
            point_size=self.point_size * 0.3,
            normal_length=self.normal_length * 0.3,
            point_alpha=self.point_alpha,
            normal_alpha=self.normal_alpha,
        )

    def set_parameters(self, title: str = None):
        self.ax.set_xlabel('X', fontsize=self.label_fontsize)
        self.ax.set_ylabel('Y', fontsize=self.label_fontsize)
        self.ax.set_zlabel('Z', fontsize=self.label_fontsize)
        # ---
        plt.title(title, fontsize=self.label_fontsize)
        # ---
        set_aspect_equal_3d(self.ax)
        self.ax.view_init(elev=self.elev, azim=self.azim)

    def show_or_save(self):
        if self.mode == 0:
            plt.show()
        elif self.mode == 1:
            plt.tight_layout()
            plt.savefig(self.save_path, dpi=500, bbox_inches='tight')
            plt.close()

    def visualize(self,
                  source_set: SourcePointSurfaceSet,
                  target_set: TargetPointSurfaceSet,
                  n_z: np.ndarray,
                  call_level: int,
                  title: str = None,
                  ):
        if self.visualize_call == -1: return
        if not (self.visualize_call <= call_level): return

        # ---- make plot object ----
        fig = plt.figure(figsize=self.figsize)
        self.ax: Axes3D = fig.add_subplot(111, projection='3d')

        # -------- whole_object ---------
        self.plot_whole_object(color='gray')
        # -------- surface ---------
        self.plot_source_surface(source_set.surface)
        self.plot_object_contact_surface(target_set, data_type="target", color='skyblue')
        # ----- correspondence -----
        self.plot_source_correspondence(source_set.correspondence)
        self.plot_target_correspondence(target_set.correspondence)
        # --------- n_app ---------
        self.plot_n_z(n_z)
        self.plot_n_app()
        # --------- save -----------
        self.set_parameters(title=title)
        self.show_or_save()

    def visualize_with_skew(self,
                            source_set_skew: SourcePointSurfaceSet,
                            source_set_rod: SourcePointSurfaceSet,
                            target_set: TargetPointSurfaceSet,
                            n_z: np.ndarray,
                            call_level: int,
                            title: str = None,
                            ):
        if self.visualize_call == -1: return
        if not (self.visualize_call <= call_level): return

        # ---- make plot object ----
        fig = plt.figure(figsize=self.figsize)
        self.ax: Axes3D = fig.add_subplot(111, projection='3d')

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
