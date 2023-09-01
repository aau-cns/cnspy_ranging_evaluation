#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Requirements:
# numpy, matplotlib
########################################################################################################################
import os
from sys import version_info

import numpy as np
from matplotlib import pyplot as plt
import pandas as pandas
import math

from cnspy_timestamp_association.TimestampAssociation import TimestampAssociation

class PlotLineStyle():
    linecolor = 'r'  # b,g,r,c,m,y,k,w
    linewidth = 1
    linestyle = '-'  # '-', '--', '-.', ':'
    marker = '.'  # '.', ',', 'o', 'v', '^', '<', '>', '1..4', 's', 'p'
    markersize = 12
    markerfacecolor = 'blue'
    label = 'x'

    def __init__(self, linecolor='r', linewidth=1, linestyle='-', marker='o', markersize=12,
                 markerfacecolor='blue', label='x'):
        self.linecolor = linecolor
        self.linewidth = linewidth
        self.linestyle = linestyle
        self.marker = marker
        self.markersize = markersize
        self.markerfacecolor = markerfacecolor
        self.label = label


class AssociateRanges:
    csv_df_gt = None
    csv_df_est = None

    data_frame_gt_matched = None
    data_frame_est_matched = None

    matches_est_gt = None  # list of tuples containing [(idx_est, idx_gt), ...]

    def __init__(self, fn_gt, fn_est, UWB_ID1=None, UWB_ID2=None, relative_timestamps=False,
                 max_difference=0.02, subsample=0, verbose=False,
                 max_range=30, range_error_val=0):
        assert (os.path.exists(fn_gt)), str("Path to fn_gt does not exist!:" + str(fn_gt))
        assert (os.path.exists((fn_est))), str("Path to fn_est does not exist!:" + str(fn_est))

        self.csv_df_gt = pandas.read_csv(fn_gt, sep='\s+|\,', comment='#', engine='python')
        self.csv_df_est = pandas.read_csv(fn_est, sep='\s+|\,', comment='#', engine='python')

        if UWB_ID1 is not None:
            self.csv_df_gt= (self.csv_df_gt.loc[self.csv_df_gt['UWB_ID1'] == UWB_ID1])
            self.csv_df_est = (self.csv_df_est.loc[self.csv_df_est['UWB_ID1'] == UWB_ID1])

        if UWB_ID2  is not None:
            self.csv_df_gt = (self.csv_df_gt.loc[self.csv_df_gt['UWB_ID2'] == UWB_ID2])
            self.csv_df_est = (self.csv_df_est.loc[self.csv_df_est['UWB_ID2'] == UWB_ID2])

        #self.csv_df_est["range_raw"] = self.csv_df_est["range_raw"].where(abs(self.csv_df_est["range_raw"]) > max_range, other=range_error_val)

        indices = (abs(self.csv_df_est["range_raw"]) > max_range)
        self.csv_df_est.loc[indices, "range_raw"] = 0

        if subsample > 1:
            subsample = round(subsample, 0)
            self.csv_df_gt= AssociateRanges.subsample_DataFrame(df=self.csv_df_gt, step=subsample, verbose=verbose)
            self.csv_df_est = AssociateRanges.subsample_DataFrame(df=self.csv_df_est, step=subsample, verbose=verbose)

        if version_info[0] < 3:
            t_vec_gt = self.csv_df_gt.as_matrix(['t'])
            t_vec_est = self.csv_df_est.as_matrix(['t'])
            t_zero = min(t_vec_gt[0], t_vec_est[0])
            if relative_timestamps:
                self.csv_df_gt[['t']] = self.csv_df_gt[['t']] - t_zero
                self.csv_df_est[['t']] = self.csv_df_est[['t']] - t_zero
        else:
            # FIX(scm): for newer versions as_matrix is deprecated, using to_numpy instead
            # from https://stackoverflow.com/questions/60164560/attributeerror-series-object-has-no-attribute-as-matrix-why-is-it-error
            t_vec_gt = self.csv_df_gt[['t']].to_numpy()
            t_vec_est = self.csv_df_est[['t']].to_numpy()
            t_zero = min(t_vec_gt[0], t_vec_est[0])
            if relative_timestamps:
                self.csv_df_gt[['t']] = self.csv_df_gt[['t']] - t_zero
                self.csv_df_est[['t']] = self.csv_df_est[['t']] - t_zero

        if relative_timestamps:
            # only relative time stamps:
            t_vec_gt = t_vec_gt - t_zero
            t_vec_est = t_vec_est - t_zero

        idx_est, idx_gt, t_est_matched, t_gt_matched = TimestampAssociation.associate_timestamps(
            t_vec_est,
            t_vec_gt,
            max_difference=max_difference,
            round_decimals=6,
            unique_timestamps=True)

        self.data_frame_est_matched = self.csv_df_est.iloc[idx_est, :]
        self.data_frame_gt_matched = self.csv_df_gt.iloc[idx_gt, :]
        self.matches_est_gt = zip(idx_est, idx_gt)

        if verbose:
            print("AssociateRanges(): {} timestamps associated.".format(len(idx_est)))


        # using zip() and * operator to
        # perform Unzipping
        # res = list(zip(*test_list))

    def plot_timestamps(self, cfg_dpi=200, cfg_title="timestamps", fig=None, ax=None,
                        colors=['r', 'g'], labels=['gt', 'est'],
                        ls_vec=[PlotLineStyle(linestyle='-'), PlotLineStyle(linestyle='-.')],
                        save_fn="",  result_dir="."):
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        if version_info[0] < 3:
            t_vec_gt = self.data_frame_gt_matched.as_matrix(['t'])
            t_vec_est = self.data_frame_est_matched.as_matrix(['t'])
        else:
            t_vec_gt = self.data_frame_gt_matched[['t']].to_numpy()
            t_vec_est = self.data_frame_est_matched[['t']].to_numpy()

        x_arr = range(len(t_vec_gt))
        AssociateRanges.ax_plot_n_dim(ax, x_arr, t_vec_gt, colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
        x_arr = range(len(t_vec_est))
        AssociateRanges.ax_plot_n_dim(ax, x_arr, t_vec_est, colors=[colors[1]], labels=[labels[1]], ls=ls_vec[1])

        ax.grid(b=True)
        ax.set_xlabel('idx')
        ax.set_ylabel('time [s]')
        AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        return fig, ax

    def plot_ranges(self, cfg_dpi=200, cfg_title="ranges", fig=None, ax=None,
                        colors=['r', 'g'], labels=['gt', 'est'],
                        ls_vec=[PlotLineStyle(linestyle='-'), PlotLineStyle(linestyle='-.')],
                        save_fn="",  result_dir="."):
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        if version_info[0] < 3:
            t_vec_gt = self.data_frame_gt_matched.as_matrix(['t'])
            t_vec_est = self.data_frame_est_matched.as_matrix(['t'])
        else:
            t_vec_gt = self.data_frame_gt_matched[['t']].to_numpy()
            t_vec_est = self.data_frame_est_matched[['t']].to_numpy()

        if version_info[0] < 3:
            r_vec_gt = self.data_frame_gt_matched.as_matrix(['range_raw'])
            r_vec_est = self.data_frame_est_matched.as_matrix(['range_raw'])
        else:
            r_vec_gt = self.data_frame_gt_matched[['range_raw']].to_numpy()
            r_vec_est = self.data_frame_est_matched[['range_raw']].to_numpy()

        #x_arr = range(len(t_vec_gt))
        AssociateRanges.ax_plot_n_dim(ax, t_vec_gt, r_vec_gt, colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
        #x_arr = range(len(t_vec_est))
        AssociateRanges.ax_plot_n_dim(ax, t_vec_est, r_vec_est, colors=[colors[1]], labels=[labels[1]], ls=ls_vec[1])

        ax.grid(b=True)
        ax.set_ylabel('range')
        ax.set_xlabel('time [s]')
        AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        return fig, ax


    def save(self, result_dir=None, prefix=None):
        if not result_dir:
            fn_est_ = str(os.path.splitext(self.csv_df_est.fn)[0]) + "_matched.csv"
            fn_gt_ = str(os.path.splitext(self.csv_df_gt.fn)[0]) + "_matched.csv"
        else:
            if not prefix:
                prefix = ""
            if not os.path.exists(result_dir):
                os.makedirs(os.path.abspath(result_dir))
            fn_est_ = str(result_dir) + '/' + str(prefix) + "est_matched.csv"
            fn_gt_ = str(result_dir) + '/' + str(prefix) + "gt_matched.csv"

        self.data_frame_est_matched.to_csv(fn_est_, sep=',', index=False)
        self.data_frame_gt_matched.to_csv(fn_gt_, sep=',', index=False)


    @staticmethod
    def subsample_DataFrame(df, step=None, num_max_points=None, verbose=False):

        num_elems = len(df.index)

        if num_max_points:
            step = 1
            if (int(num_max_points) > 0) and (int(num_max_points) < num_elems):
                step = int(math.ceil(num_elems / float(num_max_points)))

        sparse_indices = np.arange(start=0, stop=num_elems, step=step)

        if num_max_points or step:
            if verbose:
                print("subsample_DataFrame():")
                print("* len: " + str(num_elems) + ", max_num_points: " + str(
                    num_max_points) + ", subsample by: " + str(step))

            return AssociateRanges.sample_DataFrame(df, sparse_indices)
        else:
            return df.copy(True)

    @staticmethod
    def sample_DataFrame(df, indices_arr):
        num_elems = len(df.index)
        assert (len(indices_arr) <= num_elems), "sample_DataFrame():\n\t index array must be smaller " \
                                                    "equal the dataframe."
        assert (max(indices_arr) <= num_elems), "sample_DataFrame():\n\t elements in the index array " \
                                                    "must be smaller equal the dataframe."
        assert (min(indices_arr) >= 0), "sample_DataFrame():\n\t elemts in the index array " \
                                                    "must be greater equal zero."

        df_sub = df.iloc[indices_arr]
        df_sub.reset_index(inplace=True, drop=True)
        return df_sub


    @staticmethod
    def ax_plot_n_dim(ax, x_linespace, values,
                      colors=['r', 'g', 'b'],
                      labels=['x', 'y', 'z'], ls=PlotLineStyle()):
        assert len(colors) == len(labels)
        if len(colors) > 1:
            assert len(colors) == values.shape[1]
            for i in range(len(colors)):
                ax.plot(x_linespace, values[:, i],
                        color=colors[i], label=labels[i], linestyle=ls.linestyle, linewidth=ls.linewidth,
                        marker=ls.marker)
        else:
            ax.plot(x_linespace, values, color=colors[0], label=labels[0], linestyle=ls.linestyle,
                    linewidth=ls.linewidth)
    @staticmethod
    def ax_x_linespace(ax, ts=None, dist_vec=None, relative_time=True, plot_type=None, x_label_prefix=''):
        if plot_type == "plot_2D_over_dist":
            x_arr = dist_vec
            ax.set_xlabel(x_label_prefix + 'distance [m]')
        else:
            x_arr = ts
            if relative_time:
                x_arr = x_arr - x_arr[0]
                ax.set_xlabel(x_label_prefix + 'rel. time [sec]')
            else:
                ax.set_xlabel(x_label_prefix + 'time [sec]')
        return x_arr

    @staticmethod
    def show_save_figure(fig, save_fn="",  result_dir=".", dpi = 200, show = True, close_figure = False):
        assert(isinstance(fig, plt.Figure))
        plt.pause(0.01)
        plt.draw()
        plt.pause(0.01)
        if save_fn:
            if not os.path.exists(result_dir):
                os.makedirs(result_dir)

            filename = os.path.join(result_dir, save_fn)
            print("save to file: " + filename)
            plt.savefig(filename, dpi=int(dpi))
        if show:
            plt.show()
        if close_figure:
            plt.close(fig)