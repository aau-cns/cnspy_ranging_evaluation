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
import time
import numpy as np
from matplotlib import pyplot as plt
import pandas as pandas
import math

from cnspy_numpy_utils.numpy_statistics import *
from cnspy_timestamp_association.TimestampAssociation import TimestampAssociation
from cnspy_trajectory.PlotLineStyle import PlotLineStyle


class AssociateRangesCfg:
    ID1 = None
    ID2 = None
    relative_timestamps = False
    max_difference = 0.02
    subsample = 0
    verbose = False
    remove_outliers = False,
    max_range = 30
    max_range_error = 10
    range_error_val = 0
    label_timestamp = 't'
    label_ID1 = 'ID1'
    label_ID2 = 'ID2'
    label_range = 'range_raw'
    def __init__(self, ID1=None, ID2=None, relative_timestamps=False,
                 max_difference=0.02, subsample=0, verbose=False, remove_outliers=False,
                 max_range=30, range_error_val=0, label_timestamp='t',
                 label_ID1='UWB_ID1',
                 label_ID2='UWB_ID2', label_range = 'range_raw'):
        self.label_timestamp = label_timestamp
        self.label_ID1 = label_ID1
        self.label_ID2 = label_ID2
        self.label_range = label_range
        self.ID1 = ID1
        self.ID2 = ID2
        self.relative_timestamps = relative_timestamps
        self.max_difference = max_difference
        self.subsample = subsample
        self.verbose = verbose
        self.remove_outliers = remove_outliers
        self.max_range = max_range
        self.range_error_val = range_error_val
        
        
    
class AssociateRanges:
    csv_df_gt = None
    csv_df_est = None

    data_frame_gt_matched = None
    data_frame_est_matched = None

    matches_est_gt = None  # list of tuples containing [(idx_est, idx_gt), ...]

    cfg = None
    data_loaded = False

    def __init__(self):
        pass

    def __init__(self, fn_gt, fn_est, cfg):
        assert (isinstance(cfg, AssociateRangesCfg))
        self.cfg = cfg
        self.load(fn_gt, fn_est, cfg)

    def load(self, fn_gt, fn_est, cfg):
        assert (isinstance(cfg, AssociateRangesCfg))
        assert (os.path.exists(fn_gt)), str("Path to fn_gt does not exist!:" + str(fn_gt))
        assert (os.path.exists((fn_est))), str("Path to fn_est does not exist!:" + str(fn_est))

        self.csv_df_gt = pandas.read_csv(fn_gt, sep='\s+|\,', comment='#', engine='python')
        self.csv_df_est = pandas.read_csv(fn_est, sep='\s+|\,', comment='#', engine='python')

        if cfg.ID1 is not None:
            self.csv_df_gt = (self.csv_df_gt.loc[self.csv_df_gt[cfg.label_ID1] == cfg.ID1])
            self.csv_df_est = (self.csv_df_est.loc[self.csv_df_est[cfg.label_ID1] == cfg.ID1])

            if len(self.csv_df_gt) == 0 or len(self.csv_df_est) == 0:
                print("ID1=" + str(cfg.ID1) + " not found")
                return

        if cfg.ID2 is not None:
            self.csv_df_gt = (self.csv_df_gt.loc[self.csv_df_gt[cfg.label_ID2] == cfg.ID2])
            self.csv_df_est = (self.csv_df_est.loc[self.csv_df_est[cfg.label_ID2] == cfg.ID2])
            if len(self.csv_df_gt) == 0 or len(self.csv_df_est) == 0:
                print("ID2=" + str(cfg.ID2) + " not found")
                return

        if cfg.remove_outliers:
            indices1 = np.nonzero(self.csv_df_est[cfg.label_range] < self.cfg.max_range)
            indices2 = np.nonzero(self.csv_df_est[cfg.label_range] > 0)
            idc = np.intersect1d(indices1, indices2)
            num_outliers = len(self.csv_df_est.index) - len(idc)
            perc = 100.0*(num_outliers / max(1, len(self.csv_df_est.index)))
            if num_outliers:
                self.csv_df_est = AssociateRanges.sample_DataFrame(self.csv_df_est, np.intersect1d(indices1, indices2))
                print('AssociateRanges.load(): [%d] outliers (%.1f %%) removed!' % (num_outliers, perc))

        else:
            indices = ((self.csv_df_est[cfg.label_range]) < 0)
            self.csv_df_est.loc[indices, cfg.label_range] = cfg.range_error_val
            indices = (self.csv_df_est[cfg.label_range] > cfg.max_range)
            self.csv_df_est.loc[indices, cfg.label_range] = cfg.range_error_val

        if cfg.subsample > 1:
            subsample = round(cfg.subsample, 0)
            self.csv_df_gt = AssociateRanges.subsample_DataFrame(df=self.csv_df_gt, step=subsample, verbose=cfg.verbose)
            self.csv_df_est = AssociateRanges.subsample_DataFrame(df=self.csv_df_est, step=subsample, verbose=cfg.verbose)

        if version_info[0] < 3:
            t_vec_gt = self.csv_df_gt.as_matrix([cfg.label_timestamp])
            t_vec_est = self.csv_df_est.as_matrix([cfg.label_timestamp])
            t_zero = min(t_vec_gt[0], t_vec_est[0])

            if len(t_vec_gt) == 0 or len(t_vec_est) == 0:
                print("empty")
                return

            if cfg.relative_timestamps:
                self.csv_df_gt[[cfg.label_timestamp]] = self.csv_df_gt[[cfg.label_timestamp]] - t_zero
                self.csv_df_est[[cfg.label_timestamp]] = self.csv_df_est[[cfg.label_timestamp]] - t_zero
        else:
            # FIX(scm): for newer versions as_matrix is deprecated, using to_numpy instead
            # from https://stackoverflow.com/questions/60164560/attributeerror-series-object-has-no-attribute-as-matrix-why-is-it-error
            t_vec_gt = self.csv_df_gt[[cfg.label_timestamp]].to_numpy()
            t_vec_est = self.csv_df_est[[cfg.label_timestamp]].to_numpy()
            if len(t_vec_gt) == 0 or len(t_vec_est) == 0:
                print("empty")
                return

            t_zero = min(t_vec_gt[0], t_vec_est[0])
            if cfg.relative_timestamps:
                self.csv_df_gt[[cfg.label_timestamp]] = self.csv_df_gt[[cfg.label_timestamp]] - t_zero
                self.csv_df_est[[cfg.label_timestamp]] = self.csv_df_est[[cfg.label_timestamp]] - t_zero

        if cfg.relative_timestamps:
            # only relative time stamps:
            t_vec_gt = t_vec_gt - t_zero
            t_vec_est = t_vec_est - t_zero

        idx_est, idx_gt, t_est_matched, t_gt_matched = TimestampAssociation.associate_timestamps(
            t_vec_est,
            t_vec_gt,
            max_difference=cfg.max_difference,
            round_decimals=6,
            unique_timestamps=True)

        self.data_frame_est_matched = self.csv_df_est.iloc[idx_est, :]
        self.data_frame_gt_matched = self.csv_df_gt.iloc[idx_gt, :]
        self.matches_est_gt = zip(idx_est, idx_gt)

        if cfg.verbose:
            print("AssociateRanges(): {} timestamps associated.".format(len(idx_est)))

        self.data_loaded = True
        # using zip() and * operator to
        # perform Unzipping
        # res = list(zip(*test_list))

    def plot_timestamps(self, cfg_dpi=200, cfg_title="timestamps", calc_error=False, fig=None, ax=None,
                        colors=['r', 'g'], labels=['gt', 'est'],
                        ls_vec=[PlotLineStyle(linestyle='-'), PlotLineStyle(linestyle='-.')],
                        save_fn="", result_dir="."):
        if not self.data_loaded:
            return
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        if version_info[0] < 3:
            t_vec_gt = self.data_frame_gt_matched.as_matrix([self.cfg.label_timestamp])
            t_vec_est = self.data_frame_est_matched.as_matrix([self.cfg.label_timestamp])
        else:
            t_vec_gt = self.data_frame_gt_matched[[self.cfg.label_timestamp]].to_numpy()
            t_vec_est = self.data_frame_est_matched[[self.cfg.label_timestamp]].to_numpy()

        if not calc_error:
            x_arr = range(len(t_vec_gt))
            AssociateRanges.ax_plot_n_dim(ax, x_arr, t_vec_gt, colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
            x_arr = range(len(t_vec_est))
            AssociateRanges.ax_plot_n_dim(ax, x_arr, t_vec_est, colors=[colors[1]], labels=[labels[1]], ls=ls_vec[1])

            ax.grid(both=True)
            ax.set_xlabel('idx')
            ax.set_ylabel('time [s]')
        else:
            x_arr = range(len(t_vec_gt))
            # t_est = t_true + t_err
            t_vec_err = t_vec_est - t_vec_gt
            AssociateRanges.ax_plot_n_dim(ax, x_arr, t_vec_err, colors=['r'], labels=['err'], ls=ls_vec[0])
            ax.grid()
            ax.set_xlabel('idx')
            ax.set_ylabel('diff time [s]')

        AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        ax.legend()
        return fig, ax

    def plot_ranges(self, cfg_dpi=200, cfg_title="ranges", sorted=False, fig=None, ax=None,
                    colors=['r', 'g'], labels=['gt', 'est'],
                    ls_vec=[PlotLineStyle(linestyle='-'), PlotLineStyle(linestyle='-.')],
                    save_fn="", result_dir="."):
        if not self.data_loaded:
            return
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        if version_info[0] < 3:
            t_vec_gt = self.data_frame_gt_matched.as_matrix([self.cfg.label_timestamp])
            t_vec_est = self.data_frame_est_matched.as_matrix([self.cfg.label_timestamp])
        else:
            t_vec_gt = self.data_frame_gt_matched[[self.cfg.label_timestamp]].to_numpy()
            t_vec_est = self.data_frame_est_matched[[self.cfg.label_timestamp]].to_numpy()

        if version_info[0] < 3:
            r_vec_gt = self.data_frame_gt_matched.as_matrix([self.cfg.label_range])
            r_vec_est = self.data_frame_est_matched.as_matrix([self.cfg.label_range])
        else:
            r_vec_gt = self.data_frame_gt_matched[[self.cfg.label_range]].to_numpy()
            r_vec_est = self.data_frame_est_matched[[self.cfg.label_range]].to_numpy()

        if not sorted:
            # x_arr = range(len(t_vec_gt))
            AssociateRanges.ax_plot_n_dim(ax, t_vec_gt, r_vec_gt, colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
            # x_arr = range(len(t_vec_est))
            AssociateRanges.ax_plot_n_dim(ax, t_vec_est, r_vec_est, colors=[colors[1]], labels=[labels[1]], ls=ls_vec[1])

            ax.grid()
            ax.set_ylabel('range')
            ax.set_xlabel('time [s]')
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        else:
            gt_indices_sorted = np.argsort(r_vec_gt, axis=0)
            x_arr = range(len(r_vec_gt))
            AssociateRanges.ax_plot_n_dim(ax, x_arr, np.take_along_axis(r_vec_gt, gt_indices_sorted, axis=0), colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
            AssociateRanges.ax_plot_n_dim(ax, x_arr, np.take_along_axis(r_vec_est, gt_indices_sorted, axis=0), colors=[colors[1]], labels=[labels[1]], ls=ls_vec[1])
            ax.grid()
            ax.set_ylabel('range')
            ax.set_xlabel('range sorted index')
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        ax.legend()
        return fig, ax

    def get_range_error(self, sort=False, remove_outlier=True, max_error=None):
        if not self.data_loaded:
            return
        if version_info[0] < 3:
            t_vec_gt = self.data_frame_gt_matched.as_matrix([self.cfg.label_timestamp])
            t_vec_est = self.data_frame_est_matched.as_matrix([self.cfg.label_timestamp])
        else:
            t_vec_gt = self.data_frame_gt_matched[[self.cfg.label_timestamp]].to_numpy()
            t_vec_est = self.data_frame_est_matched[[self.cfg.label_timestamp]].to_numpy()

        if version_info[0] < 3:
            r_vec_gt = self.data_frame_gt_matched.as_matrix([self.cfg.label_range])
            r_vec_est = self.data_frame_est_matched.as_matrix([self.cfg.label_range])
        else:
            r_vec_gt = self.data_frame_gt_matched[[self.cfg.label_range]].to_numpy()
            r_vec_est = self.data_frame_est_matched[[self.cfg.label_range]].to_numpy()

        if not sort:
            # if r_est = r_true + r_err, then  r_err is an offset or the constant bias (gamma).
            r_vec_err = r_vec_est - r_vec_gt
            t_vec = t_vec_gt

            if max_error:
                indices = np.nonzero((abs(r_vec_err) > max_error))
                t_vec = np.delete(t_vec, indices, axis=0, )
                r_vec_err = np.delete(r_vec_err, indices, axis=0)

            return [t_vec, r_vec_err]
        else:
            # if r_est = r_true + r_err, then  r_err is an offset or the constant bias (gamma).
            r_vec_err = r_vec_est - r_vec_gt
            x_arr = r_vec_gt

            if max_error:
                indices = np.nonzero((abs(r_vec_err) > max_error))
                x_arr = np.delete(x_arr, indices, axis=0, )
                r_vec_err = np.delete(r_vec_err, indices, axis=0)

            gt_indices_sorted = np.argsort(x_arr, axis=0)
            #x_arr = range(len(r_vec_gt))
            t_vec = np.take_along_axis(x_arr, gt_indices_sorted, axis=0)
            r_vec = np.take_along_axis(r_vec_err, gt_indices_sorted, axis=0)
            return [t_vec, r_vec]

    def plot_range_error(self, cfg_dpi=200, cfg_title="ranges", sorted=False, remove_outlier=True, fig=None, ax=None,
                    colors=['r'], labels=['error'],
                    ls_vec=[PlotLineStyle(linestyle='-'), PlotLineStyle(linestyle='-.')],
                    save_fn="", result_dir="."):
        if not self.data_loaded:
            return
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        [t_vec, r_vec_err] = self.get_range_error(sort=sorted, remove_outlier=remove_outlier)
        if not sorted:
            AssociateRanges.ax_plot_n_dim(ax, t_vec, r_vec_err, colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
            ax.grid()
            ax.set_ylabel('range error (est-gt)')
            ax.set_xlabel('time [s]')
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        else:
            AssociateRanges.ax_plot_n_dim(ax, t_vec, r_vec_err,
                                          colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])

            ax.grid()
            ax.set_ylabel('range error (est-gt)')
            ax.set_xlabel('range sorted index')
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        ax.legend()
        stat = numpy_statistics(vNumpy=np.squeeze(np.asarray(r_vec_err)))
        return fig, ax, stat, r_vec_err

    def plot_range_error_histogram(self, cfg_dpi=200, fig=None, ax=None,
                                   save_fn="", result_dir=".", max_error=None, filter_histogramm=False, perc_inliers = 0.3,
                                   ID1=None, ID2=None):
        if not self.data_loaded:
            return fig, ax, None, None

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)

        if ID1 is None:
            ID1 = self.cfg.ID1
        if ID2 is None:
            ID2 = self.cfg.ID2

        [t_vec, r_vec_err] = self.get_range_error(sort=True, remove_outlier=True, max_error=max_error)
        num_bins = 50
        n, bins, patches = ax.hist(r_vec_err, num_bins, density=True, color='red', alpha=0.75, label='Histogram')

        if len(t_vec) == 0 or len(r_vec_err) == 0:
            return fig, ax, None, None

        if not filter_histogramm:
            # add a 'best fit' line
            stat = numpy_statistics(vNumpy=np.squeeze(np.asarray(r_vec_err)))
            sigma = stat['std']
            mu = stat['mean']
            y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
                 np.exp(-0.5 * (1 / sigma * (bins - mu)) ** 2))

            scaling = len(r_vec_err)/num_bins
            ax.plot(bins, y*scaling, '--', color='blue', alpha=0.75, label='PDF')
            ax.set_ylabel('num. samples normalized')
            ax.set_xlabel('error [m]')
            ax.set_title(r'Range Error Histogram ID' + str(ID1) + '-ID' + str(ID2) + ': $\mu$=' + str(round(mu, 3)) + ', $\sigma$=' + str(round(sigma, 3)))
            ax.legend()
            return fig, ax, stat, r_vec_err
        else:
            idx_n_sorted = np.argsort(n)

            # assuming a certain percentage as inliers:
            num_best_bins = int(num_bins*(0.5*perc_inliers))

            # compute the mean about the most frequent values:
            idx_best_bins = idx_n_sorted[-num_best_bins:]
            best_error_vals = bins[idx_best_bins]
            mean_best_errors = np.mean(best_error_vals, axis=0)

            # compute a boundary to remove outliers:
            min_offset_errors = mean_best_errors - np.min(best_error_vals, axis=0)
            max_offset_errors = np.max(best_error_vals, axis=0) - mean_best_errors
            print('mean_best_errors %f' % mean_best_errors)

            # remove outliers
            r_filtered_err = r_vec_err[(r_vec_err > (mean_best_errors - min_offset_errors*2.0)) &
                                       (r_vec_err < (mean_best_errors + 2.0*max_offset_errors))]

            # add a 'best fit' line
            if len(r_filtered_err) > 1:
                stat = numpy_statistics(vNumpy=np.squeeze(np.asarray(r_filtered_err)))
                num_plot_bins = int(num_bins*(perc_inliers))
                n_, bins_, patches_ = ax.hist(r_filtered_err, num_plot_bins, density=True, color='blue', alpha=0.75, label='Histogram (filtered)')
                sigma = max(0.001, stat['std'])
                mu = stat['mean']
                scaling = 1.0  #len(r_filtered_err)/num_plot_bins
                y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
                     np.exp(-0.5 * (1 / sigma * (bins_ - mu)) ** 2))
                ax.plot(bins_, y*scaling, '--', color='green', label='PDF (filtered)')
                ax.set_ylabel('num. samples normalized')
                ax.set_xlabel('error [m]')
                ax.set_title(r'Range Error Histogram (filtered) ID' + str(ID1) + '-ID' + str(ID2) + ': $\mu$=' + str(round(mu, 3)) + ', $\sigma$=' + str(round(sigma, 3)))
                ax.legend()
            else:
                stat = None
            return fig, ax, stat, r_filtered_err
        pass

    def save(self, result_dir=None, prefix=None):
        if not self.data_loaded:
            return
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
    def show_save_figure(fig, save_fn="", result_dir=".", dpi=200, show=True, close_figure=False):
        assert (isinstance(fig, plt.Figure))

        plt.pause(0.1)
        # set current active again
        plt.figure(fig.number)
        # draw
        fig.canvas.draw_idle()
        plt.pause(0.1)
        time.sleep(0.1)
        if save_fn:
            if not os.path.exists(result_dir):
                os.makedirs(result_dir)

            filename = os.path.join(result_dir, save_fn) + ".png"

            # create directories for files
            print("save to file: " + filename)
            [root, ext] = os.path.splitext(filename)
            [head, tail] = os.path.split(root)
            try:  # else already exists
                os.makedirs(head)
            except:
                pass

            #plt.savefig(fig=fig, fname=filename, dpi=float(dpi), format="png")
            fig.savefig(filename, dpi=float(dpi), format='png')

            filename = os.path.join(result_dir, save_fn) + ".svg"
            fig.savefig(filename, dpi=float(dpi), format='svg')
            print("save to file done...")
        if show:
            plt.show()
        if close_figure:
            plt.close(fig)
