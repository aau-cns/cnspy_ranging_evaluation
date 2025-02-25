#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2025, Roland Jung (roland.jung@aau.at) , AAU, CNS
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
########################################################################################################################

import time
import os
import argparse
import yaml
import csv
import numpy as np
from matplotlib import pyplot as plt
import pandas as pandas
import math

from cnspy_numpy_utils.numpy_statistics import numpy_statistics
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_ranging_evaluation.AssociateRanges import AssociateRangesCfg, AssociateRanges


class PlotTwoWayRanges:
    csv_df = None
    cfg = None
    data_loaded = False

    def __init__(self):
        pass

    def __init__(self, fn, cfg=AssociateRangesCfg()):
        assert (isinstance(cfg, AssociateRangesCfg))
        self.cfg = cfg
        self.load(fn, cfg)

    def load(self, fn, cfg):
        assert (isinstance(cfg, AssociateRangesCfg))
        assert (os.path.exists(fn)), str("PlotTwoWayRanges.load(): Path to fn does not exist!:" + str(fn))

        self.csv_df = pandas.read_csv(fn, sep='\s+|\,', comment='#', engine='python')
        if cfg.ID1 is not None:
            self.csv_df = (self.csv_df.loc[self.csv_df[cfg.label_ID1] == cfg.ID1])

            if len(self.csv_df) == 0:
                print("ID1=" + str(cfg.ID1) + " not found")
                return

        if cfg.ID2 is not None:
            self.csv_df = (self.csv_df.loc[self.csv_df[cfg.label_ID2] == cfg.ID2])
            if len(self.csv_df) == 0:
                print("ID2=" + str(cfg.ID2) + " not found")
                return

        if cfg.remove_outliers:
            indices1 = np.nonzero(self.csv_df[cfg.label_range] < self.cfg.max_range)
            indices2 = np.nonzero(self.csv_df[cfg.label_range] > 0)
            idc = np.intersect1d(indices1, indices2)
            num_outliers = len(self.csv_df.index) - len(idc)
            perc = 100.0 * (num_outliers / max(1, len(self.csv_df.index)))
            if num_outliers:
                self.csv_df_est = AssociateRanges.sample_DataFrame(self.csv_df, np.intersect1d(indices1, indices2))
                print('PlotTwoWayRanges.load(): [%d] outliers (%.1f %%) removed!' % (num_outliers, perc))

        else:
            indices = ((self.csv_df[cfg.label_range]) < 0)
            self.csv_df.loc[indices, cfg.label_range] = cfg.range_error_val
            indices = (self.csv_df[cfg.label_range] > cfg.max_range)
            self.csv_df.loc[indices, cfg.label_range] = cfg.range_error_val

        if cfg.subsample > 1:
            subsample = round(cfg.subsample, 0)
            self.csv_df = AssociateRanges.subsample_DataFrame(df=self.csv_df, step=subsample, verbose=cfg.verbose)

        # FIX(scm): for newer versions as_matrix is deprecated, using to_numpy instead
        # from https://stackoverflow.com/questions/60164560/attributeerror-series-object-has-no-attribute-as-matrix-why-is-it-error
        t_vec = self.csv_df[[cfg.label_timestamp]].to_numpy()
        if len(t_vec) == 0:
            print("PlotTwoWayRanges.load(): empty file...")
            return

        t_zero = t_vec[0]
        if cfg.relative_timestamps:
            self.csv_df[[cfg.label_timestamp]] = self.csv_df[[cfg.label_timestamp]] - t_zero

        self.data_loaded = True
        # using zip() and * operator to
        # perform Unzipping
        # res = list(zip(*test_list))

    def get_ids(self):
        if not self.data_loaded:
            return
        ID1_vec = list(set(self.csv_df[self.cfg.label_ID1]))
        ID2_vec = list(set(self.csv_df[self.cfg.label_ID2]))

        return ID1_vec, ID2_vec

    def plot_ranges(self, cfg_dpi=200, cfg_title="ranges", sorted=False, fig=None, ax=None,
                    color='g', label=['range'],
                    ID1=None, ID2=None,
                    ls=PlotLineStyle(linestyle='-'),
                    save_fn="", result_dir="."):
        if not self.data_loaded:
            return
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        if ID1 is None:
            ID1 = self.cfg.ID1
        if ID2 is None:
            ID2 = self.cfg.ID2

        df = self.csv_df
        # filter IDs:
        if ID1 is not None:
            df = (df.loc[df[self.cfg.label_ID1] == ID1])

            if len(df) == 0:
                print("ID1=" + str(ID1) + " not found")
                return

        if ID2 is not None:
            df = (df.loc[df[self.cfg.label_ID2] == ID2])
            if len(df) == 0:
                print("ID2=" + str(ID2) + " not found")
                return

        t_vec_gt = df[[self.cfg.label_timestamp]].to_numpy()
        r_vec_gt = df[[self.cfg.label_range]].to_numpy()

        if not sorted:
            # x_arr = range(len(t_vec_gt))
            AssociateRanges.ax_plot_n_dim(ax, t_vec_gt, r_vec_gt, colors=[color], labels=[label], ls=ls)

            ax.grid()
            ax.set_ylabel('range')
            ax.set_xlabel('time [s]')
            ax.set_title(r'Ranges ID' + str(ID1) + '-ID' + str(ID2))
            ax.legend()
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        else:
            gt_indices_sorted = np.argsort(r_vec_gt, axis=0)
            x_arr = range(len(r_vec_gt))
            AssociateRanges.ax_plot_n_dim(ax, x_arr, np.take_along_axis(r_vec_gt, gt_indices_sorted, axis=0),
                                          colors=[color], labels=[label], ls=ls)
            ax.grid()
            ax.set_ylabel('range')
            ax.set_xlabel('range sorted index')
            ax.set_title(r'Ranges ID' + str(ID1) + '-ID' + str(ID2))
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        ax.legend()
        return fig, ax

    def plot_range_histogram(self, cfg_dpi=200, fig=None, ax=None,
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

        df = self.csv_df

        # filter IDs:
        if ID1 is not None:
            df = (df.loc[df[self.cfg.label_ID1] == ID1])

            if len(df) == 0:
                print("ID1=" + str(ID1) + " not found")
                return

        if ID2 is not None:
            df = (df.loc[df[self.cfg.label_ID2] == ID2])
            if len(df) == 0:
                print("ID2=" + str(ID2) + " not found")
                return

        t_vec = df[[self.cfg.label_timestamp]].to_numpy()
        r_vec_err = df[[self.cfg.label_range]].to_numpy()

        num_bins = 50
        n, bins, patches = ax.hist(r_vec_err, num_bins, density=True, color='red', alpha=0.75, label='Histogram')

        if len(t_vec) == 0 or len(r_vec_err) == 0:
            return fig, ax, None, None

            # add a 'best fit' line
        stat = numpy_statistics(vNumpy=np.squeeze(np.asarray(r_vec_err)))
        sigma = stat['std']
        mu = stat['mean']
        y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
             np.exp(-0.5 * (1 / sigma * (bins - mu)) ** 2))

        scaling = len(r_vec_err) / num_bins
        ax.plot(bins, y * scaling, '--', color='blue', alpha=0.75, label='PDF')
        ax.set_ylabel('num. samples normalized')
        ax.set_xlabel('range [m]')
        ax.set_title(
            r'Range Histogram ID' + str(ID1) + '-ID' + str(ID2) + ': $\mu$=' + str(round(mu, 3)) + ', $\sigma$=' + str(
                round(sigma, 3)))
        ax.legend()

        return fig, ax, stat, r_vec_err

    def plot(self, fn,
             result_dir=None,
             verbose=False,
             plot_histograms=False,
             plot_ranges=True,
             show_plots=True,
             save_plots=True,
             ls=PlotLineStyle(),
             cfg=AssociateRangesCfg()):
        if not os.path.isfile(fn):
            print("PlotTrajectory: could not find file: %s" % fn)
            return False

        ## create result dir:
        if result_dir is None:
            folder = str(fn).replace(".csv", "")
        else:
            folder = result_dir
        folder = os.path.abspath(folder)
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass
        if verbose:
            print("* result_dir: \t " + str(folder))

        IDs1, IDs2 = self.get_ids()

        for ID1 in IDs1:
            if plot_ranges:
                fig_r = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_r.suptitle('Ranges of ID=' + str(ID1), fontsize=16)
            if plot_histograms:
                fig_h = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_h.suptitle('Histograms of ID=' + str(ID1), fontsize=16)
                pass

            n = len(IDs2)
            if ID1 in IDs2:
                n = n - 1

            sqrt_n = max(1, math.ceil(math.sqrt(n)))
            n_rows = sqrt_n
            if sqrt_n * sqrt_n < n:
                n_cols = sqrt_n + 1
            else:
                n_cols = sqrt_n

            if n_rows * n_cols <= n:
                assert (False, "something went wrong!")

            idx = 1

            for ID2 in IDs2:
                if ID1 == ID2:
                    continue

                cfg.ID1 = int(ID1)
                cfg.ID2 = int(ID2)
                cfg_title = str("ID" + str(ID1) + " to ID" + str(ID2))

                if plot_ranges:
                    ax_r = fig_r.add_subplot(n_rows, n_cols, idx)
                    self.plot_ranges(fig=fig_r, ax=ax_r, cfg_title=cfg_title, ID1=ID1, ID2=ID2)
                if plot_histograms:
                    ax_h = fig_h.add_subplot(n_rows, n_cols, idx)
                    [fig_, ax_, stat, r_vec_err_] = self.plot_range_histogram(fig=fig_h, ax=ax_h, ID1=ID1, ID2=ID2)

                # the histogram of the date
                idx += 1
                pass

            if plot_ranges:
                fig_r.tight_layout()
                if save_plots:
                    AssociateRanges.show_save_figure(fig=fig_r, result_dir=folder,
                                                     save_fn=str("Ranges_ID" + str(ID1)),
                                                     show=show_plots, close_figure=not show_plots)

            if plot_histograms:
                fig_h.tight_layout()
                if save_plots:
                    AssociateRanges.show_save_figure(fig=fig_h, result_dir=folder,
                                                     save_fn=str("Histograms_ID" + str(ID1)),
                                                     show=show_plots, close_figure=not show_plots)

        return True


def main():
    # test3: python3 TWR_ROSbag2CS.py --bagfile ../test/example.bag --topics /a01/ranging /a02/ranging--verbose --filenames ranges.csv
    parser = argparse.ArgumentParser(
        description='PlotTwoWayRanges: extract and store given topics of a rosbag into a CSV file')
    parser.add_argument('--filename', help='csv filename', default=None)
    parser.add_argument('--result_dir', help='directory to store results]',
                        default=None)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--plot_ranges', action='store_true', default=False)
    parser.add_argument('--plot_histograms', action='store_true', default=False)
    parser.add_argument('--show_plots', action='store_true', default=False)
    parser.add_argument('--save_plots', action='store_true', default=False)
    parser.add_argument('--save_metrics', action='store_true', default=False)

    tp_start = time.time()
    args = parser.parse_args()

    res = False
    if args.filename is not None and (args.plot_ranges or args.plot_histograms):
        cfg = AssociateRangesCfg(max_range=150, relative_timestamps=True,verbose=args.verbose)
        plotter = PlotTwoWayRanges(fn=args.filename, cfg=cfg)
        res = plotter.plot(fn=args.filename,
                           cfg=cfg,
                           result_dir=args.result_dir,
                           verbose=args.verbose,
                           show_plots=args.show_plots,
                           save_plots=args.save_plots,
                           plot_ranges=args.plot_ranges,
                           plot_histograms=args.plot_histograms)
    if res:
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
