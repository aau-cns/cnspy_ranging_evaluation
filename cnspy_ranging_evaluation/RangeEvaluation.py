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
# os
########################################################################################################################
import os
from sys import version_info
from datetime import datetime
import argparse
import time
import math
import yaml
import pandas as pandas
import numpy as np

import cnspy_numpy_utils.numpy_statistics
from cnspy_timestamp_association.TimestampAssociation import TimestampAssociation
from cnspy_ranging_evaluation.AssociateRanges import AssociateRanges, AssociateRangesCfg
from matplotlib import pyplot as plt

class RangeEvaluation:

    report = None
    def __init__(self,
                 fn_gt,
                 fn_est,
                 UWB_ID1_arr=[0], UWB_ID2_arr=[1],
                 result_dir=None,
                 prefix=None,
                 save_plot=True,
                 save_statistics=True,
                 show_plot=True,
                 cfg = AssociateRangesCfg(),
                 plot_timestamps=True,
                 plot_ranges=True,
                 plot_ranges_sorted=True,
                 plot_error =True,
                 plot_histogram=True,
                 verbose=False,
                 filter_histogram=True
                 ):
        if not result_dir:
            result_dir = '.'
        if not prefix:
            prefix = ''

        statistics_file = None
        if save_statistics and not plot_histogram:
            print("RangeEvaluation: Warning save_statistics can only be used in combination with plot_histogram")
            plot_histogram = True
        if save_statistics:
            if not os.path.exists(result_dir):
                os.makedirs(result_dir)
            stat_fn = os.path.join(result_dir, 'statistics.yaml')
            statistics_file = open(stat_fn, 'w')
            if verbose:
                print("RangeEvaluation: stat_fn=" + stat_fn)
            yaml.dump({'info': 'RangeEvaluation Statistics',
                       'time': datetime.now().strftime("%m/%d/%Y, %H:%M:%S"),
                       'fn_gt': fn_gt,
                       'fn_est': fn_est,
                       'filter_histogram': filter_histogram,
                       'result_dir': result_dir}, statistics_file, sort_keys=False, explicit_start=False, default_flow_style=False)
            yaml.dump({'UWB_ID1s': UWB_ID1_arr,
                       'UWB_ID2s': UWB_ID2_arr}, statistics_file, sort_keys=False, explicit_start=False, default_flow_style=True)
        fn_gt = os.path.abspath(fn_gt)
        fn_est = os.path.abspath(fn_est)

        plt.style.use('ggplot')

        SMALL_SIZE = 6
        MEDIUM_SIZE = 7
        BIGGER_SIZE = 8

        plt.rc('font', size=SMALL_SIZE)  # controls default text sizes
        plt.rc('axes', titlesize=SMALL_SIZE)  # fontsize of the axes title
        plt.rc('axes', labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
        plt.rc('xtick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
        plt.rc('ytick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
        plt.rc('legend', fontsize=SMALL_SIZE)  # legend fontsize
        plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

        for UWB_ID1 in UWB_ID1_arr:
            if plot_timestamps:
                fig_t = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_t.suptitle('Timestamps of ID=' + str(UWB_ID1), fontsize=16)
            if plot_ranges:
                fig_r = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_r.suptitle('Ranges of ID=' + str(UWB_ID1), fontsize=16)
            if plot_ranges_sorted:
                fig_rs = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_rs.suptitle('Range sorted of ID=' + str(UWB_ID1), fontsize=16)
            if plot_error:
                fig_e = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_e.suptitle('Error of ID=' + str(UWB_ID1), fontsize=16)
            if plot_histogram:
                fig_h = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_h.suptitle('Histograms of ID=' + str(UWB_ID1), fontsize=16)
            if save_statistics:
                dict_statistics_i = {'ID' : UWB_ID1, 'contant_bias_table' : dict(), 'noise_table' : dict()}
                pass

            n = len(UWB_ID2_arr)
            if UWB_ID1 in UWB_ID2_arr:
                n = n - 1

            sqrt_n = max(1, math.ceil(math.sqrt(n)))
            n_rows = sqrt_n
            if sqrt_n * sqrt_n < n:
                n_cols = sqrt_n + 1
            else:
                n_cols = sqrt_n

            if n_rows * n_cols <= n:
                assert(False, "something went wrong!")

            idx = 1
            for UWB_ID2 in UWB_ID2_arr:
                if UWB_ID1 == UWB_ID2:
                    continue

                cfg.ID1 = int(UWB_ID1)
                cfg.ID2 = int(UWB_ID2)
                cfg_title = str("ID" + str(UWB_ID1) + " to ID" + str(UWB_ID2))
                assoc = AssociateRanges(fn_gt=fn_gt, fn_est=fn_est, cfg=cfg)
                assoc.save(result_dir=result_dir, prefix=prefix+cfg_title)

                if not assoc.data_loaded:
                    print("RangeEvaluation: no data loaded for " + cfg_title)
                    continue

                if plot_timestamps:
                    ax_t = fig_t.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_timestamps(fig=fig_t, ax=ax_t, calc_error=True, cfg_title=cfg_title)

                if plot_ranges:
                    ax_r = fig_r.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_ranges(fig=fig_r, ax=ax_r, cfg_title=cfg_title)

                if plot_ranges_sorted:
                    ax_rs = fig_rs.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_ranges(fig=fig_rs, ax=ax_rs, sorted=True, cfg_title=cfg_title)
                if plot_error:
                    ax_e = fig_e.add_subplot(n_rows, n_cols, idx)
                    [fig_, ax_, stat, r_vec_err_] = assoc.plot_range_error(fig=fig_e, ax=ax_e,
                                                                        sorted=False,
                                                                        remove_outlier=True,
                                                                        cfg_title=cfg_title)
                    cnspy_numpy_utils.numpy_statistics.print_statistics(stat, desc=cfg_title + " error")
                if plot_histogram:
                    ax_h = fig_h.add_subplot(n_rows, n_cols, idx)
                    [fig_, ax_, stat, r_vec_err_] = assoc.plot_range_error_histogram(fig=fig_h,
                                                                                     ax=ax_h,
                                                                                     max_error=1,
                                                                                     filter_histogramm=filter_histogram)
                    if stat:
                        dict_statistics_i['contant_bias_table'][UWB_ID2] = round(float(stat['mean']),2)
                        dict_statistics_i['noise_table'][UWB_ID2] = round(float(stat['std']),2)

                # the histogram of the date
                idx += 1

                pass
            if verbose:
                print("* RangeEvaluation(): ranges associated!")

            # Tweak spacing to prevent clipping of ylabel
            if plot_timestamps:
                fig_t.tight_layout()
                if save_plot:
                    AssociateRanges.show_save_figure(fig=fig_t, result_dir=result_dir,
                                                     save_fn=str("Timestamps" + str(UWB_ID1)),
                                                     show=show_plot, close_figure=not show_plot)

            if plot_ranges:
               fig_r.tight_layout()
               if save_plot:
                   AssociateRanges.show_save_figure(fig=fig_r, result_dir=result_dir,
                                                    save_fn=str("Ranges_ID" + str(UWB_ID1)),
                                                    show=show_plot, close_figure=not show_plot)

            if plot_ranges_sorted:
                fig_rs.tight_layout()
                if save_plot:
                    AssociateRanges.show_save_figure(fig=fig_rs, result_dir=result_dir,
                                                     save_fn=str("Range_Sorted_ID" + str(UWB_ID1)),
                                                     show=show_plot, close_figure=not show_plot)

            if plot_error:
                fig_e.tight_layout()
                if save_plot:
                    AssociateRanges.show_save_figure(fig=fig_e, result_dir=result_dir,
                                                     save_fn=str("Errors_ID" + str(UWB_ID1)),
                                                     show=show_plot, close_figure=not show_plot)

            if plot_histogram:
                fig_h.tight_layout()
                if save_plot:
                    AssociateRanges.show_save_figure(fig=fig_h, result_dir=result_dir,
                                                     save_fn=str("Histograms_ID" + str(UWB_ID1)),
                                                     show=show_plot, close_figure=not show_plot)

            if save_statistics:
                yaml.dump(dict_statistics_i, statistics_file,explicit_start=True, default_flow_style=True)
                if verbose:
                    print(yaml.dump(dict_statistics_i,explicit_start=True, default_flow_style=True))
            pass # UWB_ID2
        pass # UWB_ID1

        if save_statistics:
            statistics_file.close()

        pass # DONE

def main():
    parser = argparse.ArgumentParser(
        description='RangeEvaluation: evaluate and estimated and true pairwise ranges')
    parser.add_argument('--fn_gt', help='input ground-truth trajectory CSV file', default="not specified")
    parser.add_argument('--fn_est', help='input estimated  trajectory CSV file', default="not specified")
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--UWB_ID1s', help='ID of TX', nargs='+', default=[0])
    parser.add_argument('--UWB_ID2s', help='ID of RX', nargs='+', default=[1])
    parser.add_argument('--prefix', help='prefix in results', default='')
    parser.add_argument('--max_timestamp_difference', help='Max difference between associated timestampes (t_gt - t_est)', default=0.03)
    parser.add_argument('--subsample', help='subsampling factor for input data (CSV)', default=0)
    parser.add_argument('--plot', action='store_true', default=True)
    parser.add_argument('--save_plot', action='store_true', default=True)
    parser.add_argument('--save_statistics', action='store_true', default=True)
    parser.add_argument('--show_plot', action='store_true', default=True)
    parser.add_argument('--relative_timestamps', action='store_true', default=False)
    parser.add_argument('--remove_outliers', action='store_true', default=False)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--max_range', help='range that classifies as outlier', default='30')
    parser.add_argument('--range_error_val', help='value assigned to outlier', default='0')
    parser.add_argument('--label_timestamp', help='timestamp label in CSV', default='t')
    parser.add_argument('--label_range', help='range label in CSV', default='range_raw')
    parser.add_argument('--label_ID1', help='ID1 label in CSV', default='UWB_ID1')
    parser.add_argument('--label_ID2', help='ID2 label in CSV', default='UWB_ID2')
    parser.add_argument('--plot_timestamps', action='store_true', default=False)
    parser.add_argument('--plot_ranges', action='store_true', default=False)
    parser.add_argument('--plot_ranges_sorted', action='store_true', default=False)
    parser.add_argument('--plot_errors', action='store_true', default=False)
    parser.add_argument('--plot_histograms', action='store_true', default=False)
    tp_start = time.time()
    args = parser.parse_args()
    cfg = AssociateRangesCfg(ID1=None,
                             ID2=None,
                             relative_timestamps=args.relative_timestamps,
                             max_difference=float(args.max_timestamp_difference),
                             subsample=int(args.subsample),
                             verbose=args.verbose,
                             remove_outliers=args.remove_outliers,
                             max_range=float(args.max_range),
                             range_error_val=float(args.range_error_val),
                             label_timestamp=args.label_timestamp,
                             label_ID1=args.label_ID1,
                             label_ID2=args.label_ID2,
                             label_range = args.label_range)

    eval = RangeEvaluation( fn_gt=args.fn_gt,
                            fn_est=args.fn_est,
                            UWB_ID1_arr=args.UWB_ID1s,
                            UWB_ID2_arr=args.UWB_ID2s,
                            cfg=cfg,
                            result_dir=args.result_dir,
                            prefix=args.prefix,
                            save_plot=args.save_plot,
                            show_plot=args.show_plot,
                            save_statistics=args.save_statistics,
                            plot_timestamps=args.plot_timestamps,
                            plot_ranges=args.plot_ranges,
                            plot_ranges_sorted=args.plot_ranges_sorted,
                            plot_error=args.plot_errors,
                            plot_histogram=args.plot_histograms,
                            )

    print(" ")
    print("finished after [%s sec]\n" % str(time.time() - tp_start))
    pass

if __name__ == "__main__":
    main()
    pass