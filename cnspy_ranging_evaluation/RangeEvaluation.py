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
import argparse
import time
import math
import pandas as pandas
import numpy as np
from cnspy_timestamp_association.TimestampAssociation import TimestampAssociation
from pkgs.f_ranging_evaluation.cnspy_ranging_evaluation.AssociateRanges import AssociateRanges
from matplotlib import pyplot as plt

class RangeEvaluation:
    report = None

    def __init__(self,
                 fn_gt,
                 fn_est,
                 UWB_ID1=0, UWB_ID2_arr=[1],
                 result_dir=None,
                 prefix=None,
                 subsample=0,
                 plot=False,
                 save_plot=False,
                 show_plot=False,
                 max_difference=0.01,
                 relative_timestamps=True,
                 verbose=False):
        if not result_dir:
            result_dir = '.'
        if not prefix:
            prefix = ''

        fn_gt = os.path.abspath(fn_gt)
        fn_est = os.path.abspath(fn_est)

        fig_t = plt.figure(figsize=(20, 15), dpi=int(200))
        fig_r = plt.figure(figsize=(20, 15), dpi=int(200))

        n_rows = len(UWB_ID2_arr)
        idx = 1
        for UWB_ID2 in UWB_ID2_arr:
            assoc = AssociateRanges(fn_gt=fn_gt, fn_est=fn_est, UWB_ID1=int(UWB_ID1), UWB_ID2=int(UWB_ID2),
                                           max_difference=max_difference,
                                           relative_timestamps=relative_timestamps,
                                           subsample=subsample, verbose=verbose)

            ax_t = fig_t.add_subplot(n_rows, 1, idx)
            ax_r = fig_r.add_subplot(n_rows, 1, idx)
            assoc.plot_timestamps(fig=fig_t, ax=ax_t)
            assoc.plot_ranges(fig=fig_r, ax=ax_r)
            idx += 1
        if verbose:
            print("* RangeEvaluation(): ranges associated!")

        assoc.save(result_dir=result_dir, prefix=prefix)
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='RangeEvaluation: evaluate and estimated trajectory against a true trajectory')
    parser.add_argument('--fn_gt', help='input ground-truth trajectory CSV file', default="not specified")
    parser.add_argument('--fn_est', help='input estimated  trajectory CSV file', default="not specified")
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--UWB_ID1', help='ID of TX', default='0')
    parser.add_argument('--UWB_ID2s', help='ID of RX', nargs='+', default=[1])
    parser.add_argument('--prefix', help='prefix in results', default='')
    parser.add_argument('--max_timestamp_difference', help='Max difference between associated timestampes (t_gt - t_est)', default=0.01)
    parser.add_argument('--subsample', help='subsampling factor for input data (CSV)', default=0)
    parser.add_argument('--plot', action='store_true', default=False)
    parser.add_argument('--save_plot', action='store_true', default=False)
    parser.add_argument('--show_plot', action='store_true', default=False)
    parser.add_argument('--relative_timestamp', action='store_true', default=False)
    parser.add_argument('--verbose', action='store_true', default=False)


    tp_start = time.time()
    args = parser.parse_args()

    eval = RangeEvaluation( fn_gt=args.fn_gt,
                            fn_est=args.fn_est,
                            UWB_ID1=int(args.UWB_ID1),
                            UWB_ID2_arr=args.UWB_ID2s,
                            result_dir=args.result_dir,
                            prefix=args.prefix,
                            max_difference=args.max_timestamp_difference,
                            subsample=int(args.subsample),
                            plot=args.plot,
                            save_plot=args.save_plot,
                            show_plot=args.show_plot,
                            relative_timestamps=args.relative_timestamp,
                            verbose=args.verbose)

    print(" ")
    print("finished after [%s sec]\n" % str(time.time() - tp_start))