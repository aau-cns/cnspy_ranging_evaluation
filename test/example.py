#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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

import os
from cnspy_ranging_evaluation.RangeEvaluation import *
from cnspy_ranging_evaluation.TWR_ROSbag2CSV import *
from cnspy_ranging_evaluation.ROSBag_TrueRanges import *

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')
RES_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_results')

class RangeEvaluationTrail:

    @staticmethod
    def test_eval():
        reprocess = True
        sequence = 'T1_A3_loiter_2m_2023-08-31-20-58-20'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        topic_list=['/d01/ranging', '/a01/ranging', '/a02/ranging', '/a03/ranging']
        fn_meas_ranges = str(RES_DATA_DIR + '/' + sequence + '-all-meas-ranges.csv')
        fn_gt_ranges = str(RES_DATA_DIR + '/' + sequence + '-all-true-ranges.csv')


        # 1) extract all measurements to CSV
        if not os.path.isfile(fn_meas_ranges) or reprocess:
            res = TWR_ROSbag2CSV.extract_to_one(bagfile_name=bagfile_in,
                                                    topic_list=topic_list,
                                                    fn=fn_meas_ranges,
                                                    result_dir=RES_DATA_DIR,
                                                    verbose=True)



        bagfile_out = str(RES_DATA_DIR + '/' + sequence + '-true-ranges.bag')

        # 2) create a clean bag file
        if not os.path.isfile(bagfile_out) or reprocess:
            res = ROSbag_TrueRanges.extract(bagfile_in_name=bagfile_in, bagfile_out_name=bagfile_out, cfg=cfg_file,
                                            stddev_range=0.001, verbose=True)

        if not os.path.isfile(fn_gt_ranges) or reprocess:
            # 3) extract all measurements from the clean bagfile
            res = TWR_ROSbag2CSV.extract_to_one(bagfile_name=bagfile_out,
                                                    topic_list=topic_list,
                                                    fn=fn_gt_ranges,
                                                    result_dir=RES_DATA_DIR,
                                                    verbose=True)


        # 4) evaluate the ranges
        cfg = AssociateRangesCfg(ID1=None,
                                 ID2=None,
                                 relative_timestamps=False,
                                 max_difference=0.03,
                                 subsample=0,
                                 verbose=True,
                                 remove_outliers=False,
                                 max_range=30,
                                 range_error_val=0,
                                 label_timestamp='t',
                                 label_ID1='UWB_ID1',
                                 label_ID2='UWB_ID2',
                                 label_range='range_raw')

        eval = RangeEvaluation(fn_gt=fn_gt_ranges,
                               fn_est=fn_meas_ranges,
                               UWB_ID1_arr=[0,1,2,3],
                               UWB_ID2_arr=[0,1,2,3],
                               cfg=cfg,
                               result_dir=str(RES_DATA_DIR + '/' + sequence + '-eval/'),
                               prefix='',
                               save_plot=True,
                               show_plot=False,
                               save_statistics=True,
                               plot_timestamps=True,
                               plot_ranges=True,
                               plot_ranges_sorted=True,
                               plot_error=True,
                               plot_histogram=True,
                               verbose=True)


    @staticmethod
    def test_range_gen_outliers():
        reprocess=True
        sequence = 'T1_A3_loiter_2m_2023-08-31-20-58-20'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        topic_list=['/d01/ranging', '/a01/ranging', '/a02/ranging', '/a03/ranging']
        fn_meas_ranges = str(RES_DATA_DIR + '/' + sequence + '-all-true-ranges-stddev0-outliers0_1.csv')
        fn_gt_ranges = str(RES_DATA_DIR + '/' + sequence + '-all-true-ranges.csv')


        bagfile_out_noisy = str(RES_DATA_DIR + '/' + sequence + '-true-ranges-stddev0-outliers0_1.bag')

        # 1) create a dirty bag file
        if not os.path.isfile(bagfile_out_noisy) or reprocess:
            res = ROSbag_TrueRanges.extract(bagfile_in_name=bagfile_in, bagfile_out_name=bagfile_out_noisy,
                                            cfg=cfg_file, stddev_range=0.001, perc_outliers=0.1, stddev_outlier=1.0,
                                            verbose=True)

        if not os.path.isfile(fn_meas_ranges) or reprocess:
            # 2) extract all measurements from the dirty bagfile
            res = TWR_ROSbag2CSV.extract_to_one(bagfile_name=bagfile_out_noisy,
                                                    topic_list=topic_list,
                                                    fn=fn_meas_ranges,
                                                    result_dir=RES_DATA_DIR,
                                                    verbose=True)

        bagfile_out = str(RES_DATA_DIR + '/' + sequence + '-true-ranges.bag')

        # 3) create a clean bag file
        if not os.path.isfile(bagfile_out) or reprocess:
            res = ROSbag_TrueRanges.extract(bagfile_in_name=bagfile_in, bagfile_out_name=bagfile_out, cfg=cfg_file,
                                            stddev_range=0.001, verbose=True)

        if not os.path.isfile(fn_gt_ranges) or reprocess:
            # 4) extract all measurements from the clean bagfile
            res = TWR_ROSbag2CSV.extract_to_one(bagfile_name=bagfile_out,
                                                    topic_list=topic_list,
                                                    fn=fn_gt_ranges,
                                                    result_dir=RES_DATA_DIR,
                                                    verbose=True)

        # 4) evaluate the ranges
        cfg = AssociateRangesCfg(ID1=None,
                                 ID2=None,
                                 relative_timestamps=False,
                                 max_difference=0.03,
                                 subsample=0,
                                 verbose=True,
                                 remove_outliers=False,
                                 max_range=30,
                                 range_error_val=0,
                                 label_timestamp='t',
                                 label_ID1='UWB_ID1',
                                 label_ID2='UWB_ID2',
                                 label_range='range_raw')

        eval = RangeEvaluation(fn_gt=fn_gt_ranges,
                               fn_est=fn_meas_ranges,
                               UWB_ID1_arr=[0,1,2,3],
                               UWB_ID2_arr=[0,1,2,3],
                               cfg=cfg,
                               result_dir=str(RES_DATA_DIR + '/' + sequence + '-eval-outliers/'),
                               prefix='',
                               save_plot=True,
                               show_plot=True,
                               save_statistics=True,
                               plot_timestamps=True,
                               plot_ranges=True,
                               plot_ranges_sorted=True,
                               plot_error=True,
                               plot_histogram=True,
                               verbose=True)


if __name__ == "__main__":
    RangeEvaluationTrail.test_eval()
    #RangeEvaluationTrail.test_range_gen_outliers()
    pass