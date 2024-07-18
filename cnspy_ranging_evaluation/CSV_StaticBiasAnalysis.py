#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2023, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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
# BASED ON: https://github.com/aau-cns/cnspy_rosbag2csv
# just install "pip install cnspy-rosbag2csv"
########################################################################################################################
import math
from plistlib import Data

from docutils.nodes import topic

import cnspy_numpy_utils
import rosbag
import time
import os
import argparse
import yaml
import pandas as pandas
import csv
from tqdm import tqdm
import numpy as np
from numpy import linalg as LA, number
from cnspy_numpy_utils.numpy_statistics import *
from matplotlib import pyplot as plt
from cnspy_trajectory.PlotLineStyle import PlotLineStyle


class DataframeCfg:

    verbose = False
    label_timestamp = 't'
    label_ID1 = 'UWB_ID1'
    label_ID2 = 'UWB_ID2'
    label_range = 'range_raw'
    def __init__(self, label_timestamp='t',
                 label_ID1='UWB_ID1',
                 label_ID2='UWB_ID2', label_range = 'range_raw'):
        self.label_timestamp = label_timestamp
        self.label_ID1 = label_ID1
        self.label_ID2 = label_ID2
        self.label_range = label_range

class StaticBiasAnalysis:
    dict_err_ij = dict()

    def __init__(self, csv_fn, cfg_fn, verbose=False, df_cfg=DataframeCfg()):
        if not os.path.isfile(csv_fn):
            print("StaticBiasAnalysis: could not find file: %s" % csv_fn)
            return False
        cfg_fn = os.path.abspath(cfg_fn)
        if not os.path.isfile(cfg_fn):
            print("StaticBiasAnalysis: could not find file: %s" % cfg_fn)
            return False
        if verbose:
            print("StaticBiasAnalysis:")
            print("* csv_fn in name: " + str(csv_fn))
            print("* cfg_fn YAML file: \t " + str(cfg_fn))

        dict_cfg = None
        with open(cfg_fn, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)

            if "abs_anchor_positions" not in dict_cfg:
                print("[abs_anchor_positions] does not exist in fn=" + cfg_fn)
                return False
            print("Read successful")

        if verbose:
            print("configuration contains:")
            print("* abs_anchor_positions:" + str(dict_cfg["abs_anchor_positions"]))

        df = pandas.read_csv(csv_fn, sep='\s+|\,', comment='#', engine='python')

        if verbose:
            print("num measurements loaded:" + str(df.size))

        # Iterate all rows using DataFrame.iterrows()
        self.dict_err_ij = dict()
        for index, row in df.iterrows():
            ID1 = row[df_cfg.label_ID1]
            ID2 = row[df_cfg.label_ID2]
            range = row[df_cfg.label_range]
            t = row[df_cfg.label_timestamp]

            if range < 0 or range > 150.0:
                continue

            if ID2 not in dict_cfg["abs_anchor_positions"]:
                #if verbose:
                    #print("ID2=" + str(ID2) + " not in abs_anchor_positions")
                    #print(str(row))
                continue
            if ID1 not in dict_cfg["abs_anchor_positions"]:
                #if verbose:
                    #print("ID1=" + str(ID1) + " not in abs_anchor_positions")
                    #print(str(row))
                continue

            if ID1 not in self.dict_err_ij:
                self.dict_err_ij[ID1] = dict()
            if ID2 not in self.dict_err_ij[ID1]:
                self.dict_err_ij[ID1][ID2] = list()

            t_GA1 = np.array(dict_cfg["abs_anchor_positions"][ID1])
            t_GA2 = np.array(dict_cfg["abs_anchor_positions"][ID2])
            d_A1A2 = LA.norm(t_GA1 - t_GA2)
            self.dict_err_ij[ID1][ID2].append(range - d_A1A2)
        pass


    def plot_ranges(self, ID1, ID2, cfg_dpi=200, cfg_title="ranges", sorted=False, fig=None, ax=None,
                    color='r', label='err',
                    ls=PlotLineStyle(linestyle='-'),
                    save_fn="", result_dir="."):
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        r_err_arr = self.dict_err_ij[ID1][ID2]
        x_arr = range(len(r_err_arr))
        StaticBiasAnalysis.ax_plot_n_dim(ax, x_arr, r_err_arr,
                                      colors=color, labels=label, ls=ls)
        ax.grid(b=True)
        ax.set_ylabel('range')
        ax.set_xlabel('range sorted index')
        StaticBiasAnalysis.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)
        pass

    def plot_error_histogram(self, ID1, ID2,
                             cfg_dpi=200, fig=None, ax=None,
                             cfg_title="histogram",
                             print_stats=True):
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        r_err_arr = self.dict_err_ij[ID1][ID2]
        x_arr = range(len(r_err_arr))
        num_bins = 50
        stat = numpy_statistics(vNumpy=np.squeeze(np.asarray(r_err_arr)))
        if print_stats:
            cnspy_numpy_utils.numpy_statistics.print_statistics(stat, desc=str(ID1) + "-" + str(ID2) + " error")
            pass
        n, bins, patches = ax.hist(r_err_arr, num_bins)

        # add a 'best fit' line
        sigma = stat['std']
        mu = stat['mean']
        y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
             np.exp(-0.5 * (1 / sigma * (bins - mu)) ** 2))
        ax.plot(bins, y, '--')
        ax.set_ylabel('Probability density')
        ax.set_title(r'Hist: ' + cfg_title + ': $\mu$=' + str(round(mu, 3)) + ', $\sigma$=' + str(round(sigma, 3)))

        pass

    def plot_histograms(self, cfg_dpi=200, result_dir="."):
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

        for ID1, dict_err_j in self.dict_err_ij.items():
            fig = plt.figure(figsize=(20, 20), dpi=int(cfg_dpi))
            idx = 1
            n = len(dict_err_j)
            sqrt_n = math.ceil(math.sqrt(n))
            n_rows = sqrt_n
            n_cols = sqrt_n
            fig.suptitle('Histograms of ID=' +str(ID1), fontsize=16)
            for ID2, list_err in dict_err_j.items():
                cfg_title = str(str(ID1) + "-" + str(ID2))
                ax = fig.add_subplot(n_rows, n_cols, idx)
                self.plot_error_histogram(ID1=ID1, ID2=ID2, cfg_title=cfg_title,fig=fig, ax=ax)
                idx += 1

            StaticBiasAnalysis.show_save_figure(fig=fig, result_dir=result_dir, save_fn=str("Histograms_ID" + str(ID1)), show=False)
            fig.tight_layout()
            plt.show
            pass

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


def main():
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='CSV_StaticBiasAnalysis: extract a given pose topic and compute ranges to N abs_anchor_positions and M rel_tag_positions, which is stored into a CSV file')
    parser.add_argument('--csv_fn', help='input bag file', required=True)
    parser.add_argument('--cfg_fn',
                        help='YAML configuration file describing the setup: {abs_anchor_positions}',
                        default="config.yaml", required=True)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--label_timestamp', help='timestamp label in CSV', default='t')
    parser.add_argument('--label_range', help='range label in CSV', default='range_raw')
    parser.add_argument('--label_ID1', help='ID1 label in CSV', default='UWB_ID1')
    parser.add_argument('--label_ID2', help='ID2 label in CSV', default='UWB_ID2')
    tp_start = time.time()
    args = parser.parse_args()

    df_cfg = DataframeCfg(label_timestamp=args.label_timestamp,
                             label_ID1=args.label_ID1,
                             label_ID2=args.label_ID2,
                             label_range = args.label_range)

    sba = StaticBiasAnalysis(csv_fn=args.csv_fn, cfg_fn=args.cfg_fn, verbose=args.verbose, df_cfg=df_cfg)
    sba.plot_histograms()
    print(" ")
    print("finished after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass

