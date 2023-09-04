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

import rosbag
import time
import os
import argparse
import yaml
import csv
from tqdm import tqdm
import numpy as np
from numpy import linalg as LA, number
from spatialmath import UnitQuaternion, SO3, SE3, Quaternion, base

from std_msgs.msg import Header, Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped


def get_key_from_value(d, val):
    keys = [k for k, v in d.items() if v == val]
    if keys:
        return keys[0]
    return None


class HistoryBuffer:
    t_vec = []
    val_vec = []

    def __init__(self, dict_t=None):
        if dict_t is not None:
            self.set_dict(dict_t)

    def set_dict(self, dict_t):
        self.t_vec = []
        self.val_vec = []
        for key, val in dict_t.items():
            self.t_vec.append(key)
            self.val_vec.append(val)

    def set(self, t_vec_, val_vec_):
        assert (len(t_vec_) == len(val_vec_))
        idx = 0
        # sort values
        dict_t = dict()
        for t_ in self.t_vec:
            dict_t[t_] = val_vec_[idx]

        self.set_dict(dict_t)

    def get_idx_before_t(self, t):
        idx = 0
        for t_ in self.t_vec:
            if t_ >= t:
                return max(0, idx - 1)
            idx += 1
        return len(self.t_vec)

    def get_idx_after_t(self, t):
        idx = 0
        for t_ in self.t_vec:
            if t_ >= t:
                return max(0, idx)
            idx += 1
        # not in list
        return -1

    def get_idx_at_t(self, t):
        idx = 0
        for t_ in self.t_vec:
            if t_ == t:
                return max(0, idx)
            idx += 1
        # not in list
        return -1

    def exists_at_t(self, t):
        try:
            return self.t_vec.index(t)
        finally:
            return None

    def get_before_t(self, t):
        idx = self.get_idx_before_t(t)
        return [self.t_vec[idx], self.val_vec[idx]]

    def get_at_t(self, t):
        idx = self.get_idx_before_t(t)
        if idx != -1:
            return [self.t_vec[idx], self.val_vec[idx]]
        else:
            return [None, None]
        pass

    def get_after_t(self, t):
        idx = self.get_idx_after_t(t)
        if idx != -1:
            return [self.t_vec[idx], self.val_vec[idx]]
        else:
            return [None, None]
        pass


class ROSbag2CSV:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_in_name,
                bagfile_out_name,
                topic_pose,
                cfg,
                std_range=0.1,
                bias_offset=0,
                bias_range=1,
                use_header_timestamp = False,
                verbose=False):
        if not os.path.isfile(bagfile_in_name):
            print("ROSbag2CSV: could not find file: %s" % bagfile_in_name)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSbag2CSV: could not find file: %s" % cfg)
            return False

        bagfile_out_name = os.path.abspath(bagfile_out_name)

        if verbose:
            print("ROSbag2CSV:")
            print("* bagfile in name: " + str(bagfile_in_name))
            print("* bagfile out name: " + str(bagfile_out_name))
            print("* topic: \t " + str(topic_pose))
            print("* cfg YAML file: \t " + str(cfg))
            print("* std_range: " + str(std_range))
            print("* bias_offset: " + str(bias_offset))
            print("* bias_range: " + str(bias_range))
            print("* use_header_timestamp: " + str(use_header_timestamp))
        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_in_name)
        except:
            if verbose:
                print("ROSbag2CSV: Unexpected error!")

            return False

        dict_cfg = None
        with open(cfg, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)

            if "rel_tag_positions" not in dict_cfg:
                print("[rel_tag_positions] does not exist in fn=" + cfg)
                return False
            if "tag_topics" not in dict_cfg:
                print("[tag_topics] does not exist in fn=" + cfg)
                return False
            if "abs_anchor_positions" not in dict_cfg:
                print("[abs_anchor_positions] does not exist in fn=" + cfg)
                return False
            print("Read successful")

        if verbose:
            print("configuration contains:")
            print("* rel_tag_positions:" + str(dict_cfg["rel_tag_positions"]))
            print("* tag_topics:" + str(dict_cfg["tag_topics"]))
            print("* abs_anchor_positions:" + str(dict_cfg["abs_anchor_positions"]))

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSbag2CSV: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## create csv file according to the topic names:
        topicName = topic_pose
        if topicName[0] != '/':
            print("ROSbag2CSV: Not a proper topic name: %s (should start with /)" % topicName)
            return False

        ## check if desired pose topic is  in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']
        topicName = topic_pose
        found = False
        for topic_info in bag_topics:
            if topic_info['topic'] == topicName:
                found = True
        if not found:
            print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        ## check if desired topics are in the bag file:
        for key, val in dict_cfg["tag_topics"].items():
            found = False
            for topic_info in bag_topics:
                if topic_info['topic'] == val:
                    found = True
            if not found:
                print("# WARNING: desired topic [" + str(val) + "] is not in bag file!")

        if verbose:
            print("\nROSbag2CSV: num messages " + str(num_messages))

        dict_poses = dict()
        round_decimals = 6
        ## extract the desired topics from the BAG file
        try:  # else already exists
            print("ROSbag2CSV: extracting poses...")
            for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                if topic == topic_pose:
                    T_GLOBAL_BODY = SE3()
                    if hasattr(msg, 'header') and hasattr(msg, 'pose'):  # POSE_STAMPED
                        t = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                        q_GB = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z]
                        T_GLOBAL_BODY = SE3.Rt(Quaternion(q_GB).unit().R, t, check=False)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'transform'):
                        t = np.array(
                            [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
                        q_GB = [msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
                                msg.transform.rotation.z]
                        T_GLOBAL_BODY = SE3.Rt(Quaternion(q_GB).unit().R, t, check=False)
                    else:
                        print("\nROSbag2CSV: unsupported message " + str(msg))
                        continue
                    timestamp = round(msg.header.stamp.to_sec(), round_decimals)
                    dict_poses[timestamp] = T_GLOBAL_BODY
                pass
        except AssertionError as error:
            print(error)
            print("ROSbag2CSV: Unexpected error while reading the bag file!\n * try: $ rosbag fix <bagfile> <fixed>")
            return False

        hist_poses = HistoryBuffer(dict_t=dict_poses)

        noise_range_arr = np.random.normal(0, std_range, size=num_messages)  # 1000 samples with normal distribution
        idx = 0
        ## extract the desired topics from the BAG file
        try:  # else already exists
            print("ROSbag2CSV: computing new range measurements...")
            with rosbag.Bag(bagfile_out_name, 'w') as outbag:
                for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                    if topic in dict_cfg["tag_topics"].values():
                        TAG_ID = get_key_from_value(dict_cfg["tag_topics"], topic)
                        text = str("topic: " + str(topic) + " has wrong expected tag id! expected=" + str(
                            TAG_ID) + " got=" + str(msg.UWB_ID1))
                        assert (int(TAG_ID) == int(msg.UWB_ID1)), text

                        t_bt = dict_cfg["rel_tag_positions"][TAG_ID]
                        T_BODY_TAG = SE3(np.array(t_bt))
                        timestamp = round(msg.header.stamp.to_sec(), round_decimals)
                        if hist_poses.exists_at_t(timestamp) is None:
                            # interpolate between poses
                            [t1, T_GLOBAL_BODY_T1] = hist_poses.get_before_t(timestamp)
                            [t2, T_GLOBAL_BODY_T2] = hist_poses.get_after_t(timestamp)
                            dt = t2 - t1
                            dt_i = timestamp - t1
                            i = dt_i / dt

                            # interpolate between poses:
                            vec_t1_t2 = T_GLOBAL_BODY_T1.delta(T_GLOBAL_BODY_T2)
                            T_GLOBAL_BODY = T_GLOBAL_BODY_T1 * SE3.Delta(vec_t1_t2 * i)
                            pass
                        else:
                            T_GLOBAL_BODY = hist_poses.get_at_t(timestamp)

                        T_GLOBAL_TAG = T_GLOBAL_BODY * T_BODY_TAG

                        if msg.UWB_ID2 in dict_cfg["abs_anchor_positions"].keys():
                            t_GA = np.array(dict_cfg["abs_anchor_positions"][msg.UWB_ID2])
                            t_TA = T_GLOBAL_TAG.t - t_GA
                            d_TA = LA.norm(t_TA)
                            msg.range_raw = bias_range * d_TA + bias_offset + noise_range_arr[idx]
                            msg.range_corr = d_TA
                            msg.R = std_range*std_range
                            idx += 1
                            pass

                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                        pass
                    else:
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                    pass
        except AssertionError as error:
            print(error)
            print("ROSbag2CSV: Unexpected error while creating bag file")
            return False

        if verbose:
            print("\nROSbag2CSV: " + str(idx) + " range measurements modified!")

        bag.close()
        return True


if __name__ == "__main__":
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_TrueRanges: extract a given pose topic and compute ranges to N abs_anchor_positions and M rel_tag_positions, which is stored into a CSV file')
    parser.add_argument('--bagfile_in', help='input bag file', required=True)
    parser.add_argument('--bagfile_out', help='output bag file', default="")
    parser.add_argument('--topic_pose', help='desired topic', required=True)
    parser.add_argument('--cfg',
                        help='YAML configuration file describing the setup: {tag_postions, anchor_positions, tag_topics}',
                        default="config.yaml", required=True)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--std_range',
                        help='standard deviation of generated measurements: z = d + white_noise(std_range)',
                        default=0.1)
    parser.add_argument('--bias_offset', help='constant offset added to generated measurements: z = d + bias_offset',
                        default=0.0)
    parser.add_argument('--bias_range',
                        help='range-based biased multiplied to generated measurements: z = bias_range * d', default=1.0)
    parser.add_argument('--use_header_timestamp', action='store_true',
                        help='overwrites the bag time with the header time stamp', default=False)
    tp_start = time.time()
    args = parser.parse_args()

    if ROSbag2CSV.extract(bagfile_in_name=args.bagfile_in,
                          bagfile_out_name=args.bagfile_out,
                          topic_pose=str(args.topic_pose), cfg=args.cfg,
                          verbose=args.verbose, std_range=float(args.std_range),
                          bias_offset=float(args.bias_offset),
                          bias_range=float(args.bias_range),
                          use_header_timestamp=args.use_header_timestamp):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
