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
from tqdm import tqdm
import numpy as np
from numpy import linalg as LA
from spatialmath import UnitQuaternion, SO3, SE3
from spatialmath.base.quaternions import qslerp
from cnspy_trajectory.ROSBag_Pose import ROSBag_Pose

from cnspy_trajectory.HistoryBuffer import get_key_from_value

class ROSbag_TrueRanges:
    def __init__(self):
        pass



    @staticmethod
    def extract(bagfile_in_name,
                bagfile_out_name,
                cfg,
                stddev_range=0.1,
                bias_offset=0,  # gamma
                bias_range=1,   # beta
                perc_outliers=0.0,
                stddev_outlier=0.5,
                use_header_timestamp=False,
                verbose=False):
        if not os.path.isfile(bagfile_in_name):
            print("ROSbag_TrueRanges: could not find file: %s" % bagfile_in_name)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSbag_TrueRanges: could not find file: %s" % cfg)
            return False

        bagfile_out_name = os.path.abspath(bagfile_out_name)

        if verbose:
            print("ROSbag_TrueRanges:")
            print("* bagfile in name: " + str(bagfile_in_name))
            print("* bagfile out name: " + str(bagfile_out_name))
            print("* cfg YAML file: \t " + str(cfg))
            print("* std_range: " + str(stddev_range))
            print("* bias_offset: " + str(bias_offset))
            print("* bias_range: " + str(bias_range))
            print("* perc_outliers: " + str(perc_outliers))
            print("* outlier_stddev: " + str(stddev_outlier))
            print("* use_header_timestamp: " + str(use_header_timestamp))
        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_in_name)
        except:
            if verbose:
                print("ROSbag_TrueRanges: Unexpected error!")

            return False

        ## create result dir:
        [root, ext] = os.path.splitext(bagfile_out_name)
        [head, tail] = os.path.split(root)
        try:  # else already exists
            os.makedirs(head)
        except:
            pass

        if verbose:
            print("* result_dir: \t " + str(head))


        pose_topics = dict()
        tag_ids = []
        dict_cfg = None
        with open(cfg, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)

            if "rel_tag_positions" not in dict_cfg:
                print("[rel_tag_positions] does not exist in fn=" + cfg)
            if "tag_topics" not in dict_cfg:
                print("[tag_topics] does not exist in fn=" + cfg)
            else:
                for key, val in dict_cfg["tag_topics"].items():
                    tag_ids.append(int(key))
            if "anchor_topics" not in dict_cfg:
                print("[anchor_topics] does not exist in fn=" + cfg)
            if "abs_anchor_positions" not in dict_cfg:
                print("[abs_anchor_positions] does not exist in fn=" + cfg)
            # get the association between tag topics and the pose_topic(s)
            if "pose_topic" not in dict_cfg and "pose_topics" not in dict_cfg:
                print("ERROR [pose_topic] or [pose_topics] does not exist in fn=" + cfg)
                return False
            elif "pose_topic" in dict_cfg:
                for key in tag_ids:
                    pose_topics[key] = dict_cfg["pose_topic"]
            elif "pose_topics" in dict_cfg:
                pose_topics = dict_cfg["pose_topics"]
            print("Read successful")

        if verbose:
            print("configuration contains:")
            if "rel_tag_positions" in dict_cfg:
                print("* rel_tag_positions:" + str(dict_cfg["rel_tag_positions"]))
            if "tag_topics" in dict_cfg:
                print("* tag_topics:" + str(dict_cfg["tag_topics"]))
            if "anchor_topics" in dict_cfg:
                print("* anchor_topics:" + str(dict_cfg["anchor_topics"]))
            if "abs_anchor_positions" in dict_cfg:
                print("* abs_anchor_positions:" + str(dict_cfg["abs_anchor_positions"]))
            print("* pose_topics:" + str(pose_topics))

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSbag_TrueRanges: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## create csv file according to the topic names:
        for key,topic_pose in pose_topics.items():
            if topic_pose[0] != '/':
                print("ROSbag_TrueRanges: Not a proper topic name: %s (should start with /)" % topic_pose)
                return False

        ## check if desired pose topic is  in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']

        ## check if desired topics are in the bag file:
        found = False
        for key, topic_pose in pose_topics.items():
            for topic_info in bag_topics:
                if topic_info['topic'] == topic_pose:
                    found = True
            if not found:
                print("# WARNING: desired topic [" + str(topic_pose) + "] is not in bag file!")

        if "tag_topics" in dict_cfg:
            for key, val in dict_cfg["tag_topics"].items():
                found = False
                for topic_info in bag_topics:
                    if topic_info['topic'] == val:
                        found = True
                if not found:
                    print("# WARNING: desired topic [" + str(val) + "] is not in bag file!")
        if "anchor_topics" in dict_cfg:
            for key, val in dict_cfg["anchor_topics"].items():
                found = False
                if val != "":
                    for topic_info in bag_topics:
                        if topic_info['topic'] == val:
                            found = True
                    if not found:
                        print("# WARNING: desired topic [" + str(val) + "] is not in bag file!")

        if verbose:
            print("\nROSbag_TrueRanges: num messages " + str(num_messages))

        round_decimals=6
        # for each unique topic_pose extract a history...
        unique_pose_topics = set(pose_topics.values())
        dict_hist_poses = dict()
        for topic_pose in unique_pose_topics:
            dict_hist_poses[topic_pose] = ROSBag_Pose.extract_pose(bag, num_messages, topic_pose_body=topic_pose,
                                                                   round_decimals=round_decimals)

        noise_range_arr = np.random.normal(0, stddev_range, size=num_messages)  # 1000 samples with normal distribution

        if perc_outliers > 0.0:
            perc_outliers = min(perc_outliers, 1.0)
            # generating offset for outliers
            outlier_offset_arr = np.random.normal(0, stddev_outlier, size=num_messages) + stddev_outlier
            indices = np.arange(num_messages)
            np.random.shuffle(indices)

            num_inliers = int(num_messages*(1.0-perc_outliers))
            if num_inliers:
                inlier_indices = indices[np.array(range(num_inliers))]
                # remove the offset from the inliers
                outlier_offset_arr[inlier_indices] = 0
        else:
            outlier_offset_arr = np.zeros(num_messages)

        idx = 0
        cnt_T2A = 0
        cnt_A2T = 0
        cnt_A2A = 0
        cnt_T2T = 0
        ## extract the desired topics from the BAG file
        try:  # else already exists
            print("ROSbag_TrueRanges: computing new range measurements...")
            with rosbag.Bag(bagfile_out_name, 'w') as outbag:
                for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                    if "tag_topics" in dict_cfg and topic in dict_cfg["tag_topics"].values():
                        msg_range_raw=0
                        msg_range_corr=0
                        msg_R=0

                        msg_id1, msg_id2 = ROSbag_TrueRanges.get_msg_ids(msg)

                        if (int(msg_id1) not in dict_cfg["tag_topics"].keys()):
                            #text = str("topic: " + str(topic) + " has wrong expected tag id! expected=" + str( TAG_ID1) + " got=" + str(msg_id1))
                            continue

                        t_bt1 = dict_cfg["rel_tag_positions"][msg_id1]
                        T_BODY_TAG1 = SE3()
                        T_BODY_TAG1.t = (np.array(t_bt1))

                        T_GLOBAL_BODY = None
                        timestamp = round(msg.header.stamp.to_sec(), round_decimals)
                        interpol = False

                        hist_poses = dict_hist_poses[pose_topics[msg_id1]]

                        T_GLOBAL_BODY = ROSBag_Pose.get_pose(hist_poses, timestamp)
                        if T_GLOBAL_BODY is None:
                            if verbose:
                                print("* skip measurement from topic=" + topic + "at t=" + str(timestamp))
                            continue

                        T_GLOBAL_TAG1 = T_GLOBAL_BODY * T_BODY_TAG1

                        # T2A
                        if "abs_anchor_positions" in dict_cfg and msg_id2 in dict_cfg["abs_anchor_positions"].keys():
                            t_GA2 = np.array(dict_cfg["abs_anchor_positions"][msg_id2])
                            t_TA = T_GLOBAL_TAG1.t - t_GA2
                            d_TA = LA.norm(t_TA)
                            msg_range_raw = bias_range * d_TA + bias_offset + noise_range_arr[idx] + outlier_offset_arr[idx]
                            msg_range_corr = d_TA
                            msg_R = stddev_range * stddev_range
                            idx += 1
                            cnt_T2A += 1
                            pass
                        # T2T
                        elif msg_id2 in dict_cfg["tag_topics"].keys():
                            t_bt2 = dict_cfg["rel_tag_positions"][msg_id2]
                            T_BODY_TAG2 = SE3()
                            T_BODY_TAG2.t = (np.array(t_bt2))

                            if pose_topics[msg_id1] == pose_topics[msg_id2]:
                                # tag2 on the same rigid body
                                T_GLOBAL_TAG2 = T_GLOBAL_BODY * T_BODY_TAG2
                            else:
                                # tag2 on different rigid body
                                hist_poses2 = dict_hist_poses[pose_topics[msg_id2]]
                                T_GLOBAL_BODY2 = ROSBag_Pose.get_pose(hist_poses2, timestamp)
                                if T_GLOBAL_BODY2 is None:
                                    if verbose:
                                        print("* skip measurement from topic=" + topic + " at t=" + str(timestamp))
                                    continue
                                T_GLOBAL_TAG2 = T_GLOBAL_BODY2 * T_BODY_TAG2

                            t_TT = T_GLOBAL_TAG1.t - T_GLOBAL_TAG2.t
                            d_T1_T2 = LA.norm(t_TT)
                            msg_range_raw = bias_range * d_T1_T2 + bias_offset + noise_range_arr[idx] + outlier_offset_arr[idx]
                            msg_range_corr = d_T1_T2
                            msg_R = stddev_range * stddev_range
                            idx += 1
                            cnt_T2T += 1
                            pass

                        msg = ROSbag_TrueRanges.replace_measurements(msg, msg_R, msg_range_corr, msg_range_raw)
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                        pass
                    elif "anchor_topics" in dict_cfg and topic in dict_cfg["anchor_topics"].values():
                        msg_range_raw=0
                        msg_range_corr=0
                        msg_R=0

                        msg_id1, msg_id2 = ROSbag_TrueRanges.get_msg_ids(msg, msg_id1, msg_id2)

                        if (int(msg_id1) not in dict_cfg["anchor_topics"].keys()):
                            #text = str("topic: " + str(topic) + " has wrong expected anchor id! expected=" + str(ANCHOR_ID1) + " got=" + str(msg_id1))
                            continue

                        t_GA1 = np.array(dict_cfg["abs_anchor_positions"][msg_id1])

                        # A2A
                        if msg_id2 in dict_cfg["abs_anchor_positions"].keys():
                            t_GA2 = np.array(dict_cfg["abs_anchor_positions"][msg_id2])
                            d_AA = LA.norm(t_GA1 - t_GA2)
                            msg_range_raw = bias_range * d_AA + bias_offset + noise_range_arr[idx] + outlier_offset_arr[idx]
                            msg_range_corr = d_AA
                            msg_R = stddev_range * stddev_range
                            idx += 1
                            cnt_A2A += 1
                            pass
                        # A2T
                        elif "tag_topics" in dict_cfg and msg_id2 in dict_cfg["tag_topics"].keys():
                            t_bt1 = dict_cfg["rel_tag_positions"][msg_id2]
                            T_BODY_TAG1 = SE3()
                            T_BODY_TAG1.t = (np.array(t_bt1))
                            timestamp = round(msg.header.stamp.to_sec(), round_decimals)
                            hist_poses = dict_hist_poses[pose_topics[msg_id2]]
                            T_GLOBAL_BODY = ROSBag_Pose.get_pose(hist_poses, timestamp)
                            if T_GLOBAL_BODY is None:
                                if verbose:
                                    print("* skip measurement from topic=" + topic + " at t=" + str(timestamp))
                                continue

                            T_GLOBAL_TAG1 = T_GLOBAL_BODY * T_BODY_TAG1
                            t_TA = T_GLOBAL_TAG1.t - t_GA1
                            d_AT = LA.norm(t_TA)
                            msg_range_raw = bias_range * d_AT + bias_offset + noise_range_arr[idx] + outlier_offset_arr[idx]
                            msg_range_corr = d_AT
                            msg_R = stddev_range * stddev_range
                            idx += 1
                            cnt_A2T += 1
                            pass

                        msg = ROSbag_TrueRanges.replace_measurements(msg, msg_R, msg_range_corr, msg_range_raw)
                        # write anchor topic
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                        pass
                    else:
                        # write other topic
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                    pass
        except AssertionError as error:
            print(error)
            print("ROSbag_TrueRanges: Unexpected error while creating bag file")
            return False

        if verbose:
            print("\nROSbag_TrueRanges: " + str(idx) + " range measurements modified!")
            print("* num. T2T = " + str(cnt_T2T))
            print("* num. T2A = " + str(cnt_T2A))
            print("* num. A2A = " + str(cnt_A2A))
            print("* num. A2T = " + str(cnt_A2T))
        bag.close()
        return True

    @staticmethod
    def replace_measurements(msg, msg_R, msg_range_corr, msg_range_raw):
        # REPLACE VALUES
        # https://github.com/aau-cns/uwb_msgs
        if hasattr(msg, 'header') and hasattr(msg, 'range_raw'):
            msg.range_raw = msg_range_raw
            msg.range_corr = msg_range_corr
            msg.R = msg_R
        #  https://github.com/decargroup/miluv/blob/main/uwb_ros/msg/RangeStamped.msg
        elif hasattr(msg, 'header') and hasattr(msg, 'range') and hasattr(msg, 'from_id'):
            msg.range = msg_range_raw
            msg.covariance = msg_R
        return msg

    @staticmethod
    def get_msg_ids(msg):
        # https://github.com/aau-cns/uwb_msgs/blob/main/msg/TwoWayRangeStamped.msg
        if hasattr(msg, 'header') and hasattr(msg, 'range_raw'):
            msg_id1 = msg.UWB_ID1
            msg_id2 = msg.UWB_ID2

        #  https://github.com/decargroup/miluv/blob/main/uwb_ros/msg/RangeStamped.msg
        elif hasattr(msg, 'header') and hasattr(msg, 'range') and hasattr(msg, 'from_id'):
            msg_id1 = msg.from_id
            msg_id2 = msg.to_id
        else:
            assert False, "ROSbag_TrueRanges: Unsupported ROS message format"
        return msg_id1, msg_id2


def main():
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_TrueRanges: compute at given range topics the true ranges to N abs_anchor_positions and M '
                    'rel_tag_positions, which is stored into a CSV file')
    parser.add_argument('--bagfile_in', help='input bag file', required=True)
    parser.add_argument('--bagfile_out', help='output bag file', default="")
    parser.add_argument('--cfg',
                        help='YAML configuration file describing the setup: {rel_tag_positions, abs_anchor_positions, pose_topics}',
                        default="config.yaml", required=True)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--std_range',
                        help='standard deviation of generated measurements: z = d + white_noise(std_range)',
                        default=0.1)
    parser.add_argument('--bias_offset', help='constant offset added to generated measurements: z = d + bias_offset',
                        default=0.0)
    parser.add_argument('--bias_range',
                        help='range-based biased multiplied to generated measurements: z = bias_range * d', default=1.0)
    parser.add_argument('--perc_outliers', help='specifies a percentage of generated outliers by modified the '
                                                'measurement: z = d + white_noise(std_range) + std_range',
                        default=0.0)
    parser.add_argument('--outlier_stddev', help='standard deviation of the outliers.',
                        default=1.0)
    parser.add_argument('--use_header_timestamp', action='store_true',
                        help='overwrites the bag time with the header time stamp', default=False)
    tp_start = time.time()
    args = parser.parse_args()

    if ROSbag_TrueRanges.extract(bagfile_in_name=args.bagfile_in, bagfile_out_name=args.bagfile_out, cfg=args.cfg,
                                 stddev_range=float(args.std_range), bias_offset=float(args.bias_offset),
                                 bias_range=float(args.bias_range), perc_outliers=float(args.perc_outliers),
                                 stddev_outlier=float(args.outlier_stddev),
                                 use_header_timestamp=args.use_header_timestamp, verbose=args.verbose):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
