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
from numpy import linalg as LA
from spatialmath import UnitQuaternion, SO3, SE3, Quaternion, base

from std_msgs.msg import Header, Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped

class ROSbag_Pose2Ranges:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_name, topic, cfg, filename, result_dir="", fn_list=[], verbose=False
                ):
        """"
        Extracts a list of topic from a rosbag file and stores each topic in a file specified in "fn_list"


        Example:
        >> args.bagfile  = "example.bag"
        >> args.topics =  ["/CS_200_MAV1/estimated_poseWithCov",  "/pose_sensor/pose"]
        >> args.verbose = True
        >> args.result_dir = "./results"
        >> args.filenames = ["mav_PoseWithCov.csv", "sensor_PoseWithCov"]
        >> ROSbag_Pose2Ranges.extract(bagfile_name=args.bagfile, topic_list=args.topics,
                      fn_list=args.filenames, result_dir=args.result_dir,
                      verbose=args.verbose, fmt=CSVSpatialFormatType(args.format)):


        Input:
        bagfile_name -- file name of the rosbag
        topic_list -- list of topic names, these must start with a "/" (absolute topic name)
        result_dir  -- root directory the files to be created (defined in fn_list)
        fn_list -- list of file names;  (not the entire path!) just name with or without extension


        Output:
        res -- boolean about success
        """

        if not os.path.isfile(bagfile_name):
            print("ROSbag_Pose2Ranges: could not find file: %s" % bagfile_name)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSbag_Pose2Ranges: could not find file: %s" % cfg)
            return False

        if verbose:
            print("ROSbag_Pose2Ranges:")
            print("* bagfile name: " + str(bagfile_name))
            print("* topic: \t " + str(topic))
            print("* cfg: \t " + str(cfg))
            print("* filename: " + str(filename))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_name)
        except:
            if verbose:
                print("ROSbag_Pose2Ranges: Unexpected error!")

            return False

        dict_cfg = None
        with open(cfg, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)
            if "rel_tag_positions" not in dict_cfg:
                print("[rel_tag_positions] does not exist in fn=" + cfg)
                return False
            if "abs_anchor_positions" not in dict_cfg:
                print("[abs_anchor_positions] does not exist in fn=" + cfg)
                return False
            print("Read successful")
        if verbose:
            print("configuration contains:")
            print("rel_tag_positions:" + str(dict_cfg["rel_tag_positions"]))
            print("abs_anchor_positions:" + str(dict_cfg["abs_anchor_positions"]))

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSbag_Pose2Ranges: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## create result dir:
        if result_dir == "":
            folder = str(bagfile_name).replace(".bag", "")
        else:
            folder = result_dir

        folder = os.path.abspath(folder)
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass

        if verbose:
            print("* result_dir: \t " + str(folder))

        ## create csv file according to the topic names:
        dict_file_writers = dict()
        dict_header_written = dict()
        dict_csvfile_hdls = dict()
        idx = 0
        topicName = topic

        if topicName[0] != '/':
            print("ROSbag_Pose2Ranges: Not a proper topic name: %s (should start with /)" % topicName)
            return False

        if not filename:
            filename = str(folder + '/') + str.replace(topicName[1:], '/', '_') + '.csv'
        else:
            fn = filename
            [root, ext] = os.path.splitext(fn)
            [head, tail] = os.path.split(root)
            if ext:
                filename = str(folder + '/') + tail + ext
            else:
                filename = str(folder + '/') + tail + '.csv'

        csvfile = open(filename, 'w+')
        file_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
        dict_file_writers[topicName] = file_writer
        dict_header_written[topicName] = False
        dict_csvfile_hdls[topicName] = csvfile

        if verbose:
            print("ROSbag_Pose2Ranges: creating csv file: %s " % filename)

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']
        topicName = topic
        found = False
        for topic_info in bag_topics:
            if topic_info['topic'] == topicName:
                found = True

        if not found:
            print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        if verbose:
            print("\nROSbag_Pose2Ranges: num messages " + str(num_messages))

        ## extract the desired topics from the BAG file
        try:  # else already exists
            for topic_, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                if topic_ == topic:

                    T_GLOBAL_BODY= SE3()
                    if hasattr(msg, 'header') and hasattr(msg, 'pose'):  # POSE_STAMPED
                        t = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
                        q_GB = [msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]
                        T_GLOBAL_BODY = SE3.Rt(Quaternion(q_GB).unit().R, t, check=False)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'transform'):
                        t = np.array([msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z])
                        q_GB = [msg.transform.rotation.w,msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z]
                        T_GLOBAL_BODY = SE3.Rt(Quaternion(q_GB).unit().R, t, check=False)
                    else:
                        print("\nROSbag_Pose2Ranges: unsupported message " + str(msg))
                        continue

                    for TAG_ID, tag_pos in dict_cfg["rel_tag_positions"].items():
                        t_BT = np.array(tag_pos)

                        T_BODY_TAG = SE3(t_BT)
                        T_GLOBAL_TAG = T_GLOBAL_BODY * T_BODY_TAG

                        for ANCHOR_ID, anchor_pos in dict_cfg["abs_anchor_positions"].items():
                            t_GA = np.array(anchor_pos)
                            t_TA = T_GLOBAL_TAG.t - t_GA
                            d_TA = LA.norm(t_TA)

                            file_writer = dict_file_writers[topic]

                            if not dict_header_written[topic]:
                                file_writer.writerow(["t", "range_raw", "range_corr", "R", "UWB_ID1", "UWB_ID2", "type_ID1", "type_ID2", "LOS"])
                                dict_header_written[topic] = True

                            content = ["%f" % msg.header.stamp.to_sec(), str(d_TA), str(d_TA), str(0), str(TAG_ID), str(ANCHOR_ID), "T", "A", "1"]

                            file_writer.writerow(content)
        except:
            print("ROSbag_Pose2Ranges: Unexpected error while reading the bag file!\n * try: $ rosbag fix <bagfile> <fixed>")
            return False
        ## CLEANUP:
        # close all csv files
        dict_csvfile_hdls[topic].close()

        # check if a topic was found by checking if the topic header was written
        topicName = topic
        if not dict_header_written[topicName]:
            print("\nROSbag_Pose2Ranges: \n\tWARNING topic [" + str(topicName) + "] was not in bag-file")
            print("\tbag file [" + str(bagfile_name) + "] contains: ")
            # print(info_dict['topics'])
            for t in info_dict['topics']:
                print(t['topic'])
            return False

        if verbose:
            print("\nROSbag_Pose2Ranges: extracting done! ")

        bag.close()
        return True


def main():
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_Pose2Ranges: extract a given pose topic and compute ranges to N abs_anchor_positions and M rel_tag_positions, which is stored into a CSV file')
    parser.add_argument('--bagfile', help='input bag file', required=True)
    parser.add_argument('--topic', help='desired topic', required=True)
    parser.add_argument('--cfg', help='YAML configuration file describing the setup: {rel_tag_positions, abs_anchor_positions}', default="config.yaml", required=True)
    parser.add_argument('--filename', help='csv filename of corresponding topic', default="")
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)

    tp_start = time.time()
    args = parser.parse_args()

    if ROSbag_Pose2Ranges.extract(bagfile_name=args.bagfile, topic=str(args.topic), cfg=args.cfg,
                          filename=args.filename, result_dir=args.result_dir,
                          verbose=args.verbose
                          ):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
