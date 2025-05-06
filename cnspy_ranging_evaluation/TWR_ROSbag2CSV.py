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

from std_msgs.msg import Header, Time
#from uwb_msgs.msg import TwoWayRangeStamped

class TWR_ROSbag2CSV:
    def __init__(self):
        pass

    #
    # extract
    #
    @staticmethod
    def extract(bagfile_name, topic_list, result_dir="", fn_list=[], verbose=False
                ):
        if not os.path.isfile(bagfile_name):
            print("TWR_ROSbag2CSV: could not find file: %s" % bagfile_name)
            return False

        if len(topic_list) < 1:
            print("TWR_ROSbag2CSV: no topics specified!")
            return False

        if fn_list:
            if len(topic_list) != len(fn_list):
                print("TWR_ROSbag2CSV: topic_list and fn_list must have the same length!")
                return False

        if verbose:
            print("TWR_ROSbag2CSV:")
            print("* bagfile name: " + str(bagfile_name))
            print("* topic_list: \t " + str(topic_list))
            print("* filename_list: " + str(fn_list))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_name)
        except:
            if verbose:
                print("TWR_ROSbag2CSV: Unexpected error!")

            return False

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("TWR_ROSbag2CSV: Unexpected error, bag file might be empty!")
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
        for topicName in topic_list:

            if topicName != "" and topicName[0] != '/':
                print("TWR_ROSbag2CSV: Not a proper topic name: %s (should start with /)" % topicName)
                continue

            if not fn_list:
                filename = str(folder + '/') + str.replace(topicName[1:], '/', '_') + '.csv'
            else:
                fn = fn_list[idx]
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
                print("TWR_ROSbag2CSV: creating csv file: %s " % filename)

            idx = idx + 1

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']
        for topicName in topic_list:
            found = False
            if topicName == "":
                continue
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found = True

            if not found:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        if verbose:
            print("\nROSbag2CSV: num messages " + str(num_messages))

        ## extract the desired topics from the BAG file
        for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
            if topic in topic_list:
                if hasattr(msg, 'header') and hasattr(msg, 'range_raw'):  # STAMPED
                    file_writer = dict_file_writers[topic]

                    if not dict_header_written[topic]:
                        file_writer.writerow(["t", "range_raw", "range_corr", "R", "UWB_ID1", "UWB_ID2", "type_ID1", "type_ID2", "LOS"])
                        dict_header_written[topic] = True

                    # HINT: conversions:
                    content = ["%f" % msg.header.stamp.to_sec(), str(msg.range_raw), str(msg.range_corr), str(msg.R), str(msg.UWB_ID1), str(msg.UWB_ID2), str(msg.type_ID1), str(msg.type_ID2), str(msg.LOS)]

                    file_writer.writerow(content)

        ## CLEANUP:
        # close all csv files
        for topicName in topic_list:
            dict_csvfile_hdls[topicName].close()

        # check if a topic was found by checking if the topic header was written
        for topicName in topic_list:
            if not dict_header_written[topicName]:
                print("\nROSbag2CSV: \n\tWARNING topic [" + str(topicName) + "] was not in bag-file")
                print("\tbag file [" + str(bagfile_name) + "] contains: ")
                # print(info_dict['topics'])
                for t in info_dict['topics']:
                    print(t['topic'])
                return False

        if verbose:
            print("\nROSbag2CSV: extracting done! ")

        bag.close()
        return True

    #
    # extract_to_one
    #
    @staticmethod
    def extract_to_one(bagfile_name, topic_list, fn, result_dir="", ext="csv", verbose=False ):
        if not os.path.isfile(bagfile_name):
            print("TWR_ROSbag2CSV: could not find file: %s" % bagfile_name)
            return False

        if len(topic_list) < 1:
            print("TWR_ROSbag2CSV:extract_to_one() no topics specified!")
            return False

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_name)
        except:
            if verbose:
                print("TWR_ROSbag2CSV: Unexpected error!")

            return False

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("TWR_ROSbag2CSV: Unexpected error, bag file might be empty!")
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

        if not fn:
          filename = str(folder + '/') + 'all_ranges.csv'
        else:
          filename = fn

        if verbose:
            print("TWR_ROSbag2CSV.extract_to_one():")
            print("* bagfile name: " + str(bagfile_name))
            print("* topic_list: \t " + str(topic_list))
            print("* filename_list: " + str(filename))

        for topicName in topic_list:
            if topicName != '' and topicName[0] != '/':
                print("TWR_ROSbag2CSV: Not a proper topic name: %s (should start with /)" % topicName)
                continue

        [root, ext] = os.path.splitext(filename)
        [head, tail] = os.path.split(root)
        if ext:
            filename = str(folder + '/') + tail + ext
        else:
            filename = str(folder + '/') + tail + '.csv'

        csvfile = open(filename, 'w+')
        file_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
        file_writer.writerow(["t", "range_raw", "range_corr", "R", "UWB_ID1", "UWB_ID2", "type_ID1", "type_ID2", "LOS"])

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']
        for topicName in topic_list:
            found = False
            if topicName != "":
                for topic_info in bag_topics:
                    if topic_info['topic'] == topicName:
                        found = True

                if not found:
                    print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        if verbose:
            print("\nTWR_ROSbag2CSV: num messages " + str(num_messages))

        ## extract the desired topics from the BAG file
        for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
            if topic in topic_list:
                # https://github.com/aau-cns/uwb_msgs/blob/main/msg/TwoWayRangeStamped.msg
                if hasattr(msg, 'header') and hasattr(msg, 'range_raw'):  # TwoWayRangeStamped
                    # HINT: conversions:
                    content = ["%f" % msg.header.stamp.to_sec(), str(msg.range_raw), str(msg.range_corr), str(msg.R), str(msg.UWB_ID1), str(msg.UWB_ID2), str(msg.type_ID1), str(msg.type_ID2), str(msg.LOS)]
                    file_writer.writerow(content)

                #  https://github.com/decargroup/miluv/blob/main/uwb_ros/msg/RangeStamped.msg
                elif hasattr(msg, 'header') and hasattr(msg, 'range') and hasattr(msg, 'from_id'):  # RangeStamped
                    # HINT: conversions:
                    content = ["%f" % msg.header.stamp.to_sec(), str(msg.range), "0", str(msg.covariance), str(msg.from_id), str(msg.to_id), "U", "U", "1"]
                    file_writer.writerow(content)
                else:
                    assert False, "ROSbag_TrueRanges: Unsupported ROS message format"
        ## CLEANUP:
        csvfile.close()


        if verbose:
            print("\nTWR_ROSbag2CS.extract_to_one(): extracting done! ")

        bag.close()
        return True


def main():
    # test3: python3 TWR_ROSbag2CS.py --bagfile ../test/example.bag --topics /a01/ranging /a02/ranging--verbose --filenames ranges.csv
    parser = argparse.ArgumentParser(
        description='TWR_ROSbag2CSV: extract and store given topics of a rosbag into a CSV file')
    parser.add_argument('--bagfile', help='input bag file', default="not specified")
    parser.add_argument('--topics', nargs='*', help='desired topics', default=[])
    parser.add_argument('--filenames', nargs='*', help='csv filename of corresponding topic', default=[])
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)

    tp_start = time.time()
    args = parser.parse_args()

    res = False;
    if args.topics is not None and args.filenames is not None:
        if len(args.topics) is not len(args.filenames):
            print("\nTWR_ROSbag2CS.extract_to_one()")
            fn = ""
            if len(args.filenames) :
              fn  = args.filenames[0]
            res = TWR_ROSbag2CSV.extract_to_one(bagfile_name=args.bagfile,
                                                topic_list=args.topics,
                                                fn=fn,
                                                result_dir=args.result_dir,
                                                verbose=args.verbose)
        else:
            print("\nTWR_ROSbag2CS.extract()")

            res = TWR_ROSbag2CSV.extract(bagfile_name=args.bagfile,
                                         topic_list=args.topics,
                                         fn_list=args.filenames,
                                         result_dir=args.result_dir,
                                         verbose=args.verbose)

    if res:
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
