#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2024, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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
from tqdm import tqdm
import numpy as np
from spatialmath import UnitQuaternion, SO3, SE3, Quaternion, base


from std_msgs.msg import Header, Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from cnspy_ranging_evaluation.HistoryBuffer import HistoryBuffer, get_key_from_value


class ROSBag_Pose:
    @staticmethod
    def extract_pose(bag, num_messages, topic_pose_body, round_decimals=6, T_BODY_SENSOR=None) -> HistoryBuffer:
        hist_poses = dict()
        ## extract the desired topics from the BAG file
        try:  # else already exists
            print("ROSBag_Pose.extract(): extracting pose...")
            cnt_poses = 0
            for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                if topic == topic_pose_body:
                    T_GLOBAL_BODY = None
                    if hasattr(msg, 'header') and hasattr(msg, 'pose') and hasattr(msg, 'twist'):  # nav_msgs/Odometry Message
                        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                        q_GB = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z]

                        q = UnitQuaternion(q_GB).unit()
                        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'pose'):  # POSE_STAMPED
                        t = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                        q_GB = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z]

                        q = UnitQuaternion(q_GB, norm=True)
                        T_GLOBAL_BODY = SE3.Rt(q.R, t, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'transform'):
                        t = np.array(
                            [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
                        q_GB = [msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
                                msg.transform.rotation.z]
                        q = UnitQuaternion(q_GB, norm=True)
                        T_GLOBAL_BODY = SE3.Rt(q.R, t, check=True)
                    else:
                        print("\nROSBag_Pose.extract(): unsupported message " + str(msg))
                        continue

                    if T_GLOBAL_BODY is not None:
                        timestamp = round(msg.header.stamp.to_sec(), round_decimals)
                        if T_BODY_SENSOR and isinstance(T_BODY_SENSOR, SE3):
                            hist_poses[timestamp] = T_GLOBAL_BODY * T_BODY_SENSOR
                        else:
                            hist_poses[timestamp] = T_GLOBAL_BODY
                        cnt_poses = cnt_poses + 1
                        pass
                pass

            if cnt_poses == 0:
                print("\nROSBag_Pose.extract(): no poses obtained!")
                return None
            else:
                print("\nROSBag_Pose.extract(): poses extracted: " + str(cnt_poses))
                pass
        except AssertionError as error:
            print(error)
            print(
                "ROSBag_Pose.extract(): Unexpected error while reading the bag file!\n * try: $ rosbag fix <bagfile> <fixed>")
            return None
        return HistoryBuffer(dict_t=hist_poses)

    @staticmethod
    def extract_poses(bag, num_messages, dict_topic_pose_body, dict_senor_topic_pose, round_decimals=6, dict_T_BODY_SENSOR=None) -> dict:
        dict_hist_poses = dict()  # map<topic<timestamp, SE3>>
        for id_, topic_ in dict_senor_topic_pose.items():
            dict_hist_poses[topic_] = dict()

        try:  # else already exists
            print("ROSBag_Pose: extracting poses...")
            cnt_poses = 0
            for topic_, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                if topic_ in dict_topic_pose_body.values():

                    id_topic = get_key_from_value(dict_topic_pose_body, topic_)
                    topic_sensor = dict_senor_topic_pose[id_topic]
                    T_GLOBAL_BODY = None
                    if hasattr(msg, 'header') and hasattr(msg, 'pose') and hasattr(msg, 'twist'):  # nav_msgs/Odometry Message
                        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                        q_GB = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z]

                        q = UnitQuaternion(q_GB).unit()
                        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'pose'):  # POSE_STAMPED
                        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                        q_GB = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z]

                        q = UnitQuaternion(q_GB).unit()
                        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'transform'):
                        p = np.array(
                            [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
                        q_GB = [msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
                                msg.transform.rotation.z]
                        q = UnitQuaternion(q_GB).unit()
                        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
                    else:
                        print("\nROSBag_Pose: unsupported message " + str(msg))
                        continue

                    if T_GLOBAL_BODY is not None:
                        timestamp = round(msg.header.stamp.to_sec(), round_decimals)

                        if dict_T_BODY_SENSOR and topic_sensor in dict_T_BODY_SENSOR.keys():
                            T_BODY_SENSOR = dict_T_BODY_SENSOR[topic_sensor]
                            dict_hist_poses[topic_sensor][timestamp] = T_GLOBAL_BODY * T_BODY_SENSOR
                        else:
                            dict_hist_poses[topic_sensor][timestamp] = T_GLOBAL_BODY
                        cnt_poses = cnt_poses + 1
                        pass
                pass

            if cnt_poses == 0:
                print("\nROSBag_Pose: no poses obtained!")
                return None
            else:
                print("\nROSBag_Pose: poses extracted: " + str(cnt_poses))

        except AssertionError as error:
            print(error)
            print(
                "ROSBag_Pose: Unexpected error while reading the bag file!\n * try: $ rosbag fix <bagfile> <fixed>")
            return None

        ## convert poses to History
        dict_history = dict() # map<topic, history>
        for topic_sensor, hist_poses in dict_hist_poses.items():
            dict_history[topic_sensor] = HistoryBuffer(dict_t=hist_poses)

        return dict_history
