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
            idx += 1

        self.set_dict(dict_t)

    def get_idx_before_t(self, t):
        idx = 0
        for t_ in self.t_vec:
            if t_ >= t:
                # if idx = 0:  not in list
                return idx - 1
            idx += 1
        # not in list
        return -1

    def get_idx_after_t(self, t):
        idx = 0
        for t_ in self.t_vec:
            if t_ >= t:
                return idx
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
        if idx != -1 and idx < len(self.t_vec):
            return [self.t_vec[idx], self.val_vec[idx]]
        else:
            return [None, None]
        pass

    def get_at_t(self, t):
        idx = self.get_idx_at_t(t)
        if idx != -1 and idx < len(self.t_vec):
            return [self.t_vec[idx], self.val_vec[idx]]
        else:
            return [None, None]
        pass

    def get_after_t(self, t):
        idx = self.get_idx_after_t(t)
        if idx != -1 and idx < len(self.t_vec):
            return [self.t_vec[idx], self.val_vec[idx]]
        else:
            return [None, None]
        pass

