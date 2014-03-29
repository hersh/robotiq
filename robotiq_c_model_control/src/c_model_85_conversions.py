#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, SRI, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2014, SRI, Inc.

import robotiq_c_model_control.range_control.

class CModel85Conversions:
    def __init__(self):
        # distance is finger separation, which is in the opposite direction from the command values.
        # Useful command values don't go all the way to 0 and 255, so we parameterize those as well.
        self.min_dist_ = 0
        self.min_dist_command_ = 230
        self.max_dist_ = 87
        self.max_dist_command_ = 13

        # For speed and force, the useful command values go all the
        # way from 0 to 255, so we don't record them here.
        self.min_speed_ = 0.013 # m/s
        self.max_speed_ = 0.100

        self.min_force_ = 30    # Newtons
        self.max_force_ = 100

    def dist_to_command(distance):
        dist_range = self.max_dist_ - self.min_dist_
        command_range = self.max_dist_command_ - self.min_dist_command_
        ratio = (distance - self.min_dist_) / dist_range
        return ratio * command_range + self.min_dist_command_

    def command_to_dist(command):
        dist_range = self.max_dist_ - self.min_dist_
        command_range = self.max_dist_command_ - self.min_dist_command_
        ratio = (command - self.min_dist_command_) / command_range
        return ratio * dist_range + self.min_dist_
        
