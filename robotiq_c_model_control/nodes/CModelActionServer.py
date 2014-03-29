#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, SRI International.
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
# Copyright (c) 2014, SRI International.
# Revision $Id$

"""@package docstring
ROS node providing an actionlib action server for controling a Robotiq
C-Model gripper using the Modbus TCP protocol.

The script takes as an argument the IP address of the gripper. It
initializes a baseCModel object and adds a comModbusTcp client to it.
It implements an actionlib action server using the "GripperCommand"
action defined in control_msgs/action/GripperCommand.action.
"""

import roslib; roslib.load_manifest('robotiq_c_model_control')
roslib.load_manifest('robotiq_modbus_tcp')
import rospy
import robotiq_c_model_control.baseCModel
import robotiq_modbus_tcp.comModbusTcp
import os, sys
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
import actionlib
import control_msgs.msg
from robotiq_c_model_control.c_model_85_conversions import CModel85Conversions

class CModelActionServer(object):
    feedback_ = control_msgs.msg.GripperCommandFeedback()
    result_   = control_msgs.msg.GripperCommandResult()
    conversions_ = CModel85Conversions()

    def __init__(self, action_name, gripper_ip_address):

        self.gripper_ = robotiq_c_model_control.baseCModel.robotiqBaseCModel()
        self.gripper_.client = robotiq_modbus_tcp.comModbusTcp.communication()

        self.gripper_.client.connectToDevice(gripper_ip_address)

        self.activateGripper()

        self.action_name_ = action_name
        self.action_server_ = actionlib.SimpleActionServer(self.action_name_,
                                                           control_msgs.msg.GripperCommandAction,
                                                           execute_cb = self.execute,
                                                           auto_start = False)
        self.action_server_.start()

    def activateGripper(self):
        # When the gripper is first activated, it goes through a
        # calibration routine where it opens and closes fully.  This
        # function triggers this and waits for it to finish.
        msg_to_gripper = outputMsg.CModel_robot_output()
        msg_to_gripper.rACT = 1 # 1 = activate, 0 = reset
        msg_to_gripper.rGTO = 1 # 1 = go to position, 0 = stop
        msg_to_gripper.rATR = 0 # 1 = automatic release in case of e-stop, 0 = normal
        msg_to_gripper.rPR = 0 # all the way open
        msg_to_gripper.rSP = 255 # maximum speed
        msg_to_gripper.rFR = 0 # minimum force

        gripper_status = self.gripper_.getStatus()

        print("Gripper activating...")

        while gripper_status.gSTA != 3:
            self.gripper_.refreshCommand(msg_to_gripper)
            self.gripper_.sendCommand()
            gripper_status = self.gripper_.getStatus()

        print("Gripper activated.")

    def execute(self, goal):
        command = goal.command

        msg_to_gripper = outputMsg.CModel_robot_output()
        msg_to_gripper.rACT = 1 # 1 = activate, 0 = reset
        msg_to_gripper.rGTO = 1 # 1 = go to position, 0 = stop
        msg_to_gripper.rATR = 0 # 1 = automatic release in case of e-stop, 0 = normal
        msg_to_gripper.rPR = self.conversions_.dist_to_command(command.position)
        msg_to_gripper.rSP = 255 # always use maximum speed (for now)
        msg_to_gripper.rFR = self.conversions_.force_to_command(command.max_effort)

        self.gripper_.refreshCommand(msg_to_gripper)
        self.gripper_.sendCommand()

        rospy.sleep(0.05) # why?

        gripper_status = self.gripper_.getStatus()

        while (not rospy.is_shutdown() and
               gripper_status.gFLT == 0 and # no fault
               gripper_status.gOBJ != 2 and # haven't stopped due to object
               gripper_status.gOBJ != 3): # haven't gotten to requested position yet

            rospy.sleep(0.05) # why?

            self.gripper_.refreshCommand(msg_to_gripper)
            self.gripper_.sendCommand()

            rospy.sleep(0.05) # why?

            gripper_status = self.gripper_.getStatus()

        # print "status at end of loop:"
        # print repr(gripper_status)

        self.result_.position = self.conversions_.command_to_dist(gripper_status.gPO)
        self.result_.effort = command.max_effort

        self.result_.stalled = (gripper_status.gOBJ == 2)
        self.result_.reached_goal = (gripper_status.gOBJ == 3)

        if gripper_status.gOBJ == 3:
            self.action_server_.set_succeeded(self.result_)
        else:
            self.action_server_.set_failed(self.result_)

if __name__ == '__main__':
    try:
        rospy.init_node('CModelActionServer')
        if len(sys.argv) < 2:
            print("USAGE: CModelActionServer.py <IP-addr-of-gripper>")
        else:
            CModelActionServer(rospy.get_name(), sys.argv[1])
            rospy.spin()
    except rospy.ROSInterruptException: pass
