#!/usr/bin/env python

#
# Software License Agreement (Modified BSD License)
#
#  Copyright (c) 2012, PAL Robotics, S.L.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of PAL Robotics, S.L. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

import roslib; roslib.load_manifest('blort_ros')
import re
import subprocess
from multiprocessing import Process
from geometry_msgs.msg import Pose
import sys
from call_SingleShot import singleShotClient
from pose_utils import *

# error in meters
max_error = Pose()
max_error.position.x = 0.3
max_error.position.y = 0.3
max_error.position.z = 0.3
max_error.orientation.x = 0.2
max_error.orientation.y = 0.2
max_error.orientation.z = 0.2
max_error.orientation.w = 0.2

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "First the path of the pose file to compare to, second parameter should be the number of tries!"
        exit(1)
    approxPose = readPose(sys.argv[1])
    print "'ve read approxPose : "
    print approxPose

    nTries = int(sys.argv[2])
    nFails = 0
    nSuccs = 0
    resultPoses = []

    for i in range(nTries):
        resultPose = singleShotClient()
        if resultPose == None:
            nFails += 1
        else:
            if poseValidate(approxPose, resultPose, max_error):
                 nSuccs += 1
                 resultPoses.append(resultPose)
            else:
                nFails += 1
    print '\nStatistics: nTries: '+ repr(nTries) + ', nFails: ' + repr(nFails) + ', nSuccs: ' + repr(nSuccs)
    if nSuccs > 0:
        print 'Average error:'
        print poseListAvgDiff(approxPose, resultPoses)

    print "\nBig Kahuna Burger. That's that Hawaiian burger joint."


