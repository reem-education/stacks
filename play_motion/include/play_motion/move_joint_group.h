/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Adolfo Rodriguez Tsouroukdissian. */
/** \author Paul Mathieu.                     */

#ifndef MOVEJOINTGROUP_H
#define MOVEJOINTGROUP_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace play_motion
{
  /** Move a joint group to a given pose.
   * The pose will be reached within a specified duration with zero velocity.
   */
  class MoveJointGroup
  {
    private:
      typedef actionlib::SimpleActionClient
        <control_msgs::FollowJointTrajectoryAction>     ActionClient;
      typedef control_msgs::FollowJointTrajectoryGoal   ActionGoal;
      typedef control_msgs::FollowJointTrajectoryResult ActionResult;
      typedef boost::shared_ptr<const ActionResult>     ActionResultPtr;
      typedef boost::function<void(int)>                Callback;
      typedef std::vector<std::string>                  JointNames;

    public:
      struct TrajPoint
      {
        std::vector<double> positions;
        std::vector<double> velocities;
        ros::Duration       time_from_start;
      };

      MoveJointGroup(const std::string& controller_name, const JointNames& joint_names);
      bool sendGoal(const std::vector<TrajPoint>& traj, const ros::Duration& duration);
      bool isControllingJoint(const std::string& joint_name);
      bool isIdle() { return !busy_; }
      void cancel() { busy_ = false; client_.cancelAllGoals(); }
      void setCallback(const Callback& cb) { active_cb_ = cb; }

      const std::vector<std::string>& getJointNames() const {return joint_names_;}
      actionlib::SimpleClientGoalState getState() {return client_.getState();}
      const std::string& getName() { return controller_name_; }

    private:
      void alCallback();

      bool                     busy_;
      ros::NodeHandle          nh_;               ///< Default node handle.
      std::string              controller_name_;  ///< Controller name.
      JointNames               joint_names_;      ///< Names of controller joints.
      ActionClient             client_;           ///< Action client used to trigger motions.
      Callback                 active_cb_;        ///< Call this when we are called back from the controller
      ros::Timer               configure_timer_;  ///< To periodically check for controller actionlib server
  };
}

#endif
