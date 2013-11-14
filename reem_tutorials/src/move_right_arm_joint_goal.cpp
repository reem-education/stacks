/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
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

// NOTE: The contents of this file are an adaptation of a similar tutorial of the pr2_arm_navigation_tutorials package:
// http://ros.org/wiki/pr2_arm_navigation_tutorials

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>

int main(int argc, char **argv)
{
  // Init the ROS node
  ros::init (argc, argv, "move_right_arm_joint_goal_test");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Action client for sending motion planing requests
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm_torso", true);

  // Wait for the action client to be connected to the server
  ROS_INFO("Connecting to server...");
  if (!move_arm.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR("Timed-out waiting for the move_arm action server.");
    return EXIT_FAILURE;
  }
  ROS_INFO("Connected to server.");

  // Prepare motion plan request with joint-space goal
  arm_navigation_msgs::MoveArmGoal goal;
  std::vector<std::string> names(9);
  names[0] = "torso_1_joint";
  names[1] = "torso_2_joint";
  names[2] = "arm_right_1_joint";
  names[3] = "arm_right_2_joint";
  names[4] = "arm_right_3_joint";
  names[5] = "arm_right_4_joint";
  names[6] = "arm_right_5_joint";
  names[7] = "arm_right_6_joint";
  names[8] = "arm_right_7_joint";

  goal.motion_plan_request.group_name = "right_arm_torso";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goal.motion_plan_request.planner_id= std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

  for (unsigned int i = 0 ; i < goal.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goal.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.05;
    goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.05;
  }

  goal.motion_plan_request.goal_constraints.joint_constraints[0].position =  0.0;
  goal.motion_plan_request.goal_constraints.joint_constraints[1].position =  0.0;
  goal.motion_plan_request.goal_constraints.joint_constraints[2].position =  1.2;
  goal.motion_plan_request.goal_constraints.joint_constraints[3].position =  0.15;
  goal.motion_plan_request.goal_constraints.joint_constraints[4].position = -1.5708;
  goal.motion_plan_request.goal_constraints.joint_constraints[5].position =  1.3;
  goal.motion_plan_request.goal_constraints.joint_constraints[6].position =  0.0;
  goal.motion_plan_request.goal_constraints.joint_constraints[7].position =  0.0;
  goal.motion_plan_request.goal_constraints.joint_constraints[8].position =  0.0;

  // Send motion plan request
  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goal);
    finished_within_time = move_arm.waitForResult(ros::Duration(15.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving joint-space goal.");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}
