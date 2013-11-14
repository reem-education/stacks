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

/** \author Jordi Pages. */

/**
 * @file
 *
 * @brief class allowing to move the following kinematic chains in REEM:
 *
 *        torso, right arm, left arm, head, right hand
 *
 */

#ifndef _PAL_REEM_UPPERBODY_VISUAL_SERVO_MOVE_PARTS_
#define _PAL_REEM_UPPERBODY_VISUAL_SERVO_MOVE_PARTS_

// ROS headers
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>

// Eigen headers
#include <Eigen/Core>

// Std C++ headers
#include <vector>

namespace pal {

  class JointGroupCommander
  {
  public:

    /**
     * @brief JointGroupCommander
     * @param controllerName
     */
    JointGroupCommander(const std::string& controllerName);

    virtual ~JointGroupCommander();

    /**
     * @brief send send command to the joint controller so that every joint in the chain
     *        reaches a desired configuration
     * @param jointsDesiredState
     * @param endVelocity
     * @param duration max time given to reach the desired pose
     */
    void send(const std::vector<double>& jointsDesiredState,
              const std::vector<double>& endVelocities,
              const ros::Duration& duration);


  private:

    /**
     * @brief updateJointNames obtain in _jointNames the names of the joints moved by the controller
     */
    void updateJointNames();

    ros::NodeHandle _nh;

    ros::Publisher _jointTrajPub;

    std::string _controllerName;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> _actionClient;
    std::vector<std::string> _jointNames; //names of controlled joints
  };


  class MoveParts
  {
  public:

    MoveParts();
    virtual ~MoveParts();

    enum handPose { HAND_OPENED = 0, HAND_PREGRASP = 1, HAND_GRASP = 2 };

    /**
     * @brief moveRightHand moves the rigth hand to a pre-defined pose
     * @param pose
     */
    void moveRightHand(handPose pose);

    enum robotPart { TORSO = 0, RIGHT_ARM = 1, HEAD = 2 };

    void move(robotPart part,
              const std::vector<double>& jointsDesiredState,
              const std::vector<double>& endVelocities,
              const ros::Duration& duration);

  protected:

    /**
     * @brief initHandTrajectories init _handTrajectories for each handPose defined
     */
    void initHandTrajectories();

    ros::NodeHandle _nh;

    bool _isSimulation;

    //right hand publisher to the joint trajectory controller
    ros::Publisher _jointTrajRightHand_pub;

    JointGroupCommander _torsoCommand;
    JointGroupCommander _rightArmCommand;
    JointGroupCommander _headCommand;

    std::vector<trajectory_msgs::JointTrajectory> _handTrajectories;

  };

} //pal

#endif //_PAL_REEM_UPPERBODY_VISUAL_SERVO_MOVE_PARTS_
