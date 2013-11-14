/****************************************************************************
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 *****************************************************************************/

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
