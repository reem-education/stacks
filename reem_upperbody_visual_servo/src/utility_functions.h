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

/** \author Don Joven Agravante
            Jordi Pages
*/

#ifndef __PAL_VISUAL_SERVO_UTILITY_FUNCTIONS__
#define __PAL_VISUAL_SERVO_UTILITY_FUNCTIONS__

// ROS headers
#include <tf_conversions/tf_kdl.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>

// KDL headers
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>

// Visp headers
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>

// Eigen headers
#include <Eigen/Core>


namespace utility_functions
{


  // Fill in the joint trajectory message for the RIGHT ARM
  void makeArmJointTrajMSG(trajectory_msgs::JointTrajectory &JTmsg,
                           KDL::JntArray &kdlArmJointsPosition,
                           vpColVector &qdot,
                           double time_from_start = 4.0);

  /**
   * @brief makeTorsoAndRightArmJointTrajMSG
   * @param kdlArmJointsPosition
   * @param qdot velocities of the 2 torso joints and the 7 right arm joints
   * @param torsoJointTrajMsg
   * @param rightArmJointTrajMsg
   * @param time_from_start
   * @param time_from_start_factor
   */
  void makeTorsoAndRightArmJointTrajMSG(const KDL::JntArray& kdlArmJointsPosition,
                                        const vpColVector &qdot,
                                        trajectory_msgs::JointTrajectory &torsoJointTrajMsg,
                                        trajectory_msgs::JointTrajectory &rightArmJointTrajMsg,
                                        double time_from_start,
                                        double time_from_start_factor);


  // Fill in the joint trajectory message for the RIGHT ARM and TORSO
  void makeRightArmTorsoJointTrajMSG(trajectory_msgs::JointTrajectory &JTmsg,
                                     KDL::JntArray &kdlArmJointsPosition,
                                     vpColVector &qdot,
                                     double time_from_start = 4.0,
                                     bool useTorso = false,
                                     double time_from_start_factor = 1.20);

  // Fill in the joint trajectory message for the LEFT ARM
  void makeArmJointTrajMSG_left(trajectory_msgs::JointTrajectory &JTmsg,
                                KDL::JntArray &kdlArmJointsPosition,
                                vpColVector &qdot,
                                double time_from_start = 4.0);

  // Fill in the joint trajectory message for the HEAD
  void makeHeadJointTrajMSG(trajectory_msgs::JointTrajectory &JTmsg,
                            KDL::JntArray &kdlHeadJointsPosition,
                            vpColVector &qdot,
                            double time_from_start = 4.0,
                            double time_from_start_factor = 1.20);

  // Fill in the marker message for RVIZ trajectory visualization
  void makeMarkerMSG(visualization_msgs::Marker &markerMSG,
                     const std::string& referenceFrame,
                     geometry_msgs::Pose markerPose,
                     double markerID,
                     bool jointLimitAvoindanceActive);

  // Fill in joint state message --> use for simulation without controllers
  void makeJointStateMSG(sensor_msgs::JointState &JSmsg, KDL::JntArray &kdlArmJointsPosition);

  // Get Joint Limits from an URDF model
  void getJointLimitsFromModel(const urdf::Model& model,
                               std::vector<double> &q_min,
                               std::vector<double> &q_max,
                               KDL::Chain &kdlArmChain);

  // Get Joint Limits from URDF file
  void getJointLimitsFromFile(std::vector<double> &q_min,
                              std::vector<double> &q_max,
                              const std::string& URDFfilename,
                              KDL::Chain &kdlArmChain);

  // Get Joint Limits from an URDF string
  void getJointLimitsFromString(std::vector<double> &q_min,
                                std::vector<double> &q_max,
                                std::string URDFstring,
                                KDL::Chain &kdlArmChain);

  // Clip velocities over the limit and print ROS warnings
  void limitVelocity(vpColVector &qdot, double velLimit = 0.1);

  void limitVelocity(std::vector<double>& qdot, double velLimit = 0.1);

  // Fill in the plot message for publishing data to be plotted in the /VS_errors topic
  void makePlotMSG(std_msgs::Float64MultiArray &plotMsg,
                   vpColVector &taskError,
                   vpColVector &qdot,
                   vpColVector &q1dot,
                   vpColVector &q2dot,
                   vpMatrix &fJe,
                   vpVelocityTwistMatrix &eVf,
                   KDL::JntArray &kdlArmPosition);

  // Fill in the plot message for publishing data corresponding to 2 tasks
  void makePlotMSG(std_msgs::Float64MultiArray &plotMsg,
                   const vpColVector &taskError_1, //error of task 1
                   const vpColVector &qdot_1,      //qdot computed to accomplish task 1
                   const vpColVector &taskError_2, //error of task 2
                   const vpColVector &qdot_2,      //qdot computed to accomplish task 2
                   const vpColVector &q1dot_1,     //q1dot of task1
                   const vpColVector &q2dot_1,     //q2dot of task1
                   const vpColVector &q1dot_2,     //q1dot of task2
                   const vpColVector &q2dot_2,     //q2dot of task2
                   double x_midPoint,        //normalized coordinate x of midpoint(hand marker, object marker)
                   double y_midPoint);       //normalized coordinate y of midpoint(hand marker, object marker)

  // Fill in the plot message for publishing data to be plotted in the /VS_errors topic
  void makePlotMSG(std_msgs::Float64MultiArray &plotMsg,
                   const vpColVector &taskError,
                   const vpColVector &v);

} // end namespace utility_functions

#endif
