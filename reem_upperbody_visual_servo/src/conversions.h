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

#ifndef _PAL_REEM_UPPERBODY_VISUAL_SERVO_CONVERSIONS_
#define _PAL_REEM_UPPERBODY_VISUAL_SERVO_CONVERSIONS_

/**
 * @brief dataype conversions between Eigen, KDL, ROS and ViSP and STL
 *
 */

// ROS headers
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

// Visp headers
#include <visp/vpHomogeneousMatrix.h>

// KDL headers
#include <kdl/jacobian.hpp>

namespace pal {

  /**
   * @brief eigenToStdVector copy a Eigen vector to a std::vector<double>
   * @param[in] eigenV
   * @param[in] initialPose optional. Initial position of the eigen vector (0-based).
   * @param[in] length optional. Number of elements to copy from the initialPose. -1 means all the vector.
   * @return
   */
  std::vector<double> eigenToStdVector(const Eigen::VectorXd& eigenV,
                                  int initialPose = 0,
                                  int length = -1);

  /**
   * @brief vispToStdVector copy a kdl vector to a
   * @param vpVector
   * @param initialPose
   * @param length
   * @return
   */
  std::vector<double> vispToStdVector(const vpColVector& vpVector,
                                  int initialPose = 0,
                                  int length = -1);

  vpColVector stdVectorToVisp(const std::vector<double>& vector);


  /**
   * @brief convertGMPoseToVispHMat
   * @param pose
   * @param HMatrix
   */
  void convertGMPoseToVispHMat(const geometry_msgs::Pose &pose,
                               vpHomogeneousMatrix &HMatrix);

  /**
   * @brief convertVispHMatToGMPose
   * @param HMatrix
   * @param pose
   */
  void convertVispHMatToGMPose(const vpHomogeneousMatrix &HMatrix,
                               geometry_msgs::Pose &pose);

  /**
   * @brief convertEigenMatToVisp
   * @param em
   * @param vmat
   */
  void convertEigenMatToVisp(const Eigen::MatrixXd& em,
                             vpMatrix& vmat);

  /**
   * @brief convertVispHMatToTF
   * @param HMatrix
   * @param TFtransform
   */
  void convertVispHMatToTF(const vpHomogeneousMatrix &HMatrix,
                           tf::Transform &TFtransform);

  /**
   * @brief convertTFtoVispHMat
   * @param TFtransform
   * @param HMatrix
   */
  void convertTFtoVispHMat(const tf::StampedTransform &TFtransform,
                           vpHomogeneousMatrix &HMatrix);

  /**
   * @brief convertKDLJacToVispMat
   * @param kdlJac
   * @param vispMat
   */
  void convertKDLJacToVispMat(const KDL::Jacobian &kdlJac,
                              vpMatrix &vispMat);

  /**
   * @brief convertKDLJacToVispMat copy contents of a KDL Jacobian matrix to a ViSP matrix
   * @param kdlJac
   * @return
   */
  vpMatrix convertKDLJacToVispMat(const KDL::Jacobian& kdlJac);

  /**
   * @brief kdlJacobianToVispJacobian convert KDL jacobian to ViSP jacobian. Both jacobians are expressed
   * in the base frame of the kinematic chain. Nevertheless, the reference point
   * of the KDL jacobian is set in the end-effector frame and in the ViSP must be set in the base link
   * @param kdlJacobian
   * @param bMe pose of the end frame of the chain expressed in the base frame of the chain
   * @return
   */
  vpMatrix kdlJacobianToVispJacobian(const KDL::Jacobian& kdlJacobian,
                                     const vpHomogeneousMatrix& bMe);







} //pal

#endif
