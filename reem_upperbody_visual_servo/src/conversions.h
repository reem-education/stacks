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
