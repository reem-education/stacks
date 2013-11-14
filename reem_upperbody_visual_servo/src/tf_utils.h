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

// ROS headers
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// KDL headers
#include <kdl/jacobian.hpp>

// Visp headers
#include <visp/vpHomogeneousMatrix.h>

// Std C++ headers
#include <string>

/** \author Jordi Pages */

namespace pal {

  /**
   * @brief waitUntilTransform gets from TF the pose of targetFrame expressed in baseFrame in a VISP homogeneous matrix
   * @param tfListener TF listener
   * @param baseFrame name of the base frame
   * @param targetFrame name of the target frame
   * @param pose pose of targetFrame expressed in baseFrame in a VISP homogeneous matrix
   * @return the time stamp of the obtained transformation
   */
  ros::Time waitUntilTransform(tf::TransformListener& tfListener,
                              const std::string& baseFrame,
                              const std::string& targetFrame,
                              vpHomogeneousMatrix& pose);

  ros::Time waitUntilTransform(tf::TransformListener& tfListener,
                              const std::string& baseFrame,
                              const std::string& targetFrame,
                              tf::StampedTransform& pose);

  /**
   * @brief getTransformFromFile get the desired pose of the hand marker in the object marker frame
   * @param fileName
   * @param pose
   */
  void getTransformFromFile(const std::string& fileName,
                            vpHomogeneousMatrix& pose);

  void publishTransform(tf::TransformBroadcaster& tfBroadcaster,
                        const tf::Transform& transform,
                        const std::string& frameId,
                        const std::string& childFrame);

  /**
   * @brief kdlJacobianToVispJacobian transforms a KDL jacobian to a ViSP jacobian. The difference is
   *        that the former is expressed in the base frame of the chain and has the reference point
   *        in the end-effector of the frame of the kinematic chain. The latter
   *        must have the reference point in the base frame. In order to do this TF is queried
   *        to get the origin of the end-effector frame expressed in the base frame.
   * @param tfListener
   * @param baseFrame
   * @param endFrame
   * @param kdlJacobian
   * @return
   */
  vpMatrix kdlJacobianToVispJacobian(tf::TransformListener& tfListener,
                                     const std::string& baseFrame,
                                     const std::string& endFrame,
                                     const KDL::Jacobian& kdlJacobian);

}
