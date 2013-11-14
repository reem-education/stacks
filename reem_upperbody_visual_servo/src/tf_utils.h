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
