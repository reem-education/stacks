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

#ifndef _PAL_REEM_UPPERBODY_VISUAL_SERVO_H_
#define _PAL_REEM_UPPERBODY_VISUAL_SERVO_H_

// ROS headers
#include <urdf/model.h>

// KDL headers
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// Boost headers
#include <boost/scoped_ptr.hpp>

// Std C++ headers
#include <vector>
#include <string>

namespace pal {

  /**
   * @brief The KinematicChain class encapsulates all the joint names, joint state and jacobians
   *        for a given kinematic chain of the robot
   */
  class KinematicChain {

  public:

    /**
     * @brief KinematicChain constructor
     * @param robotDescription string containing the URDF definition of the whole robot
     * @param endFrame name of the last frame in the chain
     * @param baseFrame name of the frame from which the chain arises. It is the frame right before the first frame
     *        in the chain
     */
    KinematicChain(const std::string& robotDescription,
                   const std::string& endFrame,
                   const std::string& baseFrame);

    /**
     * @brief ~KinematicChain
     */
    virtual ~KinematicChain();

    /**
     * @brief updateJointState update the positions of the joints in the chain from /joint_states topic
     */
    void updateJointStateFromTopic();

    /**
     * @brief getNumberOfJoints
     * @return
     */
    int getNumberOfJoints() const;

    /**
     * @brief getJointPositions
     * @return
     */
    std::vector<double> getJointPositions() const;

    KDL::JntArray getJointPositionsKdl() const;

    /**
     * @brief setJointPositions
     * @param positions
     */
    void setJointPositions(const std::vector<double>& positions);

    /**
     * @brief applyVelocities update joint positions by applying the given velocities for the
     *        given amount of time
     * @param qdot
     * @param timeStepSec
     */
    void applyVelocities(const std::vector<double>& qdot,
                         double timeStepSec);


    /**
     * @brief getLowerJointLimits
     * @return
     */
    const std::vector<double>& getLowerJointLimits() const;

    /**
     * @brief getUpperJointLimits
     * @return
     */
    const std::vector<double>& getUpperJointLimits() const;

    /**
     * @brief setLowerJointLimits
     * @param lowerLimits
     */
    void setLowerJointLimits( const std::vector<double>& lowerLimits );

    /**
     * @brief setUpperJointLimits
     * @param upperLimits
     */
    void setUpperJointLimits( const std::vector<double>& upperLimits );

    /**
     * @brief getJacobian evaluate and return the current jacobian expressed
     *        in the base frame of the chain and with the reference point in the
     *        end-effector origin of the chain
     */
    KDL::Jacobian getJacobian() const;

    std::string jointStateToStr() const;

  protected:


    /**
     * @brief updateJointNames
     */
    void updateJointNames();

    /**
     * @brief updateJointLimits
     */
    void updateJointLimits();

    /**
     * @brief initServoTask
     */
    void initServoTask();


    int _numberOfJoints;
    std::vector<std::string> _jointNames;

    KDL::Tree                _kdlJointTree;
    KDL::Chain               _kdlChain;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> _kdlJointToJacobianHead;
    KDL::JntArray            _jointsPosition;    

    std::vector<double> _lowerJointLimits, _upperJointLimits;

    urdf::Model _model;

  };

}

#endif //_PAL_REEM_UPPERBODY_VISUAL_SERVO_H_
