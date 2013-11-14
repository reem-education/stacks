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
