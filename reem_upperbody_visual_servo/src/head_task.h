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

#ifndef _PAL_REEM_UPPERBODY_VISUAL_SERVO_HEAD_TASK_H_
#define _PAL_REEM_UPPERBODY_VISUAL_SERVO_HEAD_TASK_H_

// PAL headers
#include "kinematic_chain.h"

// ViSP headers
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>

// Std C++ headers
#include <vector>

namespace pal {

  /**
   * @brief The HeadTask class  implements an eye-in-hand visual servo task of the form:
   *
   *   qdot = -λ · ( L · ^cJ(q) )^+ · e
   *
   *   (note that in the above formula the secondary task in charge of avoiding joint limits is not included)
   *
   *   where:
   *
   *   qdot: velocities of the head joints
   *   ^cJ(q): jacobian expressed in the camera frame with reference point in the origin of the camera
   *
   *          This jacobian is actually obtained with:
   *
   *          ^cJ(q) = ^cV_b · ^bJ(q)
   *
   *   ^cV_b: 6x6 twist transformation matrix from the base frame of the
   *         head kinematic chain to the camera frame
   *   ^bJ(q): jacobian expressed in the base frame and with reference point at the base frame origin
   *
   */
  class HeadTask: private vpServo {

  public:

    HeadTask(double gain);

    virtual ~HeadTask();

    /**
     * @brief setDesiredImagePoint
     * @param x
     * @param y
     * @param Z
     */
    void setDesiredImagePoint(double x, double y, double Z);

    /**
     * @brief setCurrentImagePoint
     * @param x
     * @param y
     * @param Z
     */
    void setCurrentImagePoint(double x, double y, double Z);

    double getCurrentImagePoint_x() const;
    double getCurrentImagePoint_y() const;

    /**
     * @brief computeLaw
     * @param chain
     * @param cMheadBase
     * @param cameraTwistInducedByTorso
     * @param targetPointTwistInducedByTorsoArm
     * @param enableJointLimitAvoidance
     * @param useLargeProjectionOperator
     * @return
     */
    vpColVector computeLaw(const pal::KinematicChain& chain,
                           const vpHomogeneousMatrix& cMheadBase,
                           const vpColVector& cameraTwistInducedByTorso,
                           const vpColVector& targetPointTwistInducedByTorsoArm,
                           bool enableJointLimitAvoidance,
                           bool useLargeProjectionOperator);

    /**
     * @brief get_q1dot get the joint velocities computed before applying the secondary task
     * @return
     */
    vpColVector get_q1dot() const;

    /**
     * @brief get_q2dot get the joint velocities contribution by the secondary task
     * @return
     */
    vpColVector get_q2dot() const;

    vpColVector getTaskError() const;


  protected:

    /**
     * @brief setJacobian set the Jacobian expressed in the base frame and with
     *        reference point the origin of the base frame
     */
    void setJacobian(const vpMatrix& jacobian);

    /**
     * @brief setTwistTransformation sets the 6x6 twist transformation, computed
     *        from the given homogeneous matrix, from the base frame of the head kinematic
     *        chain to the end-effector, i.e. camera, frame of the chain.
     * @param headBaseToCamera
     */
    void setBaseToCameraTwist(const vpHomogeneousMatrix& headBaseToCamera);

    vpFeaturePoint _currentImagePoint, _desiredImagePoint;

    vpColVector _q1dot, _q2dot;

    double _gain;

  };

}

#endif //_PAL_REEM_UPPERBODY_VISUAL_SERVO_HEAD_TASK_H_
