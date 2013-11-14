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
