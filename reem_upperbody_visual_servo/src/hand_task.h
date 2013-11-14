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

#ifndef _PAL_REEM_UPPERBODY_VISUAL_SERVO_HAND_TASK_H_
#define _PAL_REEM_UPPERBODY_VISUAL_SERVO_HAND_TASK_H_

// PAL headers
#include "kinematic_chain.h"

// ViSP headers
#include <visp/vpServo.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>

// Boost headers
#include <boost/scoped_ptr.hpp>

// Std C++ headers
#include <vector>

namespace pal {

  /**
   * @brief The HandTask class  implements an eye-in-hand visual servo task of the form:
   *
   *   qdot = -λ · ( L · ^hmJ(q) )^+ · e
   *
   *   (note that in the above formula the secondary task in charge of avoiding joint limits is not included)
   *
   *   where:
   *
   *   qdot: velocities of the torso+arm joints
   *
   *   e: task error. It is a 6x1 vector composed of e = [ ^hmdO_hm, thetaU(^hmdM_hm) ]
   *
   *       where ^hmdO_hm is the origin of the current hand marker frame expressed in the desired hand marker frame
   *             ^hmdM_hm is the pose of the current hand marker expressed in the desired hand marker frame
   *
   *   ^hmJ(q): jacobian expressed in the hand marker frame with reference point in the origin of the hand marker
   *
   *          This jacobian is actually obtained with:
   *
   *          ^hmJ(q) = ^hmV_b · ^bJ(q)
   *
   *   ^hmV_b: 6x6 twist transformation matrix from the base frame of the
   *         torso+arm kinematic chain to the hand marker frame
   *   ^bJ(q): jacobian expressed in the base frame of the torso+arm kinematic chain
   *           with reference point at the base frame origin
   *
   */
  class HandTask: private vpServo {

  public:

    HandTask(double gain);

    virtual ~HandTask();

    /**
     * @brief setHandMarkerPose sets the pose of the hand marker with respect the hand frame, i.e.
     *        the last frame in the torso+arm kinematic chain
     * @param hMhm hand marker frame {hm} pose in hand frame {h}
     */
    void setHandMarkerPose(const vpHomogeneousMatrix &hMhm);

    /**
     * @brief updateTransAndThetaU set the relative pose between the current hand marker frame
     *        and the desired hand marker frame
     * @param hmdMhm
     */
    void updateTransAndThetaU(vpHomogeneousMatrix& hmdMhm);

    /**
     * @brief computeLaw
     * @param chain
     * @param hmMtorsoBase
     * @param enableJointLimitAvoidance
     * @param useLargeProjectionOperator
     * @return qdot
     */
    vpColVector computeLaw(const pal::KinematicChain& chain,
                           const vpHomogeneousMatrix& hmMtorsoBase,
                           bool enableJointLimitAvoidance,
                           bool useLargeProjectionOperator);


    double getErrorNorm() const;

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
     * @brief setBaseToMarkerTwist sets the 6x6 twist transformation, computed
     *        from the given homogeneous matrix, from the base frame of the torso+arm kinematic
     *        chain to the end-effector marker, i.e. hand marker, frame of the chain.
     * @param torsoBaseToHand
     */
    void setBaseToMarkerTwist(const vpHomogeneousMatrix& torsoBaseToHand);

    /**
     * @brief setJacobian set the Jacobian of the torso+arm chain expressed in the base frame and with
     *        reference point the origin of the base frame
     */
    void setJacobian(const vpMatrix& jacobian);

    double _gain;

    vpHomogeneousMatrix _hMhm;

    boost::scoped_ptr<vpFeatureThetaU> _thetaU;
    boost::scoped_ptr<vpFeatureTranslation> _trans;

    vpColVector _q1dot, _q2dot;

  };

}

#endif //_PAL_REEM_UPPERBODY_VISUAL_SERVO_HAND_TASK_H_
