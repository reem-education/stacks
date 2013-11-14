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

/** \author Don Joven Agravante */

#ifndef __PAL_VISUAL_SERVO_SECONDARY_TASK__
#define __PAL_VISUAL_SERVO_SECONDARY_TASK__

#include <visp/vpHomogeneousMatrix.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <vector>

namespace secondary_task
{
    // Sigmoid Function --> returns the sigmoid value f(x) at the specified x
    // used for smoothing abrupt transitions between the low and high limits
    double sigmoid(double &x,
                   double &low_limit,
                   double &high_limit,
                   double multiplier = 12.0,
                   double add = 6.0);

    // The joint limit activation vector is based on a sign function to move away from the joint limit
    void makeJointLimitActivationVector(vpColVector &jnt_limit_avoidance_activation,
                                        const KDL::JntArray &kdlJointPosition,
                                        const std::vector<double> &q_max,
                                        const std::vector<double> &q_min,
                                        double JLA_rho0 = 0.1);

    // Creates a Smooth transition function from 0 to 1
    double makeSwitchingGain(double x,
                             double lower_limit,
                             double upper_limit);

    // Create a 2 sided Smooth transition function for the joint limit avoidance injection
    void makeSmoothInjectionGain(vpColVector &smooth_injection_Lambda,
                                 const KDL::JntArray &kdlJointPosition,
                                 const std::vector<double> &q_max,
                                 const std::vector<double> &q_min,
                                 double JLA_rho0 = 0.1,
                                 double JLA_rho1 = 0.5);

    // Compute contorl law using Error Norm
    void computeControlLaw_E_Norm(vpColVector &qdot_E_Norm,
                                  vpMatrix &Projector_E_Norm,
                                  const vpMatrix &taskJacobian,
                                  const vpColVector &error,
                                  double taskLambda,
                                  vpMatrix &identity);

    // Compute adaptive gain for ensuring the execution of the secondary task
    void make2ndTaskGain(vpColVector &ensure_2nd_task_Lambda,
                         vpColVector q1dot,
                         vpColVector Pg,
                         double relative_mag_2nd_task = 0.5);

    void computeSecondaryTask(const vpColVector& qdot_primary,
                              const vpMatrix& taskJacobian,
                              double taskLambda,
                              const vpColVector& taskError,
                              const KDL::JntArray& kdlJointPosition,
                              const std::vector<double>& q_min,
                              const std::vector<double>& q_max,
                              bool useLargeProjector,
                              vpColVector& qdot,   //final qdot
                              vpColVector& q1dot,  //qdot from the primary task
                              vpColVector& q2dot); //qdot from the secondary task

} // end namespace secondary_task

#endif
