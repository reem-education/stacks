/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
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

/** \author Adolfo Rodriguez Tsouroukdissian, Hilario Tome. */

// KDL
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

// Local
#include <reem_kinematics_constraint_aware/matrix_inverter.h>
#include <reem_kinematics_constraint_aware/ik_solver.h>
#include <ros/ros.h> // TODO: REMOVE!
using namespace reem_kinematics_constraint_aware;
using std::size_t;
using Eigen::VectorXd;

IkSolver::IkSolver(const KDL::Tree&            tree,
                   const EndpointNameList&     endpoint_names,
                   const EndpointCouplingList& endpoint_couplings)
{
  init(tree, endpoint_names, endpoint_couplings);
}

IkSolver::IkSolver(const KDL::Chain& chain)
{
  const std::string root_name = chain.getSegment(0).getName();
  const std::string tip_name  = chain.getSegment(chain.getNrOfSegments() - 1).getName();

  KDL::Tree tree("root");
  tree.addChain(chain, "root");

  std::vector<std::string> endpoint_names(1, tip_name);
  std::vector<EndpointCoupling> endpoint_couplings(1, Pose);

  init(tree, endpoint_names, endpoint_couplings);
}

IkSolver::~IkSolver() {}

void IkSolver::init(const KDL::Tree&                     tree,
                    const std::vector<std::string>&      endpoint_names,
                    const std::vector<EndpointCoupling>& endpoint_couplings)
{
  // Preconditions
  assert(endpoint_names.size() == endpoint_couplings.size() && "endpoints and coupling vectors size mismatch");
  // TODO: Verify that all endpoints are contained in tree

  // Enpoint names vector
  endpoint_names_ = endpoint_names;

  // Problem size
  const size_t q_dim = tree.getNrOfJoints(); // Joint space dimension
  size_t x_dim = 0;                          // Task space dimension, value assigned below

  // Populate coupled directions vector
  coupled_dirs_.resize(endpoint_names_.size());
  for (size_t i = 0; i < coupled_dirs_.size(); ++i)
  {
    if ((endpoint_couplings[i] & Position) == Position)
    {
      coupled_dirs_[i].push_back(0);
      coupled_dirs_[i].push_back(1);
      coupled_dirs_[i].push_back(2);
      x_dim += 3;
    }
    if ((endpoint_couplings[i] & Orientation) == Orientation)
    {
      coupled_dirs_[i].push_back(3);
      coupled_dirs_[i].push_back(4);
      coupled_dirs_[i].push_back(5);
      x_dim += 3;
    }
  }

  // Initialize kinematics solvers
  fk_solver_.reset(new FkSolver(tree));
  jac_solver_.reset(new JacSolver(tree));
  inverter_.reset(new Inverter(x_dim, q_dim));

  // Matrix inversion parameters TODO: Expose!
  inverter_->setLsInverseThreshold(1e-5); // NOTE: Magic values
  inverter_->setDlsInverseThreshold(1e-4);
  inverter_->setMaxDamping(0.05);

  // Default values of position solver parameters
  delta_twist_max_ = Eigen::NumTraits<double>::highest();
  delta_joint_pos_max_ = Eigen::NumTraits<double>::highest();
  velik_gain_      = 1.0;
  eps_             = Eigen::NumTraits<double>::epsilon();
  max_iter_        = 1;

  // Populate map from joint names to KDL tree indices
  joint_name_to_idx_.clear();
  const KDL::SegmentMap& tree_segments = tree.getSegments();
  for (KDL::SegmentMap::const_iterator it = tree_segments.begin(); it != tree_segments.end(); ++it)
  {
    const KDL::Joint& joint = it->second.segment.getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      joint_name_to_idx_[joint.getName()] = it->second.q_nr;
    }
  }

  // Joint space weights updater
  limits_avoider_.reset(new JointPositionLimitsAvoider(q_dim));
  limits_avoider_->setSmoothing(0.8); // NOTE: Magic value

  // Preallocate IK resources
  delta_twist_ = VectorXd::Zero(x_dim);
  delta_q_     = VectorXd::Zero(q_dim);
  q_tmp_       = VectorXd::Zero(q_dim);

  jacobian_     = Eigen::MatrixXd(x_dim, q_dim);
  jacobian_tmp_ = KDL::Jacobian(q_dim);

  Wx_ = VectorXd::Ones(x_dim);

  q_posture_           = KDL::JntArray(q_dim);
  nullspace_projector_ = Eigen::MatrixXd(q_dim, q_dim);
  identity_qdim_       = Eigen::MatrixXd::Identity(q_dim, q_dim);
}

bool IkSolver::solve(const KDL::JntArray&           q_current,
                     const std::vector<KDL::Frame>& x_desired,
                           KDL::JntArray&           q_next)
{
  // Precondition
  assert(endpoint_names_.size() == x_desired.size());

  q_next = q_current;
  q_tmp_ = q_current.data;

  // Update joint-space weight matrix: Limit effect of joints near their position limit
  limits_avoider_->resetJointLimitAvoidance().applyJointLimitAvoidance(q_next.data);

  size_t i;
  double delta_twist_norm;
  for (i = 0; i < max_iter_; ++i)
  {
    // Update current task space velocity error
    updateDeltaTwist(q_next, x_desired);
    delta_twist_norm = delta_twist_.dot(Wx_.asDiagonal() * delta_twist_); // Weighted norm
    if (delta_twist_norm < eps_) {break;}

    // Enforce task space maximum velocity through uniform scaling
    const double delta_twist_scaling = delta_twist_max_ / delta_twist_.cwiseAbs().maxCoeff();
    if (delta_twist_scaling < 1.0) {delta_twist_ *= delta_twist_scaling;}

    // Update Jacobian
    updateJacobian(q_next);

    // Velocity IK: Compute incremental joint displacement and scale according to gain

    // Prepare computation of IK with nullspace optimization
    using Eigen::MatrixXd;
    using Eigen::VectorXd;
    using Eigen::DiagonalWrapper;

    const MatrixXd& J = jacobian_;                                                         // Convenience alias
    const DiagonalWrapper<const VectorXd> Wq = limits_avoider_->getWeights().asDiagonal(); // Convenience alias
    const DiagonalWrapper<const VectorXd> Wx = Wx_.asDiagonal();                           // Convenience alias

    // Perform SVD decomposition of J W
    inverter_->compute(Wx * J * Wq);

    // Nullspace projector
    nullspace_projector_ = identity_qdim_ - Wq * inverter_->inverse() * Wx * J; // NOTE: Not rt-friendly, allocates temporaries

    // Compute incremental joint displacement
    delta_q_  = Wq * inverter_->dlsSolve(Wx * delta_twist_) + nullspace_projector_ * (q_posture_.data - q_next.data);
    delta_q_ *= velik_gain_;

    // Enforce joint space maximum velocity through uniform scaling
    const double delta_q_scaling = delta_joint_pos_max_ / delta_q_.cwiseAbs().maxCoeff();
    if (delta_q_scaling < 1.0) {delta_q_ *= delta_q_scaling;}

    // Cache value of q_next
    q_tmp_ = q_next.data;

    // Integrate joint velocity
    q_next.data += delta_q_;

    // Enforce joint position limits

    // Update joint-space weight matrix: Limit effect of joints near their position limit
    limits_avoider_->applyJointLimitAvoidance(q_next.data);

    if (!limits_avoider_->isValid(q_next.data))
    {
      // Keep last configuration that does not exceed position limits
      q_next.data = q_tmp_;
//       ROS_DEBUG_STREAM("Iteration " << i << ", not updating joint position values, weights = " << limits_avoider_->getWeights().transpose());
    }

//     ROS_DEBUG_STREAM("Iteration " << i << ", delta_twist_norm " << delta_twist_.norm() << ", delta_q_scaling " << delta_q_scaling << ", q_next " << q_next.data.transpose()); // TODO: Remove?
  }

  updateDeltaTwist(q_next, x_desired); delta_twist_norm = delta_twist_.transpose().dot(Wx_.asDiagonal() * delta_twist_); // Only needed by below debug message
  ROS_DEBUG_STREAM("Total iterations " << i << ", delta_twist_norm " << delta_twist_norm << " (eps " << eps_ << "), q " << q_next.data.transpose());

  return (i < max_iter_);
}

void IkSolver::updateDeltaTwist(const KDL::JntArray& q, const std::vector<KDL::Frame>& x_desired)
{
  KDL::Frame ith_frame;
  KDL::Twist ith_delta_twist;
  size_t x_idx = 0;
  for (size_t i = 0; i < endpoint_names_.size(); ++i)
  {
    // Forward kinematics of ith endpoint
    fk_solver_->JntToCart(q, ith_frame, endpoint_names_[i]);
    ith_delta_twist = diff(ith_frame, x_desired[i]);

    KDL::Vector rot   = ith_frame.M.GetRot();
    KDL::Vector rot_d = x_desired[i].M.GetRot();

    // Extract only task-space directions relevant to the IK problem
    const CoupledDirections& endpoint_coupled_dirs = coupled_dirs_[i];
    for (size_t j = 0; j < endpoint_coupled_dirs.size(); ++j)
    {
      delta_twist_(x_idx) = ith_delta_twist[endpoint_coupled_dirs[j]];
      ++x_idx;
    }
  }
}

void IkSolver::updateJacobian(const KDL::JntArray& q)
{
  size_t x_idx = 0;

  for (size_t i = 0; i < endpoint_names_.size(); ++i)
  {
    // Jacobian of ith endpoint
    jac_solver_->JntToJac(q, jacobian_tmp_, endpoint_names_[i]);

    // Extract only task-space directions (Jacobian rows) relevant to the IK problem
    const CoupledDirections& endpoint_coupled_dirs = coupled_dirs_[i];
    for (size_t j = 0; j < endpoint_coupled_dirs.size(); ++j)
    {
      jacobian_.row(x_idx) = jacobian_tmp_.data.row(endpoint_coupled_dirs[j]);
      ++x_idx;
    }
  }
}

JointPositionLimitsAvoider& JointPositionLimitsAvoider::setJointLimits(const Eigen::VectorXd& q_min,
                                                                       const Eigen::VectorXd& q_max,
                                                                       double activation_window)
{
  assert(w_.size() == q_min.size() && "qmin size mismatch.");
  assert(w_.size() == q_max.size() && "qmax size mismatch.");
  assert(activation_window >= 0.0 && activation_window <= 1.0 && "Activation window should belong to [0, 1].");

  q_min_             = q_min;
  q_max_             = q_max;
  q_activation_size_ = (q_max_ - q_min_).cwiseAbs() * activation_window;
  q_activation_min_  = q_min_ + q_activation_size_;
  q_activation_max_  = q_max_ - q_activation_size_;
  return *this;
}

JointPositionLimitsAvoider& JointPositionLimitsAvoider::applyJointLimitAvoidance(const Eigen::VectorXd& q)
{
  assert(q.size() == w_.size());

  typedef Eigen::VectorXd::Index Index;
  for (Index i = 0; i < q.size(); ++i)
  {
    if (q(i) <= q_min_(i))
    {
      // Minimum joint position limit violated
      w_scaling_(i) = 0.0;
    }
    else if (q(i) >= q_max_(i))
    {
      // Maximum joint position limit violated
      w_scaling_(i) = 0.0;
    }
    else if (q(i) < q_activation_min_(i))
    {
      // In the lower joint limit avoidance zone.
      const double activation = std::abs(q(i) - q_activation_min_(i)) / q_activation_size_(i); // Grows linearly from 0 to 1 as joint limit is apprached
      w_scaling_(i) = 1.0 - 1.0 / (1 + std::exp(-12.0 * activation + 6.0 ));                   // Generalized logistic function mapping [0, 1] to [1, 0]

      // If moving away from joint limit, make further progress less difficult. Transition to no scaling is low-pass
      // filtered to avoid discontinuities and reduce weight value chattering across IK iterations
      if (w_scaling_(i) > w_scaling_prev_(i))
      {
//         ROS_WARN_STREAM(">>> pre-smooth " << w_scaling_(i) ); // TODO: Remove!
        w_scaling_(i) = 1.0 - smoothing_ * (1.0 - w_scaling_(i));
//         ROS_DEBUG_STREAM(">>> " << i << " min out, q " << q(i) << ", w_scaling_prev_ " << w_scaling_prev_(i) << ", w_scaling_ " << w_scaling_(i));  // TODO: Remove!
      }
//       else  // TODO: Remove!
//       {
//         ROS_DEBUG_STREAM(">>> " << i << " min, q " << q(i) << ", w_scaling_prev_ " << w_scaling_prev_(i) << ", w_scaling_ " << w_scaling_(i));
//       }
    }
    else if (q(i) > q_activation_max_(i))
    {
      // In the upper joint limit avoidance zone
      const double activation = std::abs(q_activation_max_(i) - q(i)) / q_activation_size_(i); // Grows linearly from 0 to 1 as joint limit is apprached
      w_scaling_(i) = 1.0 - 1.0 / (1 + std::exp(-12.0 * activation + 6.0 ));                   // Generalized logistic function mapping [0, 1] to [1, 0]

      // If moving away from joint limit, make further progress less difficult. Transition to no scaling is low-pass
      // filtered to avoid discontinuities and reduce weight value chattering across IK iterations
      if (w_scaling_(i) > w_scaling_prev_(i))
      {
        w_scaling_(i) = 1.0 - smoothing_ * (1.0 - w_scaling_(i));
//         ROS_DEBUG_STREAM(">>> " << i << " max out, q " << q(i) << ", w_scaling_prev_ " << w_scaling_prev_(i) << ", w_scaling_ " << w_scaling_(i)); // TODO: Remove!
      }
//       else // TODO: Remove!
//       {
//         ROS_DEBUG_STREAM(">>> " << i << " max, q " << q(i) << ", w_scaling_prev_ " << w_scaling_prev_(i) << ", w_scaling_ " << w_scaling_(i));
//       }
    }
    else
    {
      w_scaling_(i) = 1.0;
    }
  }

  // Scale weights
  w_scaled_ = w_.cwiseProduct(w_scaling_);

  // Update previous value
  w_scaling_prev_ = w_scaling_;

//   ROS_DEBUG_STREAM(">> Joint space weights diagonal: " << w_scaled_.transpose()); // TODO: Remove!
  return *this;
}
