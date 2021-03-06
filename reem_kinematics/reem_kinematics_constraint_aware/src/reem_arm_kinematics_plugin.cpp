/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Original code: Sachin Chitta, David Lu!!, Ugo Cupcic.
 *  REEM-specific modifications: Hilario Tome, Adolfo Rodriguez Tsouroukdissian.
 */

#include <reem_kinematics_constraint_aware/reem_arm_kinematics_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;
 
const double BOUNDS_EPSILON = .00001;
 
//register ReemKinematics as a KinematicsBase implementation
PLUGINLIB_DECLARE_CLASS(reem_kinematics_constraint_aware,ReemKinematicsPlugin, reem_kinematics_constraint_aware::ReemKinematicsPlugin, kinematics::KinematicsBase)

namespace reem_kinematics_constraint_aware {

ReemKinematicsPlugin::ReemKinematicsPlugin():active_(false)
{
  srand ( time(NULL) );
}

bool ReemKinematicsPlugin::isActive()
{
  if(active_)
    return true;
  return false;
}

double ReemKinematicsPlugin::genRandomNumber(const double &min, const double &max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

KDL::JntArray ReemKinematicsPlugin::getRandomConfiguration()
{
  KDL::JntArray jnt_array;
  jnt_array.resize(dimension_);
  for(unsigned int i=0; i < dimension_; ++i)
    jnt_array(i) = genRandomNumber(joint_min_(i),joint_max_(i));
  return jnt_array;
}

KDL::JntArray ReemKinematicsPlugin::getRandomConfiguration(const KDL::JntArray& seed_state,
                                                           const unsigned int& redundancy,
                                                           const double& consistency_limit)
{
  KDL::JntArray jnt_array;
  jnt_array.resize(dimension_);
  for(unsigned int i=0; i < dimension_; i++) {
    if(i != redundancy) {
      jnt_array(i) = genRandomNumber(joint_min_(i),joint_max_(i));
    } else {
      double jmin = fmin(joint_min_(i), seed_state(i)-consistency_limit);
      double jmax = fmax(joint_max_(i), seed_state(i)+consistency_limit);
      jnt_array(i) = genRandomNumber(jmin, jmax);
    }
  }
  return jnt_array;
}

bool ReemKinematicsPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                            const unsigned int& redundancy,
                                            const double& consistency_limit,
                                            const KDL::JntArray& solution) const
{
  KDL::JntArray jnt_array;
  jnt_array.resize(dimension_);
  for(unsigned int i=0; i < dimension_; i++) {
    if(i == redundancy) {
      double jmin = fmin(joint_min_(i), seed_state(i)-consistency_limit);
      double jmax = fmax(joint_max_(i), seed_state(i)+consistency_limit);
      if(solution(i) < jmin || solution(i) > jmax) {
        return false;
      }
    }
  }
  return true;
}

bool ReemKinematicsPlugin::initialize(const std::string& group_name,
                                      const std::string& base_name,
                                      const std::string& tip_name,
                                      const double& search_discretization)
{
  setValues(group_name, base_name, tip_name, search_discretization);
  // Get URDF XML
  std::string urdf_xml, full_urdf_xml;
  ros::NodeHandle node_handle;
  ros::NodeHandle private_handle("~"+group_name);
  ROS_INFO_STREAM("Private handle registered under " << private_handle.getNamespace());
  node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
  node_handle.searchParam(urdf_xml,full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");
  std::string result;
  if (!node_handle.getParam(full_urdf_xml, result)) {
    ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  // Load and Read Models
  if (!loadModel(result)) {
    ROS_FATAL("Could not load models!");
    return false;
  }

  // Populate map from joint names to KDL tree indices
  std::map<std::string, int> joint_name_to_idx;
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Joint& joint = kdl_chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      joint_name_to_idx[joint.getName()] = i;
    }
  }

  // Space diensions
  const int q_dim = kdl_chain_.getNrOfJoints();
  const int x_dim = 6;

  // Get Solver Parameters
  int max_iterations;
  double epsilon;
  double max_delta_x;
  double max_delta_q;
  double velik_gain;

  private_handle.param("max_solver_iterations", max_iterations, 500);
  private_handle.param("max_search_iterations", max_search_iterations_, 3);
  private_handle.param("epsilon", epsilon, 1e-5);
  private_handle.param("max_delta_x", max_delta_x, 0.006);
  private_handle.param("max_delta_q", max_delta_q, 0.03);
  private_handle.param("velik_gain", velik_gain,   1.0);

  // Joint space weights diagonal matrix inverse and default posture
  Eigen::VectorXd Wqinv = Eigen::VectorXd::Ones(q_dim);
  default_posture_.resize(q_dim);
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Joint& joint = kdl_chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      private_handle.param("joint_weights/"   + joint.getName(), Wqinv(joint_name_to_idx[joint.getName()]),            1.0);
      private_handle.param("default_posture/" + joint.getName(), default_posture_[joint_name_to_idx[joint.getName()]], 0.0);
    }
  }

  // Task space weights diagonal matrix
  Eigen::VectorXd Wxinv = Eigen::VectorXd::Ones(x_dim);
  private_handle.param("task_weights/position/x", Wxinv(0), 1.0);
  private_handle.param("task_weights/position/y", Wxinv(1), 1.0);
  private_handle.param("task_weights/position/z", Wxinv(2), 1.0);
  private_handle.param("task_weights/orientation/x", Wxinv(3), 1.0);
  private_handle.param("task_weights/orientation/y", Wxinv(4), 1.0);
  private_handle.param("task_weights/orientation/z", Wxinv(5), 1.0);

  // Build Solvers
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  ik_solver_.reset(new IkSolver(kdl_chain_));
  ik_solver_->setJointPositionLimits(joint_min_.data, joint_max_.data);
  ik_solver_->setEpsilon(epsilon);
  ik_solver_->setMaxIterations(max_iterations);
  ik_solver_->setMaxDeltaPosTask(max_delta_x);
  ik_solver_->setMaxDeltaPosJoint(max_delta_q);
  ik_solver_->setVelocityIkGain(velik_gain);
  ik_solver_->setJointSpaceWeights(Wqinv);
  ik_solver_->setTaskSpaceWeights(Wxinv);
  active_ = true;
  return true;
}

bool ReemKinematicsPlugin::loadModel(const std::string xml)
{
  urdf::Model robot_model;
  KDL::Tree tree;

  if (!robot_model.initString(xml)) {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree)) {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(base_name_, tip_name_, kdl_chain_)) {
    ROS_ERROR("Could not initialize chain object");
    return false;
  }
  if (!readJoints(robot_model)) {
    ROS_FATAL("Could not read information about the joints");
    return false;
  }
  return true;
}

bool ReemKinematicsPlugin::readJoints(urdf::Model &robot_model)
{
  dimension_ = 0;
  // get joint maxs and mins
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name_);
  boost::shared_ptr<const urdf::Joint> joint;
  while (link && link->name != base_name_) {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (!joint) {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
      ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
      dimension_++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  joint_min_.resize(dimension_);
  joint_max_.resize(dimension_);
  chain_info_.joint_names.resize(dimension_);
  chain_info_.limits.resize(dimension_);
  link = robot_model.getLink(tip_name_);
  if(link)
    chain_info_.link_names.push_back(tip_name_);

  unsigned int i = 0;
  while (link && i < dimension_) {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
      ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

      float lower, upper;
      int hasLimits;
      if ( joint->type != urdf::Joint::CONTINUOUS ) {
        if(joint->safety) {
          lower = joint->safety->soft_lower_limit+BOUNDS_EPSILON; 
          upper = joint->safety->soft_upper_limit-BOUNDS_EPSILON;
        } else {
          lower = joint->limits->lower+BOUNDS_EPSILON;
          upper = joint->limits->upper-BOUNDS_EPSILON;
        }
        hasLimits = 1;
      } else {
        lower = -M_PI;
        upper = M_PI;
        hasLimits = 0;
      }
      int index = dimension_ - i -1;

      joint_min_.data[index] = lower;
      joint_max_.data[index] = upper;
      chain_info_.joint_names[index] = joint->name;
      chain_info_.limits[index].joint_name = joint->name;
      chain_info_.limits[index].has_position_limits = hasLimits;
      chain_info_.limits[index].min_position = lower;
      chain_info_.limits[index].max_position = upper;
      ++i;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  return true;
}

int ReemKinematicsPlugin::getJointIndex(const std::string &name)
{
  for (unsigned int i=0; i < chain_info_.joint_names.size(); ++i) {
    if (chain_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

int ReemKinematicsPlugin::getKDLSegmentIndex(const std::string &name)
{
  int i=0; 
  while (i < (int)kdl_chain_.getNrOfSegments()) {
    if (kdl_chain_.getSegment(i).getName() == name) {
      return i+1;
    }
    ++i;
  }
  return -1;
}

bool ReemKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                         const std::vector<double> &ik_seed_state,
                                         const std::vector<double> &posture,
                                         std::vector<double> &solution,
                                         int &error_code)
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  ROS_DEBUG_STREAM("getPositionIK1:Position request pose is " <<
                   ik_pose.position.x << " " <<
                   ik_pose.position.y << " " <<
                   ik_pose.position.z << " " <<
                   ik_pose.orientation.x << " " <<
                   ik_pose.orientation.y << " " <<
                   ik_pose.orientation.z << " " <<
                   ik_pose.orientation.w);

  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);
  //Do the inverse kinematics
  KDL::JntArray jnt_pos_in(dimension_);
  KDL::JntArray jnt_pos_out(dimension_);
  KDL::JntArray jnt_posture(dimension_);
  for(unsigned int i=0; i < dimension_; ++i)
  {
    jnt_pos_in(i)  = ik_seed_state[i];
    jnt_posture(i) = posture[i];
  }
  ik_solver_->setPosture(jnt_posture);
  bool ik_valid = ik_solver_->solve(jnt_pos_in, pose_desired, jnt_pos_out);
  ROS_DEBUG_STREAM("IK success " << ik_valid << " time " << (ros::WallTime::now()-n1).toSec());
  solution.resize(dimension_);
  // NOTE: In the original implementation, the solution is not reported if IK failed. We do populate the best estimate
  for(unsigned int i=0; i < dimension_; ++i)
  {
    solution[i] = jnt_pos_out(i);
  }
  if(ik_valid)
  {
    error_code = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    error_code = kinematics::NO_IK_SOLUTION;
    return false;
  }
}

bool ReemKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                         const std::vector<double> &ik_seed_state,
                                         std::vector<double> &solution,
                                         int &error_code)
{
  return getPositionIK(ik_pose,
                       ik_seed_state,
                       default_posture_,
                       solution,
                       error_code);
}

bool ReemKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              const double &timeout,
                                              std::vector<double> &solution,
                                              int &error_code)
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::INACTIVE;
    return false;
  }
  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);

  ROS_DEBUG_STREAM("searchPositionIK1:Position request pose is " <<
                   ik_pose.position.x << " " <<
                   ik_pose.position.y << " " <<
                   ik_pose.position.z << " " <<
                   ik_pose.orientation.x << " " << 
                   ik_pose.orientation.y << " " << 
                   ik_pose.orientation.z << " " << 
                   ik_pose.orientation.w);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  KDL::JntArray posture(dimension_);
  for(unsigned int i=0; i < dimension_; ++i)
  {
    jnt_pos_in(i) = ik_seed_state[i];
    posture(i)    = default_posture_[i];
  }
  for(int i=0; i < max_search_iterations_; ++i)
  {
    for(unsigned int j=0; j < dimension_; ++j)
    { 
      ROS_DEBUG_STREAM("seed state " << j << " " << jnt_pos_in(j));
    }
    ik_solver_->setPosture(posture);
    bool ik_valid = ik_solver_->solve(jnt_pos_in, pose_desired, jnt_pos_out);
    ROS_DEBUG_STREAM("IK success " << ik_valid << " time " << (ros::WallTime::now()-n1).toSec());
    if(ik_valid) {
      solution.resize(dimension_);
      for(unsigned int j=0; j < dimension_; ++j) {
        solution[j] = jnt_pos_out(j);
      }
      error_code = kinematics::SUCCESS;
      ROS_DEBUG_STREAM("Solved after " << i+1 << " iterations");
      return true;
    }      
    jnt_pos_in = getRandomConfiguration();
  }
  ROS_DEBUG("An IK solution could not be found");   
  error_code = kinematics::NO_IK_SOLUTION;
  return false;
}

bool ReemKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            const double &timeout,
                                            const unsigned int& redundancy,
                                            const double &consistency_limit,
                                            std::vector<double> &solution,
                                            int &error_code)
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::INACTIVE;
    return false;
  }
  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);

  ROS_DEBUG_STREAM("searchPositionIK1:Position request pose is " <<
                   ik_pose.position.x << " " <<
                   ik_pose.position.y << " " <<
                   ik_pose.position.z << " " <<
                   ik_pose.orientation.x << " " <<
                   ik_pose.orientation.y << " " <<
                   ik_pose.orientation.z << " " <<
                   ik_pose.orientation.w);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  KDL::JntArray jnt_seed_state;
  jnt_pos_in.resize(dimension_);
  KDL::JntArray posture(dimension_);
  for(unsigned int i=0; i < dimension_; ++i)
  {
    jnt_seed_state(i) = ik_seed_state[i];
    posture(i)        = default_posture_[i];
  }
  jnt_pos_in = jnt_seed_state;
  for(int i=0; i < max_search_iterations_; ++i)
  {
    for(unsigned int j=0; j < dimension_; ++j)
    {
      ROS_DEBUG_STREAM("seed state " << j << " " << jnt_pos_in(j));
    }
    ik_solver_->setPosture(posture);
    int ik_valid = ik_solver_->solve(jnt_pos_in, pose_desired, jnt_pos_out);
    ROS_DEBUG_STREAM("IK success " << ik_valid << " time " << (ros::WallTime::now()-n1).toSec());
    if(ik_valid >= 0 && checkConsistency(jnt_seed_state, redundancy, consistency_limit, jnt_pos_out)) {
      solution.resize(dimension_);
      for(unsigned int j=0; j < dimension_; j++) {
        solution[j] = jnt_pos_out(j);
      }
      error_code = kinematics::SUCCESS;
      ROS_DEBUG_STREAM("Solved after " << i+1 << " iterations");
      return true;
    }
    jnt_pos_in = getRandomConfiguration(jnt_seed_state, redundancy, consistency_limit);
  }
  ROS_DEBUG("An IK solution could not be found");
  error_code = kinematics::NO_IK_SOLUTION;
  return false;
}

bool ReemKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              const double &timeout,
                                              std::vector<double> &solution,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                              int &error_code)  
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::INACTIVE;
    return false;
  }
  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);

  ROS_DEBUG_STREAM("searchPositionIK2: Position request pose is " <<
                   ik_pose.position.x << " " <<
                   ik_pose.position.y << " " <<
                   ik_pose.position.z << " " <<
                   ik_pose.orientation.x << " " << 
                   ik_pose.orientation.y << " " << 
                   ik_pose.orientation.z << " " << 
                   ik_pose.orientation.w);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  KDL::JntArray posture(dimension_);
  for(unsigned int i=0; i < dimension_; ++i)
  {
    jnt_pos_in(i) = ik_seed_state[i];
    posture(i)    = default_posture_[i];
  }

  if(!desired_pose_callback.empty())
    desired_pose_callback(ik_pose,ik_seed_state,error_code);

  if(error_code < 0)
  {
    ROS_DEBUG("Could not find inverse kinematics for desired end-effector pose since the pose may be in collision");
    return false;
  }
  for(int i=0; i < max_search_iterations_; ++i)
  {
    ik_solver_->setPosture(posture);
    bool ik_valid = ik_solver_->solve(jnt_pos_in, pose_desired, jnt_pos_out);
    jnt_pos_in = getRandomConfiguration();
    if(!ik_valid)
      continue;
    std::vector<double> solution_local;
    solution_local.resize(dimension_);
    for(unsigned int j=0; j < dimension_; ++j)
      solution_local[j] = jnt_pos_out(j);
    solution_callback(ik_pose,solution_local,error_code);
    if(error_code == kinematics::SUCCESS)
    {
      solution = solution_local;
      ROS_DEBUG_STREAM("Solved after " << i+1 << " iterations");
      return true;
    }
  }
  ROS_DEBUG("An IK that satisifes the constraints and is collision free could not be found");   
  error_code = kinematics::NO_IK_SOLUTION;
  return false;
}

bool ReemKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            const double &timeout,
                                            const unsigned int& redundancy,
                                            const double& consistency_limit,
                                            std::vector<double> &solution,
                                            const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                            const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                            int &error_code)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::INACTIVE;
    return false;
  }
  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);

  ROS_DEBUG_STREAM("searchPositionIK2: Position request pose is " <<
                   ik_pose.position.x << " " <<
                   ik_pose.position.y << " " <<
                   ik_pose.position.z << " " <<
                   ik_pose.orientation.x << " " <<
                   ik_pose.orientation.y << " " <<
                   ik_pose.orientation.z << " " <<
                   ik_pose.orientation.w);

  //Do the IK
  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  KDL::JntArray jnt_seed_state;
  jnt_pos_in.resize(dimension_);
  KDL::JntArray posture(dimension_);
  for(unsigned int i=0; i < dimension_; ++i)
  {
    jnt_seed_state(i) = ik_seed_state[i];
    posture(i)        = default_posture_[i];
  }
  jnt_pos_in = jnt_seed_state;

  if(!desired_pose_callback.empty())
    desired_pose_callback(ik_pose,ik_seed_state,error_code);

  if(error_code < 0)
  {
    ROS_DEBUG("Could not find inverse kinematics for desired end-effector pose since the pose may be in collision");
    return false;
  }
  for(int i=0; i < max_search_iterations_; ++i)
  {
    ik_solver_->setPosture(posture);
    int ik_valid = ik_solver_->solve(jnt_pos_in,pose_desired,jnt_pos_out);
    jnt_pos_in = getRandomConfiguration(jnt_seed_state, redundancy, consistency_limit);
    if(ik_valid < 0 || !checkConsistency(jnt_seed_state, redundancy, consistency_limit, jnt_pos_out))
      continue;
    std::vector<double> solution_local;
    solution_local.resize(dimension_);
    for(unsigned int j=0; j < dimension_; j++)
      solution_local[j] = jnt_pos_out(j);
    solution_callback(ik_pose,solution_local,error_code);
    if(error_code == kinematics::SUCCESS)
    {
      solution = solution_local;
      ROS_DEBUG_STREAM("Solved after " << i+1 << " iterations");
      return true;
    }
  }
  ROS_DEBUG("An IK that satisifes the constraints and is collision free could not be found");
  error_code = kinematics::NO_IK_SOLUTION;
  return false;
}

bool ReemKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }
  
  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;
  
  jnt_pos_in.resize(dimension_);
  for(unsigned int i=0; i < dimension_; ++i)
  {
    jnt_pos_in(i) = joint_angles[i];
  }
  
  poses.resize(link_names.size());
  
  bool valid = true;
  for(unsigned int i=0; i < poses.size(); ++i)
  {
    if(fk_solver_->JntToCart(jnt_pos_in,p_out,getKDLSegmentIndex(link_names[i])) >=0)
    {
      tf::PoseKDLToMsg(p_out,poses[i]);
    }
    else
    {
      ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

std::string ReemKinematicsPlugin::getBaseFrame()
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return std::string("");
  }
  return base_name_;
}

std::string ReemKinematicsPlugin::getToolFrame()
{
  if(!active_ || chain_info_.link_names.empty())
  {
    ROS_ERROR("kinematics not active");
    return std::string("");
  }
  return chain_info_.link_names[0];
}

const std::vector<std::string>& ReemKinematicsPlugin::getJointNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return chain_info_.joint_names;
}

const std::vector<std::string>& ReemKinematicsPlugin::getLinkNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return chain_info_.link_names;
}

} // namespace
