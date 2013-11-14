/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_ROS_P3D_HH
#define GAZEBO_ROS_P3D_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <tf/transform_broadcaster.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
  class GazeboRosP3D : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRosP3D();

    /// \brief Destructor
    public: virtual ~GazeboRosP3D();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;

    /// \brief The parent Model
    private: physics::LinkPtr link_;

    /// \brief The body of the frame to display pose, twist
    private: physics::LinkPtr reference_link_;


    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;

    /// \brief ros message
    private: nav_msgs::Odometry pose_msg_;

    /// \brief store bodyname
    private: std::string link_name_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    /// FIXME: extract link name directly?
    private: std::string frame_name_;
    private: std::string tf_frame_name_;

    /// \brief allow specifying constant xyz and rpy offsets
    private: math::Pose offset_;

    /// \brief mutex to lock access to fields used in message callbacks
    private: boost::mutex lock;

    /// \brief save last_time
    private: common::Time last_time_;
    private: math::Vector3 last_vpos_;
    private: math::Vector3 last_veul_;
    private: math::Vector3 apos_;
    private: math::Vector3 aeul_;
    private: math::Vector3 last_frame_vpos_;
    private: math::Vector3 last_frame_veul_;
    private: math::Vector3 frame_apos_;
    private: math::Vector3 frame_aeul_;

    // tf broadcaster
    private: tf::TransformBroadcaster tf_br_;

    // rate control
    private: double update_rate_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu, double sigma);

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue p3d_queue_;
    private: void P3DQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    private: unsigned int seed;
  };
}
#endif
