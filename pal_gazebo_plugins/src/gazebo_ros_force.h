#ifndef GAZEBO_ROS_FORCE_H
#define GAZEBO_ROS_FORCE_H

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Wrench.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosForce Plugin XML Reference and Example

  \brief Ros Force Plugin.

  This is a Plugin that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_force.so" name="gazebo_ros_force">
          <bodyName>box_body</bodyName>
          <topicName>box_force</topicName>
        </plugin>
      </gazebo>
  \endverbatim

\{
*/

/**
           .

*/

class GazeboRosForce : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosForce();

  /// \brief Destructor
  public: virtual ~GazeboRosForce();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateChild();

  /// \brief call back when a Wrench message is published
  /// \param[in] _msg The Incoming ROS message representing the new force to exert.
  private: void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr &_msg);

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world;

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosNode;
  private: ros::Subscriber sub;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock;

  /// \brief ROS Wrench topic name inputs
  private: std::string topicName;
  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string linkName;

  /// \brief for setting ROS name space
  private: std::string robotNamespace;

  // Custom Callback Queue
  private: ros::CallbackQueue queue;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callbackQueueThread;
  /// \brief Container for the wrench force that this plugin exerts on the body.
  private: geometry_msgs::Wrench wrenchMsg;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
};
/** \} */
/// @}
}

#endif // GAZEBO_ROS_FORCE_H
