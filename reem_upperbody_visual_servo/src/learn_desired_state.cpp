//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Subscribes to TF to obtain the pose of /marker_object_frame in /marker_hand_frame and stores the pose in
//
//    $(find reem_upperbody_visual_servo)/etc/desiredPose.txt
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \author Jordi Pages. */

#include "utility_functions.h"
#include "conversions.h"
#include "tf_utils.h"

// ROS headers
#include <ros/package.h>
#include <tf/tf.h>

// Other headers
#include <yaml-cpp/yaml.h>

// Eigen
#include <Eigen/Core>

// Std C++ headers
#include <vector>
#include <string>
#include <fstream>

int main(int argc, char **argv)
{  
  ros::init(argc,argv,"learn_desired_state"); // Create and name the Node
  ros::NodeHandle node;

  std::string objectMarkerFrame = "/marker_object_frame";
  std::string handMarkerFrame = "/marker_hand_frame";
  //get pose of the object marker frame in the hand marker frame from TF
  tf::TransformListener tfListener;
  tf::StampedTransform omMhm;

  std::cout << "Waiting for both markers in TF ..." << std::endl;
  pal::waitUntilTransform(tfListener, objectMarkerFrame, handMarkerFrame, omMhm);

  std::string path = ros::package::getPath("reem_upperbody_visual_servo");
  std::string fileName = path + "/etc/desired_state.yml";

  std::cout << "Writing file " << fileName << std::endl;

  YAML::Emitter emitter;

  emitter << YAML::Comment("Pose of /marker_hand_frame in /marker_object_frame expressed with a translation vector and the angle-axis rotation");
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "trans_x" << YAML::Value << static_cast<double>(omMhm.getOrigin().x());
  emitter << YAML::Key << "trans_y" << YAML::Value << static_cast<double>(omMhm.getOrigin().y());
  emitter << YAML::Key << "trans_z" << YAML::Value << static_cast<double>(omMhm.getOrigin().z());
  emitter << YAML::Key << "q_x"  << YAML::Value << static_cast<double>(omMhm.getRotation().x());
  emitter << YAML::Key << "q_y"  << YAML::Value << static_cast<double>(omMhm.getRotation().y());
  emitter << YAML::Key << "q_z"  << YAML::Value << static_cast<double>(omMhm.getRotation().z());
  emitter << YAML::Key << "q_w"  << YAML::Value << static_cast<double>(omMhm.getRotation().w());

  emitter << YAML::EndMap;

  std::ofstream out(fileName.c_str());
  out << emitter.c_str();

  return 0;
}
