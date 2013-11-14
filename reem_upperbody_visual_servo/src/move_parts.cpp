
#include "move_parts.h"

namespace pal {


  JointGroupCommander::JointGroupCommander(const std::string& controllerName):
    _controllerName(controllerName),
    _actionClient(_controllerName + "/follow_joint_trajectory")
  {
    _jointTrajPub = _nh.advertise<trajectory_msgs::JointTrajectory>(_controllerName + "/command", 1, true);

    updateJointNames();
  }

  JointGroupCommander::~JointGroupCommander()
  {
  }

  void JointGroupCommander::send(const std::vector<double>& jointsDesiredState,
                                 const std::vector<double>& endVelocities,
                                 const ros::Duration& duration)
  {
    trajectory_msgs::JointTrajectory jointTrajMsg;

    jointTrajMsg.joint_names = _jointNames;

    trajectory_msgs::JointTrajectoryPoint jointTrajPointMsg;


    //we want to reach the given joint positions
    jointTrajPointMsg.positions  = jointsDesiredState;
    //we want to reach the desired state with 0 velocity
    jointTrajPointMsg.velocities = endVelocities;
    //and we do it in a given time
    jointTrajPointMsg.time_from_start = duration;

    jointTrajMsg.points.push_back(jointTrajPointMsg);

    jointTrajMsg.header.stamp = ros::Time::now() + ros::Duration(0.1);

    _jointTrajPub.publish(jointTrajMsg);
  }

  void JointGroupCommander::updateJointNames()
  {
    // Wait for the action client to be connected to the server
    if (!_actionClient.waitForServer(ros::Duration(5.0)))
    {
      std::runtime_error("Error in JointGroupCommander::updateJointNames: time-out trying to connect to " +
                         _controllerName + " FollowJointTrajectory action server");
    }

    // Get list of joints used by the controller
    _jointNames.clear();
    using namespace XmlRpc;
    XmlRpcValue joint_names;
    ros::NodeHandle nh(_controllerName);
    if (!nh.getParam("joints", joint_names))
      throw std::runtime_error("Error in JointGroupCommander::updateJointNames: no joints given. (namespace: " + nh.getNamespace() + ")");
    if (joint_names.getType() != XmlRpcValue::TypeArray)
      throw std::runtime_error("Error in JointGroupCommander::updateJointNames: malformed joint specification");
    for (int i = 0; i < joint_names.size(); ++i)
    {
      XmlRpcValue &name_value = joint_names[i];
      if (name_value.getType() != XmlRpcValue::TypeString)
        throw std::runtime_error("Error in JointGroupCommander::updateJointNames: array of joint names should contain all strings");
      _jointNames.push_back(static_cast<std::string>(name_value));
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  MoveParts::MoveParts():
    _torsoCommand("/torso_controller"),
    _rightArmCommand("/right_arm_controller"),
    _headCommand("/head_controller")
  {
    _jointTrajRightHand_pub = _nh.advertise<trajectory_msgs::JointTrajectory>("/right_hand_controller/command", 1, true);

    _isSimulation = false;
    //ros::param::get("/use_sim_time", _isSimulation);

    initHandTrajectories();
  }

  MoveParts::~MoveParts()
  {
  }

  void MoveParts::move(robotPart part,
                       const std::vector<double>& jointsDesiredState,
                       const std::vector<double>& endVelocities,
                       const ros::Duration& duration)
  {
    switch(part)
    {
      case TORSO:
        _torsoCommand.send(jointsDesiredState, endVelocities, duration);
        break;
      case RIGHT_ARM:
        _rightArmCommand.send(jointsDesiredState, endVelocities, duration);
        break;
      case HEAD:
        _headCommand.send(jointsDesiredState, endVelocities, duration);
        break;
    }
  }

  void MoveParts::initHandTrajectories()
  {
    trajectory_msgs::JointTrajectory jntTrajMsg;
    jntTrajMsg.joint_names.push_back("hand_right_thumb_joint");
    jntTrajMsg.joint_names.push_back("hand_right_index_joint");
    jntTrajMsg.joint_names.push_back("hand_right_middle_joint");

    if ( _isSimulation )
    {
      //when running simulation more joints must be specified
      jntTrajMsg.joint_names.push_back("hand_right_index_1_joint");
      jntTrajMsg.joint_names.push_back("hand_right_index_2_joint");
      jntTrajMsg.joint_names.push_back("hand_right_index_3_joint");
      jntTrajMsg.joint_names.push_back("hand_right_middle_1_joint");
      jntTrajMsg.joint_names.push_back("hand_right_middle_2_joint");
      jntTrajMsg.joint_names.push_back("hand_right_middle_3_joint");
    }

    //set all joints to 0
    trajectory_msgs::JointTrajectoryPoint jointTrajPointMsg;
    for (std::size_t i = 0; i < jntTrajMsg.joint_names.size(); ++i)
    {
      jointTrajPointMsg.positions.push_back(0);
      jointTrajPointMsg.velocities.push_back(0);
    }

    jointTrajPointMsg.time_from_start = ros::Duration(1.0);
    jntTrajMsg.points.push_back(jointTrajPointMsg);

    _handTrajectories.clear();

    // HAND_OPENED
    //do nothing as this pose needs all joints at 0
    _handTrajectories.push_back(jntTrajMsg);

    // HAND_PREGRASP
    jntTrajMsg.points[0].positions[0] = 1.3; //close the thumb
    _handTrajectories.push_back(jntTrajMsg);

    // HAND_GRASP
    jntTrajMsg.points[0].positions[1] = 2.9; //close the finger
    jntTrajMsg.points[0].positions[2] = 2.9; //close the middle

    _handTrajectories.push_back(jntTrajMsg);
  }

  void MoveParts::moveRightHand(handPose pose)
  {
    int poseIdx = static_cast<int>(pose);

    _handTrajectories[poseIdx].header.stamp = ros::Time::now();

//    std::cout << "Hand trajectory " << poseIdx << ":" << std::endl;
//    for (unsigned int i = 0; i < _handTrajectories[poseIdx].points.size(); ++i)
//      for (unsigned int j = 0; j < _handTrajectories[poseIdx].points[i].positions.size(); ++j)
//        std::cout << "point " << i << ": " << " position " << j << ": " << _handTrajectories[poseIdx].points[i].positions[j] << std::endl;

    _jointTrajRightHand_pub.publish(_handTrajectories[poseIdx]);
    ros::Duration(0.5).sleep();
  }

} //pal
