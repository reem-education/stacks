//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// VISUAL SERVO THE WHOLE UPPER BODY OF REEM - do both gaze and arm control simultaneously
//
//
//  Control laws (not including secondary task):
//
//   qdot(ARM+TORSO) = -λ_1 · (L · ^hmV_b · ^bJ_h)^+ · e(ARM+TORSO)
//
//   qdot(HEAD) = (L_mid(^cO_om,^cO_h) · ^cV_t · ^tJ_n)^+ · (-λ_2 · e_mid - L_mid(^cO_om,^cO_h)·^cV_b·^bJ_t·qdot(TORSO) -0.5·L_^cO_h·^cV_t·^tJ_h·qdot(ARM)
//
//  where
//
//     qdot(ARM+TORSO) is a 9x1 vector
//     qdot(ARM)       is a 7x1 vector
//     qdot(TORSO)     is a 2x1 vector
//     qdot(HEAD)      is a 2x1 vector
//     {c}             is the camera frame, i.e. the left eye of the robot
//     {h}             is the hand frame, placed in the last joint of the arm
//     {hm}            is the frame of the marker attached to the hand
//     {om}            is the frame of the marker attached to the object
//     {b}             is the base frame of the robot
//     {t}             is the frame corresponding to the second (and last) torso joint
//     {n}             is the frame corresponding to the second (and last) neck joint
//     ^hmV_b          is the twist transformation matrix from {b} to {hm}
//     ^cV_t           is the twist transformation matrix from {t} to {c}
//     ^cV_b           is the twist transformation matrix from {b} to {c}
//     ^bJ_h           is the robot jacobian of the hand frame expressed in the the robot base frame
//     ^tJ_n           is the robot jacobian of the last neck joint expressed in the second torso frame, which is the parent of the head kinematic chain according to KDL conventions
//     ^bJ_t           is the robot jacobian of the terminal torso frame expressed in the robot base frame, which is the parent of the torso kinematic chain
//     ^tJ_h           is the robot jacobian of the hand frame expressed in the second torso frame, which is the parent of the arm kinematic chain
//
//
// To plot the task error of the HEAD:
//
//   rxplot -p 100 /VS_errors/data[0]:data[1] -l "mid point x","mid point y" -t "gaze task error"
//
// To plot the HEAD joint velocities:
//
//   rxplot -p 100 /VS_errors/data[2]:data[3] -l "head 1","head 2" -t "head joint velocities (rad/s)"
//
// To plot the task error of the TORSO+ARM:
//
//   rxplot -p 100 /VS_errors/data[4]:data[5]:data[6]:data[7]:data[8]:data[9] -l "linear x","linear y","linear z","angular x","angular y","angular z" -t "task error"
//
// To plot the TORSO joint velocities:
//
//   rxplot -p 100 /VS_errors/data[10]:data[11] -l "torso 1","torso 2" -t "torso joint velocities (rad/s)"
//
// To plot the ARM joint velocities:
//
//   rxplot -p 100 /VS_errors/data[12]:data[13]:data[14]:data[15]:data[16]:data[17]:data[18] -l "arm 1","arm 2","arm 3","arm 4","arm 5","arm 6","arm 7" -t "arm joint velocities (rad/s)"
//
// PRIMARY AND SECONDARY TASK:
// ===========================
//
// To plot q1dot of HEAD joints:
//
//   rxplot -p 100 /VS_errors/data[19]:data[20] -l "head 1","head 2" -t "primary task head joint velocities (rad/s)"
//
// To plot q2dot of HEAD joints:
//
//   rxplot -p 100 /VS_errors/data[21]:data[22] -l "head 1","head 2" -t "secondary task head joint velocities (rad/s)"
//
// To plot q1dot of TORSO joints:
//
//   rxplot -p 100 /VS_errors/data[23]:data[24] -l "torso 1","torso 2" -t "primary task torso joint velocities (rad/s)"
//
// To plot q1dot of ARM joints:
//
//   rxplot -p 100 /VS_errors/data[25]:data[26]:data[27]:data[28]:data[29]:data[30]:data[31] -l "arm 1","arm 2","arm 3","arm 4","arm 5","arm 6","arm 7" -t "primary task arm joint velocities (rad/s)"
//
// To plot q2dot of TORSO joints:
//
//   rxplot -p 100 /VS_errors/data[32]:data[33] -l "torso 1","torso 2" -t "secondary task torso joint velocities (rad/s)"
//
// To plot q2dot of ARM joints:
//
//   rxplot -p 100 /VS_errors/data[34]:data[35]:data[36]:data[37]:data[38]:data[39]:data[40] -l "arm 1","arm 2","arm 3","arm 4","arm 5","arm 6","arm 7" -t "secondary task arm joint velocities (rad/s)"
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Utility Functions
#include "utility_functions.h"
#include "tf_utils.h"
#include "conversions.h"

// Secondary Task Functions
#include "secondary_task.h"

#include "move_parts.h"

// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <gazebo/GetModelState.h>
#include <gazebo/SetModelState.h>
#include <tf/tf.h>
#include <LinearMath/btMatrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <angles/angles.h>
#include <eigen3/Eigen/Eigen>

// VISP libraries
#include <visp/vpMath.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpFeaturePoint.h>

// Other headers
#include <yaml-cpp/yaml.h>

// Standard C++ headers
#include <string>
#include <cstdlib>


bool jointStatesReceived;
double upperBodyJointState[11]; //2 head joints + 2 torso joints + 7 right arm joints

void jointStatesCallback(const sensor_msgs::JointState& msg)
{ 
  int joints_obtained = 0;
  for (unsigned int i = 0; i < msg.name.size(); ++i)
  {
    if ( msg.name[i] == "head_1_joint" )
    {
      upperBodyJointState[0] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "head_2_joint" )
    {
      upperBodyJointState[1] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "torso_1_joint" )
    {
      upperBodyJointState[2] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "torso_2_joint" )
    {
      upperBodyJointState[3] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "arm_right_1_joint" )
    {
      upperBodyJointState[4] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "arm_right_2_joint" )
    {
      upperBodyJointState[5] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "arm_right_3_joint" )
    {
      upperBodyJointState[6] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "arm_right_4_joint" )
    {
      upperBodyJointState[7] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "arm_right_5_joint" )
    {
      upperBodyJointState[8] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "arm_right_6_joint" )
    {
      upperBodyJointState[9] = msg.position[i];
      ++joints_obtained;
    }
    else if ( msg.name[i] == "arm_right_7_joint" )
    {
      upperBodyJointState[10] = msg.position[i];
      ++joints_obtained;
    }
  }

  if ( joints_obtained != 11 )
    throw std::runtime_error("Not all the required joints have been read from joint_states!");

  jointStatesReceived = true;
}

void printJointStates(double *v)
{
  ROS_INFO_STREAM("head 1: " << v[0] << " head 2: " << v[1]);
  ROS_INFO_STREAM("torso 1: " << v[2] << " torso 2: " << v[3]);
  ROS_INFO_STREAM("right arm 1: " << v[4] << " arm 2: " << v[5] << " arm 3: " << v[6]);
  ROS_INFO_STREAM("      arm 4: " << v[7] << " arm 5: " << v[8] << " arm 6: " << v[9] << " arm7: " << v[10]);
  ROS_INFO(" ");
}

int main(int argc, char **argv)
{
  {
    ros::init(argc,argv,"reem_upperbody_visual_servo_upperbody"); // Create and name the Node
    ros::NodeHandle node;

    //set the right hand at pre-grasp pose
    pal::MoveParts moveParts;
    moveParts.moveRightHand(pal::MoveParts::HAND_PREGRASP);


    std::string desiredStateFileName = ros::package::getPath("reem_upperbody_visual_servo") + "/desired_state.yml";
    if ( argc > 1 )
    {
      desiredStateFileName = argv[1];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    bool STEP_BY_STEP              = false;
    bool ENABLE_HEAD               = true;
    bool ENABLE_ARM_AND_TORSO      = true;
    bool ENABLE_TORSO_COMPENSATION = true;
    bool ENABLE_ARM_COMPENSATION   = false;
    enum midPointGazing { USE_3D = 0, USE_2D = 1 };
    midPointGazing GAZING_TASK_MID_POINT = USE_3D;
    enum midPointInteractionMatrix { USE_CLASSIC_POINT_L = 0, USE_MID_POINT_L = 1 };
    midPointInteractionMatrix GAZING_TASK_INTERACTION_MATRIX = USE_MID_POINT_L;
    double lambdaTorsoArm          = 0.2;
    double lambdaHead              = 0.4;
    double maxVelocity             = 0.3;
    double timeStep                = 0.3; //seconds per iteration
    double timeFromStartFactor     = 3.0;
    bool   CLOSE_HAND_IN_GOAL      = false;
    //////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONFIGURATION
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    enum jointLimitAvoidanceAlgorithm { NONE = 0, CLASSICAL = 1, LARGE_PROJECTION = 2 };
    int JointLimAvAlg;

    if ( !node.getParam(ros::this_node::getName() + "/joint_limit_avoidance_algorithm",  JointLimAvAlg) )
      JointLimAvAlg = LARGE_PROJECTION;

    ROS_INFO_STREAM("JOINT LIMIT AVOIDANCE ALGORITHM: " << JointLimAvAlg);

    if ( !node.getParam(ros::this_node::getName() + "/step_by_step",  STEP_BY_STEP) )
      STEP_BY_STEP = false;

    ROS_INFO_STREAM("RUN STEP BY STEP: " << STEP_BY_STEP);

    if ( !node.getParam(ros::this_node::getName() + "/enable_torso_arm",  ENABLE_ARM_AND_TORSO) )
      ENABLE_ARM_AND_TORSO = false;

    ROS_INFO_STREAM("Enable torso+arm movement: " << ENABLE_ARM_AND_TORSO);

    if ( !node.getParam(ros::this_node::getName() + "/enable_head",  ENABLE_HEAD) )
      ENABLE_HEAD = false;

    ROS_INFO_STREAM("Enable head movement: " << ENABLE_HEAD);

    if ( !node.getParam(ros::this_node::getName() + "/torso_compensation_to_gaze",  ENABLE_TORSO_COMPENSATION) )
      ENABLE_TORSO_COMPENSATION = false;

    ROS_INFO_STREAM("Torso motion compensation to gaze control: " << ENABLE_TORSO_COMPENSATION);

    if ( !node.getParam(ros::this_node::getName() + "/torso_arm_compensation_to_gaze",  ENABLE_ARM_COMPENSATION) )
      ENABLE_ARM_COMPENSATION = false;

    ROS_INFO_STREAM("Torso and arm motion compensation to gaze control: " << ENABLE_ARM_COMPENSATION);

    if ( !node.getParam(ros::this_node::getName() + "/gain_torso_arm",  lambdaTorsoArm) )
      lambdaTorsoArm = 0.2;

    ROS_INFO_STREAM("Gain of toros+arm control law: " << lambdaTorsoArm);

    if ( !node.getParam(ros::this_node::getName() + "/gain_head",  lambdaHead) )
      lambdaHead = 0.2;

    ROS_INFO_STREAM("Gain of head control law: " << lambdaHead);

    if ( !node.getParam(ros::this_node::getName() + "/max_joint_velocity",  maxVelocity) )
      maxVelocity = 0.2;

    ROS_INFO_STREAM("Max joint velocity (rad/s): " << maxVelocity);

    if ( !node.getParam(ros::this_node::getName() + "/max_joint_velocity",  maxVelocity) )
      maxVelocity = 0.2;

    ROS_INFO_STREAM("Max joint velocity (rad/s): " << maxVelocity);

    if ( !node.getParam(ros::this_node::getName() + "/time_step",  timeStep) )
      timeStep = 0.3;

    ROS_INFO_STREAM("Seconds per iteration: " << timeStep);

    if ( !node.getParam(ros::this_node::getName() + "/time_from_start_factor",  timeFromStartFactor) )
      timeFromStartFactor = 0.3;

    ROS_INFO_STREAM("Time from start factor: " << timeFromStartFactor);

    if ( !node.getParam(ros::this_node::getName() + "/close_hand",  CLOSE_HAND_IN_GOAL) )
      CLOSE_HAND_IN_GOAL = false;

    ROS_INFO_STREAM("Close hand when goal reached: " << CLOSE_HAND_IN_GOAL);

    //////////////////////////////////////////////////////////////////////////////////////////////////////

    // Listen to the tf topic for the transformations
    tf::TransformListener tfListener; // Important to be here due to TF scoping
    // Broadcast to the tf topic for visualizing target pose
    static tf::TransformBroadcaster br;

    //============================= Publishers of this node =============================
    //------------for RVIZ trajectory markers
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    long long markerID = 1;
    //------------ to move the torso
    ros::Publisher jointTrajtorso_pub = node.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1, true);
    //------------ to move the arm
    ros::Publisher jointTrajArm_pub   = node.advertise<trajectory_msgs::JointTrajectory>("/right_arm_controller/command", 1, true);
    //------------ to move the head
    ros::Publisher jointTrajHead_pub  = node.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1, true);
    //------------ for the error plots
    ros::Publisher plot_pub = node.advertise<std_msgs::Float64MultiArray>("/VS_errors", 1);

    //============================= Initialize Kinematics =============================
    //------------important reference frames
    std::string eyeFrame          = "/stereo_gazebo_l_stereo_camera_optical_frame";
    std::string handFrame         = "arm_right_7_link";
    std::string headFrame         = "head_2_link";
    std::string baseFrame         = "base_link";
    std::string torsoFrame        = "torso_2_link";
    std::string arHandMarkerFrame = "marker_hand_frame";   // TF from detection, must run ar_multi node !!!
    std::string arObjMarkerFrame  = "marker_object_frame"; // TF from detection, must run ar_multi node !!!

    //------------ Read the REEM model from a ROS param
    KDL::Tree kdlJointTree;
    std::string URDFstring;
    node.param("robot_description", URDFstring, std::string());

    //------------ Make a KDL joint tree from the URDF
    if (!kdl_parser::treeFromString(URDFstring, kdlJointTree))
    {
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }

    //============================= Create a KDL joint chain =============================
    //------------for the RIGHT ARM
    KDL::Chain kdlArmChain;
    if(!kdlJointTree.getChain(baseFrame, handFrame, kdlArmChain))
    { ROS_ERROR("Failed to get arm chain"); }
    KDL::ChainJntToJacSolver kdlJointToJacobianArm(kdlArmChain);
    KDL::JntArray kdlTorsoArmPosition;
    KDL::Jacobian kdlTorsoArmJacobian;
    unsigned int number_of_torsoAndArm_joints;
    number_of_torsoAndArm_joints = kdlArmChain.getNrOfJoints();
    kdlTorsoArmPosition.resize(number_of_torsoAndArm_joints);
    kdlTorsoArmJacobian.resize(number_of_torsoAndArm_joints);

    //------------for the HEAD
    KDL::Chain kdlHeadChain;
    if(!kdlJointTree.getChain(torsoFrame, headFrame, kdlHeadChain))
    { ROS_ERROR("Failed to get head chain"); }
    KDL::ChainJntToJacSolver kdlJointToJacobianHead(kdlHeadChain);
    KDL::JntArray kdlHeadPosition;
    KDL::Jacobian kdlHeadJacobian;
    unsigned int number_of_head_joints;
    number_of_head_joints = kdlHeadChain.getNrOfJoints();
    kdlHeadPosition.resize(number_of_head_joints);
    kdlHeadJacobian.resize(number_of_head_joints);

    //------------for the TORSO (for feedforward component)
    KDL::Chain kdlTorsoChain;
    if(!kdlJointTree.getChain(baseFrame, torsoFrame, kdlTorsoChain))
    { ROS_ERROR("Failed to get torso chain"); }
    KDL::ChainJntToJacSolver kdlJointToJacobianTorso(kdlTorsoChain);
    KDL::JntArray kdlTorsoPosition;
    KDL::Jacobian kdlTorsoJacobian;
    unsigned int number_of_torso_joints;
    number_of_torso_joints = kdlTorsoChain.getNrOfJoints();
    kdlTorsoPosition.resize(number_of_torso_joints);
    kdlTorsoJacobian.resize(number_of_torso_joints);

    //------------for the RIGHT ARM without TORSO (for feedforward component)
    KDL::Chain kdlArmOnlyChain;
    if(!kdlJointTree.getChain(torsoFrame, handFrame, kdlArmOnlyChain))
    { ROS_ERROR("Failed to get armOnly chain"); }
    KDL::ChainJntToJacSolver kdlJointToJacobianArmOnly(kdlArmOnlyChain);
    KDL::JntArray kdlArmOnlyPosition;
    KDL::Jacobian kdlArmOnlyJacobian;
    unsigned int number_of_armOnly_joints;
    number_of_armOnly_joints = kdlArmOnlyChain.getNrOfJoints();
    kdlArmOnlyPosition.resize(number_of_armOnly_joints);
    kdlArmOnlyJacobian.resize(number_of_armOnly_joints);

    //============================= Read joint limits from URDF file =============================
    //------------for the RIGHT ARM
    std::vector<double> q_min_torsoAndArm;
    std::vector<double> q_max_torsoAndArm;
    q_min_torsoAndArm.resize(number_of_torsoAndArm_joints);
    q_max_torsoAndArm.resize(number_of_torsoAndArm_joints);
    utility_functions::getJointLimitsFromString(q_min_torsoAndArm, q_max_torsoAndArm, URDFstring, kdlArmChain);
    // !!! changed to try to make shoulder joint not bump into torso
    q_min_torsoAndArm.at(3) = -0.15;
    //------------for the HEAD
    std::vector<double> q_min_head;
    std::vector<double> q_max_head;
    q_min_head.resize(number_of_head_joints);
    q_max_head.resize(number_of_head_joints);
    utility_functions::getJointLimitsFromString(q_min_head, q_max_head, URDFstring, kdlHeadChain);
    //let the head lower 30 degrees, otherwise it is really difficult to reach any tabletop task
    q_max_head.at(1) = 30.0*M_PI/180.0;

    //============================= Get start positions =============================

    //////////////////////////////////////////////////////////////////////////////////////////////
    // GET START POSITION FROM joint_states
    //////////////////////////////////////////////////////////////////////////////////////////////
    jointStatesReceived = false;
    ros::Subscriber jointStateSub = node.subscribe("/joint_states", 1, jointStatesCallback);

    ROS_INFO("-------- Waiting joint states ----------");
    while ( !jointStatesReceived && ros::ok() )
    {
      ros::Duration(0.2).sleep();
      ros::spinOnce();
    }

    kdlHeadPosition.data[0] = upperBodyJointState[0];
    kdlHeadPosition.data[1] = upperBodyJointState[1];
    for (unsigned int i = 0; i < number_of_torsoAndArm_joints; ++i)
      kdlTorsoArmPosition.data[i] = upperBodyJointState[i+2];

    printJointStates(upperBodyJointState);
    //////////////////////////////////////////////////////////////////////////////////////////////

    //============================= Initialize important frame transformations =============================
    //------------Transformation to the Marker with the Camera as Reference Frame
    vpHomogeneousMatrix cMhm; // Camera Position --> Hand Marker
    pal::waitUntilTransform(tfListener, eyeFrame, arHandMarkerFrame, cMhm);

    //------------Transformation to the Robot Base Frame with the Camera as Reference Frame
    vpHomogeneousMatrix cMb;
    pal::waitUntilTransform(tfListener, eyeFrame, baseFrame, cMb);

    //------------Transformation to the Object Marker with the Camera as Reference Frame
    vpHomogeneousMatrix cMom; // Camera Position --> Object Marker
    pal::waitUntilTransform(tfListener, eyeFrame, arObjMarkerFrame, cMom);

    //------------Transformation to the Head Chain Base Frame with the Camera as Reference Frame
    vpHomogeneousMatrix cMt_head;
    pal::waitUntilTransform(tfListener, eyeFrame, torsoFrame, cMt_head);

    //----------------------------------------------------------------------------------------------------------------------
    // Transformation to the hand Frame with the marker as Reference Frame  --> detect this transformation once
    // !! might need something to make sure it is detected properly
    vpHomogeneousMatrix hmMh; // Camera Position --> Object Marker
    pal::waitUntilTransform(tfListener, arHandMarkerFrame, handFrame, hmMh);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Compute Hand position in the camera frame
    vpHomogeneousMatrix cMh;
    cMh = cMhm * hmMh;

    vpHomogeneousMatrix omMhmd;
    pal::getTransformFromFile(desiredStateFileName, omMhmd);

    // Compute Desired Hand Marker position in the camera frame
    vpHomogeneousMatrix cMhmd;
    cMhmd = cMom * omMhmd;

    // Compute Desired Hand position in the camera frame
    vpHomogeneousMatrix cMhd;
    cMhd = cMhmd * hmMh;

    //============================= INITIALIZE VISUAL FEATURES =============================
    //------------for the RIGHT ARM
    //Position-Based Task with 6 DOF - translation and orientation features
    vpHomogeneousMatrix hmdMhm;
    hmdMhm = cMhmd.inverse() * cMhm;
    vpFeatureTranslation trans(hmdMhm, vpFeatureTranslation::cdMc);
    vpFeatureThetaU thetaU(hmdMhm, vpFeatureThetaU::cdRc);

    //------------for the HEAD
    vpFeaturePoint desired_image_point;
    // x and y are coordinates in the image plane in meters
    double x_desiredImgPt = 0.0;
    double y_desiredImgPt = 0.0;
    double Z_desiredImgPt = 1.0; //Estimated depth of the point in the camera frame.
    //Set the point feature from the desired parameters.
    desired_image_point.buildFrom(x_desiredImgPt, y_desiredImgPt, Z_desiredImgPt);
    // Compute the Object Distance from Camera
    vpTranslationVector cOom;
    vpTranslationVector cOhm;
    cMom.extract(cOom);
    cMhm.extract(cOhm);

    double Z_currentImgPt = 0;
    double x_currentImgPt = 0;
    double y_currentImgPt = 0;

    if ( GAZING_TASK_MID_POINT == USE_3D )
    {
      //2) Compute the mid point in 3D and then project it to the normalized coordinates
      Z_currentImgPt =   (cOom[2]+cOhm[2]) / 2;
      x_currentImgPt = ( (cOom[0]+cOhm[0]) / 2) / Z_currentImgPt;
      y_currentImgPt = ( (cOom[1]+cOhm[1]) / 2) / Z_currentImgPt;
    }
    else if ( GAZING_TASK_MID_POINT == USE_2D )
    {
      //1) first normalize each 3D point and then compute the midpoint in normalized coordinates and take
      //   the average of the Z
      x_currentImgPt = ( cOom[0]/cOom[2] + cOhm[0]/cOhm[2]) / 2;
      y_currentImgPt = ( cOom[1]/cOom[2] + cOhm[1]/cOhm[2]) / 2;
      Z_currentImgPt = ( cOom[2]+cOhm[2]) / 2;
    }
    else
      throw std::runtime_error("Invalid GAZING_TASK_MID_POINT");

    //Set the point feature from image processing desired parameters.
    vpFeaturePoint current_image_point;
    current_image_point.buildFrom(x_currentImgPt, y_currentImgPt, Z_currentImgPt);

    //============================= INITIALIZE VISUAL SERVOING TASK =============================
    //------------for the RIGHT ARM
    vpServo task_ARM;
    // use eye-in-hand with to correctly use the relative PBVS features
    task_ARM.setServo(vpServo::EYEINHAND_L_cVe_eJe);    // Use since KDL returns fJe_ARM,
    // EYE-IN-HAND, just use cVf and fJe_ARM instead of cVe and eJe
    task_ARM.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task_ARM.addFeature(trans);
    task_ARM.addFeature(thetaU);
    task_ARM.setLambda(lambdaTorsoArm); // gain --> NEED TO TUNE THIS
    // Set the velocity twist from robot base frame in the reference marker frame
    vpHomogeneousMatrix hmMb;
    hmMb = cMhm.inverse() * cMb;
    task_ARM.set_cVe(hmMb);
    // Set the robot jacobian in the defined robot base frame
    vpMatrix fJe_ARM(kdlTorsoArmJacobian.rows(), kdlTorsoArmJacobian.columns());
    kdlJointToJacobianArm.JntToJac(kdlTorsoArmPosition, kdlTorsoArmJacobian); // Compute Jacobian at current position
    // Fix reference point: KDL sets the refpoint in the base frame of the kinematic chain. We need it in the end-effector of the chain
    vpHomogeneousMatrix bMh;
    pal::waitUntilTransform(tfListener, baseFrame, handFrame, bMh);
    vpTranslationVector bOh;
    bMh.extract(bOh);
    kdlTorsoArmJacobian.changeRefPoint(KDL::Vector(-bOh[0], -bOh[1], -bOh[2]));
    pal::convertKDLJacToVispMat(kdlTorsoArmJacobian, fJe_ARM); // KDL data --> VISP data
    task_ARM.set_eJe(fJe_ARM);

    //------------for the HEAD
    vpServo task_HEAD;
    task_HEAD.setServo(vpServo::EYEINHAND_L_cVe_eJe); // Use since KDL returns fJe,
    // EYE-IN-HAND, just use cVf and fJe instead of cVe and eJe
    task_HEAD.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task_HEAD.addFeature(current_image_point, desired_image_point);
    task_HEAD.setLambda(lambdaHead); //NEED TO TUNE THIS
    // UPDATE VELOCITY TWIST AND JACOBIAN
    task_HEAD.set_cVe(cMt_head); // just use cVf and fJe instead of cVe and eJe
    vpMatrix fJe_HEAD(kdlHeadJacobian.rows(), kdlHeadJacobian.columns());
    // Compute Jacobian at current position
    kdlJointToJacobianHead.JntToJac(kdlHeadPosition, kdlHeadJacobian);
    // Fix reference point
    vpHomogeneousMatrix tMhead;
    pal::waitUntilTransform(tfListener, torsoFrame, headFrame, tMhead);
    vpTranslationVector tOhead;
    tMhead.extract(tOhead);
    kdlHeadJacobian.changeRefPoint(KDL::Vector(-tOhead[0], -tOhead[1], -tOhead[2]));
    pal::convertKDLJacToVispMat(kdlHeadJacobian, fJe_HEAD);// KDL data --> VISP data
    task_HEAD.set_eJe(fJe_HEAD); // just use cVf and fJe instead of cVe and eJe

    //============================= INITIALIZE JOINT VELOCITY VECTORS =============================
    //------------for the RIGHT ARM
    vpColVector qdot_ARM; // aggregate joint velocity vector
    qdot_ARM.resize(number_of_torsoAndArm_joints);
    vpColVector q1dot_ARM; // joint velocities for Primary Task
    q1dot_ARM.resize(number_of_torsoAndArm_joints);
    vpColVector q2dot_ARM; // joint velocities for Secondary Task (i.e. Joint Limit Avoidance)
    q2dot_ARM.resize(number_of_torsoAndArm_joints);
    //------------for the HEAD
    vpColVector qdot_HEAD; // aggregate joint velocity vector
    qdot_HEAD.resize(number_of_head_joints);
    vpColVector q1dot_HEAD; // joint velocities for Primary Task
    q1dot_HEAD.resize(number_of_head_joints);
    vpColVector q2dot_HEAD; // joint velocities for Secondary Task (i.e. Joint Limit Avoidance)
    q2dot_HEAD.resize(number_of_head_joints);
    //------------for the TORSO (for feedforward terms)
    vpColVector qdot_TORSO;
    qdot_TORSO.resize(number_of_torso_joints);
    //------------for the ARM ONLY (for feedforward terms)
    vpColVector qdot_ARMONLY;
    qdot_ARMONLY.resize(number_of_armOnly_joints);

    //============================= DEFINE VISUAL SERVO LOOP EXECUTION TIME =============================
    // Define the loop running time --> affects the joint position command (d =  v * t)
    //        double time_step = 0.1;  // realistic frame rates are 10Hz and below
    double time_step = timeStep;  // realistic frame rates are 10Hz and below
    ros::Rate loop_rate(1.0/time_step);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////        VISUAL SERVO LOOP       ////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool goalReached = false;

    std::cout << "========== STARTING VISUAL SERVO ==========================" << std::endl;
    while( ros::ok() )
    {
      if ( goalReached && CLOSE_HAND_IN_GOAL)
      {
        static bool handClosed = false;
        if ( !handClosed )
          system("roslaunch reem_upperbody_visual_servo reem_close_right_hand.launch &");
        handClosed = true;
      }
      else
      {
        if ( STEP_BY_STEP )
        {
          //Added for stepping functionality for the visual servo loop
          char dummyVar;
          ROS_INFO("Press a key + ENTER");
          std::cin >> dummyVar;
        }

        //============================= UPDATE IMPORTANT FRAME TRANSFORMATIONS =============================
        //------------Transformation to the Robot Base Frame with the Camera as Reference Frame
        pal::waitUntilTransform(tfListener, eyeFrame, baseFrame, cMb);

        // Transformation to the Object Marker with the Camera as Reference Frame
        ros::Time transformTime = pal::waitUntilTransform(tfListener, eyeFrame, arObjMarkerFrame, cMom);

        bool objectMarkerLost = false;
        if ( (ros::Time::now() - transformTime).sec >= 1 )
          objectMarkerLost = true;

        // Compute Desired Hand position in the camera frame
        cMhmd = cMom * omMhmd;
        cMhd = cMhmd * hmMh;

        // Transformation to the Hand Marker with the Camera as Reference Frame
        transformTime = pal::waitUntilTransform(tfListener, eyeFrame, arHandMarkerFrame, cMhm);

        bool handMarkerLost = false;
        if ( (ros::Time::now() - transformTime).sec >= 1 )
          handMarkerLost = true;

        // Compute Hand position in the camera frame
        cMh = cMhm * hmMh; // needed for trajectory

        if ( objectMarkerLost || handMarkerLost )
        {
          ROS_ERROR_STREAM("Doing nothing because some marker lost. Hand marker: " << !handMarkerLost << " object marker: " << !objectMarkerLost);
        }
        /////////////////////////////////////////////////////////////////////////
        // Do servoing only if both markers have been found recently
        /////////////////////////////////////////////////////////////////////////
        else
        {
          //Transformation to the Head Chain Base Frame with the Camera as Reference Frame
          pal::waitUntilTransform(tfListener, eyeFrame, torsoFrame, cMt_head);

          //============================= UPDATE TWIST TRANSFORMATION MATRIX =============================
          //------------for the RIGHT ARM
          hmMb = cMhm.inverse() * cMb;
          task_ARM.set_cVe(hmMb);
          //------------for the HEAD
          task_HEAD.set_cVe(cMt_head);

          //============================= UPDATE ROBOT JACOBIAN =============================
          //------------for the RIGHT ARM
          // Compute Robot Jacobian at current position
          kdlJointToJacobianArm.JntToJac(kdlTorsoArmPosition, kdlTorsoArmJacobian);
          // Fix reference point
          pal::waitUntilTransform(tfListener, baseFrame, handFrame, bMh);

          vpTranslationVector bOh;
          bMh.extract(bOh);
          kdlTorsoArmJacobian.changeRefPoint(KDL::Vector(-bOh[0], -bOh[1], -bOh[2]));
          pal::convertKDLJacToVispMat(kdlTorsoArmJacobian, fJe_ARM); // KDL data --> VISP data
          task_ARM.set_eJe(fJe_ARM);

          //------------for the HEAD
          // Compute Jacobian at current position
          kdlJointToJacobianHead.JntToJac(kdlHeadPosition, kdlHeadJacobian);
          // Fix reference point
          pal::waitUntilTransform(tfListener, torsoFrame, headFrame, tMhead);
          vpTranslationVector tOhead;
          tMhead.extract(tOhead);
          kdlHeadJacobian.changeRefPoint(KDL::Vector(-tOhead[0], -tOhead[1], -tOhead[2]));
          pal::convertKDLJacToVispMat(kdlHeadJacobian, fJe_HEAD);// KDL data --> VISP data
          task_HEAD.set_eJe(fJe_HEAD); // just use cVf and fJe instead of cVe and eJe

          //============================= UPDATE VISUAL FEATURES =============================
          //------------for the RIGHT ARM
          hmdMhm = cMhmd.inverse() * cMhm;
          trans.buildFrom(hmdMhm);
          thetaU.buildFrom(hmdMhm);

          //------------for the HEAD
          //Set the point feature from the desired parameters.
          desired_image_point.buildFrom(x_desiredImgPt, y_desiredImgPt, Z_desiredImgPt);
          // Compute the Object Distance from Camera
          cMom.extract(cOom); //extract cOobjectmarker
          cMhm.extract(cOhm);

          //compute mid point between both markers in normalized coordinates:

          if ( GAZING_TASK_MID_POINT == USE_3D )
          {
            //2) Compute the mid point in 3D and then project it to the normalized coordinates
            Z_currentImgPt =   (cOom[2]+cOhm[2]) / 2;
            x_currentImgPt = ( (cOom[0]+cOhm[0]) / 2) / Z_currentImgPt;
            y_currentImgPt = ( (cOom[1]+cOhm[1]) / 2) / Z_currentImgPt;
          }
          else if ( GAZING_TASK_MID_POINT == USE_2D )
          {
            //1) first normalize each 3D point and then compute the midpoint in normalized coordinates and take
            //   the average of the Z
            x_currentImgPt = ( cOom[0]/cOom[2] + cOhm[0]/cOhm[2]) / 2;
            y_currentImgPt = ( cOom[1]/cOom[2] + cOhm[1]/cOhm[2]) / 2;
            Z_currentImgPt = ( cOom[2]+cOhm[2]) / 2;
          }
          else
            throw std::runtime_error("Invalid GAZING_TASK_MID_POINT");

          //Set the point feature from image processing desired parameters.
          current_image_point.buildFrom(x_currentImgPt, y_currentImgPt, Z_currentImgPt);

          //============================= COMPUTE VISUAL SERVO CONTROL LAW ==> FOR THE RIGHT ARM + TORSO =============================
          // Get velocities from Visual Servo
          // !! Also needed to update VISP matrices used in the control task
          qdot_ARM = task_ARM.computeControlLaw(); // qdot_ARM might be overwritten later considering secondary task
          std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
          task_ARM.print();
          std::cout << "eJe: " << std::endl << task_ARM.get_eJe() << std::endl;
          std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;

          // ----------------- EXTRACT VARIABLES FROM VISUAL SERVO TASK
          vpMatrix taskJacobian_ARM;
          taskJacobian_ARM = task_ARM.J1;
          vpColVector taskError_ARM;
          taskError_ARM = task_ARM.error;

          double error_Norm = sqrt((taskError_ARM.transpose() * taskError_ARM)[0]);

          double taskLambda;
          taskLambda = task_ARM.lambda.getLastValue();
          vpMatrix identity;
          identity.eye(number_of_torsoAndArm_joints);

          if ( JointLimAvAlg != NONE )
          {
            //compute qdot_ARM, q1dot_ARM and q2dot_ARM
            secondary_task::computeSecondaryTask(qdot_ARM,
                                                 taskJacobian_ARM,
                                                 taskLambda,
                                                 taskError_ARM,
                                                 kdlTorsoArmPosition,
                                                 q_min_torsoAndArm, q_max_torsoAndArm,
                                                 JointLimAvAlg == LARGE_PROJECTION,
                                                 qdot_ARM, q1dot_ARM, q2dot_ARM);
          } //end if ( JointLimAvAlg != NONE )

          // Rescale velocities when 1 or more is over the limit specified
          utility_functions::limitVelocity(qdot_ARM, maxVelocity); // limit in rad/sec

          if ( !ENABLE_ARM_AND_TORSO )
          {
            for (unsigned int i = 0; i < qdot_ARM.getRows(); ++i)
            {
              qdot_ARM[i]  = 0;
              q1dot_ARM[i] = 0;
              q2dot_ARM[i] = 0;
            }
          }

          /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          //             COMPUTE VISUAL SERVO CONTROL LAW FOR THE HEAD
          /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          // Get velocities from Visual Servo
          // !! Also needed to update VISP matrices used in the control task
          qdot_HEAD = task_HEAD.computeControlLaw(); // qdot not considering feed-forward compensation
          // it might be overwritten later considering secondary task

          ROS_INFO_STREAM("qdot_HEAD (without feed-forward terms):\n" << qdot_HEAD);

          // fill in separate joint positions
          for(unsigned int i=0; i<number_of_torsoAndArm_joints; i++)
          {
            if(i<number_of_torso_joints)
            {
              kdlTorsoPosition.data[i] = kdlTorsoArmPosition.data[i];
              qdot_TORSO[i] = qdot_ARM[i];
            }
            else
            {
              kdlArmOnlyPosition.data[i-number_of_torso_joints] = kdlTorsoArmPosition.data[i];
              qdot_ARMONLY[i-number_of_torso_joints] = qdot_ARM[i];
            }
          }

          //-------------------------------- feedforward terms
          //-------Torso motion compensation
          vpColVector torsoCompensation;
          // Ls
          vpMatrix Ls_midpoint = task_HEAD.L;

          // cVf
          vpVelocityTwistMatrix cVf;
          cVf.buildFrom(cMb);

          // fJe
          vpMatrix fJe_TORSO(kdlTorsoJacobian.rows(), kdlTorsoJacobian.columns());
          kdlJointToJacobianTorso.JntToJac(kdlTorsoPosition, kdlTorsoJacobian);  // Compute Jacobian at current position
          // Fix reference point
          vpHomogeneousMatrix bMt;
          pal::waitUntilTransform(tfListener, baseFrame, torsoFrame, bMt);
          vpTranslationVector bOt;
          bMt.extract(bOt);
          kdlTorsoJacobian.changeRefPoint(KDL::Vector(-bOt[0], -bOt[1], -bOt[2]));
          pal::convertKDLJacToVispMat(kdlTorsoJacobian, fJe_TORSO);// KDL data --> VISP data

          // Jt
          vpMatrix taskJacobian_torsoCompensation;
          taskJacobian_torsoCompensation = Ls_midpoint * cVf * fJe_TORSO;

          // compute final compensation
          torsoCompensation = taskJacobian_torsoCompensation * qdot_TORSO;

          //-------Arm motion compensation
          vpColVector armCompensation;

          // fJe (without torso) = Jacobian of handFrame expressed in torsoFrame
          vpMatrix fJe_ARMONLY(kdlArmOnlyJacobian.rows(), kdlArmOnlyJacobian.columns());
          kdlJointToJacobianArmOnly.JntToJac(kdlArmOnlyPosition, kdlArmOnlyJacobian);  // Compute Jacobian at current position
          // Fix reference point
          pal::waitUntilTransform(tfListener, torsoFrame, handFrame, bMh);
          bMh.extract(bOh);
          kdlArmOnlyJacobian.changeRefPoint(KDL::Vector(-bOh[0], -bOh[1], -bOh[2]));
          pal::convertKDLJacToVispMat(kdlArmOnlyJacobian, fJe_ARMONLY);// KDL data --> VISP data

          vpVelocityTwistMatrix cVt; //twist transformation from torso_2_link to camera frame
          cVt.buildFrom(cMt_head);


          if ( GAZING_TASK_INTERACTION_MATRIX == USE_CLASSIC_POINT_L )
          {
            armCompensation =  -0.5  * Ls_midpoint * cVf * fJe_ARM * qdot_ARM;
            ///////armCompensation =  -0.5  * Ls_midpoint * cVt * fJe_ARMONLY * qdot_ARMONLY;
          }
          else if ( GAZING_TASK_INTERACTION_MATRIX == USE_MID_POINT_L )
          {
            vpMatrix Ls_midPoint3D(2,6);
            Ls_midPoint3D[0][0] = -2/(cOom[2] + cOhm[2]);
            Ls_midPoint3D[0][1] = 0;
            Ls_midPoint3D[0][2] = 2*x_currentImgPt/(cOom[2] + cOhm[2]);
            Ls_midPoint3D[0][3] = x_currentImgPt*y_currentImgPt;
            Ls_midPoint3D[0][4] = -1-x_currentImgPt*x_currentImgPt;
            Ls_midPoint3D[0][5] = y_currentImgPt;

            Ls_midPoint3D[1][0] = 0;
            Ls_midPoint3D[1][1] = -2/(cOom[2] + cOhm[2]);
            Ls_midPoint3D[1][2] = 2*y_currentImgPt/(cOom[2] + cOhm[2]);
            Ls_midPoint3D[1][3] = 1+y_currentImgPt*y_currentImgPt;
            Ls_midPoint3D[1][4] = -x_currentImgPt*y_currentImgPt;
            Ls_midPoint3D[1][5] = -x_currentImgPt;

            armCompensation =  -0.5 * Ls_midPoint3D * cVf * fJe_ARM * qdot_ARM;
          }

          std::cout << "arm compensation" << std::endl << armCompensation << std::endl;

          //-------Object motion compensation
          // to be added only if a moving object is expected


          if ( !ENABLE_TORSO_COMPENSATION )
            torsoCompensation = 0;

          if ( !ENABLE_ARM_COMPENSATION )
            armCompensation = 0;

          // ----------------- EXTRACT VARIABLES FROM VISUAL SERVO TASK
          vpMatrix taskJacobian_HEAD;
          taskJacobian_HEAD = task_HEAD.J1;
          vpColVector taskError_HEAD;
          taskError_HEAD = task_HEAD.error;

          error_Norm = sqrt((taskError_HEAD.transpose() * taskError_HEAD)[0]);

          //            double taskLambda;
          taskLambda = task_HEAD.lambda.getLastValue();
          //            vpMatrix identity;
          identity.eye(number_of_head_joints);

          {
            vpMatrix taskJacPinv;
            taskJacobian_HEAD.pseudoInverse(taskJacPinv, 1e-3);
            qdot_HEAD = taskJacPinv * ( (-taskLambda*taskError_HEAD) -torsoCompensation -armCompensation);
          }

          if ( JointLimAvAlg != NONE )
          {
            secondary_task::computeSecondaryTask(qdot_HEAD,
                                                 taskJacobian_HEAD,
                                                 taskLambda,
                                                 taskError_HEAD,
                                                 kdlHeadPosition,
                                                 q_min_head, q_max_head,
                                                 JointLimAvAlg == LARGE_PROJECTION,
                                                 qdot_HEAD, q1dot_HEAD, q2dot_HEAD);
          } //end if ( JointLimAvAlg != NONE )

          // Rescale velocities when 1 or more is over the limit specified
          utility_functions::limitVelocity(qdot_HEAD, maxVelocity); // limit in rad/sec

          if ( !ENABLE_HEAD )
          {
            for (unsigned int i = 0; i < qdot_HEAD.getRows(); ++i)
            {
              qdot_HEAD[i]  = 0;
              q1dot_HEAD[i] = 0;
              q2dot_HEAD[i] = 0;
            }
          }

          /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          //                           COMPUTE NEXT JOINT POSITIONS FOR TRAJECTORY
          /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          //------------for the RIGHT ARM
          for(unsigned int i=0; i < number_of_torsoAndArm_joints; i++)
          {
            kdlTorsoArmPosition.data[i] = kdlTorsoArmPosition.data[i] + (qdot_ARM[i] * time_step) ;
            //Normalize angle to -PI to PI
            kdlTorsoArmPosition.data[i] = angles::normalize_angle(kdlTorsoArmPosition.data[i]);
          }
          //------------for the HEAD
          for(unsigned int i=0; i < number_of_head_joints; i++)
          {
            kdlHeadPosition.data[i] = kdlHeadPosition.data[i] + (qdot_HEAD[i] * time_step) ;
            //Normalize angle to -PI to PI
            kdlHeadPosition.data[i] = angles::normalize_angle(kdlHeadPosition.data[i]);
          }                    

          /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          //                             PUBLISH MESSAGES TO THE CORRESPONDING TOPICS
          /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

          moveParts.move( pal::MoveParts::TORSO,
                          pal::eigenToStdVector(kdlTorsoArmPosition.data, 0, 2),
                          pal::vispToStdVector(qdot_ARM, 0, 2),
                          ros::Duration(time_step * timeFromStartFactor));

          moveParts.move( pal::MoveParts::RIGHT_ARM,
                          pal::eigenToStdVector(kdlTorsoArmPosition.data, 2, 7),
                          pal::vispToStdVector(qdot_ARM, 2, 7),
                          ros::Duration(time_step * timeFromStartFactor));

          moveParts.move( pal::MoveParts::HEAD,
                          pal::eigenToStdVector(kdlHeadPosition.data, 0, 2),
                          pal::vispToStdVector(qdot_HEAD, 0, 2),
                          ros::Duration(time_step * timeFromStartFactor));


//          //------------for MOVING the TORSO and the RIGHT arm
//          trajectory_msgs::JointTrajectory jntTrajTorsoMSG, jntTrajRightArmMSG;
//          utility_functions::makeTorsoAndRightArmJointTrajMSG(kdlTorsoArmPosition,
//                                                              qdot_ARM,
//                                                              jntTrajTorsoMSG,
//                                                              jntTrajRightArmMSG,
//                                                              time_step,
//                                                              timeFromStartFactor);

//          jointTrajtorso_pub.publish(jntTrajTorsoMSG);
//          jointTrajArm_pub.publish(jntTrajRightArmMSG);
//          //------------for MOVING the HEAD
//          trajectory_msgs::JointTrajectory jntTrajHeadMSG;
//          utility_functions::makeHeadJointTrajMSG(jntTrajHeadMSG, kdlHeadPosition, qdot_HEAD, time_step, timeFromStartFactor);
//          jointTrajHead_pub.publish(jntTrajHeadMSG);

          // Publish Errors of Visual Servoing
          std_msgs::Float64MultiArray plotMsg;
          // get the transformation from base frame to the end-effector
          vpHomogeneousMatrix eMf; // end effector Frame --> Robot Jacobian base Frame
          pal::waitUntilTransform(tfListener, headFrame, torsoFrame, eMf);
          vpVelocityTwistMatrix eVf(eMf);
          //utility_functions::makePlotMSG(plotMsg,taskError_HEAD, qdot_HEAD, q1dot_HEAD, q2dot_HEAD, fJe_HEAD, eVf, kdlHeadPosition);
          utility_functions::makePlotMSG(plotMsg,
                                         taskError_HEAD, qdot_HEAD,
                                         taskError_ARM, qdot_ARM,
                                         q1dot_HEAD, q2dot_HEAD,
                                         q1dot_ARM, q2dot_ARM,
                                         x_currentImgPt, y_currentImgPt);
          plot_pub.publish(plotMsg);

          // Publish a RVIZ marker corresponding to the arm_right_7_joint pose obtained from TF
          vpHomogeneousMatrix cMarm_end_effector;
          pal::waitUntilTransform(tfListener, eyeFrame, handFrame, cMarm_end_effector);
          visualization_msgs::Marker marker;
          geometry_msgs::Pose markerPose;
          pal::convertVispHMatToGMPose(cMarm_end_effector, markerPose);
          utility_functions::makeMarkerMSG(marker, eyeFrame, markerPose, markerID, q2dot_ARM.sumSquare() > 0.0);
          markerID++; // define unique ID numbers
          marker_pub.publish(marker); // Publish the marker

          // Broadcast to /tf for RVIZ visualization of target pose
          tf::Transform cMhd_TF;
          pal::convertVispHMatToTF(cMhd, cMhd_TF);
          br.sendTransform( tf::StampedTransform( cMhd_TF, ros::Time::now(), eyeFrame, "target_pose" ) );

          // Broadcast to /tf for RVIZ visualization of target pose
          tf::Transform cMhmd_TF;
          pal::convertVispHMatToTF(cMhmd, cMhmd_TF);
          br.sendTransform( tf::StampedTransform( cMhmd_TF, ros::Time::now(), eyeFrame, "target_marker_pose" ) );

          // ----------------- DISPLAYS FOR DEBUGGING
          std::cout << "qdot_TORSO_ARM" << std::endl << qdot_ARM << std::endl;
          std::cout << "q1dot_TORSO_ARM" << std::endl << q1dot_ARM << std::endl;
          std::cout << "q2dot_TORSO_ARM" << std::endl << q2dot_ARM << std::endl;
          std::cout << "qdot_HEAD" << std::endl << qdot_HEAD << std::endl;
          std::cout << "q1dot_HEAD" << std::endl << q1dot_HEAD << std::endl;
          std::cout << "q2dot_HEAD" << std::endl << q2dot_HEAD << std::endl;
          std::cout << "torso compensation" << std::endl << torsoCompensation << std::endl;
          std::cout << "arm compensation" << std::endl << armCompensation << std::endl;

          std::stringstream str;
          for(unsigned int i=0; i < number_of_head_joints; i++)
            str << kdlHeadPosition.data[i] << " ";
          std::cout << "integrated head joint state: [ " << str.str() << " ]" << std::endl;
          str.str("");
          for(unsigned int i=0; i < number_of_torsoAndArm_joints; i++)
            str << kdlTorsoArmPosition.data[i] << " ";
          std::cout << "integrated torso+arm joint state: [ " << str.str() << " ]" << std::endl;

          std::cout << "------------------------------------------------" << std::endl;
          std::cout << "TORSO_ARM Task error: " << task_ARM.e.t() << " norm: " << sqrt((taskError_ARM.transpose() * taskError_ARM)[0]) <<  std::endl;
          std::cout << "HEAD Task error: "      << task_HEAD.e.t() << " norm: " << sqrt((taskError_HEAD.transpose() * taskError_HEAD)[0]) << std::endl;

          goalReached = sqrt((taskError_ARM.transpose() * taskError_ARM)[0]) < 0.02;

          for(unsigned int i=0; i < number_of_torsoAndArm_joints; i++)
          {
            if ( q2dot_ARM.data[i] != 0 )
              std::cout << "TORSO+ARM chain => Joint " << i << " is " << kdlTorsoArmPosition.data[i] << " and the valid range is [" <<  q_min_torsoAndArm[i] << ", " << q_max_torsoAndArm[i] << "]" << std::endl;
          }

          std::cout << "------------------------------------------------" << std::endl;

          std::cout << "========== iterating visual servo ==========================" << std::endl;

        } //end if both markers found

        ros::spinOnce();
        loop_rate.sleep();
      }
    } // end while(ros::ok())

    // End Visual Servo
    task_ARM.kill();
    task_HEAD.kill();
    return 0;
  }

} //end main
