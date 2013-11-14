//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// VISUAL SERVO THE WHOLE UPPER BODY OF REEM - do both gaze and arm control simultaneously
//
//
//  Control laws (not including secondary task):
//
//   qdot(ARM+TORSO) = -λ_1 · (L · ^hmV_b · ^bJ_h)^+ · e(ARM+TORSO)
//
//   qdot(HEAD) = (L_mid(^cO_om,^cO_h) · ^cV_t · ^tJ_n)^+ · (-λ_2 · e_mid -torsoFeedForward -armFeedForward)
//
//  where:
//
//     torsoFeedForward = L_mid(^cO_om,^cO_h)·^cV_b·^bJ_t·qdot(TORSO)
//     armFeedForward   = -0.5·L_^cO_h·^cV_t·^tJ_h·qdot(ARM)
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
#include "conversions.h"
#include "tf_utils.h"
#include "move_parts.h"
#include "kinematic_chain.h"
#include "head_task.h"
#include "hand_task.h"

//// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

// Standard C++ headers
#include <string>

namespace pal {

  class ReemUpperbodyServo {

  public:

    /**
   * @brief The midPointGazing enum allows chosing the point that will be followed by the gaze of the robot
   */
    enum midPointGazing { USE_3D = 0, USE_2D = 1 };

    enum midPointInteractionMatrix { USE_CLASSIC_POINT_L = 0, USE_MID_POINT_L = 1 };

    /**
    * @brief The jointLimitAvoidanceAlgorithm enum
    */
    enum jointLimitAvoidanceAlgorithm { NONE = 0, CLASSICAL = 1, LARGE_PROJECTION = 2 };


    ReemUpperbodyServo(ros::NodeHandle& nh,
                       const std::string& desiredStateFileName);

    virtual ~ReemUpperbodyServo();

    void configureParams();

    void run();

  protected:

    void initFrameNames();
    void loadDesiredState();

    void createKinematicChains();
    void createServoTasks();

    void getConstantFramesTransforms();
    void updateMovingFramesTransforms();

    void updateArmTorsoVisualFeatures();
    void runTorsoArmTask();

    void updateHeadTaskVisualFeatures();
    void runHeadTask();
    void waitKeyIfNecessary();

    void publishTrajectory();
    void publishPlots();


    /**
     * @brief cameraTwistsInducedByArmAndTorso estimate the twists that will be induced to the camera due to
     *        qdot_torso and qdot_arm
     */
    void cameraTwistsInducedByArmAndTorso();

    ros::NodeHandle _nh;

    //configuration parameters
    std::string _desiredStateFileName;

    bool _stepByStep;
    bool _enableHead;
    bool _enableArmAndTorso;
    bool _enableTorsoCompensation;
    bool _enableArmCompensation;
    midPointGazing _midPointGazing;
    midPointInteractionMatrix _gazingTaskInteractionMatrix;
    jointLimitAvoidanceAlgorithm _jointLimitAvoidance;

    double _lambdaTorsoArm;
    double _lambdaHead;
    double _maxVelocity;
    double _timeStep;
    double _timeFromStartFactor; //_timeStep * _timeFromStartFactor = max duration
    // of a motion sent to a trajectory controller
    bool _closeHandInGoal;

    //TF
    tf::TransformListener _tfListener;
    tf::TransformBroadcaster _tfBroadcaster;

    //head kinematic chain frames
    std::string _headBaseFrame;
    std::string _headEndFrame;

    //camera frame
    std::string _eyeFrame;

    //torso + arm kinematic chain frames:
    std::string _torsoArmBaseFrame;
    std::string _torsoArmEndFrame; // = hand frame

    //torso kinematic chain frames:
    std::string _torsoBaseFrame;
    std::string _torsoEndFrame;

    //arm kinematic chain frames:
    std::string _armBaseFrame;
    std::string _armEndFrame;

    //markers frames:
    std::string _handMarkerFrame;
    std::string _objectMarkerFrame;
    bool _objectMarkerLost;
    bool _handMarkerLost;

    //kinematic chains:
    boost::scoped_ptr<pal::KinematicChain> _headChain;
    boost::scoped_ptr<pal::KinematicChain> _torsoAndArmChain;
    boost::scoped_ptr<pal::KinematicChain> _torsoChain;

    //Servo tasks:
    boost::scoped_ptr<pal::HandTask> _taskHand;
    //joint velocities of the torso+arm kinematic chain:
    std::vector<double> _qdot_torsoArm;
    vpColVector _q1dot_torsoArm, _q2dot_torsoArm;
    //joint velocities splitted:
    std::vector<double> _qdot_torso, _qdot_arm;
    //joint velocities of the head
    std::vector<double> _qdot_head;
    vpColVector _q1dot_head, _q2dot_head;

    boost::scoped_ptr<pal::HeadTask> _taskHead;

    //executing motions
    pal::MoveParts _moveParts;

    //constant transformation between frames
    vpHomogeneousMatrix _omMhmd; //transformation from desired hand marker to object marker
    vpHomogeneousMatrix _hmMh;   //transformation from hand frame to hand marker frame
    vpHomogeneousMatrix _cMhd;   //transformation from desired hand frame in camera frame

    //variable transformation between frames
    vpHomogeneousMatrix _hmMtorsoArmBase;  //transformation from the torso+arm base frame in hand marker
    vpHomogeneousMatrix _hmdMhm;           //transformation from the current hand marker frame to the desired one
    vpHomogeneousMatrix _cMom;             //transformation from object marker to camera frame
    vpHomogeneousMatrix _cMhmd;            //transformation from desired hand marker to camera frame
    vpHomogeneousMatrix _cMhm;             //transformation from current hand marker to camera frame
    vpHomogeneousMatrix _cMh;              //transformation from torso+arm end frame (hand) to camera frame
    vpHomogeneousMatrix _cMheadBase;       //transformation from head base frame to camera frame
    vpHomogeneousMatrix _cMtorsoArmBaseFrame;
    vpHomogeneousMatrix _torsoBaseFrameMtorsoEndFrame;
    vpHomogeneousMatrix _torsoArmBaseFrameMtorsoArmEndFrame;

    //camera twist induced by torso motion expressed in the camera frame:
    vpColVector _cameraTwistInducedByTorso;
    //mid point between object and hand markers twist induced by torso+arm motion expressed in the camera frame
    vpColVector _midPointTwistInducedByTorsoAndArm;

    //Publisher of the hand trajectory
    ros::Publisher _trajectoryPub;
    long long _trajectoryMarkerCounter;
    //Publisher of the task plot
    ros::Publisher _plotPub;
  };


  ReemUpperbodyServo::ReemUpperbodyServo(ros::NodeHandle& nh,
                                         const std::string& desiredStateFileName):
    _nh(nh),
    _desiredStateFileName(desiredStateFileName),
    _midPointGazing(USE_3D),
    _gazingTaskInteractionMatrix(USE_MID_POINT_L),
    _objectMarkerLost(false),
    _handMarkerLost(false)
  {
    initFrameNames();
    configureParams();

    createKinematicChains();

    createServoTasks();

    _cameraTwistInducedByTorso.resize(6, true);
    _midPointTwistInducedByTorsoAndArm.resize(6, true);

    _trajectoryPub = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    _plotPub       = _nh.advertise<std_msgs::Float64MultiArray>("/VS_errors", 1);

    _trajectoryMarkerCounter = 1;

  }

  ReemUpperbodyServo::~ReemUpperbodyServo()
  {

  }

  void ReemUpperbodyServo::createKinematicChains()
  {
    std::string URDFstring;
    _nh.param("robot_description", URDFstring, std::string());

    _headChain.reset( new pal::KinematicChain(URDFstring,
                                              _headEndFrame,
                                              _headBaseFrame));

    _headChain->updateJointStateFromTopic();

    _torsoChain.reset(new pal::KinematicChain(URDFstring,
                                              _torsoEndFrame,
                                              _torsoBaseFrame));

    _torsoChain->updateJointStateFromTopic();

    _torsoAndArmChain.reset(new pal::KinematicChain(URDFstring,
                                                    _torsoArmEndFrame,
                                                    _torsoArmBaseFrame));

    std::vector<double> lowerJointLimits = _torsoAndArmChain->getLowerJointLimits();
    //changed to try to make shoulder joint not bump into torso
    lowerJointLimits[3] = -0.15;
    _torsoAndArmChain->setLowerJointLimits(lowerJointLimits);

    _torsoAndArmChain->updateJointStateFromTopic();
  }

  void ReemUpperbodyServo::createServoTasks()
  {
    _taskHand.reset( new pal::HandTask(_lambdaTorsoArm));

    _taskHead.reset( new pal::HeadTask(_lambdaHead));
    _taskHead->setDesiredImagePoint(0, 0, 1.0);
  }

  void ReemUpperbodyServo::initFrameNames()
  {
    //REEM specific frames

    _headBaseFrame      = "torso_2_link";
    _headEndFrame       = "head_2_link";

    _eyeFrame           = "/stereo_gazebo_l_stereo_camera_optical_frame";

    _torsoArmBaseFrame  = "base_link";
    _torsoArmEndFrame   = "arm_right_7_link";

    _torsoBaseFrame     = "base_link";
    _torsoEndFrame      = "torso_2_link";

    _armBaseFrame       = "torso_2_link";
    _armEndFrame        = "arm_right_7_link";

    //the following frames must be published by an arucos_ros node
    _handMarkerFrame    = "marker_hand_frame";
    _objectMarkerFrame  = "marker_object_frame";
  }

  void ReemUpperbodyServo::configureParams()
  {
    int JointLimAvAlg;

    _jointLimitAvoidance = NONE;

    if ( !_nh.getParam(ros::this_node::getName() + "/joint_limit_avoidance_algorithm",  JointLimAvAlg) )
      _jointLimitAvoidance = LARGE_PROJECTION;
    else
      _jointLimitAvoidance = static_cast<jointLimitAvoidanceAlgorithm>(JointLimAvAlg);

    ROS_INFO_STREAM("JOINT LIMIT AVOIDANCE ALGORITHM: " << _jointLimitAvoidance);

    if ( !_nh.getParam(ros::this_node::getName() + "/step_by_step",  _stepByStep) )
      _stepByStep = false;

    ROS_INFO_STREAM("RUN STEP BY STEP: " << _stepByStep);

    if ( !_nh.getParam(ros::this_node::getName() + "/enable_torso_arm",  _enableArmAndTorso) )
      _enableArmAndTorso = false;

    ROS_INFO_STREAM("Enable torso+arm movement: " << _enableArmAndTorso);

    if ( !_nh.getParam(ros::this_node::getName() + "/enable_head",  _enableHead) )
      _enableHead = false;

    ROS_INFO_STREAM("Enable head movement: " << _enableHead);

    if ( !_nh.getParam(ros::this_node::getName() + "/torso_compensation_to_gaze",  _enableTorsoCompensation) )
      _enableTorsoCompensation = false;

    ROS_INFO_STREAM("Torso motion compensation to gaze control: " << _enableTorsoCompensation);

    if ( !_nh.getParam(ros::this_node::getName() + "/torso_arm_compensation_to_gaze",  _enableArmCompensation) )
      _enableArmCompensation = false;

    ROS_INFO_STREAM("Torso and arm motion compensation to gaze control: " << _enableArmCompensation);

    if ( !_nh.getParam(ros::this_node::getName() + "/gain_torso_arm",  _lambdaTorsoArm) )
      _lambdaTorsoArm = 0.2;

    ROS_INFO_STREAM("Gain of toros+arm control law: " << _lambdaTorsoArm);

    if ( !_nh.getParam(ros::this_node::getName() + "/gain_head",  _lambdaHead) )
      _lambdaHead = 0.2;

    ROS_INFO_STREAM("Gain of head control law: " << _lambdaHead);

    if ( !_nh.getParam(ros::this_node::getName() + "/max_joint_velocity",  _maxVelocity) )
      _maxVelocity = 0.2;

    ROS_INFO_STREAM("Max joint velocity (rad/s): " << _maxVelocity);

    if ( !_nh.getParam(ros::this_node::getName() + "/max_joint_velocity",  _maxVelocity) )
      _maxVelocity = 0.2;

    ROS_INFO_STREAM("Max joint velocity (rad/s): " << _maxVelocity);

    if ( !_nh.getParam(ros::this_node::getName() + "/time_step",  _timeStep) )
      _timeStep = 0.3;

    ROS_INFO_STREAM("Seconds per iteration: " << _timeStep);

    if ( !_nh.getParam(ros::this_node::getName() + "/time_from_start_factor",  _timeFromStartFactor) )
      _timeFromStartFactor = 0.3;

    ROS_INFO_STREAM("Time from start factor: " << _timeFromStartFactor);

    if ( !_nh.getParam(ros::this_node::getName() + "/close_hand",  _closeHandInGoal) )
      _closeHandInGoal = false;

    ROS_INFO_STREAM("Close hand when goal reached: " << _closeHandInGoal);

  }

  void ReemUpperbodyServo::updateArmTorsoVisualFeatures()
  {
    _taskHand->updateTransAndThetaU(_hmdMhm);
  }

  void ReemUpperbodyServo::cameraTwistsInducedByArmAndTorso()
  {
    std::cout << "before camera twist inducec by torso" << std::endl;

    //^cv = ^cV_b · ^bJe · qdot_torso   where {b} is _torsoBaseFrame and {e} is _torsoEndFrame
    _cameraTwistInducedByTorso = vpVelocityTwistMatrix(_cMtorsoArmBaseFrame) *
                                 pal::kdlJacobianToVispJacobian(_torsoChain->getJacobian(),
                                                                _torsoBaseFrameMtorsoEndFrame) *
                                 pal::stdVectorToVisp(_qdot_torso);

    std::cout << "before mid point twist inducec by torso+arm" << std::endl;

    //^cv = ^cV_b · ^bJe · qdot_torsoArm   where {b} is _torsoArmBaseFrame and {e} is _torsoArmEndFrame
    _midPointTwistInducedByTorsoAndArm = -0.5 * vpVelocityTwistMatrix(_cMtorsoArmBaseFrame) *
                                         pal::kdlJacobianToVispJacobian(_torsoAndArmChain->getJacobian(),
                                                                        _torsoArmBaseFrameMtorsoArmEndFrame) *
                                         pal::stdVectorToVisp(_qdot_torsoArm);
  }

  void ReemUpperbodyServo::runTorsoArmTask()
  {    
    _qdot_torsoArm = pal::vispToStdVector(_taskHand->computeLaw(*_torsoAndArmChain,
                                                                _hmMtorsoArmBase,
                                                                _jointLimitAvoidance != NONE,
                                                                _jointLimitAvoidance == LARGE_PROJECTION) );

    _q1dot_torsoArm = _taskHand->get_q1dot();
    _q2dot_torsoArm = _taskHand->get_q2dot();

    std::cout << "Limiting velocity of torso+Arm joints:" << std::endl;
    utility_functions::limitVelocity(_qdot_torsoArm, _maxVelocity);

    _qdot_torso = std::vector<double>(_qdot_torsoArm.begin(),
                                      _qdot_torsoArm.begin() + 2);

    _qdot_arm = std::vector<double>(_qdot_torsoArm.begin() + 2,
                                    _qdot_torsoArm.begin() + 9);

    //compute which twists will be induced in the camera due to moving the torso and the arm
    //with the above joint velocities
    if ( _enableTorsoCompensation || _enableArmCompensation )
      cameraTwistsInducedByArmAndTorso();

    //integrate next joints positions
    _torsoAndArmChain->applyVelocities(_qdot_torsoArm,
                                       _timeStep);

    std::vector<double> torsoArmJointPositions = _torsoAndArmChain->getJointPositions();

    std::vector<double> desiredTorsoJointPositions(torsoArmJointPositions.begin(),
                                                   torsoArmJointPositions.begin() + 2);

    std::vector<double> desiredArmJointPositions(torsoArmJointPositions.begin() + 2,
                                                 torsoArmJointPositions.begin() + 9);


    for (unsigned int i = 0; i < _qdot_torso.size(); ++i)
      std::cout << "torso velocity for joint (" << i << ") = " << _qdot_torso[i] << std::endl;

    for (unsigned int i = 0; i < _qdot_arm.size(); ++i)
      std::cout << "arm velocity for joint (" << i << ") = " << _qdot_arm[i] << std::endl;


    _moveParts.move( pal::MoveParts::TORSO,
                     desiredTorsoJointPositions,  //desired joint state
                     _qdot_torso,
                     ros::Duration(_timeStep * _timeFromStartFactor) );

    _moveParts.move( pal::MoveParts::RIGHT_ARM,
                     desiredArmJointPositions,  //desired joint state
                     _qdot_arm,
                     ros::Duration(_timeStep * _timeFromStartFactor));
  }


  void ReemUpperbodyServo::updateHeadTaskVisualFeatures()
  {
    //First update visual features

    vpTranslationVector cOom, cOhm;
    _cMom.extract(cOom);
    _cMhm.extract(cOhm);

    //update the normalized point coordinates that REEM must keep looking to
    double Z_currentImgPt, x_currentImgPt, y_currentImgPt;

    if ( _midPointGazing == USE_3D )
    {
      //2) Compute the mid point in 3D and then project it to the normalized coordinates
      Z_currentImgPt =   (cOom[2]+cOhm[2]) / 2;
      x_currentImgPt = ( (cOom[0]+cOhm[0]) / 2) / Z_currentImgPt;
      y_currentImgPt = ( (cOom[1]+cOhm[1]) / 2) / Z_currentImgPt;
    }
    else if ( _midPointGazing == USE_2D )
    {
      //1) first normalize each 3D point and then compute the midpoint in normalized coordinates and take
      //   the average of the Z
      x_currentImgPt = ( cOom[0]/cOom[2] + cOhm[0]/cOhm[2]) / 2;
      y_currentImgPt = ( cOom[1]/cOom[2] + cOhm[1]/cOhm[2]) / 2;
      Z_currentImgPt = ( cOom[2]+cOhm[2]) / 2;
    }
    else
      throw std::runtime_error("Invalid GAZING_TASK_MID_POINT");

    _taskHead->setCurrentImagePoint(x_currentImgPt, y_currentImgPt, Z_currentImgPt);
  }

  void ReemUpperbodyServo::runHeadTask()
  {    
    _qdot_head = pal::vispToStdVector(_taskHead->computeLaw(*_headChain,
                                                           _cMheadBase,
                                                           _cameraTwistInducedByTorso,
                                                           _midPointTwistInducedByTorsoAndArm,
                                                           _jointLimitAvoidance != NONE,
                                                           _jointLimitAvoidance == LARGE_PROJECTION) );

    _q1dot_head = _taskHead->get_q1dot();
    _q2dot_head = _taskHead->get_q2dot();


    for (unsigned int i = 0; i < _qdot_head.size(); ++i)
      std::cout << "qdot_head(" << i << ") = " << _qdot_head[i] << std::endl;

    utility_functions::limitVelocity(_qdot_head, _maxVelocity);

    _headChain->applyVelocities(_qdot_head,
                                _timeStep);

    _moveParts.move(pal::MoveParts::HEAD,
                    _headChain->getJointPositions(), //desired joint state
                    _qdot_head,
                    ros::Duration(_timeStep * _timeFromStartFactor));
  }

  void ReemUpperbodyServo::waitKeyIfNecessary()
  {
    if ( _stepByStep )
    {
      char dummyVar;
      ROS_INFO("Press a key + ENTER");
      std::cin >> dummyVar;
    }
  }

  void ReemUpperbodyServo::run()
  {
    getConstantFramesTransforms();
    _taskHand->setHandMarkerPose(_hmMh.inverse());
    updateMovingFramesTransforms();

    loadDesiredState();

    //set the right hand at pre-grasp pose
    _moveParts.moveRightHand(pal::MoveParts::HAND_PREGRASP);

    bool stop = false;

    ros::Rate loop_rate(1.0/_timeStep);

    while ( !stop && ros::ok() )
    {
      updateMovingFramesTransforms();

      if ( !_handMarkerLost && !_objectMarkerLost )
      {      
        if ( _enableArmAndTorso )
        {
          //std::cout << "Torso+Arm joint state:" << _torsoAndArmChain->jointStateToStr() << std::endl;
          updateArmTorsoVisualFeatures();          
          runTorsoArmTask();
          //std::cout << "Torso+Arm joint new state:" << _torsoAndArmChain->jointStateToStr() << std::endl;
          if ( _closeHandInGoal && _taskHand->getErrorNorm() < 0.02 )
          {
            //TODO: close hand
          }
        }

        if ( _enableHead )
        {
          updateHeadTaskVisualFeatures();
          runHeadTask();
        }                

        waitKeyIfNecessary();

        std::cout << "Ok" << std::endl << std::endl;
      }

      publishTrajectory();
      publishPlots();

      //ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void ReemUpperbodyServo::publishTrajectory()
  {
    // Publish a RVIZ marker corresponding to the arm_right_7_joint pose obtained from TF
    vpHomogeneousMatrix cMarm_end_effector;
    pal::waitUntilTransform(_tfListener, _eyeFrame, _torsoArmEndFrame, cMarm_end_effector);
    visualization_msgs::Marker marker;
    geometry_msgs::Pose markerPose;
    pal::convertVispHMatToGMPose(cMarm_end_effector, markerPose);
    utility_functions::makeMarkerMSG(marker,
                                     _eyeFrame,
                                     markerPose,
                                     _trajectoryMarkerCounter,
                                     _taskHand->get_q2dot().sumSquare() > 0.0);

    ++_trajectoryMarkerCounter;     // define unique ID numbers
    _trajectoryPub.publish(marker); // Publish the marker
  }

  void ReemUpperbodyServo::publishPlots()
  {
    // Publish Errors of Visual Servoing
    std_msgs::Float64MultiArray plotMsg;

    utility_functions::makePlotMSG(plotMsg,
                                   _taskHead->getTaskError(), pal::stdVectorToVisp(_qdot_head),
                                   _taskHand->getTaskError(), pal::stdVectorToVisp(_qdot_torsoArm),
                                   _taskHead->get_q1dot(), _taskHead->get_q2dot(),
                                   _taskHand->get_q1dot(), _taskHand->get_q2dot(),
                                   _taskHead->getCurrentImagePoint_x(),
                                   _taskHead->getCurrentImagePoint_y());
    _plotPub.publish(plotMsg);
  }

  void ReemUpperbodyServo::getConstantFramesTransforms()
  {
    pal::waitUntilTransform(_tfListener,  _handMarkerFrame, _torsoArmEndFrame,  _hmMh);
  }

  void ReemUpperbodyServo::updateMovingFramesTransforms()
  {

    pal::waitUntilTransform(_tfListener, _handMarkerFrame,    _torsoArmBaseFrame,  _hmMtorsoArmBase);
    pal::waitUntilTransform(_tfListener, _eyeFrame,           _objectMarkerFrame,  _cMom);
    pal::waitUntilTransform(_tfListener, _eyeFrame,           _torsoArmBaseFrame,  _cMtorsoArmBaseFrame);
    pal::waitUntilTransform(_tfListener, _torsoBaseFrame,     _torsoEndFrame,      _torsoBaseFrameMtorsoEndFrame);
    pal::waitUntilTransform(_tfListener, _torsoArmBaseFrame,  _torsoArmEndFrame,   _torsoArmBaseFrameMtorsoArmEndFrame);

    ros::Time transformTime = pal::waitUntilTransform(_tfListener, _eyeFrame,     _handMarkerFrame,    _cMhm);
    _handMarkerLost = (ros::Time::now() - transformTime).sec >= 1;

    transformTime = pal::waitUntilTransform(_tfListener, _eyeFrame,     _headBaseFrame,      _cMheadBase);
    _objectMarkerLost = (ros::Time::now() - transformTime).sec >= 1;

    // Compute Desired Hand Marker position in the camera frame
    _cMhmd = _cMom *   //current object marker pose in camera frame
             _omMhmd;  //desired hand marker frame in object marker frame obtained from file

    //now compute the transformation between the current hand marker frame in the desired hand marker frame
    _hmdMhm = _cMhmd.inverse() * _cMhm;
  }


  void ReemUpperbodyServo::loadDesiredState()
  {
    pal::getTransformFromFile(_desiredStateFileName, _omMhmd);
  }
}

/////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc,argv,"reem_upperbody_visual_servo_upperbody"); // Create and name the Node
  ros::NodeHandle node;

  std::string desiredStateFileName = ros::package::getPath("reem_upperbody_visual_servo") + "/desired_state.yml";

  if ( argc > 1 )
  {
    desiredStateFileName = argv[1];
  }

  pal::ReemUpperbodyServo servo(node,
                                desiredStateFileName);

  servo.run();

  return 0;
} //end main
