
#include "kinematic_chain.h"

// PAL headers
#include "conversions.h"

// ROS headers
#include <sensor_msgs/JointState.h>
#include <ros/topic.h>
#include <angles/angles.h>

// KDL headers
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/segment.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>

// Boost headers
#include <boost/foreach.hpp>

// Std C++ headers
#include <stdexcept>

namespace pal {

  KinematicChain::KinematicChain(const std::string& robotDescription,
                                 const std::string& endFrame,
                                 const std::string& baseFrame)
  {
    if ( !kdl_parser::treeFromString(robotDescription, _kdlJointTree) )
      throw std::runtime_error("Error in KinematicChain::KinematicChain: unable to get KDL Tree from string");

    if( !_kdlJointTree.getChain(baseFrame, endFrame, _kdlChain))
      throw std::runtime_error("Error in KinematicChain::KinematicChain: unable to get specified chain from tree (" +
                               std::string("base frame: ") + baseFrame + "   end frame: " + endFrame + ")");

    _kdlJointToJacobianHead.reset(new KDL::ChainJntToJacSolver(_kdlChain));

    _numberOfJoints = _kdlChain.getNrOfJoints();
    _jointNames.resize(_numberOfJoints);
    _jointsPosition.resize(_numberOfJoints);    

    if ( !_model.initString(robotDescription) )
      throw std::runtime_error("Error in KinematicChain::KinematicChain: unable to get urdf model from the provided string");

    updateJointNames();
    updateJointLimits();
  }

  KinematicChain::~KinematicChain()
  {
  }

  void KinematicChain::updateJointNames()
  {
    int jointCounter = 0;
    for ( unsigned int i = 0; i < _kdlChain.getNrOfSegments(); ++i)
    {
      if ( _kdlChain.getSegment(i).getJoint().getType() != KDL::Joint::None )
      {
        if ( jointCounter >= _numberOfJoints )
          throw std::runtime_error("Error in KinematicChain::updateJointNames: unexpected joint");
        _jointNames[jointCounter] = _kdlChain.getSegment(i).getJoint().getName();
        //std::cout << "updating joint (" << jointCounter << ") name = " << _jointNames[jointCounter] << std::endl;
        ++jointCounter;
      }
    }
  }

  void KinematicChain::setJointPositions(const std::vector<double>& positions)
  {
    if ( _jointsPosition.data.size() != static_cast<int>(positions.size()) )
      throw std::runtime_error("Error in KinematicChain::setJointPositions: unexpected number of joints provided");

    for (int i = 0; i < _jointsPosition.data.size(); ++i)
      _jointsPosition.data[i] = positions[i];
  }

  void KinematicChain::applyVelocities(const std::vector<double>& qdot,
                                       double timeStepSec)
  {
    if ( static_cast<int>(qdot.size()) != _numberOfJoints )
      throw std::runtime_error("Error in KinematicChain::applyVelocities: qdot has bad length");

    for (int i = 0; i < _numberOfJoints; ++i)
    {
      _jointsPosition.data[i] += qdot[i] * timeStepSec;
      _jointsPosition.data[i] = angles::normalize_angle(_jointsPosition.data[i]);
    }
  }

  int KinematicChain::getNumberOfJoints() const
  {
    return _numberOfJoints;
  }

  void KinematicChain::updateJointStateFromTopic()
  {
    // Get current joint state
    sensor_msgs::JointStateConstPtr jointState =
        ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(5.0));

    if (!jointState)
      throw std::runtime_error("Error in KinematicChain::updateJointStateFromTopic: timed-out waiting for the current joint state");

    int jointCounter = 0;
    BOOST_FOREACH(const std::string& jointName, _jointNames)
    {
      bool found = false;
      std::size_t i = 0;
      while ( !found && i < jointState->name.size() )
      {
        found = jointName == jointState->name[i];
        if ( !found )
          ++i;
      }
      if ( !found )
        throw std::runtime_error("Error in KinematicChain::updateJointStateFromTopic: not possible to match joint " +
                                 jointName + " from kinematic chain to any joint in the /joint_states");
      _jointsPosition.data[jointCounter] = jointState->position[i];
      ++jointCounter;
    }
  }

  void KinematicChain::updateJointLimits()
  {
    boost::shared_ptr<const urdf::Joint> joint;

    _lowerJointLimits.clear();
    _upperJointLimits.clear();

    for(unsigned int i=0; i< _kdlChain.getNrOfJoints(); ++i)
    {
      if (_kdlChain.getSegment(i).getJoint().getType() != KDL::Joint::None)
      {
        joint = _model.getJoint(_kdlChain.getSegment(i).getJoint().getName());
        if ( joint->type != urdf::Joint::CONTINUOUS )
        {
          _lowerJointLimits.push_back(joint->limits->lower);
          _upperJointLimits.push_back(joint->limits->upper);
        }
        else
        {
          _lowerJointLimits.push_back(-3.14/2.0);
          _upperJointLimits.push_back(3.14/2.0);
        }
      }
    }
  }

  std::vector<double> KinematicChain::getJointPositions() const
  {
    return pal::eigenToStdVector(_jointsPosition.data);
  }

  KDL::JntArray KinematicChain::getJointPositionsKdl() const
  {
    return _jointsPosition;
  }

  const std::vector<double>& KinematicChain::getLowerJointLimits() const
  {
    return _lowerJointLimits;
  }

  const std::vector<double>& KinematicChain::getUpperJointLimits() const
  {
    return _upperJointLimits;
  }

  void KinematicChain::setLowerJointLimits( const std::vector<double>& lowerLimits )
  {
    _lowerJointLimits = lowerLimits;
  }

  void KinematicChain::setUpperJointLimits( const std::vector<double>& upperLimits )
  {
    _upperJointLimits = upperLimits;
  }


  KDL::Jacobian KinematicChain::getJacobian() const
  {    
    // KDL calculates the jacobian expressed in the base frame of the chain,
    // with reference point at the end effector of the chain
    KDL::Jacobian jacobian(_numberOfJoints);
    _kdlJointToJacobianHead->JntToJac(_jointsPosition, jacobian);

//    std::cout << "KDL jacobian: " << std::endl;
//    for (unsigned int r = 0; r < _jacobian.rows(); ++r)
//      for (unsigned int c = 0; c < _jacobian.columns(); ++c)
//      {
//        if ( c == 0 )
//          std::cout << std::endl;
//        std::cout << _jacobian.data(r,c) << " ";
//      }
//    std::cout << std::endl;

    return jacobian;
  }


  std::string KinematicChain::jointStateToStr() const
  {
    std::stringstream ss;
    for (int i = 0; i < _jointsPosition.data.rows(); ++i)
      ss << "  " << _jointNames[i] << ": " << _jointsPosition.data[i];
    return ss.str();
  }


}
