
#include "head_task.h"
#include "secondary_task.h"
#include "conversions.h"
#include "tf_utils.h"


namespace pal {

  HeadTask::HeadTask(double gain):
    _gain(gain)
  {
    setServo(vpServo::EYEINHAND_L_cVe_eJe);
    setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    addFeature(_currentImagePoint, _desiredImagePoint);
    setLambda(_gain);
  }

  HeadTask::~HeadTask()
  {
    kill();
  }

  void HeadTask::setDesiredImagePoint(double x, double y, double Z)
  {
    _desiredImagePoint.buildFrom(x, y, Z);
  }

  void HeadTask::setCurrentImagePoint(double x, double y, double Z)
  {
    _currentImagePoint.buildFrom(x, y, Z);
  }

  double HeadTask::getCurrentImagePoint_x() const
  {
    return _currentImagePoint.get_x();
  }

  double HeadTask::getCurrentImagePoint_y() const
  {
    return _currentImagePoint.get_y();
  }

  void HeadTask::setJacobian(const vpMatrix& jacobian)
  {
    //jacobian expressed in the base frame and reference point equal to the base frame origin
    set_eJe(const_cast<vpMatrix&>(jacobian));
  }

  void HeadTask::setBaseToCameraTwist(const vpHomogeneousMatrix& headBaseToCamera)
  {
    set_cVe(const_cast<vpHomogeneousMatrix&>(headBaseToCamera)); //implicit construction of a vpVelocityTwistMatrix
  }

  vpColVector HeadTask::computeLaw(const pal::KinematicChain& chain,
                                   const vpHomogeneousMatrix& cMheadBase,
                                   const vpColVector& cameraTwistInducedByTorso,
                                   const vpColVector& targetPointTwistInducedByTorsoArm,
                                   bool enableJointLimitAvoidance,
                                   bool useLargeProjectionOperator)
  {
    vpColVector qdot;

    //update the jacobian
    setJacobian( pal::convertKDLJacToVispMat(chain.getJacobian()) );
    //and the twist transformation from the head base frame to the camera frame
    setBaseToCameraTwist( cMheadBase );

    qdot.resize(chain.getNumberOfJoints());
    //compute head qdot without any feedforward term from torso and arm joint motions
    computeControlLaw();    

    qdot = J1.pseudoInverse() * ( (-_gain*getError()) -L*(cameraTwistInducedByTorso + targetPointTwistInducedByTorsoArm) );

    _q1dot = qdot;
    _q2dot.resize(chain.getNumberOfJoints());
    _q2dot = 0;

    if ( enableJointLimitAvoidance )
    {
      vpColVector final_qdot(chain.getNumberOfJoints());      

      secondary_task::computeSecondaryTask(qdot,
                                           vpServo::getTaskJacobian(),
                                           _gain,
                                           vpServo::getError(),
                                           chain.getJointPositionsKdl(),
                                           chain.getLowerJointLimits(),
                                           chain.getUpperJointLimits(),
                                           useLargeProjectionOperator,
                                           final_qdot,
                                           _q1dot,
                                           _q2dot);

      for (int i = 0; i < chain.getNumberOfJoints(); ++i)
      {
        if ( _q2dot[i] != 0 )
          std::cout << "q2dot(" << i << ") is " << _q2dot[i] << " because joint is at " <<
                       chain.getJointPositions()[i] << " and limits are [" <<
                       chain.getLowerJointLimits()[i] << ", " << chain.getUpperJointLimits()[i] << "]" << std::endl;
      }

      qdot = final_qdot;

    }

//    print();
//    std::cout << std::endl << "cVe: " << get_cVe() << std::endl;
//    std::cout << std::endl << "eJe: " << get_eJe() << std::endl;
//    std::cout << "qdot: " << qdot << std::endl;

    return qdot;
  }

  vpColVector HeadTask::get_q1dot() const
  {
    return _q1dot;
  }

  vpColVector HeadTask::get_q2dot() const
  {
    return _q2dot;
  }

  vpColVector HeadTask::getTaskError() const
  {
    return getError();
  }

}
