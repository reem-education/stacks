
#include "hand_task.h"
#include "conversions.h"
#include "secondary_task.h"

namespace pal {

  HandTask::HandTask(double gain):
    _gain(gain)
  {
    setServo(vpServo::EYEINHAND_L_cVe_eJe);
    setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    setLambda(_gain);
  }

  HandTask::~HandTask()
  {
    kill();
  }

  double HandTask::getErrorNorm() const
  {
    return sqrt((error.transpose() * error)[0]);
  }

  void HandTask::setHandMarkerPose(const vpHomogeneousMatrix &hMhm)
  {
    _hMhm = hMhm;
  }

  void HandTask::updateTransAndThetaU(vpHomogeneousMatrix& hmdMhm)
  {
    if ( _trans.get() == NULL )
    {
      _trans.reset( new vpFeatureTranslation(hmdMhm, vpFeatureTranslation::cdMc) );
      _thetaU.reset( new vpFeatureThetaU(hmdMhm, vpFeatureThetaU::cdRc) );

      addFeature(*_trans);
      addFeature(*_thetaU);
    }
    else
    {
      _trans->buildFrom(hmdMhm);
      _thetaU->buildFrom(hmdMhm);
    }
  }

  void HandTask::setJacobian(const vpMatrix& jacobian)
  {
    set_eJe(const_cast<vpMatrix&>(jacobian));
  }

  void HandTask::setBaseToMarkerTwist(const vpHomogeneousMatrix& torsoBaseToHand)
  {
    set_cVe(const_cast<vpHomogeneousMatrix&>(torsoBaseToHand)); //implicit construction of a vpVelocityTwistMatrix
  }

  vpColVector HandTask::computeLaw(const pal::KinematicChain& chain,
                                   const vpHomogeneousMatrix& hmMtorsoBase,
                                   bool enableJointLimitAvoidance,
                                   bool useLargeProjectionOperator)
  {
    vpColVector qdot;

    //update the jacobian
    vpHomogeneousMatrix bMe;
    bMe = (_hMhm * hmMtorsoBase).inverse();
    setJacobian( pal::kdlJacobianToVispJacobian(chain.getJacobian(), bMe) );
    //and the twist transformation from the torso+arm base frame to the end-effector (hand) marker frame
    setBaseToMarkerTwist( hmMtorsoBase );

    qdot.resize(chain.getNumberOfJoints());
    qdot = computeControlLaw();

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
        {
          std::cout << "q2dot(" << i << ") is " << _q2dot[i] << " because joint is at " <<
                       chain.getJointPositions()[i] << " and limits are [" <<
                       chain.getLowerJointLimits()[i] << ", " << chain.getUpperJointLimits()[i] << "]" << std::endl;
        }
      }

      qdot = final_qdot;

    }

    return qdot;
  }

  vpColVector HandTask::get_q1dot() const
  {
    return _q1dot;
  }

  vpColVector HandTask::get_q2dot() const
  {
    return _q2dot;
  }

  vpColVector HandTask::getTaskError() const
  {
    return getError();
  }


}
