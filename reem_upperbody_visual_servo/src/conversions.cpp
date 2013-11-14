
#include "conversions.h"

namespace pal {

  std::vector<double> eigenToStdVector(const Eigen::VectorXd& eigenV,
                                  int initialPose,
                                  int length)
  {
    if ( length == - 1)
      length = eigenV.size() - initialPose;
    std::vector<double> stdVector(length);
    for (int i = initialPose; i < initialPose + length; ++i)
      stdVector[i - initialPose] = eigenV[i];
    return stdVector;
  }

  std::vector<double> vispToStdVector(const vpColVector& vpVector,
                                  int initialPose,
                                  int length)
  {
    if ( length == - 1)
      length = vpVector.size() - initialPose;
    std::vector<double> stdVector(length);
    for (int i = initialPose; i < initialPose + length; ++i)
      stdVector[i - initialPose] = vpVector[i];
    return stdVector;
  }

  vpColVector stdVectorToVisp(const std::vector<double>& vector)
  {
    vpColVector vpVector;
    vpVector.resize(vector.size());
    for (unsigned int i = 0; i < vector.size(); ++i)
      vpVector[i] = vector[i];
    return vpVector;
  }


  void convertGMPoseToVispHMat(const geometry_msgs::Pose &pose,
                               vpHomogeneousMatrix &HMatrix)
  {
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    double angle = q.getAngle();
    HMatrix.buildFrom(pose.position.x,
                      pose.position.y,
                      pose.position.z,
                      angle*q.getAxis().x(),
                      angle*q.getAxis().y(),
                      angle*q.getAxis().z()
                      );
  }

  void convertVispHMatToGMPose(const vpHomogeneousMatrix &HMatrix,
                               geometry_msgs::Pose &pose)
  {
    vpTranslationVector Trans;
    HMatrix.extract(Trans);
    vpThetaUVector ThetaU;
    HMatrix.extract(ThetaU);
    double vpAngle;
    vpColVector vpAxis;
    ThetaU.extract(vpAngle, vpAxis);

    btVector3 btAxis;
    btScalar btAngle = vpAngle;
    if(fabs(vpAngle) < 1.0e-15) // the case of no rotation, prevents nan on btQuaternion
    {
      btAngle = 0.0;
      btAxis.setValue(1.0, 0.0, 0.0);
    }
    else
    { btAxis.setValue(vpAxis[0], vpAxis[1], vpAxis[2]); }
    btQuaternion Quat(btAxis, btAngle);
    pose.position.x = Trans[0];
    pose.position.y = Trans[1];
    pose.position.z = Trans[2];
    pose.orientation.w = Quat.w();
    pose.orientation.x = Quat.x();
    pose.orientation.y = Quat.y();
    pose.orientation.z = Quat.z();
  }

  void convertEigenMatToVisp(const Eigen::MatrixXd& em, vpMatrix& vmat)
  {
    vmat.resize(em.rows(), em.cols());
    for (int i = 0; i < em.rows(); ++i)
      for (int j = 0; j < em.cols(); ++j)
        vmat[i][j] = em(i,j);
  }

  void convertVispHMatToTF(const vpHomogeneousMatrix &HMatrix,
                           tf::Transform &TFtransform)
  {
    vpTranslationVector Trans;
    HMatrix.extract(Trans);
    vpThetaUVector ThetaU;
    HMatrix.extract(ThetaU);
    double vpAngle;
    vpColVector vpAxis;
    ThetaU.extract(vpAngle, vpAxis);

    btVector3 btAxis;
    btScalar btAngle = vpAngle;
    if(fabs(vpAngle) < 1.0e-15) // the case of no rotation, prevents nan on btQuaternion
    {
      btAngle = 0.0;
      btAxis.setValue(1.0, 0.0, 0.0);
    }
    else
    {
      btAxis.setValue(vpAxis[0], vpAxis[1], vpAxis[2]);
    }
    btQuaternion Quat(btAxis, btAngle);
    TFtransform.setOrigin( tf::Vector3(Trans[0], Trans[1], Trans[2] ) );
    TFtransform.setRotation(Quat);
  }

  void convertTFtoVispHMat(const tf::StampedTransform &TFtransform,
                           vpHomogeneousMatrix &HMatrix)
  {
    double Angle; // Theta U angle of the transform
    Angle = TFtransform.getRotation().getAngle();

    HMatrix.buildFrom(TFtransform.getOrigin().x(),
                      TFtransform.getOrigin().y(),
                      TFtransform.getOrigin().z(),
                      Angle*TFtransform.getRotation().getAxis().x(),
                      Angle*TFtransform.getRotation().getAxis().y(),
                      Angle*TFtransform.getRotation().getAxis().z()
                      );
  }

  void convertKDLJacToVispMat(const KDL::Jacobian &kdlJac,
                              vpMatrix &vispMat)
  {
    for(unsigned int i=0; i<kdlJac.rows(); ++i)
    {
      for(unsigned int j=0; j<kdlJac.columns(); ++j)
      {
        vispMat[i][j] = kdlJac.data(i,j);
      }
    }
  }

  vpMatrix convertKDLJacToVispMat(const KDL::Jacobian& kdlJac)
  {
    vpMatrix vispMat(kdlJac.rows(), kdlJac.columns());
    convertKDLJacToVispMat(kdlJac, vispMat);
    return vispMat;
  }

  vpMatrix kdlJacobianToVispJacobian(const KDL::Jacobian& kdlJacobian,
                                     const vpHomogeneousMatrix& bMe)
  {
    vpMatrix vpJacobian(kdlJacobian.rows(), kdlJacobian.columns());

    KDL::Jacobian kdlJacobianCopy = kdlJacobian;

    // Fix reference point so that it is in the base frame
    vpTranslationVector bOe; //origin of end frame in base frame
    bMe.extract(bOe);
    kdlJacobianCopy.changeRefPoint(KDL::Vector(-bOe[0], -bOe[1], -bOe[2]));
    pal::convertKDLJacToVispMat(kdlJacobianCopy, vpJacobian);// KDL data --> Visp data
    return vpJacobian;
  }


} //pal


