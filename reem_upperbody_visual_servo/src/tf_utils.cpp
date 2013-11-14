
#include "tf_utils.h"

// PAL headers
#include "conversions.h"

// Other headers
#include <yaml-cpp/yaml.h>

// Std C++ headers
#include <stdexcept>

namespace pal {

  ros::Time waitUntilTransform(tf::TransformListener& tfListener,
                               const std::string& baseFrame,
                               const std::string& targetFrame,
                               vpHomogeneousMatrix& pose)
  {
    tf::StampedTransform poseTF;
    pal::waitUntilTransform(tfListener,
                            baseFrame,
                            targetFrame,
                            poseTF);

    pal::convertTFtoVispHMat(poseTF, pose);
    return poseTF.stamp_;
  }

  ros::Time waitUntilTransform(tf::TransformListener& tfListener,
                               const std::string& baseFrame,
                               const std::string& targetFrame,
                               tf::StampedTransform& pose)
  {
    if ( tfListener.waitForTransform(baseFrame, targetFrame, ros::Time(0), ros::Duration(5), ros::Duration(0.01)) )
    {
      tfListener.lookupTransform(baseFrame, targetFrame, ros::Time(0), pose);
      return pose.stamp_;
    }
    else
      throw std::runtime_error("Error in pal::waitUntilTransform: timeout while waiting transform from " +
                               targetFrame + " to " + baseFrame);
  }


  void getTransformFromFile(const std::string& fileName,
                            vpHomogeneousMatrix& pose)
  {
    std::ifstream in(fileName.c_str());

    YAML::Parser parser(in);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    double tx, ty, tz;

    doc["trans_x"] >> tx;
    doc["trans_y"] >> ty;
    doc["trans_z"] >> tz;

    tf::Quaternion rot;
    double v;

    doc["q_x"] >> v; rot.setX(v);
    doc["q_y"] >> v; rot.setY(v);
    doc["q_z"] >> v; rot.setZ(v);
    doc["q_w"] >> v; rot.setW(v);

    tf::StampedTransform tfTrans;
    tfTrans.setOrigin( tf::Vector3(tx, ty, tz) );
    tfTrans.setRotation(rot);

    pal::convertTFtoVispHMat(tfTrans, pose);
  }

  void publishTransform(tf::TransformBroadcaster& tfBroadcaster,
                        const tf::Transform& transform,
                        const std::string& frameId,
                        const std::string& childFrame)
  {
    tfBroadcaster.sendTransform( tf::StampedTransform( transform,
                                                       ros::Time::now(),
                                                       frameId,
                                                       childFrame ) );
  }

  vpMatrix kdlJacobianToVispJacobian(tf::TransformListener& tfListener,
                                     const std::string& baseFrame,
                                     const std::string& endFrame,
                                     const KDL::Jacobian& kdlJacobian)
  {
    vpMatrix vpJacobian(kdlJacobian.rows(), kdlJacobian.columns());

    KDL::Jacobian kdlJacobianCopy = kdlJacobian;

    // Fix reference point so that it is in the base frame
    vpHomogeneousMatrix baseMend; //transformation from end frame to base frame
    pal::waitUntilTransform(tfListener, baseFrame, endFrame, baseMend);
    vpTranslationVector baseOend; //origin of end frame in base frame
    baseMend.extract(baseOend);
    kdlJacobianCopy.changeRefPoint(KDL::Vector(-baseOend[0], -baseOend[1], -baseOend[2]));
    pal::convertKDLJacToVispMat(kdlJacobianCopy, vpJacobian);// KDL data --> Visp data
    return vpJacobian;
  }

}
