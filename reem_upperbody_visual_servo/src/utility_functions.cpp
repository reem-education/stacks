
#include "conversions.h"

#include "utility_functions.h"

// ROS conversions
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <LinearMath/btQuaternion.h>

#include <fstream>


namespace utility_functions
{

    // Fill in the joint trajectory message for the RIGHT ARM
    void makeArmJointTrajMSG(trajectory_msgs::JointTrajectory &JTmsg,
                             KDL::JntArray &kdlArmJointsPosition,
                             vpColVector &qdot,
                             double time_from_start)
    {
        JTmsg.joint_names.push_back("arm_right_1_joint");
        JTmsg.joint_names.push_back("arm_right_2_joint");
        JTmsg.joint_names.push_back("arm_right_3_joint");
        JTmsg.joint_names.push_back("arm_right_4_joint");
        JTmsg.joint_names.push_back("arm_right_5_joint");
        JTmsg.joint_names.push_back("arm_right_6_joint");
        JTmsg.joint_names.push_back("arm_right_7_joint");

        // create the trajectory point
        trajectory_msgs::JointTrajectoryPoint JTpointtMSG;
        for(unsigned int i = 0; i < JTmsg.joint_names.size(); i++)
        {
            JTpointtMSG.positions.push_back(kdlArmJointsPosition.data[i]);
            JTpointtMSG.velocities.push_back(qdot[i]);
            JTpointtMSG.time_from_start = ros::Duration(time_from_start); // !!! NEED to tune this in case of shaking
        }

        // create the trajectory message
        JTmsg.points.push_back(JTpointtMSG);
        JTmsg.header.stamp = ros::Time::now();
    }

    void makeTorsoAndRightArmJointTrajMSG(const KDL::JntArray& kdlArmJointsPosition,
                                          const vpColVector &qdot,
                                          trajectory_msgs::JointTrajectory &torsoJointTrajMsg,
                                          trajectory_msgs::JointTrajectory &rightArmJointTrajMsg,
                                          double time_from_start,
                                          double time_from_start_factor)
    {
      //First create the joint trajectory message for the torso controller:
      torsoJointTrajMsg.joint_names.push_back("torso_1_joint");
      torsoJointTrajMsg.joint_names.push_back("torso_2_joint");
      // create the trajectory point
      trajectory_msgs::JointTrajectoryPoint torsoJointTrajPointMsg;
      for(unsigned int i = 0; i < torsoJointTrajMsg.joint_names.size(); i++)
      {
          torsoJointTrajPointMsg.positions.push_back(kdlArmJointsPosition.data[i]);
          torsoJointTrajPointMsg.velocities.push_back(qdot[i]);
          // Time overlapping is important to let interpolation play
          torsoJointTrajPointMsg.time_from_start = ros::Duration(time_from_start*time_from_start_factor);
      }
      // create the trajectory message
      torsoJointTrajMsg.points.push_back(torsoJointTrajPointMsg);
      torsoJointTrajMsg.header.stamp = ros::Time::now() + ros::Duration(0.1);

      //now create the joint trajectory message for the right arm controller:
      rightArmJointTrajMsg.joint_names.push_back("arm_right_1_joint");
      rightArmJointTrajMsg.joint_names.push_back("arm_right_2_joint");
      rightArmJointTrajMsg.joint_names.push_back("arm_right_3_joint");
      rightArmJointTrajMsg.joint_names.push_back("arm_right_4_joint");
      rightArmJointTrajMsg.joint_names.push_back("arm_right_5_joint");
      rightArmJointTrajMsg.joint_names.push_back("arm_right_6_joint");
      rightArmJointTrajMsg.joint_names.push_back("arm_right_7_joint");
      // create the trajectory point
      trajectory_msgs::JointTrajectoryPoint rightArmJointTrajPointMsg;
      for(unsigned int i = 0; i < rightArmJointTrajMsg.joint_names.size(); i++)
      {
          rightArmJointTrajPointMsg.positions.push_back(kdlArmJointsPosition.data[i + torsoJointTrajMsg.joint_names.size()]);
          rightArmJointTrajPointMsg.velocities.push_back(qdot[i + torsoJointTrajMsg.joint_names.size()]);
          rightArmJointTrajPointMsg.time_from_start = ros::Duration(time_from_start*time_from_start_factor); // Time overlapping is important to let interpolation play
      }
      // create the trajectory message
      rightArmJointTrajMsg.points.push_back(rightArmJointTrajPointMsg);
      rightArmJointTrajMsg.header.stamp = ros::Time::now() + ros::Duration(0.1);
    }

    // Fill in the joint trajectory message for the RIGHT ARM and TORSO
    void makeRightArmTorsoJointTrajMSG(trajectory_msgs::JointTrajectory &JTmsg,
                                       KDL::JntArray &kdlArmJointsPosition,
                                       vpColVector &qdot,
                                       double time_from_start,
                                       bool useTorso,
                                       double time_from_start_factor)
    {
        if(useTorso)
        {
            JTmsg.joint_names.push_back("torso_1_joint");
            JTmsg.joint_names.push_back("torso_2_joint");
            JTmsg.joint_names.push_back("arm_right_1_joint");
            JTmsg.joint_names.push_back("arm_right_2_joint");
            JTmsg.joint_names.push_back("arm_right_3_joint");
            JTmsg.joint_names.push_back("arm_right_4_joint");
            JTmsg.joint_names.push_back("arm_right_5_joint");
            JTmsg.joint_names.push_back("arm_right_6_joint");
            JTmsg.joint_names.push_back("arm_right_7_joint");
            // create the trajectory point
            trajectory_msgs::JointTrajectoryPoint JTpointtMSG;
            for(unsigned int i = 0; i < JTmsg.joint_names.size(); i++)
            {
                JTpointtMSG.positions.push_back(kdlArmJointsPosition.data[i]);
                JTpointtMSG.velocities.push_back(qdot[i]);
                JTpointtMSG.time_from_start = ros::Duration(time_from_start*time_from_start_factor); // Time overlapping is important to let interpolation play
            }
            // create the trajectory message
            JTmsg.points.push_back(JTpointtMSG);
            JTmsg.header.stamp = ros::Time::now() + ros::Duration(0.1);
        }
        else
        {
            JTmsg.joint_names.push_back("arm_right_1_joint");
            JTmsg.joint_names.push_back("arm_right_2_joint");
            JTmsg.joint_names.push_back("arm_right_3_joint");
            JTmsg.joint_names.push_back("arm_right_4_joint");
            JTmsg.joint_names.push_back("arm_right_5_joint");
            JTmsg.joint_names.push_back("arm_right_6_joint");
            JTmsg.joint_names.push_back("arm_right_7_joint");
            // create the trajectory point
            trajectory_msgs::JointTrajectoryPoint JTpointtMSG;
            for(unsigned int i = 0; i < JTmsg.joint_names.size(); i++)
            {
                JTpointtMSG.positions.push_back(kdlArmJointsPosition.data[i]);
                JTpointtMSG.velocities.push_back(qdot[i]);
                JTpointtMSG.time_from_start = ros::Duration(time_from_start*time_from_start_factor); // Time overlapping is important to let interpolation play
            }
            JTmsg.joint_names.push_back("torso_1_joint");
            JTmsg.joint_names.push_back("torso_2_joint");
            JTpointtMSG.positions.push_back(0.0);
            JTpointtMSG.positions.push_back(0.0);
            JTpointtMSG.velocities.push_back(0.0);
            JTpointtMSG.velocities.push_back(0.0);

            // create the trajectory message
            JTmsg.points.push_back(JTpointtMSG);
            JTmsg.header.stamp = ros::Time::now() + ros::Duration(0.1);
        }
    }

    // Fill in the joint trajectory message for the LEFT ARM
    void makeArmJointTrajMSG_left(trajectory_msgs::JointTrajectory &JTmsg,
                                  KDL::JntArray &kdlArmJointsPosition,
                                  vpColVector &qdot,
                                  double time_from_start
                                  )
    {
        JTmsg.joint_names.push_back("arm_left_1_joint");
        JTmsg.joint_names.push_back("arm_left_2_joint");
        JTmsg.joint_names.push_back("arm_left_3_joint");
        JTmsg.joint_names.push_back("arm_left_4_joint");
        JTmsg.joint_names.push_back("arm_left_5_joint");
        JTmsg.joint_names.push_back("arm_left_6_joint");
        JTmsg.joint_names.push_back("arm_left_7_joint");

        // create the trajectory point
        trajectory_msgs::JointTrajectoryPoint JTpointtMSG;
        for(unsigned int i = 0; i < JTmsg.joint_names.size(); i++)
        {
            JTpointtMSG.positions.push_back(kdlArmJointsPosition.data[i]);
            JTpointtMSG.velocities.push_back(qdot[i]);
            JTpointtMSG.time_from_start = ros::Duration(time_from_start*1.20); // Time overlapping is important to let interpolation play
        }

        // create the trajectory message
        JTmsg.points.push_back(JTpointtMSG);
        JTmsg.header.stamp = ros::Time::now();
    }

    // Fill in the joint trajectory message for the HEAD
    void makeHeadJointTrajMSG(trajectory_msgs::JointTrajectory &JTmsg,
                              KDL::JntArray &kdlHeadJointsPosition,
                              vpColVector &qdot,
                              double time_from_start,
                              double time_from_start_factor
                              )
    {
        JTmsg.joint_names.push_back("head_1_joint");
        JTmsg.joint_names.push_back("head_2_joint");

        // create the trajectory point
        trajectory_msgs::JointTrajectoryPoint JTpointtMSG;
        for(unsigned int i = 0; i < JTmsg.joint_names.size(); i++)
        {
            JTpointtMSG.positions.push_back(kdlHeadJointsPosition.data[i]);
            JTpointtMSG.velocities.push_back(qdot[i]);
            JTpointtMSG.time_from_start = ros::Duration(time_from_start*time_from_start_factor); // Time overlapping is important to let interpolation play
        }
        // create the trajectory message
        JTmsg.points.push_back(JTpointtMSG);
        JTmsg.header.stamp = ros::Time::now();
    }

    // Fill in the marker message for RVIZ trajectory visualization
    void makeMarkerMSG(visualization_msgs::Marker &markerMSG,
                       const std::string& referenceFrame,
                       geometry_msgs::Pose markerPose,
                       double markerID,
                       bool jointLimitAvoindanceActive
                       )
    {
        // Set the frame ID and timestamp.
        markerMSG.header.frame_id = referenceFrame;
        markerMSG.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.
        markerMSG.ns = "basic_shapes";
        markerMSG.id = markerID;

        // Set the marker type.
        markerMSG.type = visualization_msgs::Marker::CUBE;
        markerMSG.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        markerMSG.pose = markerPose;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        markerMSG.scale.x = 0.01;
        markerMSG.scale.y = 0.01;
        markerMSG.scale.z = 0.01;

        // Set the color (rgb) and alpha (a) by checking if the secondary task was active or not
        if( !jointLimitAvoindanceActive ) // no velocities generated by the limit avoidance secondary task
        {
            // green color markers
            markerMSG.color.r = 0.0f;
            markerMSG.color.g = 1.0f;
            markerMSG.color.b = 0.0f;
            markerMSG.color.a = 1.0;
        }
        else // secondary task is active
        {
            // red color markers
            markerMSG.color.r = 1.0f;
            markerMSG.color.g = 0.0f;
            markerMSG.color.b = 0.0f;
            markerMSG.color.a = 1.0;
        }

        // Set the duration the marker remains visible
        markerMSG.lifetime = ros::Duration(300.0);
    }

    // Fill in joint state message --> use for simulation without controllers
    void makeJointStateMSG(sensor_msgs::JointState &JSmsg,
                           KDL::JntArray &kdlArmJointsPosition)
    {
        JSmsg.name.push_back("torso_1_joint");
        JSmsg.name.push_back("torso_2_joint");
        JSmsg.name.push_back("head_1_joint");
        JSmsg.name.push_back("head_2_joint");
        JSmsg.name.push_back("arm_right_1_joint");
        JSmsg.name.push_back("arm_right_2_joint");
        JSmsg.name.push_back("arm_right_3_joint");
        JSmsg.name.push_back("arm_right_4_joint");
        JSmsg.name.push_back("arm_right_5_joint");
        JSmsg.name.push_back("arm_right_6_joint");
        JSmsg.name.push_back("arm_right_7_joint");
        JSmsg.name.push_back("arm_left_1_joint");
        JSmsg.name.push_back("arm_left_2_joint");
        JSmsg.name.push_back("arm_left_3_joint");
        JSmsg.name.push_back("arm_left_4_joint");
        JSmsg.name.push_back("arm_left_5_joint");
        JSmsg.name.push_back("arm_left_6_joint");
        JSmsg.name.push_back("arm_left_7_joint");

        for(unsigned int i = 0; i < JSmsg.name.size(); i++)
        {
            if( (i>3) & (i<11) )
            {
                JSmsg.position.push_back(kdlArmJointsPosition.data[i-4]);
    //            JSmsg.velocity.push_back(kdlArmJointsPosition.data[i-4]);
            }
            else
            {
                JSmsg.position.push_back(0.0);
    //            JSmsg.velocity.push_back(0.0);
            }
        }
        JSmsg.header.stamp = ros::Time::now();
    }

    // Get Joint Limits from an URDF model
    void getJointLimitsFromModel(const urdf::Model& model,
                                 std::vector<double> &q_min,
                                 std::vector<double> &q_max,                                 
                                 KDL::Chain &kdlArmChain
                                ) 
    {
        boost::shared_ptr<const urdf::Joint> joint;

        for(unsigned int i=0; i< kdlArmChain.getNrOfJoints(); ++i)
        {
            if (kdlArmChain.getSegment(i).getJoint().getType() != KDL::Joint::None)
            {
                joint = model.getJoint(kdlArmChain.getSegment(i).getJoint().getName());
                if ( joint->type != urdf::Joint::CONTINUOUS )
                {
                    q_min.at(i) = joint->limits->lower;
                    q_max.at(i) = joint->limits->upper;
                }
                else
                {
                    q_min.at(i) = -3.14/2.0;
                    q_max.at(i) = 3.14/2.0;
                }
            }
        }
    }

    // Get Joint Limits from URDF file
    void getJointLimitsFromFile(std::vector<double> &q_min,
                                std::vector<double> &q_max,
                                const std::string& URDFfilename,
                                KDL::Chain &kdlArmChain
                                )
    {
        urdf::Model model;
        if (!model.initFile(URDFfilename))
        { 
          ROS_ERROR("Failed to parse urdf file"); 
          return;
        }
        boost::shared_ptr<const urdf::Joint> joint;

        getJointLimitsFromModel(model, q_min, q_max, kdlArmChain);    
    }

    // Get Joint Limits from an URDF string
    void getJointLimitsFromString(std::vector<double> &q_min,
                                  std::vector<double> &q_max,
                                  std::string URDFstring,
                                  KDL::Chain &kdlArmChain
                                  )
    {
      urdf::Model model;
      if (!model.initString(URDFstring))
      {
        ROS_ERROR("Failed to parse urdf string");
        return;
      }
      getJointLimitsFromModel(model, q_min, q_max, kdlArmChain);
    }

    // Clip velocities over the limit and print ROS warnings
    void limitVelocity(vpColVector &qdot,
                       double velLimit)
    {
      std::vector<double> qdotVec = pal::vispToStdVector(qdot);

      limitVelocity(qdotVec, velLimit);

      qdot = pal::stdVectorToVisp(qdotVec);
    }

    void limitVelocity(std::vector<double>& qdot, double velLimit)
    {
      double rescaleLambda = 1.0;
      double lambdaSingle = 1.0;
      for(unsigned int i=0; i< qdot.size(); ++i)
      {
        if(qdot[i] > velLimit)
        {
          lambdaSingle = qdot[i] / velLimit;
          ROS_WARN("joint %i velocity is over velocity limit, need to rescale from %f to %f", i, qdot[i], velLimit);
        }
        else if(qdot[i] < (-velLimit))
        {
          lambdaSingle = qdot[i] / (-velLimit);
          ROS_WARN("joint %i velocity is over velocity limit, need to rescale from %f to %f", i, qdot[i], (-velLimit));
        }
        // use only the largest rescaling factor
        if (lambdaSingle > rescaleLambda)
        {
          rescaleLambda = lambdaSingle;
        }
      }

      // rescale all qdot by the largest rescale factor
      for(unsigned int i=0; i< qdot.size(); ++i)
      {
          qdot[i] = qdot[i] / rescaleLambda;
      }
    }

    // Fill in the plot message for publishing data to be plotted in the /VS_errors topic
    void makePlotMSG(std_msgs::Float64MultiArray &plotMsg,
                     vpColVector &taskError,
                     vpColVector &qdot,
                     vpColVector &q1dot,
                     vpColVector &q2dot,
                     vpMatrix &fJe,
                     vpVelocityTwistMatrix &eVf,
                     KDL::JntArray &kdlArmPosition
                     )
    {
        // add qdot - total joint velocity
        for(unsigned int i=0; i<qdot.getRows(); i++)
        {
            plotMsg.data.push_back(qdot[i]);
        }

        // add q1dot - main task
        for(unsigned int i=0; i<q1dot.getRows(); i++)
        {
            plotMsg.data.push_back(q1dot[i]);
        }

        // add q2dot - main task
        for(unsigned int i=0; i<q2dot.getRows(); i++)
        {
            plotMsg.data.push_back(q2dot[i]);
        }

        // add the end-effector velocity
        vpColVector vel;
        vel = eVf*fJe*qdot;
        for(unsigned int i=0; i<vel.getRows(); i++)
        {
            plotMsg.data.push_back(vel[i]);
        }

        // add the joint positions
        for(unsigned int i=0; i<kdlArmPosition.rows(); i++)
        {
            plotMsg.data.push_back(kdlArmPosition.data[i]);
        }

        // add the error norm
        plotMsg.data.push_back(taskError.sumSquare());

        // add the errors
        for(unsigned int i=0; i<taskError.getRows(); i++)
        {
          ROS_INFO_STREAM("=> makePlotMSG: adding taskError at position " << plotMsg.data.size());
          plotMsg.data.push_back(taskError[i]);
        }
    }

    // Fill in the plot message for publishing data corresponding to 2 tasks
    void makePlotMSG(std_msgs::Float64MultiArray &plotMsg,
                     const vpColVector &taskError_1, //error of task 1
                     const vpColVector &qdot_1,      //qdot computed to accomplish task 1
                     const vpColVector &taskError_2, //error of task 2
                     const vpColVector &qdot_2,      //qdot computed to accomplish task 2
                     const vpColVector &q1dot_1,     //q1dot of task1
                     const vpColVector &q2dot_1,     //q2dot of task1
                     const vpColVector &q1dot_2,     //q1dot of task2
                     const vpColVector &q2dot_2,     //q2dot of task2
                     double x_midPoint,        //normalized coordinate x of midpoint(hand marker, object marker)
                     double y_midPoint)        //normalized coordinate y of midpoint(hand marker, object marker)
    {
      /////////////////////////////////////////////////////////////////
      // Task 1
      /////////////////////////////////////////////////////////////////
      // add the errors
      for(unsigned int i=0; i<taskError_1.getRows(); i++)
      {
        //ROS_INFO_STREAM("=> Adding taskError_1 at position " << plotMsg.data.size());
        plotMsg.data.push_back(taskError_1[i]);
      }

      // add qdot - total joint velocity
      for(unsigned int i=0; i<qdot_1.getRows(); i++)
      {
        //ROS_INFO_STREAM("=> Adding qdot of task 1 at position " << plotMsg.data.size());
        plotMsg.data.push_back(qdot_1[i]);
      }
      /////////////////////////////////////////////////////////////////
      // Task 2
      /////////////////////////////////////////////////////////////////
      // add the errors
      for(unsigned int i=0; i<taskError_2.getRows(); i++)
      {
        //ROS_INFO_STREAM("=> Adding taskError_2 at position " << plotMsg.data.size());
        plotMsg.data.push_back(taskError_2[i]);
      }

      // add qdot - total joint velocity
      for(unsigned int i=0; i<qdot_2.getRows(); i++)
      {
        //ROS_INFO_STREAM("=> Adding qdot of task 2 at position " << plotMsg.data.size());
        plotMsg.data.push_back(qdot_2[i]);
      }
      /////////////////////////////////////////////////////////////////
      // Task 1: q1dot and q2dot
      /////////////////////////////////////////////////////////////////
      // add qdot - joint velocities of primary task of 1
      for(unsigned int i=0; i<q1dot_1.getRows(); i++)
      {
        //ROS_INFO_STREAM("=> Adding q1dot of task 1 at position " << plotMsg.data.size());
        plotMsg.data.push_back(q1dot_1[i]);
      }
      // add qdot - joint velocities of secondary task of 1
      for(unsigned int i=0; i<q2dot_1.getRows(); i++)
      {
        //ROS_INFO_STREAM("=> Adding q2dot of task 1 at position " << plotMsg.data.size());
        plotMsg.data.push_back(q2dot_1[i]);
      }
      /////////////////////////////////////////////////////////////////
      // Task 2: q1dot and q2dot
      /////////////////////////////////////////////////////////////////
      // add qdot - joint velocities of primary task of 2
      for(unsigned int i=0; i<q1dot_2.getRows(); i++)
      {
        //ROS_INFO_STREAM("=> Adding q1dot of task 2 at position " << plotMsg.data.size());
        plotMsg.data.push_back(q1dot_2[i]);
      }
      // add qdot - joint velocities of secondary task of 2
      for(unsigned int i=0; i<q2dot_2.getRows(); i++)
      {
        //ROS_INFO_STREAM("=> Adding q2dot of task 2 at position " << plotMsg.data.size());
        plotMsg.data.push_back(q2dot_2[i]);
      }

      //add midpoint normalized coordinates
      plotMsg.data.push_back(x_midPoint);
      plotMsg.data.push_back(y_midPoint);
    }


    // Fill in the plot message for publishing data to be plotted in the /VS_errors topic
    void makePlotMSG(std_msgs::Float64MultiArray &plotMsg,
                     const vpColVector &taskError,
                     const vpColVector &v)
    {
      // add the errors
      for(unsigned int i=0; i<taskError.getRows(); i++)
      {
          plotMsg.data.push_back(taskError[i]);
      }

      // add velocity twist
      for(unsigned int i=0; i<v.getRows(); i++)
      {
          plotMsg.data.push_back(v[i]);
      }
    }


} // end namespace utility_functions
