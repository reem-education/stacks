#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int main(int argc, char **argv)
{
  // Init the ROS node
  ros::init (argc, argv, "move_arm_joint_goal_test");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Action client for sending motion planing requests
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm_torso", true);

  // Wait for the action client to be connected to the server
  ROS_INFO("Connecting to server...");
  if (!move_arm.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR("Timed-out waiting for the move_arm action server.");
    return EXIT_FAILURE;
  }
  ROS_INFO("Connected to server.");

  // Prepare motion plan request with joint-space goal
  arm_navigation_msgs::MoveArmGoal goal;
  goal.motion_plan_request.group_name = "right_arm_torso";
  goal.motion_plan_request.num_planning_attempts = 3;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "base_link";
  desired_pose.link_name = "arm_right_7_link";
  desired_pose.pose.position.x = 0.30;
  desired_pose.pose.position.y = -0.3;
  desired_pose.pose.position.z = 1.13;

  desired_pose.pose.orientation.x = 0.5;
  desired_pose.pose.orientation.y = -0.5;
  desired_pose.pose.orientation.z = 0.5;
  desired_pose.pose.orientation.w = -0.5;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.05;
  desired_pose.absolute_pitch_tolerance = 0.05;
  desired_pose.absolute_yaw_tolerance = 0.05;

  // Send motion plan request
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goal);
  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goal);
    finished_within_time = move_arm.waitForResult(ros::Duration(15.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving task-space goal.");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}
