/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Marcus Ebner */

#include "iimoveit/robot_interface.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace iimoveit {

  RobotInterface::RobotInterface(ros::NodeHandle* node_handle, const std::string& planning_group, const std::vector<double>& base_pose)
      : PLANNING_GROUP_(planning_group),
        node_handle_(node_handle),
        move_group_(planning_group),
        visual_tools_("iiwa_link_0"),
        robot_state_(*move_group_.getCurrentState()),
        base_pose_jointspace_(base_pose),
        mfButtonState_(false) {
    trajectory_publisher_ = node_handle_->advertise<trajectory_msgs::JointTrajectory>("PositionJointInterface_trajectory_controller/command", 1);
    cartPoseLin_publisher_ = node_handle_->advertise<geometry_msgs::PoseStamped>("command/CartesianPoseLin", 1);
    button_subscriber_ = node_handle_->subscribe<std_msgs::String>("state/buttonEvent", 10, &RobotInterface::buttonEventCallback, this);
    mfButton_subscriber_ = node_handle_->subscribe<std_msgs::Bool>("state/MFButtonState", 1, &RobotInterface::mfButtonStateCallback, this);
    joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
    joint_names_ = move_group_.getJointNames();
    visual_tools_.loadRemoteControl();
    text_pose_ = Eigen::Affine3d::Identity();
    text_pose_.translation().z() = 1.70; // above head of iiwa
    base_pose_ = poseFromJointAngles(base_pose_jointspace_);

    ROS_INFO_NAMED("iimoveit", "Reference frame: %s", move_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("iimoveit", "End effector link: %s", move_group_.getEndEffectorLink().c_str());
  }

  RobotInterface::RobotInterface(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
      : RobotInterface(node_handle, planning_group, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}


  void RobotInterface::planAndMove(const geometry_msgs::Pose& target_pose, const std::string& pose_name, bool approvalRequired) {
    move_group_.setPoseTarget(target_pose);
    moveToCurrentTarget(pose_name, approvalRequired);
  }

  void RobotInterface::planAndMove(const geometry_msgs::Pose& target_pose, bool approvalRequired, const std::string& pose_name) {
    move_group_.setPoseTarget(target_pose);
    moveToCurrentTarget(pose_name, approvalRequired);
  }


  void RobotInterface::planAndMove(const geometry_msgs::PoseStamped& target_pose, const std::string& pose_name, bool approvalRequired) {
    move_group_.setPoseTarget(target_pose);
    moveToCurrentTarget(pose_name, approvalRequired);
  }

  void RobotInterface::planAndMove(const geometry_msgs::PoseStamped& target_pose, bool approvalRequired, const std::string& pose_name) {
    move_group_.setPoseTarget(target_pose);
    moveToCurrentTarget(pose_name, approvalRequired);
  }


  void RobotInterface::planAndMove(const std::vector<double>& joint_group_positions, const std::string& pose_name, bool approvalRequired) {
    move_group_.setJointValueTarget(joint_group_positions);
    moveToCurrentTarget(pose_name, approvalRequired);
  }

  void RobotInterface::planAndMove(const std::vector<double>& joint_group_positions, bool approvalRequired, const std::string& pose_name) {
    move_group_.setJointValueTarget(joint_group_positions);
    moveToCurrentTarget(pose_name, approvalRequired);
  }


  void RobotInterface::planAndMoveInGivenCoordSys(const geometry_msgs::PoseStamped& first_pose, double px, double py, double pz, double rx, double ry, double rz, double rw, bool approvalRequired) {
    tf::Quaternion base_quaternion(first_pose.pose.orientation.x, first_pose.pose.orientation.y, first_pose.pose.orientation.z, first_pose.pose.orientation.w);
    tf::Quaternion next_quaternion(rx, ry, rz, rw);
    tf::Quaternion result_quaternion = next_quaternion * base_quaternion;
    result_quaternion.normalize();

    //TODO Check transformation!!!
    tf::Vector3 positionInFirstFrame = tf::Vector3(px, py, pz).rotate(base_quaternion.getAxis(), base_quaternion.getAngle());

    geometry_msgs::PoseStamped target_pose = first_pose;
    target_pose.pose.position.x += positionInFirstFrame.getX();
    target_pose.pose.position.y += positionInFirstFrame.getY();
    target_pose.pose.position.z += positionInFirstFrame.getZ();
    target_pose.pose.orientation.x = result_quaternion.getX();
    target_pose.pose.orientation.y = result_quaternion.getY();
    target_pose.pose.orientation.z = result_quaternion.getZ();
    target_pose.pose.orientation.w = result_quaternion.getW();

    planAndMove(target_pose, std::string("relative pose"), approvalRequired);
  }


  void RobotInterface::planAndMoveInToolCoordSys(const geometry_msgs::Pose& relativePose, bool approvalRequired) {
    planAndMoveInGivenCoordSys(getPose(), relativePose.position.x, relativePose.position.y, relativePose.position.z, relativePose.orientation.x, relativePose.orientation.y, relativePose.orientation.z, relativePose.orientation.w, approvalRequired);
  }

  void RobotInterface::planAndMoveInToolCoordSys(double x, double y, double z, double roll, double pitch, double yaw, bool approvalRequired) {
    tf::Quaternion next_quaternion;
    next_quaternion.setEuler(yaw, pitch, roll);
    planAndMoveInGivenCoordSys(getPose(), x, y, z, next_quaternion.getX(), next_quaternion.getY(), next_quaternion.getZ(), next_quaternion.getW(), approvalRequired);
  }

  void RobotInterface::planAndMoveInToolCoordSys(double x, double y, double z, bool approvalRequired) {
    planAndMoveInGivenCoordSys(getPose(), x, y, z, 0.0, 0.0, 0.0, 1.0, approvalRequired);
  }


  void RobotInterface::planAndMoveInBasePoseCoordSys(const geometry_msgs::Pose& relativePose, bool approvalRequired) {
    planAndMoveInGivenCoordSys(base_pose_, relativePose.position.x, relativePose.position.y, relativePose.position.z, relativePose.orientation.x, relativePose.orientation.y, relativePose.orientation.z, relativePose.orientation.w, approvalRequired);
  }

  void RobotInterface::planAndMoveInBasePoseCoordSys(double x, double y, double z, double roll, double pitch, double yaw, bool approvalRequired) {
    tf::Quaternion next_quaternion;
    next_quaternion.setEuler(yaw, pitch, roll);
    planAndMoveInGivenCoordSys(base_pose_, x, y, z, next_quaternion.getX(), next_quaternion.getY(), next_quaternion.getZ(), next_quaternion.getW(), approvalRequired);
  }

  void RobotInterface::planAndMoveInBasePoseCoordSys(double x, double y, double z, bool approvalRequired) {
    planAndMoveInGivenCoordSys(base_pose_, x, y, z, 0.0, 0.0, 0.0, 1.0, approvalRequired);
  }


  void RobotInterface::planAndMoveRelativeToGivenPose(const geometry_msgs::PoseStamped& first_pose, double px, double py, double pz, double rx, double ry, double rz, double rw, bool approvalRequired) {
    tf::Quaternion base_quaternion(first_pose.pose.orientation.x, first_pose.pose.orientation.y, first_pose.pose.orientation.z, first_pose.pose.orientation.w);
    tf::Quaternion next_quaternion(rx, ry, rz, rw);
    tf::Quaternion result_quaternion = next_quaternion * base_quaternion;
    result_quaternion.normalize();

    geometry_msgs::PoseStamped target_pose = first_pose;
    target_pose.pose.position.x += px;
    target_pose.pose.position.y += py;
    target_pose.pose.position.z += pz;
    target_pose.pose.orientation.x = result_quaternion.getX();
    target_pose.pose.orientation.y = result_quaternion.getY();
    target_pose.pose.orientation.z = result_quaternion.getZ();
    target_pose.pose.orientation.w = result_quaternion.getW();

    planAndMove(target_pose, std::string("relative pose"), approvalRequired);
  }

  void RobotInterface::planAndMoveRelativeToCurrentPose(const geometry_msgs::Pose& relativePose, bool approvalRequired) {
    planAndMoveRelativeToGivenPose(getPose(), relativePose.position.x, relativePose.position.y, relativePose.position.z, relativePose.orientation.x, relativePose.orientation.y, relativePose.orientation.z, relativePose.orientation.w, approvalRequired);
  }

  void RobotInterface::planAndMoveRelativeToCurrentPose(double x, double y, double z, double roll, double pitch, double yaw, bool approvalRequired) {
    tf::Quaternion next_quaternion;
    next_quaternion.setEuler(yaw, pitch, roll);
    planAndMoveRelativeToGivenPose(getPose(), x, y, z, next_quaternion.getX(), next_quaternion.getY(), next_quaternion.getZ(), next_quaternion.getW(), approvalRequired);
  }

  void RobotInterface::planAndMoveRelativeToCurrentPose(double x, double y, double z, bool approvalRequired) {
    planAndMoveRelativeToGivenPose(getPose(), x, y, z, 0.0, 0.0, 0.0, 1.0, approvalRequired);
  }


  void RobotInterface::planAndMoveRelativeToBasePose(const geometry_msgs::Pose& relativePose, bool approvalRequired) {
    planAndMoveRelativeToGivenPose(base_pose_, relativePose.position.x, relativePose.position.y, relativePose.position.z, relativePose.orientation.x, relativePose.orientation.y, relativePose.orientation.z, relativePose.orientation.w, approvalRequired);
  }

  void RobotInterface::planAndMoveRelativeToBasePose(double x, double y, double z, double roll, double pitch, double yaw, bool approvalRequired) {
    tf::Quaternion next_quaternion;
    next_quaternion.setEuler(yaw, pitch, roll);
    planAndMoveRelativeToGivenPose(base_pose_, x, y, z, next_quaternion.getX(), next_quaternion.getY(), next_quaternion.getZ(), next_quaternion.getW(), approvalRequired);
  }

  void RobotInterface::planAndMoveRelativeToBasePose(double x, double y, double z, bool approvalRequired) {
    planAndMoveRelativeToGivenPose(base_pose_, x, y, z, 0.0, 0.0, 0.0, 1.0, approvalRequired);
  }


  void RobotInterface::planAndMoveToBasePose(bool approvalRequired) {
    planAndMove(base_pose_jointspace_, std::string("base_pose_jointspace"), approvalRequired);
  }


  bool RobotInterface::moveAlongCartesianPathInWorldCoords(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold, bool avoid_collisions, bool approvalRequired) {
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, 0.0, trajectory, avoid_collisions);
    ROS_INFO_NAMED("iimoveit", "%.2f%% of the path planned into trajectory.", fraction * 100.0);
    if (approvalRequired) waitForApproval();
    return runTrajectoryAction(trajectory.joint_trajectory);
  }


  void RobotInterface::waitForApproval() {
    visual_tools_.prompt("Continue with moving?");
  }

  void RobotInterface::publishTrajectory(const trajectory_msgs::JointTrajectory& trajectory) {
    trajectory_publisher_.publish(trajectory);
  }

  bool RobotInterface::runTrajectoryAction(const trajectory_msgs::JointTrajectory& trajectory) {
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client("PositionJointInterface_trajectory_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server to start.");
    action_client.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    //goal.joint_names = joint_names_;
    action_client.sendGoal(goal);

    bool finished_before_timeout = action_client.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = action_client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else ROS_INFO("Action did not finish before the time out.");
    return finished_before_timeout;
  }

  void RobotInterface::publishPoseGoal(const geometry_msgs::Pose& target_pose, double duration) {
    robot_state_.setFromIK(joint_model_group_, target_pose);
    trajectory_msgs::JointTrajectoryPoint  trajectory_point;
    robot_state_.copyJointGroupPositions(joint_model_group_, trajectory_point.positions);
    trajectory_point.time_from_start = ros::Duration(duration);

    trajectory_msgs::JointTrajectory single_point_trajectory;
    single_point_trajectory.joint_names = joint_names_;
    single_point_trajectory.points.push_back(trajectory_point);

    trajectory_publisher_.publish(single_point_trajectory);
  }

  void RobotInterface::publishPoseGoal(const geometry_msgs::PoseStamped& target_pose, double duration) {
    RobotInterface::publishPoseGoal(target_pose.pose, duration);
  }

  void RobotInterface::publishPoseGoalLinear(geometry_msgs::PoseStamped target_pose) {
    // If we are using the simulated robot, we'll just let him move to the goal via planAndMove
    bool sim;
    node_handle_->param("sim", sim, false);
    std::cout << "Sim: " << sim << std::endl;
    if (sim) planAndMove(target_pose, false);
    else {
      tf_listener_.transformPose("sunrise_world", target_pose, target_pose);
      std::cout << target_pose << std::endl << std::endl;
      waitForApproval();
      cartPoseLin_publisher_.publish(target_pose);
    }
  }


  void RobotInterface::buttonEventCallback(const std_msgs::String::ConstPtr& msg) {
   ROS_INFO("SmartPad button pressed: %s", msg->data.c_str());
  }


  std::vector<double> RobotInterface::getJointPositions() {
    return move_group_.getCurrentJointValues();
  }

  geometry_msgs::PoseStamped RobotInterface::getPose(const std::string& end_effector_link) {
    return move_group_.getCurrentPose(end_effector_link);
  }

  geometry_msgs::PoseStamped RobotInterface::poseFromJointAngles(const std::vector<double>& joint_group_positions) {
    robot_state_.setJointGroupPositions(joint_model_group_, joint_group_positions);
    const Eigen::Affine3d& end_effector_state = robot_state_.getGlobalLinkTransform(move_group_.getEndEffectorLink());
    Eigen::Vector3d t(end_effector_state.translation());
    Eigen::Quaterniond q(end_effector_state.rotation());
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = t[0];
    pose.pose.position.y = t[1];
    pose.pose.position.z = t[2];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    // Reset robot state to current state
    updateRobotState();
    return pose;
  }


  void RobotInterface::setBasePose(const std::vector<double>& new_base_pose) {
    base_pose_jointspace_ = new_base_pose;
    base_pose_ = poseFromJointAngles(new_base_pose);
  }

  void RobotInterface::updateRobotState() {
    robot_state_ = robot_state::RobotState(*move_group_.getCurrentState());
  }

  void RobotInterface::moveToCurrentTarget(const std::string& pose_name, bool approvalRequired) {
    bool success = (bool)move_group_.plan(movement_plan_);
    ROS_INFO_NAMED("iimoveit", "Visualizing plan to %s %s", pose_name.c_str(), success ? "" : "FAILED");
    ROS_INFO_NAMED("iimoveit", "Visualizing plan as trajectory line");
    visual_tools_.deleteAllMarkers();
    visual_tools_.publishText(text_pose_, "Planning movement to given pose", rvt::WHITE, rvt::XLARGE);
    visual_tools_.publishTrajectoryLine(movement_plan_.trajectory_, joint_model_group_);
    visual_tools_.trigger();
    if (success) {
      if (approvalRequired) waitForApproval();
      visual_tools_.publishText(text_pose_, "Moving to pose", rvt::WHITE, rvt::XLARGE);
      visual_tools_.trigger();
      move_group_.execute(movement_plan_);
      updateRobotState();
    }
  }

} // namespace iimoveit
