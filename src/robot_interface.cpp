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

namespace iimoveit {
  
  RobotInterface::RobotInterface(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
      : PLANNING_GROUP_(planning_group),
        node_handle_(node_handle),
        move_group_(planning_group),
        visual_tools_(base_frame),
        robot_state_(*move_group_.getCurrentState()) {
    trajectory_publisher_ = node_handle_->advertise<trajectory_msgs::JointTrajectory>("PositionJointInterface_trajectory_controller/command", 1);
    joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
    joint_names_ = move_group_.getJointNames();
    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();
    text_pose_ = Eigen::Affine3d::Identity();
    text_pose_.translation().z() = 1.50; // above head of iiwa

    visual_tools_.trigger();

    ROS_INFO_NAMED("iimoveit", "Reference frame: %s", move_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("iimoveit", "End effector link: %s", move_group_.getEndEffectorLink().c_str());
  }

  void RobotInterface::planAndMove(const geometry_msgs::Pose& target_pose, const std::string& pose_name, bool approvalRequired) {
    move_group_.setPoseTarget(target_pose);
    visual_tools_.publishAxisLabeled(target_pose, pose_name);
    moveToCurrentTarget(pose_name, approvalRequired);
  }

  void RobotInterface::planAndMove(const geometry_msgs::Pose& target_pose, const std::string& pose_name) {
    planAndMove(target_pose, pose_name, true);
  }

  void RobotInterface::planAndMove(const geometry_msgs::Pose& target_pose, bool approvalRequired) {
    planAndMove(target_pose, std::string("given pose"), approvalRequired);
  }

  void RobotInterface::planAndMove(const geometry_msgs::Pose& target_pose) {
    planAndMove(target_pose, std::string("given pose"), true);
  }

  void RobotInterface::planAndMove(const std::vector<double>& joint_group_positions, const std::string& pose_name, bool approvalRequired) {
    move_group_.setJointValueTarget(joint_group_positions);
    moveToCurrentTarget(pose_name, approvalRequired);
  }

  void RobotInterface::planAndMove(const std::vector<double>& joint_group_positions, const std::string& pose_name) {
    planAndMove(joint_group_positions, pose_name, true);
  }

  void RobotInterface::planAndMove(const std::vector<double>& joint_group_positions, bool approvalRequired) {
    planAndMove(joint_group_positions, std::string("given pose"), approvalRequired);
  }

  void RobotInterface::planAndMove(const std::vector<double>& joint_group_positions) {
    planAndMove(joint_group_positions, std::string("given pose"), true);
  }

  void RobotInterface::waitForApproval() {
    visual_tools_.prompt("Continue with moving?");
  }

  void RobotInterface::publishTrajectory(const trajectory_msgs::JointTrajectory& trajectory) {
    trajectory_publisher_.publish(trajectory);
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

  std::vector<double> RobotInterface::getJointPositions() {
    return move_group_.getCurrentJointValues();
  }

  geometry_msgs::PoseStamped RobotInterface::getPose(const std::string& end_effector_link) {
    return move_group_.getCurrentPose(end_effector_link);
  }

  void RobotInterface::updateRobotState() {
    robot_state_ = robot_state::RobotState(*move_group_.getCurrentState());
  }

  void RobotInterface::moveToCurrentTarget(const std::string& pose_name, bool approvalRequired) {
    bool success = (bool)move_group_.plan(movement_plan_);
    ROS_INFO_NAMED("iiwa_test", "Visualizing plan to %s %s", pose_name.c_str(), success ? "" : "FAILED");
    ROS_INFO_NAMED("iiwa_test", "Visualizing plan as trajectory line");
    visual_tools_.publishText(text_pose_, "Planning movement to given pose", rvt::WHITE, rvt::XLARGE);
    visual_tools_.publishTrajectoryLine(movement_plan_.trajectory_, joint_model_group_);
    visual_tools_.trigger();
    if (success) {
      if (approvalRequired) waitForApproval();
      visual_tools_.publishText(text_pose_, "Moving to pose", rvt::WHITE, rvt::XLARGE);
      visual_tools_.trigger();
      move_group_.move();
      updateRobotState();
    }
  }
  
} // namespace iimoveit