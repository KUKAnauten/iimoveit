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
 *   * Neither the name of SRI International nor the names of its
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


//TODO build library, test with sinfollowing
//TODO publish trajectory, publish pose (cart/joint), start action, publish text
//TODO documentation (doxygen + pdf)

#ifndef IIMOVEIT_ROBOT_INTERFACE_H_
#define IIMOVEIT_ROBOT_INTERFACE_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace iimoveit {

namespace rvt = rviz_visual_tools;

/**
 * Provides an easy to use interface to control the iiwa in combination with moveIt!.
 */
class RobotInterface {
  public:
  RobotInterface(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame);

  virtual void planAndMove(geometry_msgs::Pose& target_pose, const std::string& pose_name, bool approvalRequired);

  virtual void planAndMove(geometry_msgs::Pose& target_pose, const std::string& pose_name);

  virtual void planAndMove(geometry_msgs::Pose& target_pose, bool approvalRequired);

  virtual void planAndMove(geometry_msgs::Pose& target_pose);

  virtual void waitForApproval();

protected:
  ros::NodeHandle* node_handle_;
  ros::Publisher trajectory_publisher_;
  const std::string PLANNING_GROUP_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  const robot_state::JointModelGroup *joint_model_group_;
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  Eigen::Affine3d text_pose_;
  std::vector<std::string> joint_names_;
  robot_state::RobotState robot_state_;

  void updateRobotState();
};

} // namespace iimoveit

#endif // IIMOVEIT_ROBOT_INTERFACE_H_