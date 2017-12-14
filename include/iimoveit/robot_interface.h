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


//TODO publish trajectory, publish pose (cart/joint), start action, publish text

#ifndef IIMOVEIT_ROBOT_INTERFACE_H_
#define IIMOVEIT_ROBOT_INTERFACE_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace iimoveit {

namespace rvt = rviz_visual_tools;

/**
 * Provides an easy to use interface to control the iiwa in combination with MoveIt!.
 * All public methods are virtual, so that derived classes can overwrite them in case they need modified versions.
 */
class RobotInterface {
  public:
  /**
   * Constructs a new RobotInterface with the given parameters
   * @param node_handle The NodeHandle used to publish and subscribe to topics.
   * @param planning_group The planning group/joint model group to plan for.
   * @param base_frame Common base for all visual markers.
   */
  RobotInterface(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame);

  /**
   * Plans a motion to a given endeffector pose. It will show the planned trajectory in RViz and, if approvalRequired
   * is set to true, will ask for confirmation before moving to the goal.
   * @param target_pose The pose goal.
   * @param pose_name The name of the pose that will be shown in RViz.
   * @param approvalRequired If set to true, it will only move after confirmation in RViz.
   */
  virtual void planAndMove(geometry_msgs::Pose& target_pose, const std::string& pose_name, bool approvalRequired);

  /**
   * Overloaded planAndMove(), where approvalRequired is set to true.
   */
  virtual void planAndMove(geometry_msgs::Pose& target_pose, const std::string& pose_name);

  /**
   * Overloaded planAndMove(), where pose_name is set to 'given pose'.
   */
  virtual void planAndMove(geometry_msgs::Pose& target_pose, bool approvalRequired);

  /**
   * Overloaded planAndMove(), where approvalRequired is set to true and pose_name is set to 'given pose'.
   */
  virtual void planAndMove(geometry_msgs::Pose& target_pose);

  /**
   * Waits for the user to click 'Next' in RViz.
   */
  virtual void waitForApproval();

  /**
   * Publishes a joint trajectory to the command topic of the controller. This has to be used with care, as this method does
   * not check whether the robot is close to the starting point of the trajectory.
   * @param trajectory The trajectory to publish.
   */
  virtual void publishTrajectory(trajectory_msgs::JointTrajectory trajectory);

protected:
  ros::NodeHandle* node_handle_; /**< The NodeHandle used to publish or subscribe messages. */
  ros::Publisher trajectory_publisher_; /**< The publisher to publish trajectory messages to the command topic of the controller. */
  const std::string PLANNING_GROUP_; /**< The name of the planning group/joint model group to use. */
  moveit::planning_interface::MoveGroupInterface move_group_; /**< The MoveGroupInterface to use MoveIt!. */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_; /**< The PlanningSceneInterface to represent the planning scene. */
  const robot_state::JointModelGroup *joint_model_group_; /**< The actual JointModelGroup instance. */
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan_; /**< This is where a MoveIt! plan will be stored. */
  moveit_visual_tools::MoveItVisualTools visual_tools_; /**< Tools used to visualize in RViz. */
  Eigen::Affine3d text_pose_; /**< The position of the status text, usually above the robot's head. */
  std::vector<std::string> joint_names_; /**< A vector of joint names. */
  robot_state::RobotState robot_state_; /**< This is where the current robot state is stored. */

  /**
   * Gets the current robot state and stores it.
   */
  void updateRobotState();
};

} // namespace iimoveit

#endif // IIMOVEIT_ROBOT_INTERFACE_H_