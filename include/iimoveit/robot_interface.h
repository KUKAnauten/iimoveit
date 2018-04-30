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


//TODO publish trajectory, publish pose (cart/joint), start action, publish text, moveRelativeToCurrent

#ifndef IIMOVEIT_ROBOT_INTERFACE_H_
#define IIMOVEIT_ROBOT_INTERFACE_H_

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/tf.h>

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
   * @param base_pose Base pose in joint space coordinates (for tasks where robot often returns to one pose).
   */
  RobotInterface(ros::NodeHandle* node_handle, const std::string& planning_group, const std::vector<double>& base_pose);

  /**
   * Constructor that assumes all joint angles as 0.0 for the base pose.
   * @param node_handle The NodeHandle used to publish and subscribe to topics.
   * @param planning_group The planning group/joint model group to plan for.
   * @param !DEPRECATED! We suggest not to use this parameter at all as it might be removed in future versions. It does not have
   *        any effect anymore.
   */
  RobotInterface(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame = std::string("iiwa_link_0"));

  /**
   * Plans a motion to a given endeffector pose. It will show the planned trajectory in RViz and, if approvalRequired
   * is set to true, will ask for confirmation before moving to the goal.
   * @param target_pose The pose goal.
   * @param pose_name The name of the pose that will be shown in RViz.
   * @param approvalRequired If set to true, it will only move after confirmation in RViz.
   */
  virtual void planAndMove(const geometry_msgs::Pose& target_pose, bool approvalRequired = true, const std::string& pose_name = std::string("given pose"));

  /**
   * Deprecated. Use the one which has approvalRequired before pose_name instead.
   */
  virtual void planAndMove(const geometry_msgs::Pose& target_pose, const std::string& pose_name = std::string("given pose"), bool approvalRequired = true);


  /**
   * Plans a motion to a given endeffector pose. It will show the planned trajectory in RViz and, if approvalRequired
   * is set to true, will ask for confirmation before moving to the goal.
   * @param target_pose The pose goal.
   * @param pose_name The name of the pose that will be shown in RViz.
   * @param approvalRequired If set to true, it will only move after confirmation in RViz.
   */
  virtual void planAndMove(const geometry_msgs::PoseStamped& target_pose, bool approvalRequired = true, const std::string& pose_name = std::string("given pose"));

  /**
   * Deprecated. Use the one which has approvalRequired before pose_name instead.
   */
  virtual void planAndMove(const geometry_msgs::PoseStamped& target_pose, const std::string& pose_name = std::string("given pose"), bool approvalRequired = true);


  /**
   * Plans a motion to a given joint space goal. It will show the planned trajectory in RViz and, if approvalRequired
   * is set to true, will ask for confirmation before moving to the goal.
   * @param joint_group_positions The joint goal.
   * @param pose_name The name of the pose that will be shown in RViz.
   * @param approvalRequired If set to true, it will only move after confirmation in RViz.
   */
  virtual void planAndMove(const std::vector<double>& joint_group_positions, bool approvalRequired = true, const std::string& pose_name = std::string("given pose"));

  /**
   * Deprecated. Use the one which has approvalRequired before pose_name instead.
   */
  virtual void planAndMove(const std::vector<double>& joint_group_positions, const std::string& pose_name = std::string("given pose"), bool approvalRequired = true);

  /**
   * !UNTESTED! Plans a motion to a pose given in another coordinate frame. The coordinates of the second pose will be interpreted with the first pose as origin.
   * This method is called by the other ones that plan in a different coordinate system like the current tool pose or the base pose.
   * @param first_pose The first pose. The pose given with the other parameters will be concatenated onto this one.
   * @param px The translation in x-direction of the second pose.
   * @param py The translation in y-direction of the second pose.
   * @param pz The translation in z-direction of the second pose.
   * @param rx The x-component of the quaternion of the second pose.
   * @param ry The y-component of the quaternion of the second pose.
   * @param rz The z-component of the quaternion of the second pose.
   * @param rw The w-component of the quaternion of the second pose.
   * @param approvalRequired If set to true, it will only move after confirmation in RViz.
   */
  virtual void planAndMoveInGivenCoordSys(const geometry_msgs::PoseStamped& first_pose, double px, double py, double pz, double rx, double ry, double rz, double rw, bool approvalRequired = true);
  virtual void planAndMoveInToolCoordSys(const geometry_msgs::Pose& relativePose, bool approvalRequired = true);
  virtual void planAndMoveInToolCoordSys(double x, double y, double z, double roll, double pitch, double yaw, bool approvalRequired = true);
  virtual void planAndMoveInToolCoordSys(double x, double y, double z, bool approvalRequired = true);
  virtual void planAndMoveInBasePoseCoordSys(const geometry_msgs::Pose& relativePose, bool approvalRequired = true);
  virtual void planAndMoveInBasePoseCoordSys(double x, double y, double z, double roll, double pitch, double yaw, bool approvalRequired = true);
  virtual void planAndMoveInBasePoseCoordSys(double x, double y, double z, bool approvalRequired = true);

  /**
   * Plans a motion relative to a given pose and moves to it. The coordinates of the second pose will still be interpreted in the world frame. Only the
   * orientation of both will be concatenated. This method is called by the ones that plan relative to the current pose or base pose.
   * @param first_pose The first pose.
   * @param px The translation in x-direction of the second pose.
   * @param py The translation in y-direction of the second pose.
   * @param pz The translation in z-direction of the second pose.
   * @param rx The x-component of the quaternion of the second pose.
   * @param ry The y-component of the quaternion of the second pose.
   * @param rz The z-component of the quaternion of the second pose.
   * @param rw The w-component of the quaternion of the second pose.
   * @param approvalRequired If set to true, it will only move after confirmation in RViz.
   */
  virtual void planAndMoveRelativeToGivenPose(const geometry_msgs::PoseStamped& first_pose, double px, double py, double pz, double rx, double ry, double rz, double rw, bool approvalRequired = true);
  virtual void planAndMoveRelativeToCurrentPose(const geometry_msgs::Pose& relativePose, bool approvalRequired = true);
  virtual void planAndMoveRelativeToCurrentPose(double x, double y, double z, double roll, double pitch, double yaw, bool approvalRequired = true);
  virtual void planAndMoveRelativeToCurrentPose(double x, double y, double z, bool approvalRequired = true);
  virtual void planAndMoveRelativeToBasePose(const geometry_msgs::Pose& relativePose, bool approvalRequired = true);
  virtual void planAndMoveRelativeToBasePose(double x, double y, double z, double roll, double pitch, double yaw, bool approvalRequired = true);
  virtual void planAndMoveRelativeToBasePose(double x, double y, double z, bool approvalRequired = true);

  virtual void planAndMoveToBasePose(bool approvalRequired = true);

  virtual bool moveAlongCartesianPathInWorldCoords(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold, bool avoid_collisions = true, bool approvalRequired = true);

  /**
   * Waits for the user to click 'Next' in RViz.
   */
  virtual void waitForApproval();


  /**
   * Publishes a joint trajectory to the command topic of the controller. This has to be used with care, as this method does
   * not check whether the robot is close to the starting point of the trajectory.
   * @param trajectory The trajectory to publish.
   */
  virtual void publishTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

  /**
   * Runs a joint trajectory action and waits for the robot to finish. This has to be used with care, as this method does
   * not check whether the robot is close to the starting point of the trajectory.
   * @param trajectory The trajectory.
   */
  virtual bool runTrajectoryAction(const trajectory_msgs::JointTrajectory& trajectory);

  /**
   * Publishes a Pose to the command topic of the controller. This has to be used with care, as this method does
   * not check whether the robot is close to the starting point of the trajectory. As the command topic expects
   * trajectory messages, the pose will be put in a trajectory containing this single pose.
   * @param target_pose The pose to publish.
   * @param duration The duration value of the trajectory message.
   */
  virtual void publishPoseGoal(const geometry_msgs::Pose& target_pose, double duration);

  /**
   * @see publishPoseGoal
   */
  virtual void publishPoseGoal(const geometry_msgs::PoseStamped& target_pose, double duration);

  /**
   * Publishes a cartesian pose goal to the CartesianPoseLin topic. This does not work within the simulation as it uses
   * KUKAs linear motion functionality. This also means that there will be no obstacle avoidance as it does not plan its
   * path via MoveIt.
   * @param target_pose The pose goal.
   */
  virtual void publishPoseGoalLinear(geometry_msgs::PoseStamped target_pose);


  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) { joint_state_ptr_ = msg; }
  /**
   * This callback function is called whenever one of the buttons on the SmartPad is pressed. To make it useful, a class should be
   * derived from RobotInterface and the callback function overwritten to something useful. For the buttons to be shown on the SmartPad,
   * they have to be configured via a launch file (like in iiwa_stack/iiwa_ros/launch/toolbar_example.launch) prior to the RosSmartServo
   * application being launched on the iiwa!
   * @param msg A string containing information about which button was pressed or released.
   */
  virtual void buttonEventCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Callback function to store current button state in member variable.
   * @param msg A Bool message containing the button state read from topic mftButtonState.
   */
  void mfButtonStateCallback(const std_msgs::Bool::ConstPtr& msg) { mfButtonState_ = msg->data; }

  /**
   * Returns true if the green button on the Media Flange is being pressed.
   * @return The Media Flange user button state.
   */
  bool getMFButtonState() { return mfButtonState_; }

  /**
   * Returns the current joint positions.
   * @return A vector with the joint positions.
   */
  std::vector<double> getJointPositions();

  /**
   * Returns the current joint velocities.
   * @return A vector with the joint velocities.
   */
  std::vector<double> getJointVelocities();

  /**
   * Returns the current cartesian Pose.
   * @param end_effector_link Name of the endeffector link, if left empty, end effector link of planning group will be used.
   * @return Current Pose as geometry_msgs::PoseStamped.
   */
  geometry_msgs::PoseStamped getPose(const std::string& end_effector_link = "");

  /**
   * Calculates the robot's endeffector pose from given joint angles, relative to robot base.
   * @param joint_group_positions Pose in joint space
   * @return Pose in cartesian space
   */
  geometry_msgs::PoseStamped poseFromJointAngles(const std::vector<double>& joint_group_positions);


  geometry_msgs::PoseStamped getBasePose() const {return base_pose_;}
  void setBasePose(const std::vector<double>& new_base_pose);

  void setPathConstraints(const moveit_msgs::Constraints& constraint) {move_group_.setPathConstraints(constraint);}
  void clearPathConstraints() {move_group_.clearPathConstraints();}


protected:
  ros::NodeHandle* node_handle_; /**< The NodeHandle used to publish or subscribe messages. */
  ros::Publisher trajectory_publisher_; /**< The publisher to publish trajectory messages to the command topic of the controller. */
  ros::Publisher cartPoseLin_publisher_; /**< The publisher to publish cartesian pose goals for linear motions. */
  ros::Subscriber joint_state_subscriber_; /**< Subscribes to the joint_states topic. */
  ros::Subscriber button_subscriber_; /**< Subscribes to the button topic of the robot. */
  ros::Subscriber mfButton_subscriber_; /**< Subscribes to the button topic (Media Flange) of the robot. */
  const std::string PLANNING_GROUP_; /**< The name of the planning group/joint model group to use. */
  moveit::planning_interface::MoveGroupInterface move_group_; /**< The MoveGroupInterface to use MoveIt!. */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_; /**< The PlanningSceneInterface to represent the planning scene. */
  const robot_state::JointModelGroup *joint_model_group_; /**< The actual JointModelGroup instance. */
  moveit::planning_interface::MoveGroupInterface::Plan movement_plan_; /**< This is where a MoveIt! plan will be stored. */
  moveit_visual_tools::MoveItVisualTools visual_tools_; /**< Tools used to visualize in RViz. */
  Eigen::Affine3d text_pose_; /**< The position of the status text, usually above the robot's head. */
  std::vector<std::string> joint_names_; /**< A vector of joint names. */
  robot_state::RobotState robot_state_; /**< This is where the current robot state is stored. */
  std::vector<double> base_pose_jointspace_; /**< Base pose in joint space coordinates. */
  geometry_msgs::PoseStamped base_pose_; /**< Base pose in world coordinates. */
  bool mfButtonState_; /**< State of Media Flange UserButton. */
  sensor_msgs::JointState::ConstPtr joint_state_ptr_; /**< Current joint states. */
  tf::TransformListener tf_listener_;

  /**
   * Gets the current robot state and stores it.
   */
  void updateRobotState();

  /**
   * Plans and moves to current target.
   */
  void moveToCurrentTarget(const std::string& pose_name, bool approvalRequired);
};

} // namespace iimoveit

#endif // IIMOVEIT_ROBOT_INTERFACE_H_
