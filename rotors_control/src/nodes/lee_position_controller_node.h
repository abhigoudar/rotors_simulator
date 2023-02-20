/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <mav_msgs/msg/attitude_thrust.hpp>
#include <mav_msgs/eigen_mav_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"

namespace rotors_control {

class LeePositionControllerNode : public rclcpp::Node {
 public:
  LeePositionControllerNode(const rclcpp::NodeOptions&);
  ~LeePositionControllerNode();

  void RunController(const rclcpp::Node::SharedPtr);
  void InitializeParams();
  void Publish();

 private:

  LeePositionController lee_position_controller_;
  rclcpp::Node::SharedPtr node;
  std::string namespace_;

  // subscribers
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr cmd_multi_dof_joint_trajectory_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_velocity_reference_pub_;

  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<rclcpp::Duration> command_waiting_times_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  void TimedCommandCallback();

  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr);

  void CommandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);

  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
