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

#include <mav_msgs/default_topics.hpp>
#include "lee_position_controller_node.h"
#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerNode::LeePositionControllerNode(
  const rclcpp::NodeOptions& options)
  : Node("lee_position_controller_node", options)
  {
  }

LeePositionControllerNode::~LeePositionControllerNode() { }

void LeePositionControllerNode::RunController(const rclcpp::Node::SharedPtr node_)
{
  node = node_;
  InitializeParams();

  cmd_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      mav_msgs::default_topics::COMMAND_POSE, rclcpp::SystemDefaultsQoS(),
      std::bind(&LeePositionControllerNode::CommandPoseCallback, this,
      std::placeholders::_1));

  cmd_multi_dof_joint_trajectory_sub_ = this->create_subscription
    <trajectory_msgs::msg::MultiDOFJointTrajectory>(
    mav_msgs::default_topics::COMMAND_TRAJECTORY, rclcpp::SystemDefaultsQoS(),
    std::bind(&LeePositionControllerNode::MultiDofJointTrajectoryCallback, this,
    std::placeholders::_1));

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>
    (mav_msgs::default_topics::ODOMETRY, rclcpp::SystemDefaultsQoS(),
    std::bind(&LeePositionControllerNode::OdometryCallback, this,
    std::placeholders::_1));

  motor_velocity_reference_pub_ = this->create_publisher<mav_msgs::msg::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, rclcpp::SystemDefaultsQoS());

  command_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(0),
      std::bind(&LeePositionControllerNode::TimedCommandCallback, this));
  command_timer_->cancel();
}

void LeePositionControllerNode::InitializeParams() {
  // Read parameters from rosparam.
  GetRosParameter(node, "position_gain.x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(node, "position_gain.y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(node, "position_gain.z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(node, "velocity_gain.x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(node, "velocity_gain.y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(node, "velocity_gain.z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(node, "attitude_gain.x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(node, "attitude_gain.y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(node, "attitude_gain.z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(node, "angular_rate_gain.x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(node, "angular_rate_gain.y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(node, "angular_rate_gain.z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(node, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();
}

void LeePositionControllerNode::Publish() {
}

void LeePositionControllerNode::CommandPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
  // Clear all pending commands.
  command_timer_->cancel();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void LeePositionControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg) {
  // Clear all pending commands.
  command_timer_->cancel();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    RCLCPP_WARN(this->get_logger(),
      "Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    // command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
    command_waiting_times_.push_back(rclcpp::Duration(current_reference.time_from_start) - 
      rclcpp::Duration(reference_before.time_from_start));
  }

  // We can trigger the first command immediately.
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if(n_commands > 1) {
    command_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(command_waiting_times_.front().nanoseconds()),
      std::bind(&LeePositionControllerNode::TimedCommandCallback, this));
    command_waiting_times_.pop_front();
  }
}

void LeePositionControllerNode::TimedCommandCallback() {

  if(commands_.empty()){
    RCLCPP_WARN(this->get_logger(), "Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_->cancel();
  if(!command_waiting_times_.empty()){
  //   command_timer_.setPeriod(command_waiting_times_.front());
    command_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(command_waiting_times_.front().nanoseconds()),
      std::bind(&LeePositionControllerNode::TimedCommandCallback, this));
    command_waiting_times_.pop_front();
  }
}

void LeePositionControllerNode::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {

  RCLCPP_INFO_ONCE(this->get_logger(), "LeePositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::msg::Actuators actuator_msg;

  actuator_msg. angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg.angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg.header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_->publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto controller_node = std::make_shared 
    <rotors_control::LeePositionControllerNode>(options);
  controller_node->RunController(controller_node->shared_from_this());
  rclcpp::spin(controller_node);
  rclcpp::shutdown();
  return 0;
}
