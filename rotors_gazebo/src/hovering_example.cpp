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

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.hpp>
#include <mav_msgs/default_topics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>
    ("hovering_example", rclcpp::NodeOptions());
  //
  auto trajectory_pub = node->create_publisher
    <trajectory_msgs::msg::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY,
            rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(node->get_logger(), "Started hovering example.");

  std_srvs::srv::Empty srv;
  auto gz_phy_client = node->create_client<std_srvs::srv::Empty>
    ("/unpause_physics");
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();

  unsigned int i = 0;
  auto result =  gz_phy_client->async_send_request(req);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(10)) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Physics unpaused");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to unpause Gazebo physics");
    return 0;
  }

  // if (!unpaused) {
  //   RCLCPP_ERROR(this->get_logger(), "Could not wake up Gazebo.");
  //   return -1;
  // } else {
  //   RCLCPP_INFO(node->get_logger(),
  //     "Unpaused the Gazebo simulation.");
  // }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  std::this_thread::sleep_for(5s);

  trajectory_msgs::msg::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->get_clock()->now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
  double desired_yaw = 0.0;

  // Overwrite defaults if set as node parameters.
  // nh_private.param("x", desired_position.x(), desired_position.x());
  // nh_private.param("y", desired_position.y(), desired_position.y());
  // nh_private.param("z", desired_position.z(), desired_position.z());
  // nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  RCLCPP_INFO(node->get_logger(),
    "Publishing waypoint on namespace: [%f, %f, %f].",
     desired_position.x(), desired_position.y(), desired_position.z());
  trajectory_pub->publish(trajectory_msg);

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
