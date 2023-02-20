#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_

#include <rclcpp/rclcpp.hpp>

#include "rotors_control/parameters.h"

namespace rotors_control {

template<typename T> inline void GetRosParameter(const rclcpp::Node::SharedPtr node,
                                                 const std::string& key,
                                                 const T& default_value,
                                                 T* value) {
  if(value == nullptr)
    RCLCPP_FATAL(node->get_logger(), "Value is empty");

  bool have_parameter = node->get_parameter(key, *value);
  if (!have_parameter) {
    RCLCPP_WARN_STREAM(node->get_logger(),
     "[rosparam]: could not find parameter. Default val:  " 
      << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(),
     "Value:" << key << ": " << default_value);
  }

}

inline void GetRotorConfiguration(const rclcpp::Node::SharedPtr node,
                                  RotorConfiguration* rotor_configuration) {
  std::string rotor_configuration_string = "rotor_configuration.";
  unsigned int i = 0;
  while (node->has_parameter(rotor_configuration_string + std::to_string(i) + ".angle")) {
    if (i == 0) {
      rotor_configuration->rotors.clear();
    }
    Rotor rotor;
    std::string key;

    key = rotor_configuration_string + std::to_string(i) + ".angle";
    node->get_parameter(key, rotor.angle);
    RCLCPP_INFO_STREAM(node->get_logger(), "Val:" << "/" << key << ": " << rotor.angle);

    key = rotor_configuration_string + std::to_string(i) + ".arm_length";
    node->get_parameter(key, rotor.arm_length);
    RCLCPP_INFO_STREAM(node->get_logger(), "Val:" << "/" << key << ": " << rotor.arm_length);

    key = rotor_configuration_string + std::to_string(i) + ".rotor_force_constant";
    node->get_parameter(key, rotor.rotor_force_constant);
    RCLCPP_INFO_STREAM(node->get_logger(), "Val:" << "/" << key << ": " << rotor.rotor_force_constant);

    key = rotor_configuration_string + std::to_string(i) + ".rotor_moment_constant";
    node->get_parameter(key, rotor.rotor_moment_constant);
    RCLCPP_INFO_STREAM(node->get_logger(), "Val:" << "/" << key << ": " << rotor.rotor_moment_constant);

    key = rotor_configuration_string + std::to_string(i) + ".direction";
    node->get_parameter(key, rotor.direction);
    RCLCPP_INFO_STREAM(node->get_logger(), "Val:" << "/" << key << ": " << rotor.direction);

    rotor_configuration->rotors.push_back(rotor);
    ++i;
  }
}

inline void GetVehicleParameters(const rclcpp::Node::SharedPtr node, VehicleParameters* vehicle_parameters) {
  GetRosParameter(node, "mass",
                  vehicle_parameters->mass_,
                  &vehicle_parameters->mass_);
  GetRosParameter(node, "inertia.xx",
                  vehicle_parameters->inertia_(0, 0),
                  &vehicle_parameters->inertia_(0, 0));
  GetRosParameter(node, "inertia.xy",
                  vehicle_parameters->inertia_(0, 1),
                  &vehicle_parameters->inertia_(0, 1));
  vehicle_parameters->inertia_(1, 0) = vehicle_parameters->inertia_(0, 1);
  GetRosParameter(node, "inertia.xz",
                  vehicle_parameters->inertia_(0, 2),
                  &vehicle_parameters->inertia_(0, 2));
  vehicle_parameters->inertia_(2, 0) = vehicle_parameters->inertia_(0, 2);
  GetRosParameter(node, "inertia.yy",
                  vehicle_parameters->inertia_(1, 1),
                  &vehicle_parameters->inertia_(1, 1));
  GetRosParameter(node, "inertia.yz",
                  vehicle_parameters->inertia_(1, 2),
                  &vehicle_parameters->inertia_(1, 2));
  vehicle_parameters->inertia_(2, 1) = vehicle_parameters->inertia_(1, 2);
  GetRosParameter(node, "inertia.zz",
                  vehicle_parameters->inertia_(2, 2),
                  &vehicle_parameters->inertia_(2, 2));
  GetRotorConfiguration(node, &vehicle_parameters->rotor_configuration_);
}
}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_ */
