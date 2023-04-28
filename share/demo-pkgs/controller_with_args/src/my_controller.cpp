#include <rclcpp/rclcpp.hpp>
#include <cassert>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>

class MyController : public controller_interface::ControllerInterface
{
public:

  controller_interface::InterfaceConfiguration command_interface_configuration() const override  {
    	controller_interface::InterfaceConfiguration config;
	/**/
	return config;
  }
  
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    	controller_interface::InterfaceConfiguration config;
	/**/
	return config;
  }

	/*
  controller_interface::return_type init(const std::string& controller_name) override
  {
    RCLCPP_INFO(get_node()->get_logger(), "Hello, world!");
    return controller_interface::return_type::OK;
  }
  */

  // controller_interface::return_type update() override -> this is for Foxy!
  // this is for Humble...
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    return controller_interface::return_type::OK;
  }
  
  
  // in Humble - not in Foxy!
  CallbackReturn on_init() override {
 	return CallbackReturn::SUCCESS;
    }

  
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override {
   return CallbackReturn::SUCCESS;
   }
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override {
   return CallbackReturn::SUCCESS;
  }
  
};

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(MyController, controller_interface::ControllerInterface)
