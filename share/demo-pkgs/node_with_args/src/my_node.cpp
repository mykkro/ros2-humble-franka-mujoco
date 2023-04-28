#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("my_node");

	if (argc != 2) {
		RCLCPP_ERROR(node->get_logger(), "Usage: ros2 run my_package my_node arg");
		return 1;
	}

	std::string arg = argv[1];
	RCLCPP_INFO(node->get_logger(), "Received argument: %s", arg.c_str());

	rclcpp::shutdown();
	return 0;
}
