cmake_minimum_required(VERSION 3.5)
project(panda2_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(panda2_description REQUIRED)

# Install directories
install(DIRECTORY config launch rviz srdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
