cmake_minimum_required(VERSION 3.5)
project(pymoveit2_demos)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install examples
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/ex_collision_object.py
    ${EXAMPLES_DIR}/ex_gripper_command.py
    ${EXAMPLES_DIR}/ex_gripper.py
    ${EXAMPLES_DIR}/ex_joint_goal.py
    ${EXAMPLES_DIR}/ex_pose_goal.py
    ${EXAMPLES_DIR}/ex_servo.py
    ${EXAMPLES_DIR}/ex_servo2.py
    DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY ${EXAMPLES_DIR}/assets
  DESTINATION lib/${PROJECT_NAME}/
)

# Setup the project
ament_package()
