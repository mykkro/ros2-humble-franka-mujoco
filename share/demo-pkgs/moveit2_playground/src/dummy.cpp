#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("dummy");

  RCLCPP_INFO(logger, "[01] rclcpp::init");

  auto const node = std::make_shared<rclcpp::Node>(
    "dummy",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  /*
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_;

  // this will not work from outside a class....
  moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());

  // Planning component associated with a single motion group
  planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>("panda_arm", moveit_cpp_);

  // Parameters set on this node
  plan_parameters_.load(this->shared_from_this());

  RCLCPP_INFO(logger, "[02] get planning_component, load parameters");
  */


  //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // get robotModel
  auto const robotModel = move_group_interface.getRobotModel();
  RCLCPP_INFO(logger, "[02] get robot model");

  // get kinematics state
  // From: https://ros-planning.github.io/moveit_tutorials/doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(robotModel));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = robotModel->getJointModelGroup("panda_arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();  
  RCLCPP_INFO(logger, "[02] get kinematic state objects");

  std::vector<double> joint_values;

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  // all zeros? something is not right...
  RCLCPP_INFO(logger, "[03] got joint values");


  /* Print end-effector pose. Remember that this is in the model frame */
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");
  RCLCPP_INFO_STREAM(logger, "Translation: \n" << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(logger, "Rotation: \n" << end_effector_state.rotation() << "\n");

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                              kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                              reference_point_position, jacobian);
  RCLCPP_INFO_STREAM(logger, "Jacobian: \n" << jacobian << "\n");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  // still zeros!
  RCLCPP_INFO(logger, "[04] got joint values");

  /* Print end-effector pose. Remember that this is in the model frame */
  const Eigen::Isometry3d& end_effector_state_2 = kinematic_state->getGlobalLinkTransform("panda_link8");
  RCLCPP_INFO_STREAM(logger, "Translation: \n" << end_effector_state_2.translation() << "\n");
  RCLCPP_INFO_STREAM(logger, "Rotation: \n" << end_effector_state_2.rotation() << "\n");
  // it reports still the same translation, rotation matrix is close to identity matrix with y and z factors negated


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}