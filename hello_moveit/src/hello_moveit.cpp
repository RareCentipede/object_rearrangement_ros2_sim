#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[]){
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  RCLCPP_INFO(logger, "Hello MoveIt!");

  // MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  geometry_msgs::msg::Pose msg;
  auto const target_pose = [&msg]{
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Plan to the target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan if successful
  if (success){
    RCLCPP_INFO(logger, "Plan found, executing...");
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Motion complete.");
  } else {
    RCLCPP_ERROR(logger, "Planning failed.");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}