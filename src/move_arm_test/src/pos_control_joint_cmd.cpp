#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "interbotix_xs_msgs/JointGroupCommand.h"

// Run launch file: ros2 launch interbotix_xsarm_ros_control xsarm_ros_control.launch.py robot_model:=px100
// Msgs are read by node '/px100/xs_sdk'

// Unlock arm: ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"
// Lock arm:   ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
 
int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pos_control_joint_cmd",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Send to /px100/commands/joint_group  
  // name:      position
  // waist:         1.59
  // shoulder -     1.38
  // elbow          0.39
  // wrist_angle   -0.73
  // gripper        0.00
  // left_finger    0.01
  // right_finger  -0.01
  interbotix_xs_msgs::msg::JointGroupCommand joint_msg;
  joint_msg.name = "all";
  joint_msg.cmd = {1.59, 1.38, 0.39,-0.73,0.00,0.00,0.00}; // Some example commands

  auto publisher = node->create_publisher<std_msgs::msg::String>("px100/commands/joint_group", 10);
  publisher->publish(joint_msg);

  // Give time to publish
  rclcpp::spin_some(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}