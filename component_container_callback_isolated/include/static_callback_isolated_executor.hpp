#pragma once
#include "rclcpp/rclcpp.hpp"

class StaticCallbackIsolatedExecutor {
public:
  void add_node(const rclcpp::Node::SharedPtr &node);
  void add_node(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &node);

  void spin();
  void spin_node();

  void remove_node(const rclcpp::Node::SharedPtr &node);
  void remove_node(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &node);

  
private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::vector<rclcpp::executors::SingleThreadedExecutor::SharedPtr> executors;
};