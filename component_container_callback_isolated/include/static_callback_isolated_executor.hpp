#pragma once
#include "rclcpp/rclcpp.hpp"

class StaticCallbackIsolatedExecutor {
public:
  //StaticCallbackIsolatedExecutor::StaticCallbackIsolatedExecutor();
  void add_node(const rclcpp::Node::SharedPtr &node);
  void add_node(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &node);
  void spin();
  void remove_node(const rclcpp::Node::SharedPtr &node);
  void remove_node(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &node);
  
private:
  //rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::vector<rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr> executors;
};