#include <memory>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/syscall.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include "ros2_thread_configurator.hpp"

namespace rclcpp_components
{

class ComponentManagerNode : public rclcpp_components::ComponentManager {

    struct ExecutorWrapper {
    explicit ExecutorWrapper(std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor)
      : executor(executor), thread_initialized(false) {}

    ExecutorWrapper(const ExecutorWrapper&) = delete;
    ExecutorWrapper& operator=(const ExecutorWrapper&) = delete;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread thread;
    std::atomic_bool thread_initialized;
  };

public:
  template<typename... Args>
  ComponentManagerNode(Args&&... args) : rclcpp_components::ComponentManager(std::forward<Args>(args)...) {
    client_publisher_ = ros2_thread_configurator::create_client_publisher();
  }

  ~ComponentManagerNode();

protected:
  void add_node_to_executor(uint64_t node_id) override;
  void remove_node_from_executor(uint64_t node_id) override;
private:
  void cancel_executor(ExecutorWrapper &executor_wrapper);
  std::unordered_map<uint64_t, std::list<ExecutorWrapper>> node_id_to_executor_wrappers_;
  rclcpp::Publisher<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr client_publisher_;
};

ComponentManagerNode::~ComponentManagerNode() {
  if (node_wrappers_.size() == 0) return;

  for (auto &p : node_id_to_executor_wrappers_) {
    auto &executor_wrappers = p.second;

    for (auto &executor_wrapper : executor_wrappers) {
      cancel_executor(executor_wrapper);
    }
  }

  node_wrappers_.clear();
}

void ComponentManagerNode::add_node_to_executor(uint64_t node_id){
  auto node = node_wrappers_[node_id].get_node_base_interface();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  auto it = node_id_to_executor_wrappers_[node_id].begin();
  it = node_id_to_executor_wrappers_[node_id].emplace(it, executor);
  auto &executor_wrapper = *it;

  executor_wrapper.thread = std::thread([&executor_wrapper, node, this](){
    auto tid = syscall(SYS_gettid);

    for (int i = 0; i < 3; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ros2_thread_configurator::publish_callback_group_info(this->client_publisher_, tid, ros2_thread_configurator::create_node_id(node));
    }

    executor_wrapper.thread_initialized = true;
    executor_wrapper.executor->spin();
  });
}

void ComponentManagerNode::remove_node_from_executor(uint64_t node_id) {
  auto it = node_id_to_executor_wrappers_.find(node_id);
  if (it == node_id_to_executor_wrappers_.end()) return;

  for (ExecutorWrapper &executor_wrapper : it->second) {
    cancel_executor(executor_wrapper);
  }

  node_id_to_executor_wrappers_.erase(it);
}

void ComponentManagerNode::cancel_executor(ExecutorWrapper &executor_wrapper) {
  if (!executor_wrapper.thread_initialized) {
    auto context = this->get_node_base_interface()->get_context();

    while (!executor_wrapper.executor->is_spinning() && rclcpp::ok(context)) {
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
  }

  executor_wrapper.executor->cancel();
  executor_wrapper.thread.join();
}

} //rclcpp_components name space

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManagerNode>();
  exec->add_node(node);
  exec->spin();
}