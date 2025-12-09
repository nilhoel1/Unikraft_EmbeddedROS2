#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class UnikraftLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit UnikraftLifecycleNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("unikraft_ros2_node", options)
  {
    RCLCPP_INFO(get_logger(), "UnikraftLifecycleNode constructed");
  }

  // Lifecycle callback: on_configure
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring node...");
    
    // Initialize publishers, subscribers, etc. here
    timer_ = create_wall_timer(
      1s,
      [this]() {
        RCLCPP_INFO(get_logger(), "Node is active and running on Unikraft!");
      });
    
    // Deactivate timer until node is activated
    timer_->cancel();
    
    RCLCPP_INFO(get_logger(), "Node configured successfully");
    return CallbackReturn::SUCCESS;
  }

  // Lifecycle callback: on_activate
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating node...");
    
    // Start timers, activate publishers, etc.
    if (timer_) {
      timer_->reset();
    }
    
    RCLCPP_INFO(get_logger(), "Node activated successfully");
    return CallbackReturn::SUCCESS;
  }

  // Lifecycle callback: on_deactivate
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating node...");
    
    // Stop timers, deactivate publishers, etc.
    if (timer_) {
      timer_->cancel();
    }
    
    RCLCPP_INFO(get_logger(), "Node deactivated successfully");
    return CallbackReturn::SUCCESS;
  }

  // Lifecycle callback: on_cleanup
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up node...");
    
    // Clean up resources
    timer_.reset();
    
    RCLCPP_INFO(get_logger(), "Node cleaned up successfully");
    return CallbackReturn::SUCCESS;
  }

  // Lifecycle callback: on_shutdown
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Shutting down node...");
    
    // Release all resources
    timer_.reset();
    
    RCLCPP_INFO(get_logger(), "Node shut down successfully");
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ROS 2 node on Unikraft with MultiThreadedExecutor");

  // Create executor with multiple threads
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(),
    4  // Number of threads
  );

  // Create lifecycle node
  auto node = std::make_shared<UnikraftLifecycleNode>(rclcpp::NodeOptions());

  // Add node to executor
  executor->add_node(node->get_node_base_interface());

  RCLCPP_INFO(rclcpp::get_logger("main"), "Node added to MultiThreadedExecutor");

  // Trigger lifecycle transitions
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Node configured and activated, spinning...");

  // Spin the executor
  executor->spin();

  // Cleanup
  rclcpp::shutdown();

  return 0;
}
