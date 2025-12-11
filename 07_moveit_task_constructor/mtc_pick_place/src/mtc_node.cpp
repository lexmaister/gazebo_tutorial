#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>

// FIX 2: Include the full definition for RobotModel
#include <moveit/robot_model/robot_model.h>

namespace mtc = moveit::task_constructor;

// No need to inherit from std::enable_shared_from_this here anymore,
// as rclcpp::Node already does.
class MtcInitNode : public rclcpp::Node
{
public:
  MtcInitNode(const rclcpp::NodeOptions& options)
    : Node("mtc_node", options) // Corrected name to match logs
  {
    RCLCPP_INFO(this->get_logger(), "MTC Node constructor finished.");
  }

  void initialize()
  {
    RCLCPP_INFO(this->get_logger(), "MTC Node post-constructor initialization started.");
    try
    {
      task_ = std::make_unique<mtc::Task>("MtcInitializationTask");

      // FIX 1: Use the unambiguous way to get a shared pointer to the node
      task_->loadRobotModel(this->get_node_base_interface());

      task_->setProperty("group", "panda_arm");

      if (task_->getRobotModel())
      {
        RCLCPP_INFO(this->get_logger(), "Robot model '%s' loaded successfully for the task.",
                    task_->getRobotModel()->getName().c_str());
        RCLCPP_INFO(this->get_logger(), "Task initialized successfully.");
      }
      else
      {
        throw std::runtime_error("Failed to load robot model");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
    }
  }

private:
  std::unique_ptr<mtc::Task> task_;
};

// Main function remains the same
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<MtcInitNode>(options);
  node->initialize();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}