#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

// Use a shorter namespace for convenience
namespace mtc = moveit::task_constructor;

// Create a class for our node
class MtcInitNode : public rclcpp::Node
{
public:
  // Constructor
  MtcInitNode(const rclcpp::NodeOptions& options)
    : Node("mtc_init_node", options), task_{}
  {
    RCLCPP_INFO(this->get_logger(), "MTC Init Node started.");

    // This is the core of the initialization
    try
    {
      initializeTask();
      RCLCPP_INFO(this->get_logger(), "Task initialized successfully.");
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
    }
  }

private:
  void initializeTask()
  {
    // 1. Create the MTC Task object
    // The task name is used for logging and identification
    task_ = std::make_unique<mtc::Task>("MtcInitializationTask");

    // 2. Load the robot model
    // This is where MTC "catches" the robot info. It looks for the
    // `/robot_description` parameter published by move_group.
    // We pass 'this' (the node) to provide the ROS context.
    task_->loadRobotModel(shared_from_this());

    // 3. (Optional but good practice) Add properties to the task
    // These are stored with the task solution and can be useful for debugging
    task_->setProperty("group", "panda_arm"); // Example: Specify the main planning group

    // At this point, the task is initialized and knows about the robot.
    // You can now add stages, but for this example, we stop here.

    // You can verify that the robot model was loaded correctly
    if (task_->getRobotModel())
    {
      RCLCPP_INFO(this->get_logger(), "Robot model '%s' loaded successfully for the task.",
                  task_->getRobotModel()->getName().c_str());
    }
    else
    {
      // This will throw an exception, but it's good practice to check
      throw std::runtime_error("Failed to load robot model");
    }
  }

  // Member variable to hold the task
  std::unique_ptr<mtc::Task> task_;
};

// Main function to spin up the node
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // We need to set node options to automatically declare parameters from overrides
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // Create and spin the node
  auto node = std::make_shared<MtcInitNode>(options);
  rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}