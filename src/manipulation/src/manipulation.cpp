#include <memory>

#include "robotic_manipulation/arm_controller.h"
#include "robotic_manipulation/rosbot_controller.h"

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto armController = std::make_shared<ArmController>();

  auto rosbot = std::make_shared<ROSBotController>();
  rosbot->Begin(*armController);

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}