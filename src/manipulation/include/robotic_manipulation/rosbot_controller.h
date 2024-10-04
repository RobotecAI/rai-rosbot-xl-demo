#pragma once

#include "robotic_manipulation/arm_controller.h"

class ROSBotController {
public:
  ROSBotController();
  ~ROSBotController();

  void Begin(ArmController &arm);

  void Rest(ArmController &arm);
  void Open(ArmController &arm);
  void Close(ArmController &arm);
  void Wave(ArmController &arm);
  void GiveCan(ArmController &arm);

private:
  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};