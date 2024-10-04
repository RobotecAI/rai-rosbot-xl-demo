#pragma once

#include <moveit/move_group_interface/move_group_interface.h>

class ArmController {
public:
  ArmController();
  ~ArmController();

  geometry_msgs::msg::Pose CalculatePose(double x, double y, double z,
                                         double r = 0.0);

  void
  MoveThroughWaypoints(std::vector<geometry_msgs::msg::Pose> const &waypoints);
  void MoveToPose(geometry_msgs::msg::Pose pose);

  void Open();
  void Close();

  std::vector<double> GetEffectorPose();
  bool GetGripper();

  std::vector<double> CaptureJointValues();
  void SetJointValues(std::vector<double> const &jointValues);

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_arm;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_hand;

  std::atomic_bool gripper = false;

  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};
