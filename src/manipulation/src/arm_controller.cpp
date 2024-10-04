#include "robotic_manipulation/arm_controller.h"

#include <Eigen/Geometry>

ArmController::ArmController() {
  m_node = rclcpp::Node::make_shared("manipulator");
  m_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  m_executor.add_node(m_node);
  m_spinner = std::thread([this]() { m_executor.spin(); });

  m_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      m_node, "manipulator");
  m_hand = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      m_node, "gripper");

  m_arm->setMaxVelocityScalingFactor(1.0);
  m_arm->setMaxAccelerationScalingFactor(1.0);
  m_arm->setPoseReferenceFrame("link1");

  m_arm->setGoalOrientationTolerance(0.05);
  m_arm->setGoalPositionTolerance(0.0001);
}

ArmController::~ArmController() {
  m_executor.cancel();
  if (m_spinner.joinable()) {
    m_spinner.join();
  }
}

void ArmController::MoveToPose(geometry_msgs::msg::Pose pose) {
  m_arm->setPoseTarget(pose);
  while (m_arm->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to move to pose");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

geometry_msgs::msg::Pose ArmController::CalculatePose(double x, double y,
                                                      double z, double r) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(r, Eigen::Vector3d::UnitY()));
  quat = pitchAngle * quat;

  auto angle = std::atan2(y, x);
  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  quat = yawAngle * quat;

  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  return pose;
}

void ArmController::MoveThroughWaypoints(
    std::vector<geometry_msgs::msg::Pose> const &waypoints) {
  auto logger = m_node->get_logger();
  moveit_msgs::msg::RobotTrajectory trajectory;
  if (m_arm->computeCartesianPath(waypoints, 0.01, 0.0, trajectory) == -1) {
    RCLCPP_ERROR(logger,
                 "MoveThroughWaypoints: Failed to compute Cartesian path");
    return;
  }

  while (m_arm->execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "MoveThroughWaypoints: Failed to execute trajectory");
  }
}

void ArmController::Open() {
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"gripper_left_joint"};
  joint_state.position = {0.019};
  m_hand->setJointValueTarget(joint_state);

  while (m_hand->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to open hand");
  }
}

void ArmController::Close() {
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"gripper_left_joint"};
  joint_state.position = {-0.01};
  m_hand->setJointValueTarget(joint_state);

  auto error = m_hand->move();
  while (error != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to close hand");
    if (error == moveit::core::MoveItErrorCode::CONTROL_FAILED) {
      break;
    }
    error = m_hand->move();
  }
}

std::vector<double> ArmController::GetEffectorPose() {
  auto pose = m_arm->getCurrentPose().pose;
  auto rotation = tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 m(rotation);
  m.getRPY(pose.orientation.x, pose.orientation.y, pose.orientation.z, 0);
  return {pose.position.x,    pose.position.y,    pose.position.z,
          pose.orientation.x, pose.orientation.y, pose.orientation.z};
};

bool ArmController::GetGripper() { return gripper.load(); }

std::vector<double> ArmController::CaptureJointValues() {
  return m_arm->getCurrentJointValues();
}

void ArmController::SetJointValues(std::vector<double> const &jointValues) {
  m_arm->setJointValueTarget(jointValues);
  if (m_arm->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to set joint values");
  }
}