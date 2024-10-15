#include "robotic_manipulation/rosbot_controller.h"

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "rai_interfaces/srv/manipulator_move_to.hpp"

ROSBotController::ROSBotController() {
  m_node = rclcpp::Node::make_shared("manipulator_service");
  m_executor.add_node(m_node);
  m_spinner = std::thread([&]() { m_executor.spin(); });
}

ROSBotController::~ROSBotController() {
  m_executor.cancel();
  if (m_spinner.joinable()) {
    m_spinner.join();
  }
}

void ROSBotController::Begin(ArmController &arm) {
  auto logger = m_node->get_logger();

  auto service = m_node->create_service<rai_interfaces::srv::ManipulatorMoveTo>(
    "/manipulator_move_to",
    [&](const std::shared_ptr<rai_interfaces::srv::ManipulatorMoveTo::Request> request,
        std::shared_ptr<rai_interfaces::srv::ManipulatorMoveTo::Response> response) {
      RCLCPP_INFO(logger, "Received move request");

      response->success = false;

      //arm.SetReferenceFrame(request->target_pose.header.frame_id);
      //RCLCPP_INFO(logger, "Set reference frame to: %s", request->target_pose.header.frame_id.c_str());

      // Print current pose
      auto current_pose = arm.GetEffectorPose();
      auto [current_x, current_y, current_z, current_rx, current_ry, current_rz] =
          std::make_tuple(current_pose[0], current_pose[1], current_pose[2],
                          current_pose[3], current_pose[4], current_pose[5]);

      RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", current_x, current_y,
                  current_z, current_rx, current_ry, current_rz);
      RCLCPP_INFO(logger, "Target pose: %f %f %f %f %f %f", 
                  request->target_pose.pose.position.x, 
                  request->target_pose.pose.position.y, 
                  request->target_pose.pose.position.z,
                  request->target_pose.pose.orientation.x,
                  request->target_pose.pose.orientation.y,
                  request->target_pose.pose.orientation.z);

      if (request->initial_gripper_state) {
        arm.Open();
      } else {
        arm.Close();
      }

      {
        auto above_current = arm.GetEffectorPose();
        above_current[2] = std::min(0.30, std::max(above_current[2] + 0.03, 0.20));
        RCLCPP_INFO(logger, "Above current: %f %f %f", above_current[0], above_current[1], above_current[2]);
        if (!arm.MoveToPose(arm.CalculatePose(above_current[0], above_current[1], above_current[2])))
          return;

        auto above_target = request->target_pose.pose;
        above_target.position.z = std::min(0.30, std::max(above_target.position.z + 0.10, 0.20));
        if (!arm.MoveToPose(arm.CalculatePose(above_target.position.x, above_target.position.y, above_target.position.z)) ||
            !arm.MoveToPose(arm.CalculatePose(request->target_pose.pose.position.x, request->target_pose.pose.position.y, request->target_pose.pose.position.z)))
          return;

        if (request->final_gripper_state) {
          arm.Open();
        } else {
          arm.Close();
        }
      }

      response->success = true;
    }
  );
  auto state_subscription =
      m_node->create_subscription<geometry_msgs::msg::Pose>(
          "/goal_state", 10, [&](geometry_msgs::msg::Pose::SharedPtr msg) {
            std::string action = "";
            for (auto i : {msg->position.x, msg->position.y, msg->position.z}) {
              action += std::to_string(i) + " ";
            }
            RCLCPP_INFO(logger, "Action: %s", action.c_str());

            double x = msg->position.x;
            double y = msg->position.y;
            double z = msg->position.z;
            double r = 0.0;

            arm.MoveToPose({arm.CalculatePose(x, y, z, r)});

            auto current_pose = arm.GetEffectorPose();
            auto [current_x, current_y, current_z, current_rx, current_ry,
                  current_rz] =
                std::make_tuple(current_pose[0], current_pose[1],
                                current_pose[2], current_pose[3],
                                current_pose[4], current_pose[5]);

            RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", current_x,
                        current_y, current_z, current_rx, current_ry,
                        current_rz);
          });
  auto pickup_subscription =
      m_node->create_subscription<geometry_msgs::msg::Pose>(
          "/pickup_from", 10, [&](geometry_msgs::msg::Pose::SharedPtr msg) {
            std::string action = "";
            for (auto i : {msg->position.x, msg->position.y, msg->position.z}) {
              action += std::to_string(i) + " ";
            }
            RCLCPP_INFO(logger, "Action: %s", action.c_str());

            double x = msg->position.x;
            double y = msg->position.y;
            double z = msg->position.z;
            double r = 0.0;

            arm.Open();
            arm.MoveToPose({arm.CalculatePose(x, y, z + 0.1, r)});
            arm.MoveToPose({arm.CalculatePose(x, y, z, r)});
            arm.Close();
            arm.MoveToPose(arm.CalculatePose(x, y, 0.2));
            arm.MoveToPose(arm.CalculatePose(-0.053, -0.115, 0.12));
            arm.Open();
            arm.MoveToPose(arm.CalculatePose(-0.225, 0.0, 0.25));
          });
  auto animation_subscription =
      m_node->create_subscription<std_msgs::msg::String>(
          "/animation", 10, [&](std_msgs::msg::String::SharedPtr msg) {
            std::string animation = msg->data;
            RCLCPP_INFO(logger, "Animation: %s", animation.c_str());

            if      (animation == "rest")     Rest(arm);
            else if (animation == "open")     Open(arm);
            else if (animation == "close")    Close(arm);
            else if (animation == "wave")     Wave(arm);
            else if (animation == "give_can") GiveCan(arm);

            for (auto i : arm.GetEffectorPose()) {
              RCLCPP_INFO(logger, "%f", i);
            }
          });

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void ROSBotController::Rest(ArmController &arm) {
  auto logger = m_node->get_logger();
  RCLCPP_INFO(logger, "Moving to rest position.");
  arm.MoveToPose(arm.CalculatePose(-0.225, 0.0, 0.25));
}

void ROSBotController::Open(ArmController &arm) {
  auto logger = m_node->get_logger();
  RCLCPP_INFO(logger, "Opening hand.");
  arm.Open();
}

void ROSBotController::Close(ArmController &arm) {
  auto logger = m_node->get_logger();
  RCLCPP_INFO(logger, "Closing hand.");
  arm.Close();
}

void ROSBotController::Wave(ArmController &arm) {
  auto logger = m_node->get_logger();
  RCLCPP_INFO(logger, "Waving hand.");
  arm.MoveToPose(arm.CalculatePose(-0.187, -0.002, 0.318, -1.3));
  arm.MoveToPose(arm.CalculatePose(-0.179, -0.055, 0.318, -1.3));
  arm.MoveToPose(arm.CalculatePose(-0.187, -0.002, 0.318, -1.3));
  arm.MoveToPose(arm.CalculatePose(-0.179, -0.055, 0.318, -1.3));
  arm.MoveToPose(arm.CalculatePose(-0.187, -0.002, 0.318, -1.3));
  Rest(arm);
}

void ROSBotController::GiveCan(ArmController &arm) {
  auto logger = m_node->get_logger();
  RCLCPP_INFO(logger, "Giving can.");
  arm.MoveToPose(arm.CalculatePose(-0.053, -0.115, 0.17));
  arm.Open();
  arm.MoveToPose(arm.CalculatePose(-0.053, -0.115, 0.13));
  arm.Close();
  arm.MoveToPose(arm.CalculatePose(-0.053, -0.115, 0.23));
  arm.MoveToPose(arm.CalculatePose(-0.006, -0.218, 0.187));
  arm.MoveToPose(arm.CalculatePose(-0.002, -0.325, 0.007));
  arm.Open();
  arm.MoveToPose(arm.CalculatePose(-0.002, -0.325, 0.12));
  arm.MoveToPose(arm.CalculatePose(0.0, -0.25, 0.25));
  arm.MoveToPose(arm.CalculatePose(-0.225, 0.0, 0.25));
}