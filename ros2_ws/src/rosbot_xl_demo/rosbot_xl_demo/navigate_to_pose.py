# Copyright (c) Contributors to the Open 3D Engine Project.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from gazebo_msgs.srv import SpawnEntity

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from threading import Thread

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('position', [0.0, 0.0, 0.0])
        
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.position = self.get_parameter('position').get_parameter_value().double_array_value

        action_name = f'/{self.robot_namespace}/navigate_to_pose'
        self.get_logger().info(f'Waiting for action server: {action_name}')
        self.cli = ActionClient(self, NavigateToPose, action_name)
        self.cli.wait_for_server()

    def send_request(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.position[0]
        goal_pose.pose.position.y = self.position[1]
        goal_pose.pose.position.z = self.position[2]
        goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.orientation.z = 0.0

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.get_logger().info(f'Sending goal: {goal}')
        future = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error('Result is None')
            return
        self.get_logger().info(f'Goal result: {result}')

def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()
    navigator.send_request()
    navigator.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()