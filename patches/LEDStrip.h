/*
 * Copyright (c) Robotec.ai.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/map.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace Husarion
{
    class LEDStrip : public AZ::Component
    {
    public:
        AZ_COMPONENT(LEDStrip, "{6C2D9FCD-3204-4457-B4A5-1594D94409C8}");
        LEDStrip();
        ~LEDStrip() = default;

        static void Reflect(AZ::ReflectContext* context);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void Activate() override;
        void Deactivate() override;

    private:
        AZStd::vector<AZStd::pair<int, AZ::EntityId>> m_leds;
        float m_intensity = 1.0f;
        AZStd::map<int, AZ::EntityId> m_lightSet;

        ROS2::TopicConfiguration m_topicConfiguration;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_subscription;
    };
} // namespace Husarion
