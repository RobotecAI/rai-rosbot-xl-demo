/*
 * Copyright (c) Robotec.ai.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "LEDStrip.h"

#include <Atom/Feature/CoreLights/PhotometricValue.h>
#include <AtomLyIntegration/CommonFeatures/CoreLights/AreaLightBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Math/Color.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Communication/QoS.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace Husarion
{
    LEDStrip::LEDStrip()
    {
        m_topicConfiguration.m_topic = "led_strip";
        m_topicConfiguration.m_type = "sensor_msgs/msg/image";
    }

    void LEDStrip::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LEDStrip>()
                ->Version(1)
                ->Field("Leds", &LEDStrip::m_leds)
                ->Field("intensity", &LEDStrip::m_intensity)
                ->Field("TopicConfiguration", &LEDStrip::m_topicConfiguration);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<LEDStrip>("LEDStrip", "LED Strip")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Husarion")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LEDStrip::m_intensity, "Intensity", "Intensity")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LEDStrip::m_leds, "LEDs", "LEDs")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LEDStrip::m_topicConfiguration, "Topic Configuration", "Topic Configuration");
            }
        }
    }

    void LEDStrip::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void LEDStrip::Activate()
    {
        auto node = ROS2::ROS2Interface::Get()->GetNode();
        if (!node)
        {
            AZ_Error("LightController", false, "ROS2 node is not available. Component will not be activated.");
            return;
        }

        auto* ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
        if (!ros2Frame)
        {
            AZ_Error(
                "LightController",
                false,
                "ROS2FrameComponent is not available. Component will not be "
                "activated.");
            return;
        }

        AZStd::string topicName = ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_topicConfiguration.m_topic);

        m_subscription = node->create_subscription<sensor_msgs::msg::Image>(
            topicName.c_str(),
            m_topicConfiguration.GetQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                AZStd::unordered_set<AZStd::string> supportedTypes{ "mono8", "rgb8", "rgba8", "8UC1", "8UC3", "8UC4" };
                if (!supportedTypes.contains(msg->encoding.c_str()))
                {
                    AZ_Error("Husarion - LED strip", false, "Unsupported image encoding.");
                    return;
                }

                const int channels = sensor_msgs::image_encodings::numChannels(msg->encoding);

                int pixel = 0;
                for (int h = 0; h < msg->height; ++h)
                {
                    for (int w = 0; w < msg->width; ++w)
                    {
                        const int index = pixel * channels;
                        if (m_lightSet.contains(pixel))
                        {
                            AZ::Color color;
                            if (channels == 1)
                            {
                                AZ::u8 c = msg->data[index];
                                color = AZ::Color(c, c, c, 255);
                            }
                            else if (channels == 3)
                            {
                                AZ::u8 r = msg->data[index + 0];
                                AZ::u8 g = msg->data[index + 1];
                                AZ::u8 b = msg->data[index + 2];
                                color = AZ::Color(r, g, b, 255);
                            }
                            else if (channels == 4)
                            {
                                AZ::u8 r = msg->data[index + 0];
                                AZ::u8 g = msg->data[index + 1];
                                AZ::u8 b = msg->data[index + 2];
                                AZ::u8 a = msg->data[index + 3];
                                color = AZ::Color(r, g, b, a);
                            }
                            AZ::Render::AreaLightRequestBus::Event(
                                m_lightSet[pixel], &AZ::Render::AreaLightRequestBus::Events::SetColor, color);
                        }
                        ++pixel;
                    }
                }
            });

        m_lightSet.clear();
        for (auto& led : m_leds)
        {
            m_lightSet[led.first] = led.second;
        }

        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                for (auto& led : m_leds)
                {
                    AZ::Render::AreaLightRequestBus::Event(
                        led.second,
                        &AZ::Render::AreaLightRequestBus::Events::SetIntensityAndMode,
                        m_intensity,
                        AZ::Render::PhotometricUnit::Lumen);
                }
            });
    }

    void LEDStrip::Deactivate()
    {
        m_subscription.reset();
    }
} // namespace Husarion
