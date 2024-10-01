
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
            AZ_Error("LightController", false, "ROS2FrameComponent is not available. Component will not be activated.");
            return;
        }

        AZStd::string topicName = ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_topicConfiguration.m_topic);

        m_subscription = node->create_subscription<sensor_msgs::msg::Image>(
            topicName.c_str(),
            m_topicConfiguration.GetQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                if (msg->encoding.c_str() != AZStd::string("8UC1"))
                {
                    AZ_Error("Husarion - LED strip", false, "Unsupported image encoding. Only 8UC1 is supported.");
                    return;
                }
                if (msg->width != 3)
                {
                    AZ_Error("Husarion - LED strip", false, "Unsupported image width. Only 3 is supported.");
                    return;
                }
                if (msg->height != 18)
                {
                    AZ_Error("Husarion - LED strip", false, "Unsupported image height. Only 18 is supported.");
                    return;
                }

                for (int h = 0; h < msg->height; ++h)
                {
                    AZ::u8 red = msg->data[h * msg->step + 0];
                    AZ::u8 green = msg->data[h * msg->step + 1];
                    AZ::u8 blue = msg->data[h * msg->step + 2];

                    AZ::Color lightColor(red, green, blue, 255);
                    if (m_lightSet.contains(h))
                    {
                        AZ::Render::AreaLightRequestBus::Event(
                            m_lightSet[h], &AZ::Render::AreaLightRequestBus::Events::SetColor, lightColor);
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
