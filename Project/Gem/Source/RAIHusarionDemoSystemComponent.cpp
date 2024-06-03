
#include <AzCore/Serialization/SerializeContext.h>

#include "RAIHusarionDemoSystemComponent.h"

#include <RAIHusarionDemo/RAIHusarionDemoTypeIds.h>

namespace RAIHusarionDemo
{
    AZ_COMPONENT_IMPL(RAIHusarionDemoSystemComponent, "RAIHusarionDemoSystemComponent",
        RAIHusarionDemoSystemComponentTypeId);

    void RAIHusarionDemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RAIHusarionDemoSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void RAIHusarionDemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RAIHusarionDemoService"));
    }

    void RAIHusarionDemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RAIHusarionDemoService"));
    }

    void RAIHusarionDemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RAIHusarionDemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RAIHusarionDemoSystemComponent::RAIHusarionDemoSystemComponent()
    {
        if (RAIHusarionDemoInterface::Get() == nullptr)
        {
            RAIHusarionDemoInterface::Register(this);
        }
    }

    RAIHusarionDemoSystemComponent::~RAIHusarionDemoSystemComponent()
    {
        if (RAIHusarionDemoInterface::Get() == this)
        {
            RAIHusarionDemoInterface::Unregister(this);
        }
    }

    void RAIHusarionDemoSystemComponent::Init()
    {
    }

    void RAIHusarionDemoSystemComponent::Activate()
    {
        RAIHusarionDemoRequestBus::Handler::BusConnect();
    }

    void RAIHusarionDemoSystemComponent::Deactivate()
    {
        RAIHusarionDemoRequestBus::Handler::BusDisconnect();
    }
}
