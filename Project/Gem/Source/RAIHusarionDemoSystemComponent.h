
#pragma once

#include <AzCore/Component/Component.h>

#include <RAIHusarionDemo/RAIHusarionDemoBus.h>

namespace RAIHusarionDemo
{
    class RAIHusarionDemoSystemComponent
        : public AZ::Component
        , protected RAIHusarionDemoRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(RAIHusarionDemoSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        RAIHusarionDemoSystemComponent();
        ~RAIHusarionDemoSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // RAIHusarionDemoRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
