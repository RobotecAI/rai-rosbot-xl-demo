
#pragma once

#include <RAIHusarionDemo/RAIHusarionDemoTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RAIHusarionDemo
{
    class RAIHusarionDemoRequests
    {
    public:
        AZ_RTTI(RAIHusarionDemoRequests, RAIHusarionDemoRequestsTypeId);
        virtual ~RAIHusarionDemoRequests() = default;
        // Put your public methods here
    };

    class RAIHusarionDemoBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RAIHusarionDemoRequestBus = AZ::EBus<RAIHusarionDemoRequests, RAIHusarionDemoBusTraits>;
    using RAIHusarionDemoInterface = AZ::Interface<RAIHusarionDemoRequests>;

} // namespace RAIHusarionDemo
