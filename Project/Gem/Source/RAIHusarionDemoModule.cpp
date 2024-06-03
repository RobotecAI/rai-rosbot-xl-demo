
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "RAIHusarionDemoSystemComponent.h"

#include <RAIHusarionDemo/RAIHusarionDemoTypeIds.h>

namespace RAIHusarionDemo
{
    class RAIHusarionDemoModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(RAIHusarionDemoModule, RAIHusarionDemoModuleTypeId, AZ::Module);
        AZ_CLASS_ALLOCATOR(RAIHusarionDemoModule, AZ::SystemAllocator);

        RAIHusarionDemoModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                RAIHusarionDemoSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RAIHusarionDemoSystemComponent>(),
            };
        }
    };
}// namespace RAIHusarionDemo

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), RAIHusarionDemo::RAIHusarionDemoModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_RAIHusarionDemo, RAIHusarionDemo::RAIHusarionDemoModule)
#endif
