#include "JoltC/JoltC.h"

#include "Jolt/Jolt.h"
#include "Jolt/Core/Memory.h"
#include "Jolt/Core/Factory.h"

JPC_API void JPC_RegisterDefaultAllocator() {
	JPH::RegisterDefaultAllocator();
}

JPC_API void JPC_FactoryInit() {
	JPH::Factory::sInstance = new JPH::Factory();
}