#include "JoltC/JoltC.h"

#include "Jolt/Jolt.h"
#include "Jolt/Core/Memory.h"
#include "Jolt/Core/Factory.h"

void JPC_RegisterDefaultAllocator() {
	JPH::RegisterDefaultAllocator();
}

void JPC_FactoryInit() {
	JPH::Factory::sInstance = new JPH::Factory();
}