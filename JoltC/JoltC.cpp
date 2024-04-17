#include "JoltC/JoltC.h"

#include "Jolt/Jolt.h"

#include "Jolt/Core/Factory.h"
#include "Jolt/Core/Memory.h"
#include "Jolt/Core/TempAllocator.h"
#include "Jolt/RegisterTypes.h"
#include "Jolt/Core/Core.h"

static auto to_jpc(JPH::TempAllocatorImpl *in) { return reinterpret_cast<JPC_TempAllocatorImpl*>(in); }
static auto to_jph(JPC_TempAllocatorImpl *in) { return reinterpret_cast<JPH::TempAllocatorImpl*>(in); }

JPC_API void JPC_RegisterDefaultAllocator() {
	JPH::RegisterDefaultAllocator();
}

JPC_API void JPC_FactoryInit() {
	JPH::Factory::sInstance = new JPH::Factory();
}

JPC_API void JPC_RegisterTypes() {
	JPH::RegisterTypes();
}

JPC_API JPC_TempAllocatorImpl* JPC_TempAllocatorImpl_new(unsigned int size) {
	return to_jpc(new JPH::TempAllocatorImpl(size));
}

JPC_API void JPC_TempAllocatorImpl_delete(JPC_TempAllocatorImpl* object) {
	delete to_jph(object);
}