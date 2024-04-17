#include "Jolt/Jolt.h"

#include "Jolt/Core/Core.h"
#include "Jolt/Core/Factory.h"
#include "Jolt/Core/JobSystemThreadPool.h"
#include "Jolt/Core/Memory.h"
#include "Jolt/Core/TempAllocator.h"
#include "Jolt/RegisterTypes.h"

#include "JoltC/JoltC.h"

#define OPAQUE_WRAPPER(c_type, cpp_type) \
	static auto to_jpc(cpp_type *in) { return reinterpret_cast<c_type*>(in); } \
	static auto to_jph(c_type *in) { return reinterpret_cast<cpp_type*>(in); }

OPAQUE_WRAPPER(JPC_TempAllocatorImpl, JPH::TempAllocatorImpl)

JPC_API void JPC_RegisterDefaultAllocator() {
	JPH::RegisterDefaultAllocator();
}

JPC_API void JPC_FactoryInit() {
	JPH::Factory::sInstance = new JPH::Factory();
}

JPC_API void JPC_RegisterTypes() {
	JPH::RegisterTypes();
}

JPC_API JPC_TempAllocatorImpl* JPC_TempAllocatorImpl_new(uint size) {
	return to_jpc(new JPH::TempAllocatorImpl(size));
}

JPC_API void JPC_TempAllocatorImpl_delete(JPC_TempAllocatorImpl* object) {
	delete to_jph(object);
}

OPAQUE_WRAPPER(JPC_JobSystemThreadPool, JPH::JobSystemThreadPool)

JPC_API JPC_JobSystemThreadPool* JPC_JobSystemThreadPool_new2(
	uint inMaxJobs,
	uint inMaxBarriers)
{
	return to_jpc(new JPH::JobSystemThreadPool(inMaxJobs, inMaxBarriers));
}

JPC_API JPC_JobSystemThreadPool* JPC_JobSystemThreadPool_new3(
	uint inMaxJobs,
	uint inMaxBarriers,
	int inNumThreads)
{
	return to_jpc(new JPH::JobSystemThreadPool(inMaxJobs, inMaxBarriers, inNumThreads));
}

JPC_API void JPC_JobSystemThreadPool_delete(JPC_JobSystemThreadPool* object) {
	delete to_jph(object);
}