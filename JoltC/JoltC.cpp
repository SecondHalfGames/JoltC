#include "Jolt/Jolt.h"

#include "Jolt/Core/Core.h"
#include "Jolt/Core/Factory.h"
#include "Jolt/Core/JobSystemThreadPool.h"
#include "Jolt/Core/Memory.h"
#include "Jolt/Core/TempAllocator.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/RegisterTypes.h"

#include "JoltC/JoltC.h"

#define OPAQUE_WRAPPER(c_type, cpp_type) \
	static auto to_jpc(cpp_type *in) { return reinterpret_cast<c_type*>(in); } \
	static auto to_jph(c_type *in) { return reinterpret_cast<cpp_type*>(in); }

#define DESTRUCTOR(c_type) \
	JPC_API void c_type##_delete(c_type* object) { \
		delete to_jph(object); \
	}

template<typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type 
{
	return static_cast<typename std::underlying_type<E>::type>(e);
}

JPC_API void JPC_RegisterDefaultAllocator() {
	JPH::RegisterDefaultAllocator();
}

JPC_API void JPC_FactoryInit() {
	JPH::Factory::sInstance = new JPH::Factory();
}

JPC_API void JPC_FactoryDelete() {
	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;
}

JPC_API void JPC_RegisterTypes() {
	JPH::RegisterTypes();
}

JPC_API void JPC_UnregisterTypes() {
	JPH::UnregisterTypes();
}

////////////////////////////////////////////////////////////////////////////////
// TempAllocatorImpl

OPAQUE_WRAPPER(JPC_TempAllocatorImpl, JPH::TempAllocatorImpl)
DESTRUCTOR(JPC_TempAllocatorImpl)

JPC_API JPC_TempAllocatorImpl* JPC_TempAllocatorImpl_new(uint size) {
	return to_jpc(new JPH::TempAllocatorImpl(size));
}

////////////////////////////////////////////////////////////////////////////////
// JobSystemThreadPool

OPAQUE_WRAPPER(JPC_JobSystemThreadPool, JPH::JobSystemThreadPool)
DESTRUCTOR(JPC_JobSystemThreadPool)

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

////////////////////////////////////////////////////////////////////////////////
// BroadPhaseLayerInterface

static auto to_jpc(JPH::BroadPhaseLayer in) { return in.GetValue(); }
static auto to_jph(JPC_BroadPhaseLayer in) { return JPH::BroadPhaseLayer(in); }

class JPC_BroadPhaseLayerInterface_Impl final : public JPH::BroadPhaseLayerInterface {
public:
	explicit JPC_BroadPhaseLayerInterface_Impl(JPC_BroadPhaseLayerInterface in) : self(in.self), fns(in.fns) {}

	virtual uint GetNumBroadPhaseLayers() const override {
		return fns.GetNumBroadPhaseLayers(self);
	}

	virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
		return to_jph(fns.GetBroadPhaseLayer(self, inLayer));
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char * GetBroadPhaseLayerName([[maybe_unused]] JPH::BroadPhaseLayer inLayer) const override {
		return "FIXME";
	}
#endif

private:
	const void* self;
	JPC_BroadPhaseLayerInterfaceFns fns;
};

static JPC_BroadPhaseLayerInterface_Impl to_jph(JPC_BroadPhaseLayerInterface in) {
	return JPC_BroadPhaseLayerInterface_Impl(in);
}

////////////////////////////////////////////////////////////////////////////////
// ObjectVsBroadPhaseLayerFilter

class JPC_ObjectVsBroadPhaseLayerFilter_Impl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
	explicit JPC_ObjectVsBroadPhaseLayerFilter_Impl(JPC_ObjectVsBroadPhaseLayerFilter in) : self(in.self), fns(in.fns) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
		return fns.ShouldCollide(self, inLayer1, to_jpc(inLayer2));
	}

private:
	const void* self;
	JPC_ObjectVsBroadPhaseLayerFilterFns fns;
};

static JPC_ObjectVsBroadPhaseLayerFilter_Impl to_jph(JPC_ObjectVsBroadPhaseLayerFilter in) {
	return JPC_ObjectVsBroadPhaseLayerFilter_Impl(in);
}

////////////////////////////////////////////////////////////////////////////////
// JPC_ObjectLayerPairFilter

class JPC_ObjectLayerPairFilter_Impl final : public JPH::ObjectLayerPairFilter {
public:
	explicit JPC_ObjectLayerPairFilter_Impl(JPC_ObjectLayerPairFilter in) : self(in.self), fns(in.fns) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override {
		return fns.ShouldCollide(self, inLayer1, inLayer2);
	}

private:
	const void* self;
	JPC_ObjectLayerPairFilterFns fns;
};

static JPC_ObjectLayerPairFilter_Impl to_jph(JPC_ObjectLayerPairFilter in) {
	return JPC_ObjectLayerPairFilter_Impl(in);
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsSystem

OPAQUE_WRAPPER(JPC_PhysicsSystem, JPH::PhysicsSystem)
DESTRUCTOR(JPC_PhysicsSystem)

JPC_API JPC_PhysicsSystem* JPC_PhysicsSystem_new() {
	return to_jpc(new JPH::PhysicsSystem());
}

JPC_API void JPC_PhysicsSystem_Init(
	JPC_PhysicsSystem* self,
	uint inMaxBodies,
	uint inNumBodyMutexes,
	uint inMaxBodyPairs,
	uint inMaxContactConstraints,
	JPC_BroadPhaseLayerInterface inBroadPhaseLayerInterface,
	JPC_ObjectVsBroadPhaseLayerFilter inObjectVsBroadPhaseLayerFilter,
	JPC_ObjectLayerPairFilter inObjectLayerPairFilter)
{
	auto impl_inBroadPhaseLayerInterface = to_jph(inBroadPhaseLayerInterface);
	auto impl_inObjectVsBroadPhaseLayerFilter = to_jph(inObjectVsBroadPhaseLayerFilter);
	auto impl_inObjectLayerPairFilter = to_jph(inObjectLayerPairFilter);

	to_jph(self)->Init(
		inMaxBodies,
		inNumBodyMutexes,
		inMaxBodyPairs,
		inMaxContactConstraints,
		impl_inBroadPhaseLayerInterface,
		impl_inObjectVsBroadPhaseLayerFilter,
		impl_inObjectLayerPairFilter);
}

JPC_API JPC_PhysicsUpdateError JPC_PhysicsSystem_Update(
	JPC_PhysicsSystem* self,
	float inDeltaTime,
	int inCollisionSteps,
	JPC_TempAllocatorImpl *inTempAllocator,
	JPC_JobSystemThreadPool *inJobSystem)
{
	auto res = to_jph(self)->Update(
		inDeltaTime,
		inCollisionSteps,
		to_jph(inTempAllocator),
		to_jph(inJobSystem));

	return to_integral(res);
}