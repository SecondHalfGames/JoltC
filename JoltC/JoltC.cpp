#include <Jolt/Jolt.h>

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

#include <Jolt/Renderer/DebugRendererSimple.h>

#include <JoltC/JoltC.h>

#define OPAQUE_WRAPPER(c_type, cpp_type) \
	static c_type* to_jpc(cpp_type *in) { return reinterpret_cast<c_type*>(in); } \
	static const c_type* to_jpc(const cpp_type *in) { return reinterpret_cast<const c_type*>(in); } \
	static cpp_type* to_jph(c_type *in) { return reinterpret_cast<cpp_type*>(in); } \
	static const cpp_type* to_jph(const c_type *in) { return reinterpret_cast<const cpp_type*>(in); }

#define DESTRUCTOR(c_type) \
	JPC_API void c_type##_delete(c_type* object) { \
		delete to_jph(object); \
	}

#define ENUM_CONVERSION(c_type, cpp_type) \
	static c_type to_jpc(cpp_type in) { return static_cast<c_type>(in); } \
	static cpp_type to_jph(c_type in) { return static_cast<cpp_type>(in); }

#define LAYOUT_COMPATIBLE(c_type, cpp_type) \
	static c_type to_jpc(cpp_type in) { \
		c_type out; \
		memcpy(&out, &in, sizeof(c_type)); \
		return out; \
	} \
	static cpp_type to_jph(c_type in) { \
		cpp_type out; \
		memcpy(&out, &in, sizeof(cpp_type)); \
		return out; \
	} \
	static_assert(sizeof(c_type) == sizeof(cpp_type), "size of " #c_type " did not match size of " #cpp_type); \
	static_assert(alignof(c_type) == alignof(cpp_type), "align of " #c_type " did not match align of " #cpp_type); \
	static_assert(!std::is_polymorphic_v<cpp_type>, #cpp_type " is polymorphic and cannot be made layout compatible");

template<typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type 
{
	return static_cast<typename std::underlying_type<E>::type>(e);
}

ENUM_CONVERSION(JPC_MotionType, JPH::EMotionType)
ENUM_CONVERSION(JPC_AllowedDOFs, JPH::EAllowedDOFs)
ENUM_CONVERSION(JPC_Activation, JPH::EActivation)
ENUM_CONVERSION(JPC_BodyType, JPH::EBodyType)

OPAQUE_WRAPPER(JPC_PhysicsSystem, JPH::PhysicsSystem)
DESTRUCTOR(JPC_PhysicsSystem)

OPAQUE_WRAPPER(JPC_BodyInterface, JPH::BodyInterface)

OPAQUE_WRAPPER(JPC_TempAllocatorImpl, JPH::TempAllocatorImpl)
DESTRUCTOR(JPC_TempAllocatorImpl)

OPAQUE_WRAPPER(JPC_JobSystemThreadPool, JPH::JobSystemThreadPool)
DESTRUCTOR(JPC_JobSystemThreadPool)

OPAQUE_WRAPPER(JPC_ConvexShapeSettings, JPH::ConvexShapeSettings)
OPAQUE_WRAPPER(JPC_ShapeSettings, JPH::ShapeSettings)
OPAQUE_WRAPPER(JPC_Shape, JPH::Shape)
OPAQUE_WRAPPER(JPC_Body, JPH::Body)

OPAQUE_WRAPPER(JPC_VertexList, JPH::VertexList)
DESTRUCTOR(JPC_VertexList)

OPAQUE_WRAPPER(JPC_IndexedTriangleList, JPH::IndexedTriangleList)
DESTRUCTOR(JPC_IndexedTriangleList)

OPAQUE_WRAPPER(JPC_String, JPH::String)
DESTRUCTOR(JPC_String)

OPAQUE_WRAPPER(JPC_BoxShapeSettings, JPH::BoxShapeSettings)
DESTRUCTOR(JPC_BoxShapeSettings)

OPAQUE_WRAPPER(JPC_SphereShapeSettings, JPH::SphereShapeSettings)
DESTRUCTOR(JPC_SphereShapeSettings)

LAYOUT_COMPATIBLE(JPC_BodyManager_DrawSettings, JPH::BodyManager::DrawSettings)

static auto to_jpc(JPH::BroadPhaseLayer in) { return in.GetValue(); }
static auto to_jph(JPC_BroadPhaseLayer in) { return JPH::BroadPhaseLayer(in); }

static JPC_Vec3 to_jpc(JPH::Vec3 in) {
	return JPC_Vec3{in.GetX(), in.GetY(), in.GetZ(), in.GetZ()};
}
static JPH::Vec3 to_jph(JPC_Vec3 in) {
	return JPH::Vec3(in.x, in.y, in.z);
}

static JPC_DVec3 to_jpc(JPH::DVec3 in) {
	return JPC_DVec3{in.GetX(), in.GetY(), in.GetZ(), in.GetZ()};
}
static JPH::DVec3 to_jph(JPC_DVec3 in) {
	return JPH::DVec3(in.x, in.y, in.z);
}

static JPC_Quat to_jpc(JPH::Quat in) {
	return JPC_Quat{in.GetX(), in.GetY(), in.GetZ(), in.GetW()};
}
static JPH::Quat to_jph(JPC_Quat in) {
	return JPH::Quat(in.x, in.y, in.z, in.w);
}

static JPC_Color to_jpc(JPH::Color in) {
	return JPC_Color{in.r, in.g, in.b, in.a};
}
static JPH::Color to_jph(JPC_Color in) {
	return JPH::Color(in.r, in.g, in.b, in.a);
}

static JPC_BodyID to_jpc(JPH::BodyID in) {
	return in.GetIndexAndSequenceNumber();
}
static JPH::BodyID to_jph(JPC_BodyID in) {
	return JPH::BodyID(in);
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
// VertexList == Array<Float3> == std::vector<Float3>

JPC_API JPC_VertexList* JPC_VertexList_new(const JPC_Float3* storage, size_t len) {
	const JPH::Float3* new_storage = (const JPH::Float3*)storage;
	return to_jpc(new JPH::VertexList(new_storage, new_storage + len));
}

////////////////////////////////////////////////////////////////////////////////
// IndexedTriangleList == Array<IndexedTriangle> == std::vector<IndexedTriangle>

JPC_API JPC_IndexedTriangleList* JPC_IndexedTriangleList_new(const JPC_IndexedTriangle* storage, size_t len) {
	const JPH::IndexedTriangle* new_storage = (const JPH::IndexedTriangle*)storage;
	return to_jpc(new JPH::IndexedTriangleList(new_storage, new_storage + len));
}

////////////////////////////////////////////////////////////////////////////////
// TempAllocatorImpl

JPC_API JPC_TempAllocatorImpl* JPC_TempAllocatorImpl_new(uint size) {
	return to_jpc(new JPH::TempAllocatorImpl(size));
}

////////////////////////////////////////////////////////////////////////////////
// JobSystemThreadPool

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

class JPC_BroadPhaseLayerInterfaceBridge final : public JPH::BroadPhaseLayerInterface {
public:
	explicit JPC_BroadPhaseLayerInterfaceBridge(const void *self, JPC_BroadPhaseLayerInterfaceFns fns) : self(self), fns(fns) {}

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

OPAQUE_WRAPPER(JPC_BroadPhaseLayerInterface, JPC_BroadPhaseLayerInterfaceBridge)
DESTRUCTOR(JPC_BroadPhaseLayerInterface)

JPC_API JPC_BroadPhaseLayerInterface* JPC_BroadPhaseLayerInterface_new(
	const void *self,
	JPC_BroadPhaseLayerInterfaceFns fns)
{
	return to_jpc(new JPC_BroadPhaseLayerInterfaceBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// ObjectVsBroadPhaseLayerFilter

class JPC_ObjectVsBroadPhaseLayerFilterBridge final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
	explicit JPC_ObjectVsBroadPhaseLayerFilterBridge(const void *self, JPC_ObjectVsBroadPhaseLayerFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
		return fns.ShouldCollide(self, inLayer1, to_jpc(inLayer2));
	}

private:
	const void* self;
	JPC_ObjectVsBroadPhaseLayerFilterFns fns;
};

OPAQUE_WRAPPER(JPC_ObjectVsBroadPhaseLayerFilter, JPC_ObjectVsBroadPhaseLayerFilterBridge)
DESTRUCTOR(JPC_ObjectVsBroadPhaseLayerFilter)

JPC_API JPC_ObjectVsBroadPhaseLayerFilter* JPC_ObjectVsBroadPhaseLayerFilter_new(
	const void *self,
	JPC_ObjectVsBroadPhaseLayerFilterFns fns)
{
	return to_jpc(new JPC_ObjectVsBroadPhaseLayerFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// JPC_ObjectLayerPairFilter

class JPC_ObjectLayerPairFilterBridge final : public JPH::ObjectLayerPairFilter {
public:
	explicit JPC_ObjectLayerPairFilterBridge(const void *self, JPC_ObjectLayerPairFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override {
		return fns.ShouldCollide(self, inLayer1, inLayer2);
	}

private:
	const void* self;
	JPC_ObjectLayerPairFilterFns fns;
};

OPAQUE_WRAPPER(JPC_ObjectLayerPairFilter, JPC_ObjectLayerPairFilterBridge)
DESTRUCTOR(JPC_ObjectLayerPairFilter)

JPC_API JPC_ObjectLayerPairFilter* JPC_ObjectLayerPairFilter_new(
	const void *self,
	JPC_ObjectLayerPairFilterFns fns)
{
	return to_jpc(new JPC_ObjectLayerPairFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// BodyManager::DrawSettings

JPC_API void JPC_BodyManager_DrawSettings_default(JPC_BodyManager_DrawSettings* object) {
	*object = to_jpc(JPH::BodyManager::DrawSettings());
}

////////////////////////////////////////////////////////////////////////////////
// DebugRendererSimple

class JPC_DebugRendererSimpleBridge final : public JPH::DebugRendererSimple {
public:
	explicit JPC_DebugRendererSimpleBridge(const void *self, JPC_DebugRendererSimpleFns fns) : self(self), fns(fns) {}

	virtual void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override {
		fns.DrawLine(self, to_jpc(inFrom), to_jpc(inTo), to_jpc(inColor));
	}

	virtual void DrawText3D(
		[[maybe_unused]] JPH::RVec3Arg inPosition,
		[[maybe_unused]] const std::string_view &inString,
		[[maybe_unused]] JPH::ColorArg inColor = JPH::Color::sWhite,
		[[maybe_unused]] float inHeight = 0.5f) override
	{
		// TODO
	}

private:
	const void* self;
	JPC_DebugRendererSimpleFns fns;
};

OPAQUE_WRAPPER(JPC_DebugRendererSimple, JPC_DebugRendererSimpleBridge)
DESTRUCTOR(JPC_DebugRendererSimple)

JPC_API JPC_DebugRendererSimple* JPC_DebugRendererSimple_new(
	const void *self,
	JPC_DebugRendererSimpleFns fns)
{
	return to_jpc(new JPC_DebugRendererSimpleBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// String

JPC_API const char* JPC_String_c_str(JPC_String* self) {
	return to_jph(self)->c_str();
}

////////////////////////////////////////////////////////////////////////////////
// Shape

JPC_API uint32_t JPC_Shape_GetRefCount(JPC_Shape* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_Shape_AddRef(JPC_Shape* self) {
	return to_jph(self)->AddRef();
}

JPC_API void JPC_Shape_Release(JPC_Shape* self) {
	return to_jph(self)->Release();
}

////////////////////////////////////////////////////////////////////////////////
// ShapeSettings

JPC_API bool JPC_ShapeSettings_Create(
	JPC_ShapeSettings* self,
	JPC_Shape** outShape,
	JPC_String** outError)
{
	auto res = to_jph(self)->Create();

	if (res.HasError()) {
		if (outError != nullptr) {
			JPH::String* created = new JPH::String(std::move(res.GetError()));
			*outError = to_jpc(created);
		}

		return false;
	} else {
		*outShape = to_jpc(res.Get());

		return true;
	}
}

////////////////////////////////////////////////////////////////////////////////
// ConvexShapeSettings

JPC_API float JPC_ConvexShapeSettings_GetDensity(JPC_ConvexShapeSettings* self) {
	return to_jph(self)->mDensity;
}

JPC_API void JPC_ConvexShapeSettings_SetDensity(JPC_ConvexShapeSettings* self, float inDensity) {
	to_jph(self)->SetDensity(inDensity);
}

////////////////////////////////////////////////////////////////////////////////
// BoxShapeSettings

JPC_API JPC_BoxShapeSettings* JPC_BoxShapeSettings_new(JPC_Vec3 inHalfExtent) {
	return to_jpc(new JPH::BoxShapeSettings(to_jph(inHalfExtent)));
}

////////////////////////////////////////////////////////////////////////////////
// SphereShapeSettings

JPC_API JPC_SphereShapeSettings* JPC_SphereShapeSettings_new(float inRadius) {
	auto settings = new JPH::SphereShapeSettings{};
	settings->mRadius = inRadius;

	return to_jpc(settings);
}

////////////////////////////////////////////////////////////////////////////////
// BodyCreationSettings

JPC_API void JPC_BodyCreationSettings_default(JPC_BodyCreationSettings* settings) {
	settings->Position = JPC_RVec3{0, 0, 0};
	settings->Rotation = JPC_Quat{0, 0, 0, 1};
	settings->LinearVelocity = JPC_Vec3{0, 0, 0};
	settings->AngularVelocity = JPC_Vec3{0, 0, 0};

	settings->UserData = 0;
	settings->ObjectLayer = 0;

	settings->MotionType = JPC_MOTION_TYPE_DYNAMIC;
	settings->AllowedDOFs = JPC_ALLOWED_DOFS_ALL;
}

////////////////////////////////////////////////////////////////////////////////
// Body

JPC_API JPC_BodyID JPC_Body_GetID(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetID());
}

JPC_API JPC_BodyType JPC_Body_GetBodyType(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetBodyType());
}

JPC_API bool JPC_Body_IsRigidBody(const JPC_Body* self) {
	return to_jph(self)->IsRigidBody();
}

JPC_API bool JPC_Body_IsSoftBody(const JPC_Body* self) {
	return to_jph(self)->IsSoftBody();
}

JPC_API bool JPC_Body_IsActive(const JPC_Body* self) {
	return to_jph(self)->IsActive();
}

JPC_API bool JPC_Body_IsStatic(const JPC_Body* self) {
	return to_jph(self)->IsStatic();
}

JPC_API bool JPC_Body_IsKinematic(const JPC_Body* self) {
	return to_jph(self)->IsKinematic();
}

JPC_API bool JPC_Body_IsDynamic(const JPC_Body* self) {
	return to_jph(self)->IsDynamic();
}

JPC_API bool JPC_Body_CanBeKinematicOrDynamic(const JPC_Body* self) {
	return to_jph(self)->CanBeKinematicOrDynamic();
}

JPC_API void JPC_Body_SetIsSensor(JPC_Body* self, bool inIsSensor) {
	return to_jph(self)->SetIsSensor(inIsSensor);
}

JPC_API bool JPC_Body_IsSensor(const JPC_Body* self) {
	return to_jph(self)->IsSensor();
}

JPC_API void JPC_Body_SetCollideKinematicVsNonDynamic(JPC_Body* self, bool inCollide) {
	return to_jph(self)->SetCollideKinematicVsNonDynamic(inCollide);
}

JPC_API bool JPC_Body_GetCollideKinematicVsNonDynamic(const JPC_Body* self) {
	return to_jph(self)->GetCollideKinematicVsNonDynamic();
}

JPC_API void JPC_Body_SetUseManifoldReduction(JPC_Body* self, bool inUseReduction) {
	return to_jph(self)->SetUseManifoldReduction(inUseReduction);
}

JPC_API bool JPC_Body_GetUseManifoldReduction(const JPC_Body* self) {
	return to_jph(self)->GetUseManifoldReduction();
}

JPC_API bool JPC_Body_GetUseManifoldReductionWithBody(const JPC_Body* self, const JPC_Body* inBody2) {
	return to_jph(self)->GetUseManifoldReductionWithBody(*to_jph(inBody2));
}

JPC_API void JPC_Body_SetApplyGyroscopicForce(JPC_Body* self, bool inApply) {
	return to_jph(self)->SetApplyGyroscopicForce(inApply);
}

JPC_API bool JPC_Body_GetApplyGyroscopicForce(const JPC_Body* self) {
	return to_jph(self)->GetApplyGyroscopicForce();
}

JPC_API void JPC_Body_SetEnhancedInternalEdgeRemoval(JPC_Body* self, bool inApply) {
	return to_jph(self)->SetEnhancedInternalEdgeRemoval(inApply);
}

JPC_API bool JPC_Body_GetEnhancedInternalEdgeRemoval(const JPC_Body* self) {
	return to_jph(self)->GetEnhancedInternalEdgeRemoval();
}

JPC_API bool JPC_Body_GetEnhancedInternalEdgeRemovalWithBody(const JPC_Body* self, const JPC_Body* inBody2) {
	return to_jph(self)->GetEnhancedInternalEdgeRemovalWithBody(*to_jph(inBody2));
}

JPC_API JPC_MotionType JPC_Body_GetMotionType(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetMotionType());
}

JPC_API void JPC_Body_SetMotionType(JPC_Body* self, JPC_MotionType inMotionType) {
	return to_jph(self)->SetMotionType(to_jph(inMotionType));
}

JPC_API JPC_BroadPhaseLayer JPC_Body_GetBroadPhaseLayer(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetBroadPhaseLayer());
}

JPC_API JPC_ObjectLayer JPC_Body_GetObjectLayer(const JPC_Body* self) {
	return to_jph(self)->GetObjectLayer();
}

// JPC_API const CollisionGroup & JPC_Body_GetCollisionGroup(const JPC_Body* self);
// JPC_API CollisionGroup & JPC_Body_GetCollisionGroup(JPC_Body* self);
// JPC_API void JPC_Body_SetCollisionGroup(JPC_Body* self, const CollisionGroup &inGroup);

JPC_API bool JPC_Body_GetAllowSleeping(const JPC_Body* self) {
	return to_jph(self)->GetAllowSleeping();
}

JPC_API void JPC_Body_SetAllowSleeping(JPC_Body* self, bool inAllow) {
	return to_jph(self)->SetAllowSleeping(inAllow);
}

JPC_API void JPC_Body_ResetSleepTimer(JPC_Body* self) {
	return to_jph(self)->ResetSleepTimer();
}

JPC_API float JPC_Body_GetFriction(const JPC_Body* self) {
	return to_jph(self)->GetFriction();
}

JPC_API void JPC_Body_SetFriction(JPC_Body* self, float inFriction) {
	return to_jph(self)->SetFriction(inFriction);
}

JPC_API float JPC_Body_GetRestitution(const JPC_Body* self) {
	return to_jph(self)->GetRestitution();
}

JPC_API void JPC_Body_SetRestitution(JPC_Body* self, float inRestitution) {
	return to_jph(self)->SetRestitution(inRestitution);
}

JPC_API JPC_Vec3 JPC_Body_GetLinearVelocity(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetLinearVelocity());
}

JPC_API void JPC_Body_SetLinearVelocity(JPC_Body* self, JPC_Vec3 inLinearVelocity) {
	return to_jph(self)->SetLinearVelocity(to_jph(inLinearVelocity));
}

JPC_API void JPC_Body_SetLinearVelocityClamped(JPC_Body* self, JPC_Vec3 inLinearVelocity) {
	return to_jph(self)->SetLinearVelocityClamped(to_jph(inLinearVelocity));
}

JPC_API JPC_Vec3 JPC_Body_GetAngularVelocity(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAngularVelocity());
}

JPC_API void JPC_Body_SetAngularVelocity(JPC_Body* self, JPC_Vec3 inAngularVelocity) {
	return to_jph(self)->SetAngularVelocity(to_jph(inAngularVelocity));
}

JPC_API void JPC_Body_SetAngularVelocityClamped(JPC_Body* self, JPC_Vec3 inAngularVelocity) {
	return to_jph(self)->SetAngularVelocityClamped(to_jph(inAngularVelocity));
}

JPC_API JPC_Vec3 JPC_Body_GetPointVelocityCOM(const JPC_Body* self, JPC_Vec3 inPointRelativeToCOM) {
	return to_jpc(to_jph(self)->GetPointVelocityCOM(to_jph(inPointRelativeToCOM)));
}

JPC_API JPC_Vec3 JPC_Body_GetPointVelocity(const JPC_Body* self, JPC_RVec3 inPoint) {
	return to_jpc(to_jph(self)->GetPointVelocity(to_jph(inPoint)));
}

// JPC_API void JPC_Body_AddForce(JPC_Body* self, JPC_Vec3 inForce);
// JPC_API void JPC_Body_AddForce(JPC_Body* self, JPC_Vec3 inForce, JPC_RVec3 inPosition);

JPC_API void JPC_Body_AddTorque(JPC_Body* self, JPC_Vec3 inTorque) {
	return to_jph(self)->AddTorque(to_jph(inTorque));
}

JPC_API JPC_Vec3 JPC_Body_GetAccumulatedForce(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAccumulatedForce());
}

JPC_API JPC_Vec3 JPC_Body_GetAccumulatedTorque(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAccumulatedTorque());
}

JPC_API void JPC_Body_ResetForce(JPC_Body* self) {
	return to_jph(self)->ResetForce();
}

JPC_API void JPC_Body_ResetTorque(JPC_Body* self) {
	return to_jph(self)->ResetTorque();
}

JPC_API void JPC_Body_ResetMotion(JPC_Body* self) {
	return to_jph(self)->ResetMotion();
}

// JPC_API Mat44 JPC_Body_GetInverseInertia(const JPC_Body* self);

// JPC_API void JPC_Body_AddImpulse(JPC_Body* self, JPC_Vec3 inImpulse);
// JPC_API void JPC_Body_AddImpulse(JPC_Body* self, JPC_Vec3 inImpulse, JPC_RVec3 inPosition);

JPC_API void JPC_Body_AddAngularImpulse(JPC_Body* self, JPC_Vec3 inAngularImpulse) {
	return to_jph(self)->AddAngularImpulse(to_jph(inAngularImpulse));
}

JPC_API void JPC_Body_MoveKinematic(JPC_Body* self, JPC_RVec3 inTargetPosition, JPC_Quat inTargetRotation, float inDeltaTime) {
	return to_jph(self)->MoveKinematic(to_jph(inTargetPosition), to_jph(inTargetRotation), inDeltaTime);
}

JPC_API bool JPC_Body_ApplyBuoyancyImpulse(JPC_Body* self, JPC_RVec3 inSurfacePosition, JPC_Vec3 inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, JPC_Vec3 inFluidVelocity, JPC_Vec3 inGravity, float inDeltaTime) {
	return to_jph(self)->ApplyBuoyancyImpulse(to_jph(inSurfacePosition), to_jph(inSurfaceNormal), inBuoyancy, inLinearDrag, inAngularDrag, to_jph(inFluidVelocity), to_jph(inGravity), inDeltaTime);
}

JPC_API bool JPC_Body_IsInBroadPhase(const JPC_Body* self) {
	return to_jph(self)->IsInBroadPhase();
}

JPC_API bool JPC_Body_IsCollisionCacheInvalid(const JPC_Body* self) {
	return to_jph(self)->IsCollisionCacheInvalid();
}

JPC_API const JPC_Shape* JPC_Body_GetShape(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetShape());
}

JPC_API JPC_RVec3 JPC_Body_GetPosition(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetPosition());
}

JPC_API JPC_Quat JPC_Body_GetRotation(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetRotation());
}

// JPC_API RMat44 JPC_Body_GetWorldTransform(const JPC_Body* self);

JPC_API JPC_RVec3 JPC_Body_GetCenterOfMassPosition(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetCenterOfMassPosition());
}

// JPC_API RMat44 JPC_Body_GetCenterOfMassTransform(const JPC_Body* self);
// JPC_API RMat44 JPC_Body_GetInverseCenterOfMassTransform(const JPC_Body* self);
// JPC_API const AABox & JPC_Body_GetWorldSpaceBounds(const JPC_Body* self);
// JPC_API const MotionProperties *JPC_Body_GetMotionProperties(const JPC_Body* self)
// JPC_API MotionProperties * JPC_Body_GetMotionProperties(JPC_Body* self);
// JPC_API const MotionProperties *JPC_Body_GetMotionPropertiesUnchecked(const JPC_Body* self)
// JPC_API MotionProperties * JPC_Body_GetMotionPropertiesUnchecked(JPC_Body* self);

JPC_API uint64_t JPC_Body_GetUserData(const JPC_Body* self) {
	return to_jph(self)->GetUserData();
}

JPC_API void JPC_Body_SetUserData(JPC_Body* self, uint64_t inUserData) {
	return to_jph(self)->SetUserData(inUserData);
}

// JPC_API JPC_Vec3 JPC_Body_GetWorldSpaceSurfaceNormal(const JPC_Body* self, const SubShapeID &inSubShapeID, JPC_RVec3 inPosition);
// JPC_API TransformedShape JPC_Body_GetTransformedShape(const JPC_Body* self);
// JPC_API BodyCreationSettings JPC_Body_GetBodyCreationSettings(const JPC_Body* self);
// JPC_API SoftBodyCreationSettings JPC_Body_GetSoftBodyCreationSettings(const JPC_Body* self);

////////////////////////////////////////////////////////////////////////////////
// BodyInterface

JPC_API JPC_Body* JPC_BodyInterface_CreateBody(JPC_BodyInterface* self, JPC_BodyCreationSettings* inSettingsC) {
	JPH::BodyCreationSettings inSettings{};
	inSettings.mPosition = to_jph(inSettingsC->Position);
	inSettings.mRotation = to_jph(inSettingsC->Rotation);
	inSettings.mLinearVelocity = to_jph(inSettingsC->LinearVelocity);
	inSettings.mAngularVelocity = to_jph(inSettingsC->AngularVelocity);

	inSettings.mUserData = inSettingsC->UserData;
	inSettings.mObjectLayer = inSettingsC->ObjectLayer;

	inSettings.mMotionType = to_jph(inSettingsC->MotionType);
	inSettings.mAllowedDOFs = to_jph(inSettingsC->AllowedDOFs);

	inSettings.SetShape(to_jph(inSettingsC->Shape));

	JPH::Body* body = to_jph(self)->CreateBody(inSettings);
	return to_jpc(body);
}

JPC_API void JPC_BodyInterface_AddBody(JPC_BodyInterface* self, JPC_BodyID inBodyID, JPC_Activation inActivationMode) {
	to_jph(self)->AddBody(to_jph(inBodyID), to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_RemoveBody(JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	to_jph(self)->RemoveBody(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_DestroyBody(JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	to_jph(self)->DestroyBody(to_jph(inBodyID));
}

JPC_API bool JPC_BodyInterface_IsActive(JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	return to_jph(self)->IsActive(to_jph(inBodyID));
}

JPC_API JPC_RVec3 JPC_BodyInterface_GetCenterOfMassPosition(JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetCenterOfMassPosition(to_jph(inBodyID)));
}

JPC_API JPC_Vec3 JPC_BodyInterface_GetLinearVelocity(JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetLinearVelocity(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_SetLinearVelocity(JPC_BodyInterface* self, JPC_BodyID inBodyID, JPC_Vec3 inVelocity) {
	return to_jph(self)->SetLinearVelocity(to_jph(inBodyID), to_jph(inVelocity));
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsSystem

JPC_API JPC_PhysicsSystem* JPC_PhysicsSystem_new() {
	return to_jpc(new JPH::PhysicsSystem());
}

JPC_API void JPC_PhysicsSystem_Init(
	JPC_PhysicsSystem* self,
	uint inMaxBodies,
	uint inNumBodyMutexes,
	uint inMaxBodyPairs,
	uint inMaxContactConstraints,
	JPC_BroadPhaseLayerInterface* inBroadPhaseLayerInterface,
	JPC_ObjectVsBroadPhaseLayerFilter* inObjectVsBroadPhaseLayerFilter,
	JPC_ObjectLayerPairFilter* inObjectLayerPairFilter)
{
	JPC_BroadPhaseLayerInterfaceBridge* impl_inBroadPhaseLayerInterface = to_jph(inBroadPhaseLayerInterface);
	JPC_ObjectVsBroadPhaseLayerFilterBridge* impl_inObjectVsBroadPhaseLayerFilter = to_jph(inObjectVsBroadPhaseLayerFilter);
	JPC_ObjectLayerPairFilterBridge* impl_inObjectLayerPairFilter = to_jph(inObjectLayerPairFilter);

	to_jph(self)->Init(
		inMaxBodies,
		inNumBodyMutexes,
		inMaxBodyPairs,
		inMaxContactConstraints,
		*impl_inBroadPhaseLayerInterface,
		*impl_inObjectVsBroadPhaseLayerFilter,
		*impl_inObjectLayerPairFilter);
}

JPC_API JPC_BodyInterface* JPC_PhysicsSystem_GetBodyInterface(JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBodyInterface());
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

JPC_API void JPC_PhysicsSystem_DrawBodies(
	JPC_PhysicsSystem* self,
	JPC_BodyManager_DrawSettings* inSettings,
	JPC_DebugRendererSimple* inRenderer,
	[[maybe_unused]] const void* inBodyFilter)
{
	to_jph(self)->DrawBodies(to_jph(*inSettings), to_jph(inRenderer), nullptr);
}