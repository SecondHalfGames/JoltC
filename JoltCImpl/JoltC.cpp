#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <Jolt/Renderer/DebugRendererSimple.h>

#include <JoltC/JoltC.h>

#define JPC_IMPL static

#define OPAQUE_WRAPPER(c_type, cpp_type) \
	static c_type* to_jpc(cpp_type *in) { return reinterpret_cast<c_type*>(in); } \
	static const c_type* to_jpc(const cpp_type *in) { return reinterpret_cast<const c_type*>(in); } \
	static cpp_type* to_jph(c_type *in) { return reinterpret_cast<cpp_type*>(in); } \
	static const cpp_type* to_jph(const c_type *in) { return reinterpret_cast<const cpp_type*>(in); } \
	static cpp_type** to_jph(c_type **in) { return reinterpret_cast<cpp_type**>(in); }

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
	static c_type* to_jpc(cpp_type* in) { \
		return reinterpret_cast<c_type*>(in); \
	} \
	static cpp_type* to_jph(c_type* in) { \
		return reinterpret_cast<cpp_type*>(in); \
	} \
	static const c_type* to_jpc(const cpp_type* in) { \
		return reinterpret_cast<const c_type*>(in); \
	} \
	static const cpp_type* to_jph(const c_type* in) { \
		return reinterpret_cast<const cpp_type*>(in); \
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
ENUM_CONVERSION(JPC_MotionQuality, JPH::EMotionQuality)
ENUM_CONVERSION(JPC_OverrideMassProperties, JPH::EOverrideMassProperties)
ENUM_CONVERSION(JPC_ShapeType, JPH::EShapeType)
ENUM_CONVERSION(JPC_ShapeSubType, JPH::EShapeSubType)
ENUM_CONVERSION(JPC_SpringMode, JPH::ESpringMode)
ENUM_CONVERSION(JPC_MotorState, JPH::EMotorState)
ENUM_CONVERSION(JPC_ValidateResult, JPH::ValidateResult)

OPAQUE_WRAPPER(JPC_PhysicsSystem, JPH::PhysicsSystem)
DESTRUCTOR(JPC_PhysicsSystem)

OPAQUE_WRAPPER(JPC_BodyInterface, JPH::BodyInterface)
OPAQUE_WRAPPER(JPC_BodyLockInterface, JPH::BodyLockInterface)
OPAQUE_WRAPPER(JPC_BodyLockRead, JPH::BodyLockRead)
OPAQUE_WRAPPER(JPC_BodyLockWrite, JPH::BodyLockWrite)
OPAQUE_WRAPPER(JPC_BodyLockMultiRead, JPH::BodyLockMultiRead)
OPAQUE_WRAPPER(JPC_BodyLockMultiWrite, JPH::BodyLockMultiWrite)
OPAQUE_WRAPPER(JPC_NarrowPhaseQuery, JPH::NarrowPhaseQuery)

OPAQUE_WRAPPER(JPC_TempAllocatorImpl, JPH::TempAllocatorImpl)
DESTRUCTOR(JPC_TempAllocatorImpl)

OPAQUE_WRAPPER(JPC_JobSystem, JPH::JobSystem)
DESTRUCTOR(JPC_JobSystem)

OPAQUE_WRAPPER(JPC_JobSystemThreadPool, JPH::JobSystemThreadPool)
DESTRUCTOR(JPC_JobSystemThreadPool)

OPAQUE_WRAPPER(JPC_JobSystemSingleThreaded, JPH::JobSystemSingleThreaded)
DESTRUCTOR(JPC_JobSystemSingleThreaded)

OPAQUE_WRAPPER(JPC_Shape, JPH::Shape)
OPAQUE_WRAPPER(JPC_CompoundShape, JPH::CompoundShape)
OPAQUE_WRAPPER(JPC_Body, JPH::Body)

OPAQUE_WRAPPER(JPC_VertexList, JPH::VertexList)
DESTRUCTOR(JPC_VertexList)

OPAQUE_WRAPPER(JPC_IndexedTriangleList, JPH::IndexedTriangleList)
DESTRUCTOR(JPC_IndexedTriangleList)

OPAQUE_WRAPPER(JPC_String, JPH::String)
DESTRUCTOR(JPC_String)

LAYOUT_COMPATIBLE(JPC_BodyManager_DrawSettings, JPH::BodyManager::DrawSettings)

LAYOUT_COMPATIBLE(JPC_ShapeCastSettings, JPH::ShapeCastSettings)
LAYOUT_COMPATIBLE(JPC_CollideShapeSettings, JPH::CollideShapeSettings)

LAYOUT_COMPATIBLE(JPC_BodyID, JPH::BodyID)

static auto to_jpc(JPH::BroadPhaseLayer in) { return in.GetValue(); }
static auto to_jph(JPC_BroadPhaseLayer in) { return JPH::BroadPhaseLayer(in); }

static JPC_Vec2 to_jpc(JPH::Vector<2> in) {
	return JPC_Vec2{in[0], in[1]};
}
static JPH::Vector<2> to_jph(JPC_Vec2 in) {
	JPH::Vector<2> out;
	out[0] = in.x;
	out[1] = in.y;
	return out;
}

static JPC_Vec3 to_jpc(JPH::Vec3 in) {
	return JPC_Vec3{in.GetX(), in.GetY(), in.GetZ(), in.GetZ()};
}
static JPH::Vec3 to_jph(JPC_Vec3 in) {
	return JPH::Vec3(in.x, in.y, in.z);
}

static JPC_Vec4 to_jpc(JPH::Vec4 in) {
	return JPC_Vec4{in.GetX(), in.GetY(), in.GetZ(), in.GetW()};
}
static JPH::Vec4 to_jph(JPC_Vec4 in) {
	return JPH::Vec4(in.x, in.y, in.z, in.w);
}

static JPH::Array<JPH::Vec3> to_jph(const JPC_Vec3* src, size_t n) {
	JPH::Array<JPH::Vec3> vec;
	vec.resize(n);

	if (src != nullptr) {
		memcpy(vec.data(), src, n * sizeof(*src));
	}

	return vec;
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

static JPC_Mat44 to_jpc(JPH::Mat44 in) {
	JPC_Mat44 out;
	in.StoreFloat4x4(reinterpret_cast<JPH::Float4*>(&out));
	return out;
}
static JPH::Mat44 to_jph(JPC_Mat44 in) {
	return JPH::Mat44::sLoadFloat4x4Aligned(reinterpret_cast<const JPH::Float4*>(&in));
}

static JPC_DMat44 to_jpc(JPH::DMat44 in) {
	JPC_DMat44 out;
	out.col[0] = to_jpc(in.GetColumn4(0));
	out.col[1] = to_jpc(in.GetColumn4(1));
	out.col[2] = to_jpc(in.GetColumn4(2));
	out.col3 = to_jpc(in.GetTranslation());
	return out;
}
static JPH::DMat44 to_jph(JPC_DMat44 in) {
	JPH::DVec3 col3 = to_jph(in.col3);

	JPH::DMat44 out(
		to_jph(in.col[0]),
		to_jph(in.col[1]),
		to_jph(in.col[2]),
		col3);
	return out;
}

static JPC_Color to_jpc(JPH::Color in) {
	return JPC_Color{in.r, in.g, in.b, in.a};
}
static JPH::Color to_jph(JPC_Color in) {
	return JPH::Color(in.r, in.g, in.b, in.a);
}

static JPH::RayCast to_jph(JPC_RayCast in) {
	return JPH::RayCast(to_jph(in.Origin), to_jph(in.Direction));
}

static JPH::RRayCast to_jph(JPC_RRayCast in) {
	return JPH::RRayCast(to_jph(in.Origin), to_jph(in.Direction));
}

static JPH::RShapeCast to_jph(JPC_RShapeCast in) {
	return JPH::RShapeCast(
		to_jph(in.Shape),
		to_jph(in.Scale),
		to_jph(in.CenterOfMassStart),
		to_jph(in.Direction));
}

static JPH::SubShapeID JPC_SubShapeID_to_jph(JPC_SubShapeID in) {
	JPH::SubShapeID out;
	out.SetValue(in);
	return out;
}

static JPC_SubShapeID to_jpc(JPH::SubShapeID in) {
	return in.GetValue();
}

static JPC_RayCastResult to_jpc(JPH::RayCastResult in) {
	JPC_RayCastResult out{0};
	out.BodyID = to_jpc(in.mBodyID);
	out.Fraction = in.mFraction;
	out.SubShapeID2 = to_jpc(in.mSubShapeID2);

	return out;
}

JPC_IMPL JPC_ShapeCastResult JPC_ShapeCastResult_to_jpc(JPH::ShapeCastResult in) {
	JPC_ShapeCastResult out{};
	// CollideShapeResult
	out.ContactPointOn1 = to_jpc(in.mContactPointOn1);
	out.ContactPointOn2 = to_jpc(in.mContactPointOn2);
	out.PenetrationAxis = to_jpc(in.mPenetrationAxis);
	out.PenetrationDepth = in.mPenetrationDepth;
	out.SubShapeID1 = to_jpc(in.mSubShapeID1);
	out.SubShapeID2 = to_jpc(in.mSubShapeID2);
	out.BodyID2 = to_jpc(in.mBodyID2);
	// Face Shape1Face;
	// Face Shape2Face;

	// ShapeCastResult
	out.Fraction = in.mFraction;
	out.IsBackFaceHit = in.mIsBackFaceHit;

	return out;
}

JPC_IMPL JPH::ShapeCastSettings JPC_ShapeCastSettings_to_jph(JPC_ShapeCastSettings in) {
	JPH::ShapeCastSettings out{};

	// JPH::CollideSettingsBase
	// EActiveEdgeMode ActiveEdgeMode;
	// ECollectFacesMode CollectFacesMode;
	out.mCollisionTolerance = in.CollisionTolerance;
	out.mPenetrationTolerance = in.PenetrationTolerance;
	out.mActiveEdgeMovementDirection = to_jph(in.ActiveEdgeMovementDirection);

	// JPH::ShapeCastSettings
	out.mBackFaceModeTriangles = static_cast<JPH::EBackFaceMode>(in.BackFaceModeTriangles);
	out.mBackFaceModeConvex = static_cast<JPH::EBackFaceMode>(in.BackFaceModeConvex);
	out.mUseShrunkenShapeAndConvexRadius = in.UseShrunkenShapeAndConvexRadius;
	out.mReturnDeepestPoint = in.ReturnDeepestPoint;

	return out;
}

JPC_IMPL JPC_CollideShapeResult JPC_CollideShapeResult_to_jpc(JPH::CollideShapeResult in) {
	JPC_CollideShapeResult out{};
	// CollideShapeResult
	out.ContactPointOn1 = to_jpc(in.mContactPointOn1);
	out.ContactPointOn2 = to_jpc(in.mContactPointOn2);
	out.PenetrationAxis = to_jpc(in.mPenetrationAxis);
	out.PenetrationDepth = in.mPenetrationDepth;
	out.SubShapeID1 = to_jpc(in.mSubShapeID1);
	out.SubShapeID2 = to_jpc(in.mSubShapeID2);
	out.BodyID2 = to_jpc(in.mBodyID2);
	// Face Shape1Face;
	// Face Shape2Face;

	return out;
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
// JobSystemSingleThreaded

JPC_API JPC_JobSystemSingleThreaded* JPC_JobSystemSingleThreaded_new(uint inMaxJobs) {
	return to_jpc(new JPH::JobSystemSingleThreaded(inMaxJobs));
}

////////////////////////////////////////////////////////////////////////////////
// CollisionGroup

JPC_IMPL JPC_CollisionGroup JPC_CollisionGroup_to_jpc(const JPH::CollisionGroup* input);

class JPC_GroupFilterBridge final : public JPH::GroupFilter {
public:
	explicit JPC_GroupFilterBridge(const void *self, JPC_GroupFilterFns fns) : self(self), fns(fns) {}

	bool CanCollide(const JPH::CollisionGroup &inGroup1, const JPH::CollisionGroup &inGroup2) const override {
		JPC_CollisionGroup jpcGroup1 = JPC_CollisionGroup_to_jpc(&inGroup1);
		JPC_CollisionGroup jpcGroup2 = JPC_CollisionGroup_to_jpc(&inGroup2);

		return fns.CanCollide(self, &jpcGroup1, &jpcGroup2);
	}

	void SaveBinaryState([[maybe_unused]] JPH::StreamOut &inStream) const override {}
	void RestoreBinaryState([[maybe_unused]] JPH::StreamIn &inStream) override {}

private:
	const void* self;
	JPC_GroupFilterFns fns;
};

OPAQUE_WRAPPER(JPC_GroupFilter, JPC_GroupFilterBridge)
DESTRUCTOR(JPC_GroupFilter)

JPC_IMPL JPH::CollisionGroup JPC_CollisionGroup_to_jph(const JPC_CollisionGroup* self) {
	const JPC_GroupFilterBridge* filter_group = to_jph(self->GroupFilter);

	JPH::CollisionGroup group(filter_group, self->GroupID, self->SubGroupID);
	return group;
}

JPC_IMPL JPC_CollisionGroup JPC_CollisionGroup_to_jpc(const JPH::CollisionGroup* input) {
	JPC_CollisionGroup group{};
	group.GroupFilter; // NOTE: This member doesn't matter for callers of this function
	group.GroupID = input->GetGroupID();
	group.SubGroupID = input->GetSubGroupID();
	return group;
}

JPC_API JPC_GroupFilter* JPC_GroupFilter_new(
	const void *self,
	JPC_GroupFilterFns fns)
{
	return to_jpc(new JPC_GroupFilterBridge(self, fns));
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
// BroadPhaseLayerFilter

class JPC_BroadPhaseLayerFilterBridge final : public JPH::BroadPhaseLayerFilter {
public:
	explicit JPC_BroadPhaseLayerFilterBridge(const void *self, JPC_BroadPhaseLayerFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override {
		return fns.ShouldCollide(self, to_jpc(inLayer));
	}

private:
	const void* self;
	JPC_BroadPhaseLayerFilterFns fns;
};

OPAQUE_WRAPPER(JPC_BroadPhaseLayerFilter, JPC_BroadPhaseLayerFilterBridge)
DESTRUCTOR(JPC_BroadPhaseLayerFilter)

JPC_API JPC_BroadPhaseLayerFilter* JPC_BroadPhaseLayerFilter_new(
	const void *self,
	JPC_BroadPhaseLayerFilterFns fns)
{
	return to_jpc(new JPC_BroadPhaseLayerFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// ObjectLayerFilter

class JPC_ObjectLayerFilterBridge final : public JPH::ObjectLayerFilter {
public:
	explicit JPC_ObjectLayerFilterBridge(const void *self, JPC_ObjectLayerFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer) const override {
		return fns.ShouldCollide(self, inLayer);
	}

private:
	const void* self;
	JPC_ObjectLayerFilterFns fns;
};

OPAQUE_WRAPPER(JPC_ObjectLayerFilter, JPC_ObjectLayerFilterBridge)
DESTRUCTOR(JPC_ObjectLayerFilter)

JPC_API JPC_ObjectLayerFilter* JPC_ObjectLayerFilter_new(
	const void *self,
	JPC_ObjectLayerFilterFns fns)
{
	return to_jpc(new JPC_ObjectLayerFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// BodyFilter

class JPC_BodyFilterBridge final : public JPH::BodyFilter {
public:
	explicit JPC_BodyFilterBridge(const void *self, JPC_BodyFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(const JPH::BodyID &inBodyID) const override {
		return fns.ShouldCollide(self, to_jpc(inBodyID));
	}

	virtual bool ShouldCollideLocked(const JPH::Body &inBody) const override {
		return fns.ShouldCollideLocked(self, to_jpc(&inBody));
	}

private:
	const void* self;
	JPC_BodyFilterFns fns;
};

OPAQUE_WRAPPER(JPC_BodyFilter, JPC_BodyFilterBridge)
DESTRUCTOR(JPC_BodyFilter)

JPC_API JPC_BodyFilter* JPC_BodyFilter_new(
	const void *self,
	JPC_BodyFilterFns fns)
{
	return to_jpc(new JPC_BodyFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// ShapeFilter

class JPC_ShapeFilterBridge final : public JPH::ShapeFilter {
public:
	explicit JPC_ShapeFilterBridge(const void *self, JPC_ShapeFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeIDOfShape2) const override {
		if (fns.ShouldCollide == nullptr) {
			return true;
		}

		return fns.ShouldCollide(self, to_jpc(inShape2), to_jpc(inSubShapeIDOfShape2));
	}

	virtual bool ShouldCollide(
		const JPH::Shape *inShape1, const JPH::SubShapeID &inSubShapeIDOfShape1,
		const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeIDOfShape2) const override
	{
		if (fns.ShouldCollideTwoShapes == nullptr) {
			return true;
		}

		return fns.ShouldCollideTwoShapes(self,
			to_jpc(inShape1), to_jpc(inSubShapeIDOfShape1),
			to_jpc(inShape2), to_jpc(inSubShapeIDOfShape2));
	}

private:
	const void* self;
	JPC_ShapeFilterFns fns;
};

OPAQUE_WRAPPER(JPC_ShapeFilter, JPC_ShapeFilterBridge)
DESTRUCTOR(JPC_ShapeFilter)

JPC_API JPC_ShapeFilter* JPC_ShapeFilter_new(
	const void *self,
	JPC_ShapeFilterFns fns)
{
	return to_jpc(new JPC_ShapeFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// SimShapeFilter

class JPC_SimShapeFilterBridge final : public JPH::SimShapeFilter {
public:
	explicit JPC_SimShapeFilterBridge(const void *self, JPC_SimShapeFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(
		const JPH::Body &inBody1, const JPH::Shape *inShape1, const JPH::SubShapeID &inSubShapeIDOfShape1,
		const JPH::Body &inBody2, const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeIDOfShape2) const override
	{
		if (fns.ShouldCollide == nullptr) {
			return true;
		}

		return fns.ShouldCollide(self,
			to_jpc(&inBody1), to_jpc(inShape1), to_jpc(inSubShapeIDOfShape1),
			to_jpc(&inBody2), to_jpc(inShape2), to_jpc(inSubShapeIDOfShape2));
	}

private:
	const void* self;
	JPC_SimShapeFilterFns fns;
};

OPAQUE_WRAPPER(JPC_SimShapeFilter, JPC_SimShapeFilterBridge)
DESTRUCTOR(JPC_SimShapeFilter)

JPC_API JPC_SimShapeFilter* JPC_SimShapeFilter_new(
	const void *self,
	JPC_SimShapeFilterFns fns)
{
	return to_jpc(new JPC_SimShapeFilterBridge(self, fns));
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
// JPC_ContactListener

class JPC_ContactListenerBridge final : public JPH::ContactListener {
public:
	explicit JPC_ContactListenerBridge(void *self, JPC_ContactListenerFns fns) : self(self), fns(fns) {}

	JPH::ValidateResult OnContactValidate(
		const JPH::Body &inBody1,
		const JPH::Body &inBody2,
		JPH::RVec3Arg inBaseOffset,
		const JPH::CollideShapeResult &inCollisionResult) override
	{
		if (fns.OnContactValidate != nullptr) {
			JPC_CollideShapeResult collisionResult = JPC_CollideShapeResult_to_jpc(inCollisionResult);
			return to_jph(fns.OnContactValidate(self, to_jpc(&inBody1), to_jpc(&inBody2), to_jpc(inBaseOffset), &collisionResult));
		}
		return ContactListener::OnContactValidate(inBody1, inBody2, inBaseOffset, inCollisionResult);
	}

	void OnContactAdded(
		const JPH::Body &inBody1,
		const JPH::Body &inBody2,
		const JPH::ContactManifold &inManifold,
		JPH::ContactSettings &ioSettings) override
	{
		if (fns.OnContactAdded != nullptr) {
			const auto* cManifold = reinterpret_cast<const JPC_ContactManifold*>(&inManifold);
			auto* cSettings = reinterpret_cast<JPC_ContactSettings*>(&ioSettings);

			fns.OnContactAdded(self, to_jpc(&inBody1), to_jpc(&inBody2), cManifold, cSettings);
		}
	}

	void OnContactPersisted(
		const JPH::Body &inBody1,
		const JPH::Body &inBody2,
		const JPH::ContactManifold &inManifold,
		JPH::ContactSettings &ioSettings) override
	{
		if (fns.OnContactPersisted != nullptr) {
			const auto* cManifold = reinterpret_cast<const JPC_ContactManifold*>(&inManifold);
			auto* cSettings = reinterpret_cast<JPC_ContactSettings*>(&ioSettings);

			fns.OnContactPersisted(self, to_jpc(&inBody1), to_jpc(&inBody2), cManifold, cSettings);
		}
	}

	void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override {
		if (fns.OnContactRemoved != nullptr) {
			const auto* cSubShapePair = reinterpret_cast<const JPC_SubShapeIDPair*>(&inSubShapePair);

			fns.OnContactRemoved(self, cSubShapePair);
		}
	}

private:
	void* self;
	JPC_ContactListenerFns fns;
};

OPAQUE_WRAPPER(JPC_ContactListener, JPC_ContactListenerBridge)
DESTRUCTOR(JPC_ContactListener)

JPC_API JPC_ContactListener* JPC_ContactListener_new(
	void *self,
	JPC_ContactListenerFns fns)
{
	return to_jpc(new JPC_ContactListenerBridge(self, fns));
}

JPC_API void JPC_EstimateCollisionResponse(
	const JPC_Body* inBody1,
	const JPC_Body* inBody2,
	const JPC_ContactManifold* inManifold,
	JPC_CollisionEstimationResult* outResult,
	float inCombinedFriction,
	float inCombinedRestitution,
	float inMinVelocityForRestitution,	///< = 1.0f
	uint inNumIterations				///< = 10
) {
	const auto* jphManifold = reinterpret_cast<const JPH::ContactManifold*>(inManifold);
	auto* jphResult = reinterpret_cast<JPH::CollisionEstimationResult*>(outResult);

	JPH::EstimateCollisionResponse(
		*to_jph(inBody1),
		*to_jph(inBody2),
		*jphManifold,
		*jphResult,
		inCombinedFriction,
		inCombinedRestitution,
		inMinVelocityForRestitution,
		inNumIterations);
}

////////////////////////////////////////////////////////////////////////////////
// JPC_CastShapeCollector

class JPC_CastShapeCollectorBridge;
OPAQUE_WRAPPER(JPC_CastShapeCollector, JPC_CastShapeCollectorBridge)

class JPC_CastShapeCollectorBridge final : public JPH::CastShapeCollector {
	using ResultType = JPH::ShapeCastResult;

public:
	explicit JPC_CastShapeCollectorBridge(void *self, JPC_CastShapeCollectorFns fns) : self(self), fns(fns) {}

	void Reset() override {
		JPH::CastShapeCollector::Reset();

		if (fns.Reset != nullptr) {
			fns.Reset(self);
		}
	}

	void AddHit(const ResultType &inResult) override {
		JPC_ShapeCastResult result = JPC_ShapeCastResult_to_jpc(inResult);
		JPC_CastShapeCollector *base = to_jpc(this);

		fns.AddHit(self, base, &result);
	}

private:
	void* self;
	JPC_CastShapeCollectorFns fns;
};

DESTRUCTOR(JPC_CastShapeCollector)

JPC_API JPC_CastShapeCollector* JPC_CastShapeCollector_new(
	void *self,
	JPC_CastShapeCollectorFns fns)
{
	return to_jpc(new JPC_CastShapeCollectorBridge(self, fns));
}

JPC_API void JPC_CastShapeCollector_UpdateEarlyOutFraction(JPC_CastShapeCollector* self, float inFraction) {
	to_jph(self)->UpdateEarlyOutFraction(inFraction);
}

////////////////////////////////////////////////////////////////////////////////
// JPC_CollideShapeCollector

class JPC_CollideShapeCollectorBridge;
OPAQUE_WRAPPER(JPC_CollideShapeCollector, JPC_CollideShapeCollectorBridge)

class JPC_CollideShapeCollectorBridge final : public JPH::CollideShapeCollector {
	using ResultType = JPH::CollideShapeResult;

public:
	explicit JPC_CollideShapeCollectorBridge(void *self, JPC_CollideShapeCollectorFns fns) : self(self), fns(fns) {}

	void Reset() override {
		JPH::CollideShapeCollector::Reset();

		if (fns.Reset != nullptr) {
			fns.Reset(self);
		}
	}

	void AddHit(const ResultType &inResult) override {
		JPC_CollideShapeResult result = JPC_CollideShapeResult_to_jpc(inResult);
		JPC_CollideShapeCollector *base = to_jpc(this);

		fns.AddHit(self, base, &result);
	}

private:
	void* self;
	JPC_CollideShapeCollectorFns fns;
};

DESTRUCTOR(JPC_CollideShapeCollector)

JPC_API JPC_CollideShapeCollector* JPC_CollideShapeCollector_new(
	void *self,
	JPC_CollideShapeCollectorFns fns)
{
	return to_jpc(new JPC_CollideShapeCollectorBridge(self, fns));
}

JPC_API void JPC_CollideShapeCollector_UpdateEarlyOutFraction(JPC_CollideShapeCollector* self, float inFraction) {
	to_jph(self)->UpdateEarlyOutFraction(inFraction);
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
// Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_Constraint, JPH::Constraint);

// RefTarget<Constraint>
JPC_API uint32_t JPC_Constraint_GetRefCount(const JPC_Constraint* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_Constraint_AddRef(const JPC_Constraint* self) {
	to_jph(self)->AddRef();
}

JPC_API void JPC_Constraint_Release(const JPC_Constraint* self) {
	to_jph(self)->Release();
}

// Constraint
JPC_API void JPC_Constraint_delete(JPC_Constraint* self) {
	delete to_jph(self);
}

// JPC_API JPC_ConstraintType JPC_Constraint_GetType(const JPC_Constraint* self);
// JPC_API JPC_ConstraintSubType JPC_Constraint_GetSubType(const JPC_Constraint* self);

JPC_API uint32_t JPC_Constraint_GetConstraintPriority(const JPC_Constraint* self) {
	return to_jph(self)->GetConstraintPriority();
}

JPC_API void JPC_Constraint_SetConstraintPriority(JPC_Constraint* self, uint32_t inPriority) {
	to_jph(self)->SetConstraintPriority(inPriority);
}

JPC_API uint JPC_Constraint_GetNumVelocityStepsOverride(const JPC_Constraint* self) {
	return to_jph(self)->GetNumVelocityStepsOverride();
}

JPC_API void JPC_Constraint_SetNumVelocityStepsOverride(JPC_Constraint* self, uint inN) {
	to_jph(self)->SetNumVelocityStepsOverride(inN);
}

JPC_API uint JPC_Constraint_GetNumPositionStepsOverride(const JPC_Constraint* self) {
	return to_jph(self)->GetNumPositionStepsOverride();
}

JPC_API void JPC_Constraint_SetNumPositionStepsOverride(JPC_Constraint* self, uint inN) {
	to_jph(self)->SetNumPositionStepsOverride(inN);
}

JPC_API bool JPC_Constraint_GetEnabled(const JPC_Constraint* self) {
	return to_jph(self)->GetEnabled();
}

JPC_API void JPC_Constraint_SetEnabled(JPC_Constraint* self, bool inEnabled) {
	to_jph(self)->SetEnabled(inEnabled);
}

JPC_API uint64_t JPC_Constraint_GetUserData(const JPC_Constraint* self) {
	return to_jph(self)->GetUserData();
}

JPC_API void JPC_Constraint_SetUserData(JPC_Constraint* self, uint64_t inUserData) {
	to_jph(self)->SetUserData(inUserData);
}

JPC_API void JPC_Constraint_NotifyShapeChanged(JPC_Constraint* self, JPC_BodyID inBodyID, JPC_Vec3 inDeltaCOM) {
	to_jph(self)->NotifyShapeChanged(to_jph(inBodyID), to_jph(inDeltaCOM));
}

////////////////////////////////////////////////////////////////////////////////
// TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_TwoBodyConstraint, JPH::TwoBodyConstraint);

JPC_API JPC_Body* JPC_TwoBodyConstraint_GetBody1(const JPC_TwoBodyConstraint* self) {
	return to_jpc(to_jph(self)->GetBody1());
}

JPC_API JPC_Body* JPC_TwoBodyConstraint_GetBody2(const JPC_TwoBodyConstraint* self) {
	return to_jpc(to_jph(self)->GetBody2());
}

JPC_API JPC_Mat44 JPC_TwoBodyConstraint_GetConstraintToBody1Matrix(const JPC_TwoBodyConstraint* self) {
	return to_jpc(to_jph(self)->GetConstraintToBody1Matrix());
}

JPC_API JPC_Mat44 JPC_TwoBodyConstraint_GetConstraintToBody2Matrix(const JPC_TwoBodyConstraint* self) {
	return to_jpc(to_jph(self)->GetConstraintToBody2Matrix());
}

////////////////////////////////////////////////////////////////////////////////
// FixedConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_FixedConstraint, JPH::FixedConstraint);

JPC_API JPC_Vec3 JPC_FixedConstraint_GetTotalLambdaPosition(const JPC_FixedConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API JPC_Vec3 JPC_FixedConstraint_GetTotalLambdaRotation(const JPC_FixedConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

////////////////////////////////////////////////////////////////////////////////
// SixDOFConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_SixDOFConstraint, JPH::SixDOFConstraint);

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTranslationLimitsMin(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTranslationLimitsMin());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTranslationLimitsMax(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTranslationLimitsMax());
}

JPC_API void JPC_SixDOFConstraint_SetTranslationLimits(JPC_SixDOFConstraint* self, JPC_Vec3 inLimitMin, JPC_Vec3 inLimitMax) {
	to_jph(self)->SetTranslationLimits(to_jph(inLimitMin), to_jph(inLimitMax));
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetRotationLimitsMin(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetRotationLimitsMin());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetRotationLimitsMax(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetRotationLimitsMax());
}

JPC_API void JPC_SixDOFConstraint_SetRotationLimits(JPC_SixDOFConstraint* self, JPC_Vec3 inLimitMin, JPC_Vec3 inLimitMax) {
	to_jph(self)->SetRotationLimits(to_jph(inLimitMin), to_jph(inLimitMax));
}

JPC_API float JPC_SixDOFConstraint_GetLimitsMin(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	return to_jph(self)->GetLimitsMin((JPH::SixDOFConstraint::EAxis)inAxis);
}

JPC_API float JPC_SixDOFConstraint_GetLimitsMax(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	return to_jph(self)->GetLimitsMax((JPH::SixDOFConstraint::EAxis)inAxis);
}

JPC_API bool JPC_SixDOFConstraint_IsFreeAxis(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis);

// const SpringSettings & GetLimitsSpringSettings(JPC_SixDOFConstraint_Axis inAxis) const { JPH_ASSERT(inAxis < JPC_SixDOFConstraint_Axis::NumTranslation); return mLimitsSpringSettings[inAxis]; }
// void SetLimitsSpringSettings(JPC_SixDOFConstraint_Axis inAxis, const SpringSettings& inLimitsSpringSettings) { JPH_ASSERT(inAxis < JPC_SixDOFConstraint_Axis::NumTranslation); mLimitsSpringSettings[inAxis] = inLimitsSpringSettings; CacheHasSpringLimits(); }

JPC_API void JPC_SixDOFConstraint_SetMaxFriction(JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis, float inFriction) {
	to_jph(self)->SetMaxFriction((JPH::SixDOFConstraint::EAxis)inAxis, inFriction);
}

JPC_API float JPC_SixDOFConstraint_GetMaxFriction(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	return to_jph(self)->GetMaxFriction((JPH::SixDOFConstraint::EAxis)inAxis);
}

JPC_API JPC_Quat JPC_SixDOFConstraint_GetRotationInConstraintSpace(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetRotationInConstraintSpace());
}

/// Motor settings
// MotorSettings & GetMotorSettings(EAxis inAxis)
// const MotorSettings & GetMotorSettings(EAxis inAxis) const

// void SetMotorState(EAxis inAxis, EMotorState inState);
// EMotorState GetMotorState(EAxis inAxis) const

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTargetVelocityCS(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetVelocityCS());
}

JPC_API void JPC_SixDOFConstraint_SetTargetVelocityCS(JPC_SixDOFConstraint* self, JPC_Vec3 inVelocity) {
	to_jph(self)->SetTargetVelocityCS(to_jph(inVelocity));
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTargetAngularVelocityCS(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetAngularVelocityCS());
}

JPC_API void JPC_SixDOFConstraint_SetTargetAngularVelocityCS(JPC_SixDOFConstraint* self, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetTargetAngularVelocityCS(to_jph(inAngularVelocity));
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTargetPositionCS(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetPositionCS());
}

JPC_API void JPC_SixDOFConstraint_SetTargetPositionCS(JPC_SixDOFConstraint* self, JPC_Vec3 inPosition) {
	to_jph(self)->SetTargetPositionCS(to_jph(inPosition));
}

JPC_API JPC_Quat JPC_SixDOFConstraint_GetTargetOrientationCS(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetOrientationCS());
}

JPC_API void JPC_SixDOFConstraint_SetTargetOrientationCS(JPC_SixDOFConstraint* self, JPC_Quat inOrientation) {
	to_jph(self)->SetTargetOrientationCS(to_jph(inOrientation));
}

JPC_API void JPC_SixDOFConstraint_SetTargetOrientationBS(JPC_SixDOFConstraint* self, JPC_Quat inOrientation) {
	to_jph(self)->SetTargetOrientationBS(to_jph(inOrientation));
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTotalLambdaPosition(JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTotalLambdaRotation(JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTotalLambdaMotorTranslation(JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaMotorTranslation());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTotalLambdaMotorRotation(JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaMotorRotation());
}

////////////////////////////////////////////////////////////////////////////////
// HingeConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_HingeConstraint, JPH::HingeConstraint);

JPC_API JPC_Constraint* JPC_HingeConstraint_to_Constraint(JPC_HingeConstraint* self) {
	return (JPC_Constraint*)(self);
}

JPC_API void JPC_HingeConstraint_SetMotorState(JPC_HingeConstraint* self, JPC_MotorState inState) {
	to_jph(self)->SetMotorState(to_jph(inState));
}

JPC_API JPC_MotorState JPC_HingeConstraint_GetMotorState(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetMotorState());
}

JPC_API void JPC_HingeConstraint_SetTargetAngularVelocity(JPC_HingeConstraint* self, float inAngularVelocity) {
	to_jph(self)->SetTargetAngularVelocity(inAngularVelocity);
}

JPC_API float JPC_HingeConstraint_GetTargetAngularVelocity(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetTargetAngularVelocity();
}

JPC_API void JPC_HingeConstraint_SetTargetAngle(JPC_HingeConstraint* self, float inAngle) {
	to_jph(self)->SetTargetAngle(inAngle);
}

JPC_API float JPC_HingeConstraint_GetTargetAngle(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetTargetAngle();
}

JPC_API JPC_Vec3 JPC_HingeConstraint_GetTotalLambdaPosition(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API JPC_Vec2 JPC_HingeConstraint_GetTotalLambdaRotation(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

JPC_API float JPC_HingeConstraint_GetTotalLambdaRotationLimits(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetTotalLambdaRotationLimits();
}

JPC_API float JPC_HingeConstraint_GetTotalLambdaMotor(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetTotalLambdaMotor();
}


////////////////////////////////////////////////////////////////////////////////
// SliderConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_SliderConstraint, JPH::SliderConstraint);

JPC_API void JPC_SliderConstraint_SetMotorState(JPC_SliderConstraint* self, JPC_MotorState inState) {
	to_jph(self)->SetMotorState(to_jph(inState));
}

JPC_API JPC_MotorState JPC_SliderConstraint_GetMotorState(const JPC_SliderConstraint* self) {
	return to_jpc(to_jph(self)->GetMotorState());
}

JPC_API void JPC_SliderConstraint_SetTargetVelocity(JPC_SliderConstraint* self, float inVelocity) {
	to_jph(self)->SetTargetVelocity(inVelocity);
}

JPC_API float JPC_SliderConstraint_GetTargetVelocity(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetTargetVelocity();
}

JPC_API void JPC_SliderConstraint_SetTargetPosition(JPC_SliderConstraint* self, float inPosition) {
	to_jph(self)->SetTargetPosition(inPosition);
}

JPC_API float JPC_SliderConstraint_GetTargetPosition(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetTargetPosition();
}

JPC_API JPC_Vec2 JPC_SliderConstraint_GetTotalLambdaPosition(const JPC_SliderConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API float JPC_SliderConstraint_GetTotalLambdaPositionLimits(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetTotalLambdaPositionLimits();
}

JPC_API JPC_Vec3 JPC_SliderConstraint_GetTotalLambdaRotation(const JPC_SliderConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

JPC_API float JPC_SliderConstraint_GetTotalLambdaMotor(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetTotalLambdaMotor();
}


////////////////////////////////////////////////////////////////////////////////
// ConstraintSettings

JPC_IMPL void JPC_ConstraintSettings_to_jpc(
	JPC_ConstraintSettings* outJpc,
	const JPH::ConstraintSettings* inJph)
{
	outJpc->Enabled = inJph->mEnabled;
	outJpc->ConstraintPriority = inJph->mConstraintPriority;
	outJpc->NumVelocityStepsOverride = inJph->mNumVelocityStepsOverride;
	outJpc->NumPositionStepsOverride = inJph->mNumPositionStepsOverride;
	outJpc->DrawConstraintSize = inJph->mDrawConstraintSize;
	outJpc->UserData = inJph->mUserData;
}

JPC_IMPL void JPC_ConstraintSettings_to_jph(
	const JPC_ConstraintSettings* inJpc,
	JPH::ConstraintSettings* outJph)
{
	outJph->mEnabled = inJpc->Enabled;
	outJph->mConstraintPriority = inJpc->ConstraintPriority;
	outJph->mNumVelocityStepsOverride = inJpc->NumVelocityStepsOverride;
	outJph->mNumPositionStepsOverride = inJpc->NumPositionStepsOverride;
	outJph->mDrawConstraintSize = inJpc->DrawConstraintSize;
	outJph->mUserData = inJpc->UserData;
}

JPC_API void JPC_ConstraintSettings_default(JPC_ConstraintSettings* settings) {
	JPH::ConstraintSettings defaultSettings{};
	JPC_ConstraintSettings_to_jpc(settings, &defaultSettings);
}

////////////////////////////////////////////////////////////////////////////////
// SpringSettings

JPC_IMPL void JPC_SpringSettings_to_jpc(
	JPC_SpringSettings* outJpc,
	const JPH::SpringSettings* inJph)
{
	outJpc->Mode = to_jpc(inJph->mMode);
	outJpc->FrequencyOrStiffness = inJph->mFrequency;
	outJpc->Damping = inJph->mDamping;
}

JPC_IMPL void JPC_SpringSettings_to_jph(
	const JPC_SpringSettings* inJpc,
	JPH::SpringSettings* outJph)
{
	outJph->mMode = to_jph(inJpc->Mode);
	outJph->mFrequency = inJpc->FrequencyOrStiffness;
	outJph->mDamping = inJpc->Damping;
}

JPC_API void JPC_SpringSettings_default(JPC_SpringSettings* settings) {
	JPH::SpringSettings defaultSettings{};
	JPC_SpringSettings_to_jpc(settings, &defaultSettings);
}

////////////////////////////////////////////////////////////////////////////////
// MotorSettings

JPC_IMPL void JPC_MotorSettings_to_jpc(
	JPC_MotorSettings* outJpc,
	const JPH::MotorSettings* inJph)
{
	JPC_SpringSettings_to_jpc(&outJpc->SpringSettings, &inJph->mSpringSettings);
	outJpc->MinForceLimit = inJph->mMinForceLimit;
	outJpc->MaxForceLimit = inJph->mMaxForceLimit;
	outJpc->MinTorqueLimit = inJph->mMinTorqueLimit;
	outJpc->MaxTorqueLimit = inJph->mMaxTorqueLimit;
}

JPC_IMPL void JPC_MotorSettings_to_jph(
	const JPC_MotorSettings* inJpc,
	JPH::MotorSettings* outJph)
{
	JPC_SpringSettings_to_jph(&inJpc->SpringSettings, &outJph->mSpringSettings);
	outJph->mMinForceLimit = inJpc->MinForceLimit;
	outJph->mMaxForceLimit = inJpc->MaxForceLimit;
	outJph->mMinTorqueLimit = inJpc->MinTorqueLimit;
	outJph->mMaxTorqueLimit = inJpc->MaxTorqueLimit;
}

JPC_API void JPC_MotorSettings_default(JPC_MotorSettings* settings) {
	JPH::MotorSettings defaultSettings{};
	JPC_MotorSettings_to_jpc(settings, &defaultSettings);
}

////////////////////////////////////////////////////////////////////////////////
// FixedConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_FixedConstraintSettings_to_jpc(
	JPC_FixedConstraintSettings* outJpc,
	const JPH::FixedConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->AutoDetectPoint = inJph->mAutoDetectPoint;
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->AxisX1 = to_jpc(inJph->mAxisX1);
	outJpc->AxisY1 = to_jpc(inJph->mAxisY1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->AxisX2 = to_jpc(inJph->mAxisX2);
	outJpc->AxisY2 = to_jpc(inJph->mAxisY2);
}

JPC_IMPL void JPC_FixedConstraintSettings_to_jph(
	const JPC_FixedConstraintSettings* inJpc,
	JPH::FixedConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mAutoDetectPoint = inJpc->AutoDetectPoint;
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mAxisX1 = to_jph(inJpc->AxisX1);
	outJph->mAxisY1 = to_jph(inJpc->AxisY1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mAxisX2 = to_jph(inJpc->AxisX2);
	outJph->mAxisY2 = to_jph(inJpc->AxisY2);
}

JPC_API void JPC_FixedConstraintSettings_default(JPC_FixedConstraintSettings* settings) {
	JPH::FixedConstraintSettings defaultSettings{};
	JPC_FixedConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_Constraint* JPC_FixedConstraintSettings_Create(
	const JPC_FixedConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::FixedConstraintSettings jphSettings;
	JPC_FixedConstraintSettings_to_jph(self, &jphSettings);

	JPH::FixedConstraint* outJph = new JPH::FixedConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_Constraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// SixDOFConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_SixDOFConstraintSettings_to_jpc(
	JPC_SixDOFConstraintSettings* outJpc,
	const JPH::SixDOFConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Position1 = to_jpc(inJph->mPosition1);
	outJpc->AxisX1 = to_jpc(inJph->mAxisX1);
	outJpc->AxisY1 = to_jpc(inJph->mAxisY1);
	outJpc->Position2 = to_jpc(inJph->mPosition2);
	outJpc->AxisX2 = to_jpc(inJph->mAxisX2);
	outJpc->AxisY2 = to_jpc(inJph->mAxisY2);
	std::copy(inJph->mMaxFriction, inJph->mMaxFriction + 6, outJpc->MaxFriction);
	std::copy(inJph->mLimitMin, inJph->mLimitMin + 6, outJpc->LimitMin);
	std::copy(inJph->mLimitMax, inJph->mLimitMax + 6, outJpc->LimitMax);

	// TODO: LimitsSpringSettings
}

JPC_IMPL void JPC_SixDOFConstraintSettings_to_jph(
	const JPC_SixDOFConstraintSettings* inJpc,
	JPH::SixDOFConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPosition1 = to_jph(inJpc->Position1);
	outJph->mAxisX1 = to_jph(inJpc->AxisX1);
	outJph->mAxisY1 = to_jph(inJpc->AxisY1);
	outJph->mPosition2 = to_jph(inJpc->Position2);
	outJph->mAxisX2 = to_jph(inJpc->AxisX2);
	outJph->mAxisY2 = to_jph(inJpc->AxisY2);
	std::copy(inJpc->MaxFriction, inJpc->MaxFriction + 6, outJph->mMaxFriction);
	std::copy(inJpc->LimitMin, inJpc->LimitMin + 6, outJph->mLimitMin);
	std::copy(inJpc->LimitMax, inJpc->LimitMax + 6, outJph->mLimitMax);

	// TODO: LimitsSpringSettings
}

JPC_API void JPC_SixDOFConstraintSettings_default(JPC_SixDOFConstraintSettings* settings) {
	JPH::SixDOFConstraintSettings defaultSettings{};
	JPC_SixDOFConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_Constraint* JPC_SixDOFConstraintSettings_Create(
	const JPC_SixDOFConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::SixDOFConstraintSettings jphSettings;
	JPC_SixDOFConstraintSettings_to_jph(self, &jphSettings);

	JPH::SixDOFConstraint* outJph = new JPH::SixDOFConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_Constraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// HingeConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_HingeConstraintSettings_to_jpc(
	JPC_HingeConstraintSettings* outJpc,
	const JPH::HingeConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->HingeAxis1 = to_jpc(inJph->mHingeAxis1);
	outJpc->NormalAxis1 = to_jpc(inJph->mNormalAxis1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->HingeAxis2 = to_jpc(inJph->mHingeAxis2);
	outJpc->NormalAxis2 = to_jpc(inJph->mNormalAxis2);
	outJpc->LimitsMin = inJph->mLimitsMin;
	outJpc->LimitsMax = inJph->mLimitsMax;
	JPC_SpringSettings_to_jpc(&outJpc->LimitsSpringSettings, &inJph->mLimitsSpringSettings);
	outJpc->MaxFrictionTorque = inJph->mMaxFrictionTorque;
	JPC_MotorSettings_to_jpc(&outJpc->MotorSettings, &inJph->mMotorSettings);
}

JPC_IMPL void JPC_HingeConstraintSettings_to_jph(
	const JPC_HingeConstraintSettings* inJpc,
	JPH::HingeConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mHingeAxis1 = to_jph(inJpc->HingeAxis1);
	outJph->mNormalAxis1 = to_jph(inJpc->NormalAxis1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mHingeAxis2 = to_jph(inJpc->HingeAxis2);
	outJph->mNormalAxis2 = to_jph(inJpc->NormalAxis2);
	outJph->mLimitsMin = inJpc->LimitsMin;
	outJph->mLimitsMax = inJpc->LimitsMax;
	JPC_SpringSettings_to_jph(&inJpc->LimitsSpringSettings, &outJph->mLimitsSpringSettings);
	outJph->mMaxFrictionTorque = inJpc->MaxFrictionTorque;
	JPC_MotorSettings_to_jph(&inJpc->MotorSettings, &outJph->mMotorSettings);
}

JPC_API void JPC_HingeConstraintSettings_default(JPC_HingeConstraintSettings* settings) {
	JPH::HingeConstraintSettings defaultSettings{};
	JPC_HingeConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_HingeConstraint* JPC_HingeConstraintSettings_Create(
	const JPC_HingeConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::HingeConstraintSettings jphSettings;
	JPC_HingeConstraintSettings_to_jph(self, &jphSettings);

	JPH::HingeConstraint* outJph = new JPH::HingeConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_HingeConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// DistanceConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_DistanceConstraintSettings_to_jpc(
	JPC_DistanceConstraintSettings* outJpc,
	const JPH::DistanceConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->MinDistance = inJph->mMinDistance;
	outJpc->MaxDistance = inJph->mMaxDistance;
	// TODO: Spring settings
}

JPC_IMPL void JPC_DistanceConstraintSettings_to_jph(
	const JPC_DistanceConstraintSettings* inJpc,
	JPH::DistanceConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mMinDistance = inJpc->MinDistance;
	outJph->mMaxDistance = inJpc->MaxDistance;
	// TODO: Spring settings
}

JPC_API void JPC_DistanceConstraintSettings_default(JPC_DistanceConstraintSettings* settings) {
	JPH::DistanceConstraintSettings defaultSettings{};
	JPC_DistanceConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_Constraint* JPC_DistanceConstraintSettings_Create(
	const JPC_DistanceConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
	{
		JPH::DistanceConstraintSettings jphSettings;
		JPC_DistanceConstraintSettings_to_jph(self, &jphSettings);

		JPH::DistanceConstraint* outJph = new JPH::DistanceConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
		return (JPC_Constraint*)outJph;
	}

////////////////////////////////////////////////////////////////////////////////
// SliderConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_SliderConstraintSettings_to_jpc(
	JPC_SliderConstraintSettings* outJpc,
	const JPH::SliderConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->AutoDetectPoint = inJph->mAutoDetectPoint;
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->SliderAxis1 = to_jpc(inJph->mSliderAxis1);
	outJpc->NormalAxis1 = to_jpc(inJph->mNormalAxis1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->SliderAxis2 = to_jpc(inJph->mSliderAxis2);
	outJpc->NormalAxis2 = to_jpc(inJph->mNormalAxis2);
	outJpc->LimitsMin = inJph->mLimitsMin;
	outJpc->LimitsMax = inJph->mLimitsMax;
	JPC_SpringSettings_to_jpc(&outJpc->LimitsSpringSettings, &inJph->mLimitsSpringSettings);
	outJpc->MaxFrictionForce = inJph->mMaxFrictionForce;
	JPC_MotorSettings_to_jpc(&outJpc->MotorSettings, &inJph->mMotorSettings);
}

JPC_IMPL void JPC_SliderConstraintSettings_to_jph(
	const JPC_SliderConstraintSettings* inJpc,
	JPH::SliderConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mAutoDetectPoint = inJpc->AutoDetectPoint;
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mSliderAxis1 = to_jph(inJpc->SliderAxis1);
	outJph->mNormalAxis1 = to_jph(inJpc->NormalAxis1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mSliderAxis2 = to_jph(inJpc->SliderAxis2);
	outJph->mNormalAxis2 = to_jph(inJpc->NormalAxis2);
	outJph->mLimitsMin = inJpc->LimitsMin;
	outJph->mLimitsMax = inJpc->LimitsMax;
	JPC_SpringSettings_to_jph(&inJpc->LimitsSpringSettings, &outJph->mLimitsSpringSettings);
	outJph->mMaxFrictionForce = inJpc->MaxFrictionForce;
	JPC_MotorSettings_to_jph(&inJpc->MotorSettings, &outJph->mMotorSettings);
}

JPC_API void JPC_SliderConstraintSettings_default(JPC_SliderConstraintSettings* settings) {
	JPH::SliderConstraintSettings defaultSettings{};
	JPC_SliderConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_SliderConstraint* JPC_SliderConstraintSettings_Create(
	const JPC_SliderConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::SliderConstraintSettings jphSettings;
	JPC_SliderConstraintSettings_to_jph(self, &jphSettings);

	JPH::SliderConstraint* outJph = new JPH::SliderConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_SliderConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// Shape -> RefTarget<Shape>

// RefTarget<Shape>
JPC_API uint32_t JPC_Shape_GetRefCount(const JPC_Shape* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_Shape_AddRef(const JPC_Shape* self) {
	to_jph(self)->AddRef();
}

JPC_API void JPC_Shape_Release(const JPC_Shape* self) {
	to_jph(self)->Release();
}

// Shape
JPC_API uint64_t JPC_Shape_GetUserData(const JPC_Shape* self) {
	return to_jph(self)->GetUserData();
}

JPC_API void JPC_Shape_SetUserData(JPC_Shape* self, uint64_t userData) {
	to_jph(self)->SetUserData(userData);
}

JPC_API JPC_ShapeType JPC_Shape_GetType(const JPC_Shape* self) {
	return to_jpc(to_jph(self)->GetType());
}

JPC_API JPC_ShapeSubType JPC_Shape_GetSubType(const JPC_Shape* self) {
	return to_jpc(to_jph(self)->GetSubType());
}

JPC_API uint64_t JPC_Shape_GetSubShapeUserData(const JPC_Shape* self, JPC_SubShapeID inSubShapeID) {
	return to_jph(self)->GetSubShapeUserData(JPC_SubShapeID_to_jph(inSubShapeID));
}

JPC_API JPC_Vec3 JPC_Shape_GetCenterOfMass(const JPC_Shape* self) {
	return to_jpc(to_jph(self)->GetCenterOfMass());
}

JPC_API float JPC_Shape_GetVolume(const JPC_Shape* self) {
	return to_jph(self)->GetVolume();
}

////////////////////////////////////////////////////////////////////////////////
// CompoundShape

JPC_API const JPC_Shape* JPC_CompoundShape_GetSubShape_Shape(
	const JPC_CompoundShape* self,
	uint inIdx)
{
	return to_jpc(to_jph(self)->GetSubShape(inIdx).mShape.GetPtr());
}

JPC_API uint32_t JPC_CompoundShape_GetSubShapeIndexFromID(
	const JPC_CompoundShape* self,
	JPC_SubShapeID inSubShapeID,
	JPC_SubShapeID* outRemainder)
{
	JPH::SubShapeID jphRemainder;
	uint32_t res = to_jph(self)->GetSubShapeIndexFromID(JPC_SubShapeID_to_jph(inSubShapeID), jphRemainder);
	*outRemainder = to_jpc(jphRemainder);
	return res;
}

////////////////////////////////////////////////////////////////////////////////
// ShapeSettings

// Unpack a ShapeResult into a bool and two pointers to be friendlier to C.
static bool HandleShapeResult(JPH::ShapeSettings::ShapeResult res, JPC_Shape** outShape, JPC_String** outError) {
	if (res.HasError()) {
		if (outError != nullptr) {
			JPH::String* created = new JPH::String(std::move(res.GetError()));
			*outError = to_jpc(created);
		}

		return false;
	} else {
		JPH::Ref<JPH::Shape> shape = res.Get();
		shape->AddRef();
		*outShape = to_jpc((JPH::Shape*)shape);

		return true;
	}
}

////////////////////////////////////////////////////////////////////////////////
// TriangleShapeSettings

static void to_jph(const JPC_TriangleShapeSettings* input, JPH::TriangleShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mV1 = to_jph(input->V1);
	output->mV2 = to_jph(input->V2);
	output->mV3 = to_jph(input->V3);
	output->mConvexRadius = input->ConvexRadius;
}

JPC_API void JPC_TriangleShapeSettings_default(JPC_TriangleShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->V1 = {0};
	object->V2 = {0};
	object->V3 = {0};
	object->ConvexRadius = 0.0;
}

JPC_API bool JPC_TriangleShapeSettings_Create(const JPC_TriangleShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::TriangleShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// MeshShapeSettings

JPC_IMPL void JPC_MeshShapeSettings_to_jpc_borrowed(
	JPC_MeshShapeSettings* outJpc,
	const JPH::MeshShapeSettings* inJph)
{
	outJpc->UserData = inJph->mUserData;

	outJpc->TriangleVertices = (JPC_Float3*)inJph->mTriangleVertices.data();
	outJpc->TriangleVerticesLen = inJph->mTriangleVertices.size();
	outJpc->IndexedTriangles = (JPC_IndexedTriangle*)inJph->mIndexedTriangles.data();
	outJpc->IndexedTrianglesLen = inJph->mIndexedTriangles.size();
}

JPC_IMPL void JPC_MeshShapeSettings_to_jph(
	const JPC_MeshShapeSettings* inJpc,
	JPH::MeshShapeSettings* outJph)
{
	outJph->mUserData = inJpc->UserData;

	auto triangleVertices = (const JPH::Float3*)inJpc->TriangleVertices;
	outJph->mTriangleVertices = JPH::VertexList(triangleVertices, triangleVertices + inJpc->TriangleVerticesLen);

	auto indexedTriangles = (const JPH::IndexedTriangle*)inJpc->IndexedTriangles;
	outJph->mIndexedTriangles = JPH::IndexedTriangleList(indexedTriangles, indexedTriangles + inJpc->IndexedTrianglesLen);
}

JPC_API void JPC_MeshShapeSettings_default(JPC_MeshShapeSettings* object) {
	JPH::MeshShapeSettings settings;
	JPC_MeshShapeSettings_to_jpc_borrowed(object, &settings);

	// Overwrite all pointers and lengths so that the default value doesn't
	// contain pointers to freed memory.
	object->TriangleVertices = nullptr;
	object->TriangleVerticesLen = 0;
	object->IndexedTriangles = nullptr;
	object->IndexedTrianglesLen = 0;
}

JPC_API bool JPC_MeshShapeSettings_Create(const JPC_MeshShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::MeshShapeSettings settings;
	JPC_MeshShapeSettings_to_jph(self, &settings);

	// MeshShapeSettings calls Sanitize in its default constructor, but we don't
	// have constructors in C. It's probably fine to always Sanitize.
	settings.Sanitize();

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// BoxShapeSettings

static void to_jph(const JPC_BoxShapeSettings* input, JPH::BoxShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mHalfExtent = to_jph(input->HalfExtent);
	output->mConvexRadius = input->ConvexRadius;
}

JPC_API void JPC_BoxShapeSettings_default(JPC_BoxShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->HalfExtent = JPC_Vec3{0};
	object->ConvexRadius = 0.0;
}

JPC_API bool JPC_BoxShapeSettings_Create(const JPC_BoxShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::BoxShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// SphereShapeSettings

static void to_jph(const JPC_SphereShapeSettings* input, JPH::SphereShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mRadius = input->Radius;
}

JPC_API void JPC_SphereShapeSettings_default(JPC_SphereShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->Radius = 0.0;
}

JPC_API bool JPC_SphereShapeSettings_Create(const JPC_SphereShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::SphereShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// CapsuleShapeSettings

static void to_jph(const JPC_CapsuleShapeSettings* input, JPH::CapsuleShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mRadius = input->Radius;
	output->mHalfHeightOfCylinder = input->HalfHeightOfCylinder;
}

JPC_API void JPC_CapsuleShapeSettings_default(JPC_CapsuleShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->Radius = 0.0;
	object->HalfHeightOfCylinder = 0.0;
}

JPC_API bool JPC_CapsuleShapeSettings_Create(const JPC_CapsuleShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::CapsuleShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// CylinderShapeSettings

static void to_jph(const JPC_CylinderShapeSettings* input, JPH::CylinderShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mHalfHeight = input->HalfHeight;
	output->mRadius = input->Radius;
	output->mConvexRadius = input->ConvexRadius;
}

JPC_API void JPC_CylinderShapeSettings_default(JPC_CylinderShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->HalfHeight = 0.0;
	object->Radius = 0.0;
	object->ConvexRadius = 0.0;
}

JPC_API bool JPC_CylinderShapeSettings_Create(const JPC_CylinderShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::CylinderShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// ConvexHullShapeSettings

static void to_jph(const JPC_ConvexHullShapeSettings* input, JPH::ConvexHullShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mPoints = to_jph(input->Points, input->PointsLen);
	output->mMaxConvexRadius = input->MaxConvexRadius;
	output->mMaxErrorConvexRadius = input->MaxErrorConvexRadius;
	output->mHullTolerance = input->HullTolerance;
}

JPC_API void JPC_ConvexHullShapeSettings_default(JPC_ConvexHullShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->Points = nullptr;
	object->PointsLen = 0;
	object->MaxConvexRadius = 0.0;
	object->MaxErrorConvexRadius = 0.05f;
	object->HullTolerance = 1.0e-3f;
}

JPC_API bool JPC_ConvexHullShapeSettings_Create(const JPC_ConvexHullShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::ConvexHullShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// CompoundShape::SubShapeSettings

static JPH::CompoundShapeSettings::SubShapeSettings to_jph(const JPC_SubShapeSettings* input) {
	const JPH::Shape* shape = to_jph(input->Shape);

	JPH::CompoundShapeSettings::SubShapeSettings output;
	output.mShape = nullptr;
	output.mShapePtr = shape;
	output.mPosition = to_jph(input->Position);
	output.mRotation = to_jph(input->Rotation);
	output.mUserData = input->UserData;
	return output;
}

static JPH::Array<JPH::CompoundShapeSettings::SubShapeSettings> to_jph(const JPC_SubShapeSettings* src, size_t n) {
	JPH::Array<JPH::CompoundShapeSettings::SubShapeSettings> vec;
	vec.reserve(n);

	for (size_t i = 0; i < n; i++) {
		vec.push_back(to_jph(&src[i]));
	}

	return vec;
}

JPC_API void JPC_SubShapeSettings_default(JPC_SubShapeSettings* object) {
	object->Shape = nullptr;
	object->Position = JPC_Vec3{0};
	object->Rotation = JPC_Quat{0, 0, 0, 1};
	object->UserData = 0;
}

////////////////////////////////////////////////////////////////////////////////
// StaticCompoundShapeSettings -> CompoundShapeSettings -> ShapeSettings

static void to_jph(const JPC_StaticCompoundShapeSettings* input, JPH::StaticCompoundShapeSettings* output) {
	output->mUserData = input->UserData;

	output->mSubShapes = to_jph(input->SubShapes, input->SubShapesLen);
}

JPC_API void JPC_StaticCompoundShapeSettings_default(JPC_StaticCompoundShapeSettings* object) {
	object->UserData = 0;

	object->SubShapes = nullptr;
	object->SubShapesLen = 0;
}

JPC_API bool JPC_StaticCompoundShapeSettings_Create(const JPC_StaticCompoundShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::StaticCompoundShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// MutableCompoundShape -> CompoundShape -> Shape

JPC_IMPL JPH::MutableCompoundShape* JPC_MutableCompoundShape_to_jph(JPC_MutableCompoundShape* self) {
	return reinterpret_cast<JPH::MutableCompoundShape*>(self);
}

JPC_API uint JPC_MutableCompoundShape_AddShape(
	JPC_MutableCompoundShape* self,
	JPC_Vec3 inPosition,
	JPC_Quat inRotation,
	const JPC_Shape* inShape,
	uint32_t inUserData)
{
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	return self_jph->AddShape(to_jph(inPosition), to_jph(inRotation), to_jph(inShape), inUserData);
}

JPC_API void JPC_MutableCompoundShape_RemoveShape(JPC_MutableCompoundShape* self, uint inIndex) {
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	self_jph->RemoveShape(inIndex);
}

JPC_API void JPC_MutableCompoundShape_ModifyShape(JPC_MutableCompoundShape* self, uint inIndex, JPC_Vec3 inPosition, JPC_Quat inRotation) {
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	self_jph->ModifyShape(inIndex, to_jph(inPosition), to_jph(inRotation));
}

JPC_API void JPC_MutableCompoundShape_ModifyShape2(JPC_MutableCompoundShape* self, uint inIndex, JPC_Vec3 inPosition, JPC_Quat inRotation, const JPC_Shape* inShape) {
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	self_jph->ModifyShape(inIndex, to_jph(inPosition), to_jph(inRotation), to_jph(inShape));
}

JPC_API void JPC_MutableCompoundShape_AdjustCenterOfMass(JPC_MutableCompoundShape* self) {
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	self_jph->AdjustCenterOfMass();
}

////////////////////////////////////////////////////////////////////////////////
// MutableCompoundShapeSettings -> CompoundShapeSettings -> ShapeSettings

static void to_jph(const JPC_MutableCompoundShapeSettings* input, JPH::MutableCompoundShapeSettings* output) {
	output->mUserData = input->UserData;

	output->mSubShapes = to_jph(input->SubShapes, input->SubShapesLen);
}

JPC_API void JPC_MutableCompoundShapeSettings_default(JPC_MutableCompoundShapeSettings* object) {
	object->UserData = 0;

	object->SubShapes = nullptr;
	object->SubShapesLen = 0;
}

JPC_API bool JPC_MutableCompoundShapeSettings_Create(const JPC_MutableCompoundShapeSettings* self, JPC_MutableCompoundShape** outShape, JPC_String** outError) {
	JPH::MutableCompoundShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), (JPC_Shape**)outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// BodyCreationSettings

static JPH::BodyCreationSettings to_jph(const JPC_BodyCreationSettings* settings) {
	JPH::BodyCreationSettings output{};

	output.mPosition = to_jph(settings->Position);
	output.mRotation = to_jph(settings->Rotation);
	output.mLinearVelocity = to_jph(settings->LinearVelocity);
	output.mAngularVelocity = to_jph(settings->AngularVelocity);
	output.mUserData = settings->UserData;
	output.mObjectLayer = settings->ObjectLayer;
	// CollisionGroup
	output.mMotionType = to_jph(settings->MotionType);
	output.mAllowedDOFs = to_jph(settings->AllowedDOFs);
	output.mAllowDynamicOrKinematic = settings->AllowDynamicOrKinematic;
	output.mIsSensor = settings->IsSensor;
	output.mCollideKinematicVsNonDynamic = settings->CollideKinematicVsNonDynamic;
	output.mUseManifoldReduction = settings->UseManifoldReduction;
	output.mApplyGyroscopicForce = settings->ApplyGyroscopicForce;
	output.mMotionQuality = to_jph(settings->MotionQuality);
	output.mEnhancedInternalEdgeRemoval = settings->EnhancedInternalEdgeRemoval;
	output.mAllowSleeping = settings->AllowSleeping;
	output.mFriction = settings->Friction;
	output.mRestitution = settings->Restitution;
	output.mLinearDamping = settings->LinearDamping;
	output.mAngularDamping = settings->AngularDamping;
	output.mMaxLinearVelocity = settings->MaxLinearVelocity;
	output.mMaxAngularVelocity = settings->MaxAngularVelocity;
	output.mGravityFactor = settings->GravityFactor;
	output.mNumVelocityStepsOverride = settings->NumVelocityStepsOverride;
	output.mNumPositionStepsOverride = settings->NumPositionStepsOverride;
	output.mOverrideMassProperties = to_jph(settings->OverrideMassProperties);
	output.mInertiaMultiplier = settings->InertiaMultiplier;
	// output.mMassPropertiesOverride = settings->MassPropertiesOverride;
	output.SetShape(to_jph(settings->Shape));

	return output;
}

JPC_API void JPC_BodyCreationSettings_default(JPC_BodyCreationSettings* settings) {
	JPH::BodyCreationSettings defaultSettings{};

	settings->Position = to_jpc(defaultSettings.mPosition);
	settings->Rotation = to_jpc(defaultSettings.mRotation);
	settings->LinearVelocity = to_jpc(defaultSettings.mLinearVelocity);
	settings->AngularVelocity = to_jpc(defaultSettings.mAngularVelocity);
	settings->UserData = defaultSettings.mUserData;
	settings->ObjectLayer = defaultSettings.mObjectLayer;
	// CollisionGroup
	settings->MotionType = to_jpc(defaultSettings.mMotionType);
	settings->AllowedDOFs = to_jpc(defaultSettings.mAllowedDOFs);
	settings->AllowDynamicOrKinematic = defaultSettings.mAllowDynamicOrKinematic;
	settings->IsSensor = defaultSettings.mIsSensor;
	settings->CollideKinematicVsNonDynamic = defaultSettings.mCollideKinematicVsNonDynamic;
	settings->UseManifoldReduction = defaultSettings.mUseManifoldReduction;
	settings->ApplyGyroscopicForce = defaultSettings.mApplyGyroscopicForce;
	settings->MotionQuality = to_jpc(defaultSettings.mMotionQuality);
	settings->EnhancedInternalEdgeRemoval = defaultSettings.mEnhancedInternalEdgeRemoval;
	settings->AllowSleeping = defaultSettings.mAllowSleeping;
	settings->Friction = defaultSettings.mFriction;
	settings->Restitution = defaultSettings.mRestitution;
	settings->LinearDamping = defaultSettings.mLinearDamping;
	settings->AngularDamping = defaultSettings.mAngularDamping;
	settings->MaxLinearVelocity = defaultSettings.mMaxLinearVelocity;
	settings->MaxAngularVelocity = defaultSettings.mMaxAngularVelocity;
	settings->GravityFactor = defaultSettings.mGravityFactor;
	settings->NumVelocityStepsOverride = defaultSettings.mNumVelocityStepsOverride;
	settings->NumPositionStepsOverride = defaultSettings.mNumPositionStepsOverride;
	settings->OverrideMassProperties = to_jpc(defaultSettings.mOverrideMassProperties);
	settings->InertiaMultiplier = defaultSettings.mInertiaMultiplier;
	// MassPropertiesOverride
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
	to_jph(self)->SetIsSensor(inIsSensor);
}

JPC_API bool JPC_Body_IsSensor(const JPC_Body* self) {
	return to_jph(self)->IsSensor();
}

JPC_API void JPC_Body_SetCollideKinematicVsNonDynamic(JPC_Body* self, bool inCollide) {
	to_jph(self)->SetCollideKinematicVsNonDynamic(inCollide);
}

JPC_API bool JPC_Body_GetCollideKinematicVsNonDynamic(const JPC_Body* self) {
	return to_jph(self)->GetCollideKinematicVsNonDynamic();
}

JPC_API void JPC_Body_SetUseManifoldReduction(JPC_Body* self, bool inUseReduction) {
	to_jph(self)->SetUseManifoldReduction(inUseReduction);
}

JPC_API bool JPC_Body_GetUseManifoldReduction(const JPC_Body* self) {
	return to_jph(self)->GetUseManifoldReduction();
}

JPC_API bool JPC_Body_GetUseManifoldReductionWithBody(const JPC_Body* self, const JPC_Body* inBody2) {
	return to_jph(self)->GetUseManifoldReductionWithBody(*to_jph(inBody2));
}

JPC_API void JPC_Body_SetApplyGyroscopicForce(JPC_Body* self, bool inApply) {
	to_jph(self)->SetApplyGyroscopicForce(inApply);
}

JPC_API bool JPC_Body_GetApplyGyroscopicForce(const JPC_Body* self) {
	return to_jph(self)->GetApplyGyroscopicForce();
}

JPC_API void JPC_Body_SetEnhancedInternalEdgeRemoval(JPC_Body* self, bool inApply) {
	to_jph(self)->SetEnhancedInternalEdgeRemoval(inApply);
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
	to_jph(self)->SetMotionType(to_jph(inMotionType));
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
	to_jph(self)->SetAllowSleeping(inAllow);
}

JPC_API void JPC_Body_ResetSleepTimer(JPC_Body* self) {
	to_jph(self)->ResetSleepTimer();
}

JPC_API float JPC_Body_GetFriction(const JPC_Body* self) {
	return to_jph(self)->GetFriction();
}

JPC_API void JPC_Body_SetFriction(JPC_Body* self, float inFriction) {
	to_jph(self)->SetFriction(inFriction);
}

JPC_API float JPC_Body_GetRestitution(const JPC_Body* self) {
	return to_jph(self)->GetRestitution();
}

JPC_API void JPC_Body_SetRestitution(JPC_Body* self, float inRestitution) {
	to_jph(self)->SetRestitution(inRestitution);
}

JPC_API JPC_Vec3 JPC_Body_GetLinearVelocity(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetLinearVelocity());
}

JPC_API void JPC_Body_SetLinearVelocity(JPC_Body* self, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->SetLinearVelocity(to_jph(inLinearVelocity));
}

JPC_API void JPC_Body_SetLinearVelocityClamped(JPC_Body* self, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->SetLinearVelocityClamped(to_jph(inLinearVelocity));
}

JPC_API JPC_Vec3 JPC_Body_GetAngularVelocity(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAngularVelocity());
}

JPC_API void JPC_Body_SetAngularVelocity(JPC_Body* self, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetAngularVelocity(to_jph(inAngularVelocity));
}

JPC_API void JPC_Body_SetAngularVelocityClamped(JPC_Body* self, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetAngularVelocityClamped(to_jph(inAngularVelocity));
}

JPC_API JPC_Vec3 JPC_Body_GetPointVelocityCOM(const JPC_Body* self, JPC_Vec3 inPointRelativeToCOM) {
	return to_jpc(to_jph(self)->GetPointVelocityCOM(to_jph(inPointRelativeToCOM)));
}

JPC_API JPC_Vec3 JPC_Body_GetPointVelocity(const JPC_Body* self, JPC_RVec3 inPoint) {
	return to_jpc(to_jph(self)->GetPointVelocity(to_jph(inPoint)));
}

JPC_API void JPC_Body_AddForce(JPC_Body* self, JPC_Vec3 inForce) {
	to_jph(self)->AddForce(to_jph(inForce));
}

// overload of Body::AddForce
JPC_API void JPC_Body_AddForceAtPoint(JPC_Body* self, JPC_Vec3 inForce, JPC_RVec3 inPosition) {
	to_jph(self)->AddForce(to_jph(inForce), to_jph(inPosition));
}

JPC_API void JPC_Body_AddTorque(JPC_Body* self, JPC_Vec3 inTorque) {
	to_jph(self)->AddTorque(to_jph(inTorque));
}

JPC_API JPC_Vec3 JPC_Body_GetAccumulatedForce(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAccumulatedForce());
}

JPC_API JPC_Vec3 JPC_Body_GetAccumulatedTorque(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAccumulatedTorque());
}

JPC_API void JPC_Body_ResetForce(JPC_Body* self) {
	to_jph(self)->ResetForce();
}

JPC_API void JPC_Body_ResetTorque(JPC_Body* self) {
	to_jph(self)->ResetTorque();
}

JPC_API void JPC_Body_ResetMotion(JPC_Body* self) {
	to_jph(self)->ResetMotion();
}

JPC_API void JPC_Body_GetInverseInertia(const JPC_Body* self, JPC_Mat44* outMatrix) {
	to_jph(self)->GetInverseInertia().StoreFloat4x4(reinterpret_cast<JPH::Float4*>(outMatrix));
}

JPC_API void JPC_Body_AddImpulse(JPC_Body* self, JPC_Vec3 inImpulse) {
	to_jph(self)->AddImpulse(to_jph(inImpulse));
}

JPC_API void JPC_Body_AddImpulse2(JPC_Body* self, JPC_Vec3 inImpulse, JPC_RVec3 inPosition) {
	to_jph(self)->AddImpulse(to_jph(inImpulse), to_jph(inPosition));
}

JPC_API void JPC_Body_AddAngularImpulse(JPC_Body* self, JPC_Vec3 inAngularImpulse) {
	to_jph(self)->AddAngularImpulse(to_jph(inAngularImpulse));
}

JPC_API void JPC_Body_MoveKinematic(JPC_Body* self, JPC_RVec3 inTargetPosition, JPC_Quat inTargetRotation, float inDeltaTime) {
	to_jph(self)->MoveKinematic(to_jph(inTargetPosition), to_jph(inTargetRotation), inDeltaTime);
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

JPC_API JPC_RMat44 JPC_Body_GetWorldTransform(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetWorldTransform());
}

JPC_API JPC_RVec3 JPC_Body_GetCenterOfMassPosition(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetCenterOfMassPosition());
}

JPC_API JPC_RMat44 JPC_Body_GetCenterOfMassTransform(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetCenterOfMassTransform());
}

JPC_API JPC_RMat44 JPC_Body_GetInverseCenterOfMassTransform(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetInverseCenterOfMassTransform());
}

// JPC_API const AABox & JPC_Body_GetWorldSpaceBounds(const JPC_Body* self);
// JPC_API const MotionProperties *JPC_Body_GetMotionProperties(const JPC_Body* self)
// JPC_API MotionProperties * JPC_Body_GetMotionProperties(JPC_Body* self);
// JPC_API const MotionProperties *JPC_Body_GetMotionPropertiesUnchecked(const JPC_Body* self)
// JPC_API MotionProperties * JPC_Body_GetMotionPropertiesUnchecked(JPC_Body* self);

JPC_API uint64_t JPC_Body_GetUserData(const JPC_Body* self) {
	return to_jph(self)->GetUserData();
}

JPC_API void JPC_Body_SetUserData(JPC_Body* self, uint64_t inUserData) {
	to_jph(self)->SetUserData(inUserData);
}

JPC_API JPC_Vec3 JPC_Body_GetWorldSpaceSurfaceNormal(const JPC_Body* self, JPC_SubShapeID inSubShapeID, JPC_RVec3 inPosition) {
	JPH::SubShapeID jph_id = JPC_SubShapeID_to_jph(inSubShapeID);

	return to_jpc(to_jph(self)->GetWorldSpaceSurfaceNormal(jph_id, to_jph(inPosition)));
}

// JPC_API TransformedShape JPC_Body_GetTransformedShape(const JPC_Body* self);
// JPC_API BodyCreationSettings JPC_Body_GetBodyCreationSettings(const JPC_Body* self);
// JPC_API SoftBodyCreationSettings JPC_Body_GetSoftBodyCreationSettings(const JPC_Body* self);

////////////////////////////////////////////////////////////////////////////////
// BodyLockRead

JPC_API JPC_BodyLockRead* JPC_BodyLockRead_new(const JPC_BodyLockInterface* interface, JPC_BodyID bodyID) {
	JPH::BodyLockRead* lockRead = new JPH::BodyLockRead(*to_jph(interface), to_jph(bodyID));
	return to_jpc(lockRead);
}

JPC_API void JPC_BodyLockRead_delete(JPC_BodyLockRead* self) {
	delete to_jph(self);
}

JPC_API bool JPC_BodyLockRead_Succeeded(JPC_BodyLockRead* self) {
	return to_jph(self)->Succeeded();
}

JPC_API const JPC_Body* JPC_BodyLockRead_GetBody(JPC_BodyLockRead* self) {
	return to_jpc(&to_jph(self)->GetBody());
}

////////////////////////////////////////////////////////////////////////////////
// BodyLockWrite

JPC_API JPC_BodyLockWrite* JPC_BodyLockWrite_new(const JPC_BodyLockInterface* interface, JPC_BodyID bodyID) {
	JPH::BodyLockWrite* lockWrite = new JPH::BodyLockWrite(*to_jph(interface), to_jph(bodyID));
	return to_jpc(lockWrite);
}

JPC_API void JPC_BodyLockWrite_delete(JPC_BodyLockWrite* self) {
	delete to_jph(self);
}

JPC_API bool JPC_BodyLockWrite_Succeeded(JPC_BodyLockWrite* self) {
	return to_jph(self)->Succeeded();
}

JPC_API JPC_Body* JPC_BodyLockWrite_GetBody(JPC_BodyLockWrite* self) {
	return to_jpc(&to_jph(self)->GetBody());
}

////////////////////////////////////////////////////////////////////////////////
// BodyLockMultiRead

typedef struct JPC_BodyLockMultiRead JPC_BodyLockMultiRead;

JPC_API JPC_BodyLockMultiRead* JPC_BodyLockMultiRead_new(
	const JPC_BodyLockInterface* interface,
	const JPC_BodyID *inBodyIDs,
	int inNumber)
{
	JPH::BodyLockMultiRead* lockRead = new JPH::BodyLockMultiRead(*to_jph(interface), to_jph(inBodyIDs), inNumber);
	return to_jpc(lockRead);
}

JPC_API void JPC_BodyLockMultiRead_delete(JPC_BodyLockMultiRead* self) {
	delete to_jph(self);
}

JPC_API const JPC_Body* JPC_BodyLockMultiRead_GetBody(JPC_BodyLockMultiRead* self, int inBodyIndex) {
	return to_jpc(to_jph(self)->GetBody(inBodyIndex));
}

////////////////////////////////////////////////////////////////////////////////
// BodyLockMultiWrite

typedef struct JPC_BodyLockMultiWrite JPC_BodyLockMultiWrite;

JPC_API JPC_BodyLockMultiWrite* JPC_BodyLockMultiWrite_new(
	const JPC_BodyLockInterface* interface,
	const JPC_BodyID *inBodyIDs,
	int inNumber)
{
	JPH::BodyLockMultiWrite* lockWrite = new JPH::BodyLockMultiWrite(*to_jph(interface), to_jph(inBodyIDs), inNumber);
	return to_jpc(lockWrite);
}

JPC_API void JPC_BodyLockMultiWrite_delete(JPC_BodyLockMultiWrite* self) {
	delete to_jph(self);
}

JPC_API JPC_Body* JPC_BodyLockMultiWrite_GetBody(JPC_BodyLockMultiWrite* self, int inBodyIndex) {
	return to_jpc(to_jph(self)->GetBody(inBodyIndex));
}

////////////////////////////////////////////////////////////////////////////////
// BodyInterface

JPC_API JPC_Body* JPC_BodyInterface_CreateBody(JPC_BodyInterface* self, const JPC_BodyCreationSettings* inSettings) {
	return to_jpc(to_jph(self)->CreateBody(to_jph(inSettings)));
}

// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBody(JPC_BodyInterface *self, const SoftBodyCreationSettings &inSettings);

JPC_API JPC_Body* JPC_BodyInterface_CreateBodyWithID(JPC_BodyInterface *self, JPC_BodyID inBodyID, const JPC_BodyCreationSettings* inSettings) {
	return to_jpc(to_jph(self)->CreateBodyWithID(to_jph(inBodyID), to_jph(inSettings)));
}

// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBodyWithID(JPC_BodyInterface *self, JPC_BodyID inBodyID, const SoftBodyCreationSettings* inSettings);

JPC_API JPC_Body* JPC_BodyInterface_CreateBodyWithoutID(const JPC_BodyInterface *self, const JPC_BodyCreationSettings* inSettings) {
	return to_jpc(to_jph(self)->CreateBodyWithoutID(to_jph(inSettings)));
}

// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBodyWithoutID(const JPC_BodyInterface *self, const SoftBodyCreationSettings* inSettings);

JPC_API void JPC_BodyInterface_DestroyBodyWithoutID(const JPC_BodyInterface *self, JPC_Body *inBody) {
	to_jph(self)->DestroyBodyWithoutID(to_jph(inBody));
}

JPC_API bool JPC_BodyInterface_AssignBodyID(JPC_BodyInterface *self, JPC_Body *ioBody) {
	return to_jph(self)->AssignBodyID(to_jph(ioBody));
}

// JPC_API bool JPC_BodyInterface_AssignBodyID(JPC_BodyInterface *self, JPC_Body *ioBody, JPC_BodyID inBodyID);

JPC_API JPC_Body* JPC_BodyInterface_UnassignBodyID(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->UnassignBodyID(to_jph(inBodyID)));
}

// JPC_API void JPC_BodyInterface_UnassignBodyIDs(JPC_BodyInterface *self, const JPC_BodyID *inBodyIDs, int inNumber, JPC_Body **outBodies) {
// 	return to_jph(self)->UnassignBodyIDs(to_jph(inBodyIDs), inNumber, to_jph(outBodies));
// }

JPC_API void JPC_BodyInterface_DestroyBody(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->DestroyBody(to_jph(inBodyID));
}

// JPC_API void JPC_BodyInterface_DestroyBodies(JPC_BodyInterface *self, const JPC_BodyID *inBodyIDs, int inNumber) {
// 	return to_jph(self)->DestroyBodies(to_jph(inBodyIDs), int inNumber);
// }

JPC_API void JPC_BodyInterface_AddBody(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Activation inActivationMode) {
	to_jph(self)->AddBody(to_jph(inBodyID), to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_RemoveBody(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->RemoveBody(to_jph(inBodyID));
}

JPC_API bool JPC_BodyInterface_IsAdded(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->IsAdded(to_jph(inBodyID));
}

JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddBody(JPC_BodyInterface *self, const JPC_BodyCreationSettings* inSettings, JPC_Activation inActivationMode) {
	return to_jpc(to_jph(self)->CreateAndAddBody(to_jph(inSettings), to_jph(inActivationMode)));
}

// JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddSoftBody(JPC_BodyInterface *self, const SoftBodyCreationSettings &inSettings, JPC_Activation inActivationMode);

JPC_API void* JPC_BodyInterface_AddBodiesPrepare(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber) {
	return to_jph(self)->AddBodiesPrepare(to_jph(ioBodies), inNumber);
}

JPC_API void JPC_BodyInterface_AddBodiesFinalize(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber, void* inAddState, JPC_Activation inActivationMode) {
	to_jph(self)->AddBodiesFinalize(to_jph(ioBodies), inNumber, inAddState, to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_AddBodiesAbort(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber, void* inAddState) {
	to_jph(self)->AddBodiesAbort(to_jph(ioBodies), inNumber, inAddState);
}

JPC_API void JPC_BodyInterface_RemoveBodies(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber) {
	to_jph(self)->RemoveBodies(to_jph(ioBodies), inNumber);
}

JPC_API void JPC_BodyInterface_ActivateBody(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->ActivateBody(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_ActivateBodies(JPC_BodyInterface *self, JPC_BodyID *inBodyIDs, int inNumber) {
	to_jph(self)->ActivateBodies(to_jph(inBodyIDs), inNumber);
}

// JPC_API void JPC_BodyInterface_ActivateBodiesInAABox(JPC_BodyInterface *self, const AABox &inBox, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter);

JPC_API void JPC_BodyInterface_DeactivateBody(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->DeactivateBody(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_DeactivateBodies(JPC_BodyInterface *self, JPC_BodyID *inBodyIDs, int inNumber) {
	to_jph(self)->DeactivateBodies(to_jph(inBodyIDs), inNumber);
}

JPC_API bool JPC_BodyInterface_IsActive(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->IsActive(to_jph(inBodyID));
}

// TwoBodyConstraint * JPC_BodyInterface_CreateConstraint(JPC_BodyInterface *self, const TwoBodyConstraintSettings *inSettings, JPC_BodyID inBodyID1, JPC_BodyID inBodyID2);
// JPC_API void JPC_BodyInterface_ActivateConstraint(JPC_BodyInterface *self, const TwoBodyConstraint *inConstraint);

JPC_API const JPC_Shape* JPC_BodyInterface_GetShape(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	// NOTE: This pointer will only be alive as long as BodyInterface holds onto it!
	return to_jpc(to_jph(self)->GetShape(to_jph(inBodyID)).GetPtr());
}

JPC_API void JPC_BodyInterface_SetShape(const JPC_BodyInterface *self, JPC_BodyID inBodyID, const JPC_Shape *inShape, bool inUpdateMassProperties, JPC_Activation inActivationMode) {
	to_jph(self)->SetShape(to_jph(inBodyID), to_jph(inShape), inUpdateMassProperties, to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_NotifyShapeChanged(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inPreviousCenterOfMass, bool inUpdateMassProperties, JPC_Activation inActivationMode) {
	to_jph(self)->NotifyShapeChanged(to_jph(inBodyID), to_jph(inPreviousCenterOfMass), inUpdateMassProperties, to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_SetObjectLayer(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_ObjectLayer inLayer) {
	to_jph(self)->SetObjectLayer(to_jph(inBodyID), inLayer);
}

JPC_API JPC_ObjectLayer JPC_BodyInterface_GetObjectLayer(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetObjectLayer(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetPositionAndRotation(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Activation inActivationMode) {
	to_jph(self)->SetPositionAndRotation(to_jph(inBodyID), to_jph(inPosition), to_jph(inRotation), to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_SetPositionAndRotationWhenChanged(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Activation inActivationMode) {
	to_jph(self)->SetPositionAndRotationWhenChanged(to_jph(inBodyID), to_jph(inPosition), to_jph(inRotation), to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_GetPositionAndRotation(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 *outPosition, JPC_Quat *outRotation) {
	JPH::RVec3 outPos{};
	JPH::Quat outRot{};

	to_jph(self)->GetPositionAndRotation(to_jph(inBodyID), outPos, outRot);

	*outPosition = to_jpc(outPos);
	*outRotation = to_jpc(outRot);
}

JPC_API void JPC_BodyInterface_SetPosition(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Activation inActivationMode) {
	to_jph(self)->SetPosition(to_jph(inBodyID), to_jph(inPosition), to_jph(inActivationMode));
}

JPC_API JPC_RVec3 JPC_BodyInterface_GetPosition(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetPosition(to_jph(inBodyID)));
}

JPC_API JPC_RVec3 JPC_BodyInterface_GetCenterOfMassPosition(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetCenterOfMassPosition(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_SetRotation(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Quat inRotation, JPC_Activation inActivationMode) {
	to_jph(self)->SetRotation(to_jph(inBodyID), to_jph(inRotation), to_jph(inActivationMode));
}

JPC_API JPC_Quat JPC_BodyInterface_GetRotation(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetRotation(to_jph(inBodyID)));
}

JPC_API JPC_RMat44 JPC_BodyInterface_GetWorldTransform(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetWorldTransform(to_jph(inBodyID)));
}

JPC_API JPC_RMat44 JPC_BodyInterface_GetCenterOfMassTransform(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetCenterOfMassTransform(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_MoveKinematic(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inTargetPosition, JPC_Quat inTargetRotation, float inDeltaTime) {
	to_jph(self)->MoveKinematic(to_jph(inBodyID), to_jph(inTargetPosition), to_jph(inTargetRotation), inDeltaTime);
}

JPC_API void JPC_BodyInterface_SetLinearAndAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetLinearAndAngularVelocity(to_jph(inBodyID), to_jph(inLinearVelocity), to_jph(inAngularVelocity));
}

JPC_API void JPC_BodyInterface_GetLinearAndAngularVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 *outLinearVelocity, JPC_Vec3 *outAngularVelocity) {
	JPH::Vec3 outLinVel;
	JPH::Vec3 outAngVel;

	to_jph(self)->GetLinearAndAngularVelocity(to_jph(inBodyID), outLinVel, outAngVel);

	*outLinearVelocity = to_jpc(outLinVel);
	*outAngularVelocity = to_jpc(outAngVel);
}

JPC_API void JPC_BodyInterface_SetLinearVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->SetLinearVelocity(to_jph(inBodyID), to_jph(inLinearVelocity));
}

JPC_API JPC_Vec3 JPC_BodyInterface_GetLinearVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetLinearVelocity(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_AddLinearVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->AddLinearVelocity(to_jph(inBodyID), to_jph(inLinearVelocity));
}

JPC_API void JPC_BodyInterface_AddLinearAndAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->AddLinearAndAngularVelocity(to_jph(inBodyID), to_jph(inLinearVelocity), to_jph(inAngularVelocity));
}

JPC_API void JPC_BodyInterface_SetAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetAngularVelocity(to_jph(inBodyID), to_jph(inAngularVelocity));
}

JPC_API JPC_Vec3 JPC_BodyInterface_GetAngularVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetAngularVelocity(to_jph(inBodyID)));
}

JPC_API JPC_Vec3 JPC_BodyInterface_GetPointVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPoint) {
	return to_jpc(to_jph(self)->GetPointVelocity(to_jph(inBodyID), to_jph(inPoint)));
}

JPC_API void JPC_BodyInterface_SetPositionRotationAndVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetPositionRotationAndVelocity(to_jph(inBodyID), to_jph(inPosition), to_jph(inRotation), to_jph(inLinearVelocity), to_jph(inAngularVelocity));
}

JPC_API void JPC_BodyInterface_AddForce(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce) {
	to_jph(self)->AddForce(to_jph(inBodyID), to_jph(inForce));
}

// overload of BodyInterface::AddForce
JPC_API void JPC_BodyInterface_AddForceAtPoint(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce, JPC_RVec3 inPoint) {
	to_jph(self)->AddForce(to_jph(inBodyID), to_jph(inForce), to_jph(inPoint));
}

JPC_API void JPC_BodyInterface_AddTorque(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inTorque) {
	to_jph(self)->AddTorque(to_jph(inBodyID), to_jph(inTorque));
}

JPC_API void JPC_BodyInterface_AddForceAndTorque(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce, JPC_Vec3 inTorque) {
	to_jph(self)->AddForceAndTorque(to_jph(inBodyID), to_jph(inForce), to_jph(inTorque));
}

JPC_API void JPC_BodyInterface_AddImpulse(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inImpulse) {
	to_jph(self)->AddImpulse(to_jph(inBodyID), to_jph(inImpulse));
}

JPC_API void JPC_BodyInterface_AddImpulse3(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inImpulse, JPC_RVec3 inPoint) {
	to_jph(self)->AddImpulse(to_jph(inBodyID), to_jph(inImpulse), to_jph(inPoint));
}

JPC_API void JPC_BodyInterface_AddAngularImpulse(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inAngularImpulse) {
	to_jph(self)->AddAngularImpulse(to_jph(inBodyID), to_jph(inAngularImpulse));
}

JPC_API JPC_BodyType JPC_BodyInterface_GetBodyType(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetBodyType(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_SetMotionType(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_MotionType inMotionType, JPC_Activation inActivationMode) {
	to_jph(self)->SetMotionType(to_jph(inBodyID), to_jph(inMotionType), to_jph(inActivationMode));
}

JPC_API JPC_MotionType JPC_BodyInterface_GetMotionType(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetMotionType(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_SetMotionQuality(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_MotionQuality inMotionQuality) {
	to_jph(self)->SetMotionQuality(to_jph(inBodyID), to_jph(inMotionQuality));
}

JPC_API JPC_MotionQuality JPC_BodyInterface_GetMotionQuality(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetMotionQuality(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_GetInverseInertia(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Mat44 *outMatrix) {
	to_jph(self)->GetInverseInertia(to_jph(inBodyID)).StoreFloat4x4(reinterpret_cast<JPH::Float4*>(outMatrix));
}

JPC_API void JPC_BodyInterface_SetRestitution(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inRestitution) {
	to_jph(self)->SetRestitution(to_jph(inBodyID), inRestitution);
}

JPC_API float JPC_BodyInterface_GetRestitution(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetRestitution(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetFriction(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inFriction) {
	to_jph(self)->SetFriction(to_jph(inBodyID), inFriction);
}

JPC_API float JPC_BodyInterface_GetFriction(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetFriction(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetGravityFactor(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inGravityFactor) {
	to_jph(self)->SetGravityFactor(to_jph(inBodyID), inGravityFactor);
}

JPC_API float JPC_BodyInterface_GetGravityFactor(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetGravityFactor(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetUseManifoldReduction(JPC_BodyInterface *self, JPC_BodyID inBodyID, bool inUseReduction) {
	to_jph(self)->SetUseManifoldReduction(to_jph(inBodyID), inUseReduction);
}

JPC_API bool JPC_BodyInterface_GetUseManifoldReduction(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetUseManifoldReduction(to_jph(inBodyID));
}

// TransformedShape JPC_BodyInterface_GetTransformedShape(const JPC_BodyInterface *self, JPC_BodyID inBodyID);

JPC_API uint64_t JPC_BodyInterface_GetUserData(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetUserData(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetUserData(const JPC_BodyInterface *self, JPC_BodyID inBodyID, uint64_t inUserData) {
	to_jph(self)->SetUserData(to_jph(inBodyID), inUserData);
}

// const PhysicsMaterial* JPC_BodyInterface_GetMaterial(const JPC_BodyInterface *self, JPC_BodyID inBodyID, const SubShapeID &inSubShapeID);

JPC_API void JPC_BodyInterface_InvalidateContactCache(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->InvalidateContactCache(to_jph(inBodyID));
}

////////////////////////////////////////////////////////////////////////////////
// NarrowPhaseQuery

JPC_API bool JPC_NarrowPhaseQuery_CastRay(const JPC_NarrowPhaseQuery* self, JPC_NarrowPhaseQuery_CastRayArgs* args) {
	JPH::RayCastResult result;

	JPH::RayCastSettings settings;

	JPH::BroadPhaseLayerFilter defaultBplFilter{};
	const JPH::BroadPhaseLayerFilter* bplFilter = &defaultBplFilter;
	if (args->BroadPhaseLayerFilter != nullptr) {
		bplFilter = to_jph(args->BroadPhaseLayerFilter);
	}

	JPH::ObjectLayerFilter defaultOlFilter{};
	const JPH::ObjectLayerFilter* olFilter = &defaultOlFilter;
	if (args->ObjectLayerFilter != nullptr) {
		olFilter = to_jph(args->ObjectLayerFilter);
	}

	JPH::BodyFilter defaultBodyFilter{};
	const JPH::BodyFilter* bodyFilter = &defaultBodyFilter;
	if (args->BodyFilter != nullptr) {
		bodyFilter = to_jph(args->BodyFilter);
	}

	JPH::ShapeFilter defaultShapeFilter{};
	const JPH::ShapeFilter* shapeFilter = &defaultShapeFilter;
	if (args->ShapeFilter != nullptr) {
		shapeFilter = to_jph(args->ShapeFilter);
	}

	JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;

	to_jph(self)->CastRay(
		to_jph(args->Ray),
		settings,
		collector,
		*bplFilter,
		*olFilter,
		*bodyFilter,
		*shapeFilter);

	bool hit = collector.HadHit();
	if (hit) {
		args->Result = to_jpc(collector.mHit);
	}

	return hit;
}

JPC_API void JPC_ShapeCastSettings_default(JPC_ShapeCastSettings* object) {
	JPH::ShapeCastSettings defaultSettings{};
	*object = to_jpc(defaultSettings);
}

JPC_API void JPC_NarrowPhaseQuery_CastShape(const JPC_NarrowPhaseQuery* self, JPC_NarrowPhaseQuery_CastShapeArgs* args) {
	JPH::ShapeCastSettings settings = to_jph(args->Settings);

	JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> defaultCollector{};
	JPH::CastShapeCollector* collector = &defaultCollector;
	if (args->Collector != nullptr) {
		collector = to_jph(args->Collector);
	}

	JPH::BroadPhaseLayerFilter defaultBplFilter{};
	const JPH::BroadPhaseLayerFilter* bplFilter = &defaultBplFilter;
	if (args->BroadPhaseLayerFilter != nullptr) {
		bplFilter = to_jph(args->BroadPhaseLayerFilter);
	}

	JPH::ObjectLayerFilter defaultOlFilter{};
	const JPH::ObjectLayerFilter* olFilter = &defaultOlFilter;
	if (args->ObjectLayerFilter != nullptr) {
		olFilter = to_jph(args->ObjectLayerFilter);
	}

	JPH::BodyFilter defaultBodyFilter{};
	const JPH::BodyFilter* bodyFilter = &defaultBodyFilter;
	if (args->BodyFilter != nullptr) {
		bodyFilter = to_jph(args->BodyFilter);
	}

	JPH::ShapeFilter defaultShapeFilter{};
	const JPH::ShapeFilter* shapeFilter = &defaultShapeFilter;
	if (args->ShapeFilter != nullptr) {
		shapeFilter = to_jph(args->ShapeFilter);
	}

	to_jph(self)->CastShape(
		to_jph(args->ShapeCast),
		settings,
		to_jph(args->BaseOffset),
		*collector,
		*bplFilter,
		*olFilter,
		*bodyFilter,
		*shapeFilter);
}

JPC_API void JPC_CollideShapeSettings_default(JPC_CollideShapeSettings* object) {
	JPH::CollideShapeSettings defaultSettings{};
	*object = to_jpc(defaultSettings);
}

JPC_API void JPC_NarrowPhaseQuery_CollideShape(const JPC_NarrowPhaseQuery* self, JPC_NarrowPhaseQuery_CollideShapeArgs* args) {
	JPH::CollideShapeSettings settings = to_jph(args->Settings);

	JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> defaultCollector{};
	JPH::CollideShapeCollector* collector = &defaultCollector;
	if (args->Collector != nullptr) {
		collector = to_jph(args->Collector);
	}

	JPH::BroadPhaseLayerFilter defaultBplFilter{};
	const JPH::BroadPhaseLayerFilter* bplFilter = &defaultBplFilter;
	if (args->BroadPhaseLayerFilter != nullptr) {
		bplFilter = to_jph(args->BroadPhaseLayerFilter);
	}

	JPH::ObjectLayerFilter defaultOlFilter{};
	const JPH::ObjectLayerFilter* olFilter = &defaultOlFilter;
	if (args->ObjectLayerFilter != nullptr) {
		olFilter = to_jph(args->ObjectLayerFilter);
	}

	JPH::BodyFilter defaultBodyFilter{};
	const JPH::BodyFilter* bodyFilter = &defaultBodyFilter;
	if (args->BodyFilter != nullptr) {
		bodyFilter = to_jph(args->BodyFilter);
	}

	JPH::ShapeFilter defaultShapeFilter{};
	const JPH::ShapeFilter* shapeFilter = &defaultShapeFilter;
	if (args->ShapeFilter != nullptr) {
		shapeFilter = to_jph(args->ShapeFilter);
	}

	to_jph(self)->CollideShape(
		to_jph(args->Shape),
		to_jph(args->ShapeScale),
		to_jph(args->CenterOfMassTransform),
		settings,
		to_jph(args->BaseOffset),
		*collector,
		*bplFilter,
		*olFilter,
		*bodyFilter,
		*shapeFilter);
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

JPC_API void JPC_PhysicsSystem_OptimizeBroadPhase(JPC_PhysicsSystem* self) {
	to_jph(self)->OptimizeBroadPhase();
}

JPC_API void JPC_PhysicsSystem_AddConstraint(JPC_PhysicsSystem* self, JPC_Constraint* constraint) {
	to_jph(self)->AddConstraint(to_jph(constraint));
}

JPC_API void JPC_PhysicsSystem_RemoveConstraint(JPC_PhysicsSystem* self, JPC_Constraint* constraint) {
	to_jph(self)->RemoveConstraint(to_jph(constraint));
}

JPC_API JPC_BodyInterface* JPC_PhysicsSystem_GetBodyInterface(JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBodyInterface());
}

JPC_API const JPC_BodyLockInterface* JPC_PhysicsSystem_GetBodyLockInterface(JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBodyLockInterface());
}

JPC_API const JPC_NarrowPhaseQuery* JPC_PhysicsSystem_GetNarrowPhaseQuery(const JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetNarrowPhaseQuery());
}

JPC_API JPC_PhysicsUpdateError JPC_PhysicsSystem_Update(
	JPC_PhysicsSystem* self,
	float inDeltaTime,
	int inCollisionSteps,
	JPC_TempAllocatorImpl *inTempAllocator,
	JPC_JobSystem *inJobSystem)
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

JPC_API void JPC_PhysicsSystem_DrawConstraints(
	JPC_PhysicsSystem* self,
	JPC_DebugRendererSimple* inRenderer)
{
	to_jph(self)->DrawConstraints(to_jph(inRenderer));
}


JPC_API void JPC_PhysicsSystem_SetSimShapeFilter(
	JPC_PhysicsSystem* self,
	const JPC_SimShapeFilter* inShapeFilter)
{
	to_jph(self)->SetSimShapeFilter(to_jph(inShapeFilter));
}

JPC_API void JPC_PhysicsSystem_SetContactListener(
	JPC_PhysicsSystem* self,
	JPC_ContactListener* inContactListener)
{
	to_jph(self)->SetContactListener(to_jph(inContactListener));
}
