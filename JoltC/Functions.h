#pragma once

#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef _MSC_VER
	#define JPC_API extern __declspec(dllexport)
#else
	#define JPC_API
#endif

static float JPC_PI = 3.14159265358979323846f;

// C-compatible typedefs that match Jolt's internal primitive typedefs.
#define uint unsigned int

#ifdef __cplusplus
extern "C" {
#endif

JPC_API void JPC_RegisterDefaultAllocator();
JPC_API void JPC_FactoryInit();
JPC_API void JPC_FactoryDelete();
JPC_API void JPC_RegisterTypes();
JPC_API void JPC_UnregisterTypes();

////////////////////////////////////////////////////////////////////////////////
// Primitive types

typedef struct JPC_Float3 {
	float x;
	float y;
	float z;
} JPC_Float3;

ENSURE_SIZE_ALIGN(JPC_Float3, JPH::Float3)

typedef struct JPC_Vec3 {
	alignas(16) float x;
	float y;
	float z;
	float _w;
} JPC_Vec3;

ENSURE_SIZE_ALIGN(JPC_Vec3, JPH::Vec3)

typedef struct JPC_Vec4 {
	alignas(16) float x;
	float y;
	float z;
	float w;
} JPC_Vec4;

ENSURE_SIZE_ALIGN(JPC_Vec4, JPH::Vec4)

typedef struct JPC_DVec3 {
	alignas(32) double x;
	double y;
	double z;
	double _w;
} JPC_DVec3;

ENSURE_SIZE_ALIGN(JPC_DVec3, JPH::DVec3)

typedef struct JPC_Quat {
	alignas(16) float x;
	float y;
	float z;
	float w;
} JPC_Quat;

ENSURE_SIZE_ALIGN(JPC_Quat, JPH::Quat)

typedef struct JPC_Mat44 {
	alignas(16) JPC_Vec4 col[3];
	JPC_Vec3 col3;
} JPC_Mat44;

ENSURE_SIZE_ALIGN(JPC_Mat44, JPH::Mat44)

typedef struct JPC_DMat44 {
	alignas(32) JPC_Vec4 col[3];
	JPC_DVec3 col3;
} JPC_DMat44;

ENSURE_SIZE_ALIGN(JPC_DMat44, JPH::DMat44)

typedef struct JPC_Color {
	alignas(uint32_t) uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
} JPC_Color;

ENSURE_SIZE_ALIGN(JPC_Color, JPH::Color)

#ifdef JPC_DOUBLE_PRECISION
	typedef JPC_DVec3 JPC_RVec3;
	typedef JPC_DMat44 JPC_RMat44;
	typedef double Real;
#else
	typedef JPC_Vec3 JPC_RVec3;
	typedef JPC_Mat44 JPC_RMat44;
	typedef float Real;
#endif

ENSURE_SIZE_ALIGN(JPC_RVec3, JPH::RVec3)

typedef uint32_t JPC_BodyID;
ENSURE_SIZE_ALIGN(JPC_BodyID, JPH::BodyID)

typedef uint32_t JPC_SubShapeID;
ENSURE_SIZE_ALIGN(JPC_SubShapeID, JPH::SubShapeID)

typedef uint8_t JPC_BroadPhaseLayer;
ENSURE_SIZE_ALIGN(JPC_BroadPhaseLayer, JPH::BroadPhaseLayer)

#ifndef JPC_OBJECT_LAYER_BITS
	#define JPC_OBJECT_LAYER_BITS 16
#endif

#if JPC_OBJECT_LAYER_BITS == 16
	typedef uint16_t JPC_ObjectLayer;
#elif JPC_OBJECT_LAYER_BITS == 32
	typedef uint32_t JPC_ObjectLayer;
#else
	#error "JPC_OBJECT_LAYER_BITS must be 16 or 32"
#endif

ENSURE_SIZE_ALIGN(JPC_ObjectLayer, JPH::ObjectLayer)

typedef struct JPC_IndexedTriangleNoMaterial {
	uint32_t idx[3];
} JPC_IndexedTriangleNoMaterial;

ENSURE_SIZE_ALIGN(JPC_IndexedTriangleNoMaterial, JPH::IndexedTriangleNoMaterial)

typedef struct JPC_IndexedTriangle {
	uint32_t idx[3];
	uint32_t materialIndex;
} JPC_IndexedTriangle;

ENSURE_SIZE_ALIGN(JPC_IndexedTriangle, JPH::IndexedTriangle)

typedef struct JPC_RayCast {
	JPC_Vec3 Origin;
	JPC_Vec3 Direction;
} JPC_RayCast;

typedef struct JPC_RRayCast {
	JPC_RVec3 Origin;
	JPC_Vec3 Direction;
} JPC_RRayCast;

typedef struct JPC_RayCastResult {
	JPC_BodyID BodyID;
	float Fraction;
	JPC_SubShapeID SubShapeID2;
} JPC_RayCastResult;

typedef struct JPC_ShapeCastResult {
	// From CollideShapeResult
	JPC_Vec3 ContactPointOn1;
	JPC_Vec3 ContactPointOn2;
	JPC_Vec3 PenetrationAxis;
	float PenetrationDepth;
	// SubShapeID SubShapeID1;
	// SubShapeID SubShapeID2;
	JPC_BodyID BodyID2;
	// Face Shape1Face;
	// Face Shape2Face;

	// From ShapeCastResult
	float Fraction;
	bool IsBackFaceHit;
} JPC_ShapeCastResult;

typedef struct JPC_Body JPC_Body;

////////////////////////////////////////////////////////////////////////////////
// VertexList == Array<Float3> == std::vector<Float3>

typedef struct JPC_VertexList JPC_VertexList;

JPC_API JPC_VertexList* JPC_VertexList_new(const JPC_Float3* storage, size_t len);
JPC_API void JPC_VertexList_delete(JPC_VertexList* object);

////////////////////////////////////////////////////////////////////////////////
// IndexedTriangleList == Array<IndexedTriangle> == std::vector<IndexedTriangle>

typedef struct JPC_IndexedTriangleList JPC_IndexedTriangleList;

JPC_API JPC_IndexedTriangleList* JPC_IndexedTriangleList_new(const JPC_IndexedTriangle* storage, size_t len);
JPC_API void JPC_IndexedTriangleList_delete(JPC_IndexedTriangleList* object);

////////////////////////////////////////////////////////////////////////////////
// Shape -> RefTarget

typedef struct JPC_Shape JPC_Shape;

JPC_API uint32_t JPC_Shape_GetRefCount(const JPC_Shape* self);
JPC_API void JPC_Shape_AddRef(const JPC_Shape* self);
JPC_API void JPC_Shape_Release(const JPC_Shape* self);

JPC_API uint64_t JPC_Shape_GetUserData(const JPC_Shape* self);
JPC_API void JPC_Shape_SetUserData(JPC_Shape* self, uint64_t userData);

JPC_API JPC_ShapeType JPC_Shape_GetType(const JPC_Shape* self);
JPC_API JPC_ShapeSubType JPC_Shape_GetSubType(const JPC_Shape* self);

JPC_API JPC_Vec3 JPC_Shape_GetCenterOfMass(const JPC_Shape* self);

////////////////////////////////////////////////////////////////////////////////
// CompoundShape -> Shape -> RefTarget

typedef struct JPC_CompoundShape JPC_CompoundShape;

JPC_API uint32_t JPC_CompoundShape_GetSubShapeIndexFromID(
	const JPC_CompoundShape* self,
	JPC_SubShapeID inSubShapeID,
	JPC_SubShapeID* outRemainder);

////////////////////////////////////////////////////////////////////////////////
// TempAllocatorImpl

typedef struct JPC_TempAllocatorImpl JPC_TempAllocatorImpl;

JPC_API JPC_TempAllocatorImpl* JPC_TempAllocatorImpl_new(uint size);
JPC_API void JPC_TempAllocatorImpl_delete(JPC_TempAllocatorImpl* object);

////////////////////////////////////////////////////////////////////////////////
// JobSystemThreadPool

typedef struct JPC_JobSystemThreadPool JPC_JobSystemThreadPool;

JPC_API JPC_JobSystemThreadPool* JPC_JobSystemThreadPool_new2(
	uint inMaxJobs,
	uint inMaxBarriers);
JPC_API JPC_JobSystemThreadPool* JPC_JobSystemThreadPool_new3(
	uint inMaxJobs,
	uint inMaxBarriers,
	int inNumThreads);

JPC_API void JPC_JobSystemThreadPool_delete(JPC_JobSystemThreadPool* object);

////////////////////////////////////////////////////////////////////////////////
// BroadPhaseLayerInterface

typedef struct JPC_BroadPhaseLayerInterfaceFns {
	uint (*GetNumBroadPhaseLayers)(const void *self);
	JPC_BroadPhaseLayer (*GetBroadPhaseLayer)(const void *self, JPC_ObjectLayer inLayer);
} JPC_BroadPhaseLayerInterfaceFns;

typedef struct JPC_BroadPhaseLayerInterface JPC_BroadPhaseLayerInterface;

JPC_API JPC_BroadPhaseLayerInterface* JPC_BroadPhaseLayerInterface_new(
	const void *self,
	JPC_BroadPhaseLayerInterfaceFns fns);

JPC_API void JPC_BroadPhaseLayerInterface_delete(JPC_BroadPhaseLayerInterface* object);

////////////////////////////////////////////////////////////////////////////////
// BroadPhaseLayerFilter

typedef struct JPC_BroadPhaseLayerFilterFns {
	bool (*ShouldCollide)(const void *self, JPC_BroadPhaseLayer inLayer);
} JPC_BroadPhaseLayerFilterFns;

typedef struct JPC_BroadPhaseLayerFilter JPC_BroadPhaseLayerFilter;

JPC_API JPC_BroadPhaseLayerFilter* JPC_BroadPhaseLayerFilter_new(
	const void *self,
	JPC_BroadPhaseLayerFilterFns fns);

JPC_API void JPC_BroadPhaseLayerFilter_delete(JPC_BroadPhaseLayerFilter* object);

////////////////////////////////////////////////////////////////////////////////
// ObjectLayerFilter

typedef struct JPC_ObjectLayerFilterFns {
	bool (*ShouldCollide)(const void *self, JPC_ObjectLayer inLayer);
} JPC_ObjectLayerFilterFns;

typedef struct JPC_ObjectLayerFilter JPC_ObjectLayerFilter;

JPC_API JPC_ObjectLayerFilter* JPC_ObjectLayerFilter_new(
	const void *self,
	JPC_ObjectLayerFilterFns fns);

JPC_API void JPC_ObjectLayerFilter_delete(JPC_ObjectLayerFilter* object);

////////////////////////////////////////////////////////////////////////////////
// BodyFilter

typedef struct JPC_BodyFilterFns {
	bool (*ShouldCollide)(const void *self, JPC_BodyID inBodyID);
	bool (*ShouldCollideLocked)(const void *self, const JPC_Body *inBodyID);
} JPC_BodyFilterFns;

typedef struct JPC_BodyFilter JPC_BodyFilter;

JPC_API JPC_BodyFilter* JPC_BodyFilter_new(
	const void *self,
	JPC_BodyFilterFns fns);

JPC_API void JPC_BodyFilter_delete(JPC_BodyFilter* object);

////////////////////////////////////////////////////////////////////////////////
// ShapeFilter

typedef struct JPC_ShapeFilterFns {
	bool (*ShouldCollide)(const void *self, const JPC_Shape *inShape2, JPC_SubShapeID inSubShapeIDOfShape2);

	// virtual bool			ShouldCollide([[maybe_unused]] const Shape *inShape1, [[maybe_unused]] const SubShapeID &inSubShapeIDOfShape1, [[maybe_unused]] const Shape *inShape2, [[maybe_unused]] const SubShapeID &inSubShapeIDOfShape2) const
} JPC_ShapeFilterFns;

typedef struct JPC_ShapeFilter JPC_ShapeFilter;

JPC_API JPC_ShapeFilter* JPC_ShapeFilter_new(
	const void *self,
	JPC_ShapeFilterFns fns);

JPC_API void JPC_ShapeFilter_delete(JPC_ShapeFilter* object);

////////////////////////////////////////////////////////////////////////////////
// ObjectVsBroadPhaseLayerFilter

typedef struct JPC_ObjectVsBroadPhaseLayerFilterFns {
	bool (*ShouldCollide)(const void *self, JPC_ObjectLayer inLayer1, JPC_BroadPhaseLayer inLayer2);
} JPC_ObjectVsBroadPhaseLayerFilterFns;

typedef struct JPC_ObjectVsBroadPhaseLayerFilter JPC_ObjectVsBroadPhaseLayerFilter;

JPC_API JPC_ObjectVsBroadPhaseLayerFilter* JPC_ObjectVsBroadPhaseLayerFilter_new(
	const void *self,
	JPC_ObjectVsBroadPhaseLayerFilterFns fns);

JPC_API void JPC_ObjectVsBroadPhaseLayerFilter_delete(JPC_ObjectVsBroadPhaseLayerFilter* object);

////////////////////////////////////////////////////////////////////////////////
// ObjectLayerPairFilter

typedef struct JPC_ObjectLayerPairFilterFns {
	bool (*ShouldCollide)(const void *self, JPC_ObjectLayer inLayer1, JPC_ObjectLayer inLayer2);
} JPC_ObjectLayerPairFilterFns;

typedef struct JPC_ObjectLayerPairFilter JPC_ObjectLayerPairFilter;

JPC_API JPC_ObjectLayerPairFilter* JPC_ObjectLayerPairFilter_new(
	const void *self,
	JPC_ObjectLayerPairFilterFns fns);

JPC_API void JPC_ObjectLayerPairFilter_delete(JPC_ObjectLayerPairFilter* object);

////////////////////////////////////////////////////////////////////////////////
// ContactListener

typedef struct JPC_ContactPoints {
	uint length;
	JPC_Vec3 points[64];
} JPC_ContactPoints;

ENSURE_SIZE_ALIGN(JPC_ContactPoints, JPH::ContactPoints)

typedef struct JPC_ContactManifold {
	JPC_RVec3 BaseOffset;
	JPC_Vec3 WorldSpaceNormal;
	float PenetrationDepth;
	JPC_SubShapeID SubShapeID1;
	JPC_SubShapeID SubShapeID2;
	JPC_ContactPoints RelativeContactPointsOn1;
	JPC_ContactPoints RelativeContactPointsOn2;
} JPC_ContactManifold;

ENSURE_SIZE_ALIGN(JPC_ContactManifold, JPH::ContactManifold)
ENSURE_NORMAL_FIELD(  ContactManifold, BaseOffset)
ENSURE_NORMAL_FIELD(  ContactManifold, WorldSpaceNormal)
ENSURE_NORMAL_FIELD(  ContactManifold, PenetrationDepth)
ENSURE_NORMAL_FIELD(  ContactManifold, SubShapeID1)
ENSURE_NORMAL_FIELD(  ContactManifold, SubShapeID2)
ENSURE_NORMAL_FIELD(  ContactManifold, RelativeContactPointsOn1)
ENSURE_NORMAL_FIELD(  ContactManifold, RelativeContactPointsOn2)

typedef struct JPC_ContactSettings {
	float CombinedFriction;
	float CombinedRestitution;
	float InvMassScale1;
	float InvInertiaScale1;
	float InvMassScale2;
	float InvInertiaScale2;
	bool IsSensor;
	JPC_Vec3 RelativeLinearSurfaceVelocity;
	JPC_Vec3 RelativeAngularSurfaceVelocity;
} JPC_ContactSettings;

ENSURE_SIZE_ALIGN(JPC_ContactSettings, JPH::ContactSettings)
ENSURE_NORMAL_FIELD(  ContactSettings, CombinedFriction)
ENSURE_NORMAL_FIELD(  ContactSettings, CombinedRestitution)
ENSURE_NORMAL_FIELD(  ContactSettings, InvMassScale1)
ENSURE_NORMAL_FIELD(  ContactSettings, InvInertiaScale1)
ENSURE_NORMAL_FIELD(  ContactSettings, InvMassScale2)
ENSURE_NORMAL_FIELD(  ContactSettings, InvInertiaScale2)
ENSURE_NORMAL_FIELD(  ContactSettings, IsSensor)
ENSURE_NORMAL_FIELD(  ContactSettings, RelativeLinearSurfaceVelocity)
ENSURE_NORMAL_FIELD(  ContactSettings, RelativeAngularSurfaceVelocity)

typedef struct JPC_SubShapeIDPair {
	JPC_BodyID Body1ID;
	JPC_SubShapeID SubShapeID1;
	JPC_BodyID Body2ID;
	JPC_SubShapeID SubShapeID2;
} JPC_SubShapeIDPair;

ENSURE_SIZE_ALIGN(JPC_SubShapeIDPair, JPH::SubShapeIDPair)

typedef struct JPC_CollideShapeSettings {
	// CollideSettingsBase
	JPC_ActiveEdgeMode ActiveEdgeMode;
	JPC_CollectFacesMode CollectFacesMode;
	float CollisionTolerance;
	float PenetrationTolerance;
	JPC_Vec3 ActiveEdgeMovementDirection;

	// CollideShapeSettings
	float MaxSeparationDistance;
	JPC_BackFaceMode BackFaceMode;
} JPC_CollideShapeSettings;

ENSURE_SIZE_ALIGN(JPC_CollideShapeSettings, JPH::CollideShapeSettings)

typedef struct JPC_ContactListenerFns {
	// ValidateResult (*OnContactValidate)(
	// 	void *self,
	// 	const Body &inBody1,
	// 	const Body &inBody2,
	// 	RVec3Arg inBaseOffset,
	// 	const CollideShapeResult &inCollisionResult);

	void (*OnContactAdded)(
		void *self,
		const JPC_Body *inBody1,
		const JPC_Body *inBody2,
		const JPC_ContactManifold *inManifold,
		JPC_ContactSettings *ioSettings);

	void (*OnContactPersisted)(
		void *self,
		const JPC_Body *inBody1,
		const JPC_Body *inBody2,
		const JPC_ContactManifold *inManifold,
		JPC_ContactSettings *ioSettings);

	void (*OnContactRemoved)(
		void *self,
		const JPC_SubShapeIDPair *inSubShapePair);
} JPC_ContactListenerFns;

typedef struct JPC_ContactListener JPC_ContactListener;

JPC_API JPC_ContactListener* JPC_ContactListener_new(
	void *self,
	JPC_ContactListenerFns fns);

JPC_API void JPC_ContactListener_delete(JPC_ContactListener* object);

////////////////////////////////////////////////////////////////////////////////
// CastShapeCollector

typedef struct JPC_CastShapeCollector JPC_CastShapeCollector;

typedef struct JPC_CastShapeCollectorFns {
	void (*Reset)(void *self);
	void (*AddHit)(void *self, JPC_CastShapeCollector *base, const JPC_ShapeCastResult *Result);
} JPC_CastShapeCollectorFns;

JPC_API JPC_CastShapeCollector* JPC_CastShapeCollector_new(
	void *self,
	JPC_CastShapeCollectorFns fns);

JPC_API void JPC_CastShapeCollector_delete(JPC_CastShapeCollector* object);

JPC_API void JPC_CastShapeCollector_UpdateEarlyOutFraction(JPC_CastShapeCollector *self, float inFraction);

////////////////////////////////////////////////////////////////////////////////
// DrawSettings

typedef struct JPC_BodyManager_DrawSettings {
	bool mDrawGetSupportFunction;
	bool mDrawSupportDirection;
	bool mDrawGetSupportingFace;
	bool mDrawShape;
	bool mDrawShapeWireframe;
	JPC_ShapeColor mDrawShapeColor;
	bool mDrawBoundingBox;
	bool mDrawCenterOfMassTransform;
	bool mDrawWorldTransform;
	bool mDrawVelocity;
	bool mDrawMassAndInertia;
	bool mDrawSleepStats;
	bool mDrawSoftBodyVertices;
	bool mDrawSoftBodyVertexVelocities;
	bool mDrawSoftBodyEdgeConstraints;
	bool mDrawSoftBodyBendConstraints;
	bool mDrawSoftBodyVolumeConstraints;
	bool mDrawSoftBodySkinConstraints;
	bool mDrawSoftBodyLRAConstraints;
	bool mDrawSoftBodyPredictedBounds;
	JPC_SoftBodyConstraintColor DrawSoftBodyConstraintColor;
} JPC_BodyManager_DrawSettings;

ENSURE_SIZE_ALIGN(JPC_BodyManager_DrawSettings, JPH::BodyManager::DrawSettings)

JPC_API void JPC_BodyManager_DrawSettings_default(JPC_BodyManager_DrawSettings* object);

////////////////////////////////////////////////////////////////////////////////
// DebugRendererSimple

typedef struct JPC_DebugRendererSimpleFns {
	void (*DrawLine)(const void *self, JPC_RVec3 inFrom, JPC_RVec3 inTo, JPC_Color inColor);
} JPC_DebugRendererSimpleFns;

typedef struct JPC_DebugRendererSimple JPC_DebugRendererSimple;

JPC_API JPC_DebugRendererSimple* JPC_DebugRendererSimple_new(
	const void *self,
	JPC_DebugRendererSimpleFns fns);

JPC_API void JPC_DebugRendererSimple_delete(JPC_DebugRendererSimple* object);

////////////////////////////////////////////////////////////////////////////////
// String

typedef struct JPC_String JPC_String;

JPC_API void JPC_String_delete(JPC_String* self);
JPC_API const char* JPC_String_c_str(JPC_String* self);

////////////////////////////////////////////////////////////////////////////////
// TriangleShapeSettings

typedef struct JPC_TriangleShapeSettings {
	// ShapeSettings
	uint64_t UserData;

	// ConvexShapeSettings
	// TODO: Material
	float Density;

	// TriangleShapeSettings
	JPC_Vec3 V1;
	JPC_Vec3 V2;
	JPC_Vec3 V3;
	float ConvexRadius;
} JPC_TriangleShapeSettings;

JPC_API void JPC_TriangleShapeSettings_default(JPC_TriangleShapeSettings* object);
JPC_API bool JPC_TriangleShapeSettings_Create(const JPC_TriangleShapeSettings* self, JPC_Shape** outShape, JPC_String** outError);

////////////////////////////////////////////////////////////////////////////////
// BoxShapeSettings -> ConvexShapeSettings -> ShapeSettings

typedef struct JPC_BoxShapeSettings {
	// ShapeSettings
	uint64_t UserData;

	// ConvexShapeSettings
	// TODO: Material
	float Density;

	// BoxShapeSettings
	JPC_Vec3 HalfExtent;
	float ConvexRadius;
} JPC_BoxShapeSettings;

JPC_API void JPC_BoxShapeSettings_default(JPC_BoxShapeSettings* object);
JPC_API bool JPC_BoxShapeSettings_Create(const JPC_BoxShapeSettings* self, JPC_Shape** outShape, JPC_String** outError);

////////////////////////////////////////////////////////////////////////////////
// SphereShapeSettings -> ConvexShapeSettings -> ShapeSettings

typedef struct JPC_SphereShapeSettings {
	// ShapeSettings
	uint64_t UserData;

	// ConvexShapeSettings
	// TODO: Material
	float Density;

	// SphereShapeSettings
	float Radius;
} JPC_SphereShapeSettings;

JPC_API void JPC_SphereShapeSettings_default(JPC_SphereShapeSettings* object);
JPC_API bool JPC_SphereShapeSettings_Create(const JPC_SphereShapeSettings* self, JPC_Shape** outShape, JPC_String** outError);

////////////////////////////////////////////////////////////////////////////////
// CapsuleShapeSettings -> ConvexShapeSettings -> ShapeSettings

typedef struct JPC_CapsuleShapeSettings {
	// ShapeSettings
	uint64_t UserData;

	// ConvexShapeSettings
	// TODO: Material
	float Density;

	// CapsuleShapeSettings
	float Radius;
	float HalfHeightOfCylinder;
} JPC_CapsuleShapeSettings;

JPC_API void JPC_CapsuleShapeSettings_default(JPC_CapsuleShapeSettings* object);
JPC_API bool JPC_CapsuleShapeSettings_Create(const JPC_CapsuleShapeSettings* self, JPC_Shape** outShape, JPC_String** outError);

////////////////////////////////////////////////////////////////////////////////
// CylinderShapeSettings -> ConvexShapeSettings -> ShapeSettings

typedef struct JPC_CylinderShapeSettings {
	// ShapeSettings
	uint64_t UserData;

	// ConvexShapeSettings
	// TODO: Material
	float Density;

	// CylinderShapeSettings
	float HalfHeight;
	float Radius;
	float ConvexRadius;
} JPC_CylinderShapeSettings;

JPC_API void JPC_CylinderShapeSettings_default(JPC_CylinderShapeSettings* object);
JPC_API bool JPC_CylinderShapeSettings_Create(const JPC_CylinderShapeSettings* self, JPC_Shape** outShape, JPC_String** outError);

////////////////////////////////////////////////////////////////////////////////
// ConvexHullShapeSettings -> ConvexShapeSettings -> ShapeSettings

typedef struct JPC_ConvexHullShapeSettings {
	// ShapeSettings
	uint64_t UserData;

	// ConvexShapeSettings
	// TODO: Material
	float Density;

	// ConvexHullShapeSettings
	const JPC_Vec3* Points;
	size_t PointsLen;
	float MaxConvexRadius;
	float MaxErrorConvexRadius;
	float HullTolerance;
} JPC_ConvexHullShapeSettings;

JPC_API void JPC_ConvexHullShapeSettings_default(JPC_ConvexHullShapeSettings* object);
JPC_API bool JPC_ConvexHullShapeSettings_Create(const JPC_ConvexHullShapeSettings* self, JPC_Shape** outShape, JPC_String** outError);

////////////////////////////////////////////////////////////////////////////////
// CompoundShape::SubShapeSettings

typedef struct JPC_SubShapeSettings {
	const JPC_Shape* Shape;
	JPC_Vec3 Position;
	JPC_Quat Rotation;
	uint32_t UserData;
} JPC_SubShapeSettings;

JPC_API void JPC_SubShapeSettings_default(JPC_SubShapeSettings* object);

////////////////////////////////////////////////////////////////////////////////
// StaticCompoundShapeSettings -> CompoundShapeSettings -> ShapeSettings

typedef struct JPC_StaticCompoundShapeSettings {
	// ShapeSettings
	uint64_t UserData;

	// CompoundShapeSettings
	const JPC_SubShapeSettings* SubShapes;
	size_t SubShapesLen;

	// StaticCompoundShapeSettings
	// (no fields)
} JPC_StaticCompoundShapeSettings;

JPC_API void JPC_StaticCompoundShapeSettings_default(JPC_StaticCompoundShapeSettings* object);
JPC_API bool JPC_StaticCompoundShapeSettings_Create(const JPC_StaticCompoundShapeSettings* self, JPC_Shape** outShape, JPC_String** outError);

////////////////////////////////////////////////////////////////////////////////
// MutableCompoundShape -> CompoundShapeSettings -> ShapeSettings

typedef struct JPC_MutableCompoundShapeSettings {
	// ShapeSettings
	uint64_t UserData;

	// CompoundShapeSettings
	const JPC_SubShapeSettings* SubShapes;
	size_t SubShapesLen;

	// MutableCompoundShape
	// (no fields)
} JPC_MutableCompoundShapeSettings;

JPC_API void JPC_MutableCompoundShapeSettings_default(JPC_MutableCompoundShapeSettings* object);
JPC_API bool JPC_MutableCompoundShapeSettings_Create(const JPC_MutableCompoundShapeSettings* self, JPC_Shape** outShape, JPC_String** outError);

////////////////////////////////////////////////////////////////////////////////
// BodyCreationSettings

typedef struct JPC_BodyCreationSettings {
	JPC_RVec3 Position;
	JPC_Quat Rotation;
	JPC_Vec3 LinearVelocity;
	JPC_Vec3 AngularVelocity;
	uint64_t UserData;
	JPC_ObjectLayer ObjectLayer;
	// CollisionGroup CollisionGroup;
	JPC_MotionType MotionType;
	JPC_AllowedDOFs AllowedDOFs;
	bool AllowDynamicOrKinematic;
	bool IsSensor;
	bool CollideKinematicVsNonDynamic;
	bool UseManifoldReduction;
	bool ApplyGyroscopicForce;
	JPC_MotionQuality MotionQuality;
	bool EnhancedInternalEdgeRemoval;
	bool AllowSleeping;
	float Friction;
	float Restitution;
	float LinearDamping;
	float AngularDamping;
	float MaxLinearVelocity;
	float MaxAngularVelocity;
	float GravityFactor;
	uint NumVelocityStepsOverride;
	uint NumPositionStepsOverride;
	JPC_OverrideMassProperties OverrideMassProperties;
	float InertiaMultiplier;

	// MassProperties MassPropertiesOverride;

	const JPC_Shape* Shape;
} JPC_BodyCreationSettings;

JPC_API void JPC_BodyCreationSettings_default(JPC_BodyCreationSettings* settings);

typedef struct JPC_BodyCreationSettings JPC_BodyCreationSettings;

JPC_API JPC_BodyCreationSettings* JPC_BodyCreationSettings_new();

////////////////////////////////////////////////////////////////////////////////
// Body

JPC_API JPC_BodyID JPC_Body_GetID(const JPC_Body* self);
JPC_API JPC_BodyType JPC_Body_GetBodyType(const JPC_Body* self);
JPC_API bool JPC_Body_IsRigidBody(const JPC_Body* self);
JPC_API bool JPC_Body_IsSoftBody(const JPC_Body* self);
JPC_API bool JPC_Body_IsActive(const JPC_Body* self);
JPC_API bool JPC_Body_IsStatic(const JPC_Body* self);
JPC_API bool JPC_Body_IsKinematic(const JPC_Body* self);
JPC_API bool JPC_Body_IsDynamic(const JPC_Body* self);
JPC_API bool JPC_Body_CanBeKinematicOrDynamic(const JPC_Body* self);
JPC_API void JPC_Body_SetIsSensor(JPC_Body* self, bool inIsSensor);
JPC_API bool JPC_Body_IsSensor(const JPC_Body* self);
JPC_API void JPC_Body_SetCollideKinematicVsNonDynamic(JPC_Body* self, bool inCollide);
JPC_API bool JPC_Body_GetCollideKinematicVsNonDynamic(const JPC_Body* self);
JPC_API void JPC_Body_SetUseManifoldReduction(JPC_Body* self, bool inUseReduction);
JPC_API bool JPC_Body_GetUseManifoldReduction(const JPC_Body* self);
JPC_API bool JPC_Body_GetUseManifoldReductionWithBody(const JPC_Body* self, const JPC_Body* inBody2);
JPC_API void JPC_Body_SetApplyGyroscopicForce(JPC_Body* self, bool inApply);
JPC_API bool JPC_Body_GetApplyGyroscopicForce(const JPC_Body* self);
JPC_API void JPC_Body_SetEnhancedInternalEdgeRemoval(JPC_Body* self, bool inApply);
JPC_API bool JPC_Body_GetEnhancedInternalEdgeRemoval(const JPC_Body* self);
JPC_API bool JPC_Body_GetEnhancedInternalEdgeRemovalWithBody(const JPC_Body* self, const JPC_Body* inBody2);
JPC_API JPC_MotionType JPC_Body_GetMotionType(const JPC_Body* self);
JPC_API void JPC_Body_SetMotionType(JPC_Body* self, JPC_MotionType inMotionType);
JPC_API JPC_BroadPhaseLayer JPC_Body_GetBroadPhaseLayer(const JPC_Body* self);
JPC_API JPC_ObjectLayer JPC_Body_GetObjectLayer(const JPC_Body* self);

// JPC_API const CollisionGroup & JPC_Body_GetCollisionGroup(const JPC_Body* self);
// JPC_API CollisionGroup & JPC_Body_GetCollisionGroup(JPC_Body* self);
// JPC_API void JPC_Body_SetCollisionGroup(JPC_Body* self, const CollisionGroup &inGroup);

JPC_API bool JPC_Body_GetAllowSleeping(const JPC_Body* self);
JPC_API void JPC_Body_SetAllowSleeping(JPC_Body* self, bool inAllow);
JPC_API void JPC_Body_ResetSleepTimer(JPC_Body* self);
JPC_API float JPC_Body_GetFriction(const JPC_Body* self);
JPC_API void JPC_Body_SetFriction(JPC_Body* self, float inFriction);
JPC_API float JPC_Body_GetRestitution(const JPC_Body* self);
JPC_API void JPC_Body_SetRestitution(JPC_Body* self, float inRestitution);
JPC_API JPC_Vec3 JPC_Body_GetLinearVelocity(const JPC_Body* self);
JPC_API void JPC_Body_SetLinearVelocity(JPC_Body* self, JPC_Vec3 inLinearVelocity);
JPC_API void JPC_Body_SetLinearVelocityClamped(JPC_Body* self, JPC_Vec3 inLinearVelocity);
JPC_API JPC_Vec3 JPC_Body_GetAngularVelocity(const JPC_Body* self);
JPC_API void JPC_Body_SetAngularVelocity(JPC_Body* self, JPC_Vec3 inAngularVelocity);
JPC_API void JPC_Body_SetAngularVelocityClamped(JPC_Body* self, JPC_Vec3 inAngularVelocity);
JPC_API JPC_Vec3 JPC_Body_GetPointVelocityCOM(const JPC_Body* self, JPC_Vec3 inPointRelativeToCOM);
JPC_API JPC_Vec3 JPC_Body_GetPointVelocity(const JPC_Body* self, JPC_RVec3 inPoint);
JPC_API void JPC_Body_AddForce(JPC_Body* self, JPC_Vec3 inForce);
// overload of Body::AddForce
JPC_API void JPC_Body_AddForceAtPoint(JPC_Body* self, JPC_Vec3 inForce, JPC_RVec3 inPosition);
JPC_API void JPC_Body_AddTorque(JPC_Body* self, JPC_Vec3 inTorque);
JPC_API JPC_Vec3 JPC_Body_GetAccumulatedForce(const JPC_Body* self);
JPC_API JPC_Vec3 JPC_Body_GetAccumulatedTorque(const JPC_Body* self);
JPC_API void JPC_Body_ResetForce(JPC_Body* self);
JPC_API void JPC_Body_ResetTorque(JPC_Body* self);
JPC_API void JPC_Body_ResetMotion(JPC_Body* self);
JPC_API void JPC_Body_GetInverseInertia(const JPC_Body* self, JPC_Mat44* outMatrix);
JPC_API void JPC_Body_AddImpulse(JPC_Body* self, JPC_Vec3 inImpulse);
JPC_API void JPC_Body_AddImpulse2(JPC_Body* self, JPC_Vec3 inImpulse, JPC_RVec3 inPosition);
JPC_API void JPC_Body_AddAngularImpulse(JPC_Body* self, JPC_Vec3 inAngularImpulse);
JPC_API void JPC_Body_MoveKinematic(JPC_Body* self, JPC_RVec3 inTargetPosition, JPC_Quat inTargetRotation, float inDeltaTime);
JPC_API bool JPC_Body_ApplyBuoyancyImpulse(JPC_Body* self, JPC_RVec3 inSurfacePosition, JPC_Vec3 inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, JPC_Vec3 inFluidVelocity, JPC_Vec3 inGravity, float inDeltaTime);
JPC_API bool JPC_Body_IsInBroadPhase(const JPC_Body* self);
JPC_API bool JPC_Body_IsCollisionCacheInvalid(const JPC_Body* self);
JPC_API const JPC_Shape* JPC_Body_GetShape(const JPC_Body* self);
JPC_API JPC_RVec3 JPC_Body_GetPosition(const JPC_Body* self);
JPC_API JPC_Quat JPC_Body_GetRotation(const JPC_Body* self);
JPC_API JPC_RMat44 JPC_Body_GetWorldTransform(const JPC_Body* self);
JPC_API JPC_RVec3 JPC_Body_GetCenterOfMassPosition(const JPC_Body* self);

JPC_API JPC_RMat44 JPC_Body_GetCenterOfMassTransform(const JPC_Body* self);
JPC_API JPC_RMat44 JPC_Body_GetInverseCenterOfMassTransform(const JPC_Body* self);

// JPC_API const AABox & JPC_Body_GetWorldSpaceBounds(const JPC_Body* self);
// JPC_API const MotionProperties *JPC_Body_GetMotionProperties(const JPC_Body* self)
// JPC_API MotionProperties * JPC_Body_GetMotionProperties(JPC_Body* self);
// JPC_API const MotionProperties *JPC_Body_GetMotionPropertiesUnchecked(const JPC_Body* self)
// JPC_API MotionProperties * JPC_Body_GetMotionPropertiesUnchecked(JPC_Body* self);

JPC_API uint64_t JPC_Body_GetUserData(const JPC_Body* self);
JPC_API void JPC_Body_SetUserData(JPC_Body* self, uint64_t inUserData);

// JPC_API JPC_Vec3 JPC_Body_GetWorldSpaceSurfaceNormal(const JPC_Body* self, const SubShapeID &inSubShapeID, JPC_RVec3 inPosition);
// JPC_API TransformedShape JPC_Body_GetTransformedShape(const JPC_Body* self);
// JPC_API BodyCreationSettings JPC_Body_GetBodyCreationSettings(const JPC_Body* self);
// JPC_API SoftBodyCreationSettings JPC_Body_GetSoftBodyCreationSettings(const JPC_Body* self);

////////////////////////////////////////////////////////////////////////////////
// BodyInterface

typedef struct JPC_BodyInterface JPC_BodyInterface;

JPC_API JPC_Body* JPC_BodyInterface_CreateBody(JPC_BodyInterface* self, const JPC_BodyCreationSettings* inSettings);
JPC_API JPC_Body* JPC_BodyInterface_CreateBodyWithID(JPC_BodyInterface *self, JPC_BodyID inBodyID, const JPC_BodyCreationSettings* inSettings);
JPC_API JPC_Body* JPC_BodyInterface_CreateBodyWithoutID(const JPC_BodyInterface *self, const JPC_BodyCreationSettings* inSettings);

// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBody(JPC_BodyInterface *self, const SoftBodyCreationSettings &inSettings);
// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBodyWithID(JPC_BodyInterface *self, JPC_BodyID inBodyID, const SoftBodyCreationSettings* inSettings);
// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBodyWithoutID(const JPC_BodyInterface *self, const SoftBodyCreationSettings* inSettings);

JPC_API void JPC_BodyInterface_DestroyBodyWithoutID(const JPC_BodyInterface *self, JPC_Body *inBody);
JPC_API bool JPC_BodyInterface_AssignBodyID(JPC_BodyInterface *self, JPC_Body *ioBody);

// JPC_API bool JPC_BodyInterface_AssignBodyID(JPC_BodyInterface *self, JPC_Body *ioBody, JPC_BodyID inBodyID);

JPC_API JPC_Body* JPC_BodyInterface_UnassignBodyID(JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_UnassignBodyIDs(JPC_BodyInterface *self, const JPC_BodyID *inBodyIDs, int inNumber, JPC_Body **outBodies);
JPC_API void JPC_BodyInterface_DestroyBody(JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_DestroyBodies(JPC_BodyInterface *self, const JPC_BodyID *inBodyIDs, int inNumber);
JPC_API void JPC_BodyInterface_AddBody(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Activation inActivationMode);
JPC_API void JPC_BodyInterface_RemoveBody(JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API bool JPC_BodyInterface_IsAdded(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddBody(JPC_BodyInterface *self, const JPC_BodyCreationSettings* inSettings, JPC_Activation inActivationMode);

// JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddSoftBody(JPC_BodyInterface *self, const SoftBodyCreationSettings &inSettings, JPC_Activation inActivationMode);

JPC_API void* JPC_BodyInterface_AddBodiesPrepare(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber);
JPC_API void JPC_BodyInterface_AddBodiesFinalize(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber, void* inAddState, JPC_Activation inActivationMode);
JPC_API void JPC_BodyInterface_AddBodiesAbort(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber, void* inAddState);
JPC_API void JPC_BodyInterface_RemoveBodies(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber);
JPC_API void JPC_BodyInterface_ActivateBody(JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_ActivateBodies(JPC_BodyInterface *self, JPC_BodyID *inBodyIDs, int inNumber);

// JPC_API void JPC_BodyInterface_ActivateBodiesInAABox(JPC_BodyInterface *self, const AABox &inBox, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter);

JPC_API void JPC_BodyInterface_DeactivateBody(JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_DeactivateBodies(JPC_BodyInterface *self, JPC_BodyID *inBodyIDs, int inNumber);
JPC_API bool JPC_BodyInterface_IsActive(const JPC_BodyInterface *self, JPC_BodyID inBodyID);

// TwoBodyConstraint * JPC_BodyInterface_CreateConstraint(JPC_BodyInterface *self, const TwoBodyConstraintSettings *inSettings, JPC_BodyID inBodyID1, JPC_BodyID inBodyID2);
// JPC_API void JPC_BodyInterface_ActivateConstraint(JPC_BodyInterface *self, const TwoBodyConstraint *inConstraint);
JPC_API const JPC_Shape* JPC_BodyInterface_GetShape(const JPC_BodyInterface *self, JPC_BodyID inBodyID);

JPC_API void JPC_BodyInterface_SetShape(const JPC_BodyInterface *self, JPC_BodyID inBodyID, const JPC_Shape *inShape, bool inUpdateMassProperties, JPC_Activation inActivationMode);
JPC_API void JPC_BodyInterface_NotifyShapeChanged(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inPreviousCenterOfMass, bool inUpdateMassProperties, JPC_Activation inActivationMode);
JPC_API void JPC_BodyInterface_SetObjectLayer(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_ObjectLayer inLayer);
JPC_API JPC_ObjectLayer JPC_BodyInterface_GetObjectLayer(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_SetPositionAndRotation(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Activation inActivationMode);
JPC_API void JPC_BodyInterface_SetPositionAndRotationWhenChanged(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Activation inActivationMode);
JPC_API void JPC_BodyInterface_GetPositionAndRotation(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 *outPosition, JPC_Quat *outRotation);
JPC_API void JPC_BodyInterface_SetPosition(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Activation inActivationMode);
JPC_API JPC_RVec3 JPC_BodyInterface_GetPosition(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API JPC_RVec3 JPC_BodyInterface_GetCenterOfMassPosition(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_SetRotation(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Quat inRotation, JPC_Activation inActivationMode);
JPC_API JPC_Quat JPC_BodyInterface_GetRotation(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API JPC_RMat44 JPC_BodyInterface_GetWorldTransform(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API JPC_RMat44 JPC_BodyInterface_GetCenterOfMassTransform(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_MoveKinematic(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inTargetPosition, JPC_Quat inTargetRotation, float inDeltaTime);
JPC_API void JPC_BodyInterface_SetLinearAndAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity);
JPC_API void JPC_BodyInterface_GetLinearAndAngularVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 *outLinearVelocity, JPC_Vec3 *outAngularVelocity);
JPC_API void JPC_BodyInterface_SetLinearVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity);
JPC_API JPC_Vec3 JPC_BodyInterface_GetLinearVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_AddLinearVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity);
JPC_API void JPC_BodyInterface_AddLinearAndAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity);
JPC_API void JPC_BodyInterface_SetAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inAngularVelocity);
JPC_API JPC_Vec3 JPC_BodyInterface_GetAngularVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API JPC_Vec3 JPC_BodyInterface_GetPointVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPoint);
JPC_API void JPC_BodyInterface_SetPositionRotationAndVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity);
JPC_API void JPC_BodyInterface_AddForce(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce);
// overload of BodyInterface::AddForce
JPC_API void JPC_BodyInterface_AddForceAtPoint(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce, JPC_RVec3 inPoint);
JPC_API void JPC_BodyInterface_AddTorque(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inTorque);
JPC_API void JPC_BodyInterface_AddForceAndTorque(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce, JPC_Vec3 inTorque);
JPC_API void JPC_BodyInterface_AddImpulse(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inImpulse);
JPC_API void JPC_BodyInterface_AddImpulse3(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inImpulse, JPC_RVec3 inPoint);
JPC_API void JPC_BodyInterface_AddAngularImpulse(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inAngularImpulse);
JPC_API JPC_BodyType JPC_BodyInterface_GetBodyType(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_SetMotionType(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_MotionType inMotionType, JPC_Activation inActivationMode);
JPC_API JPC_MotionType JPC_BodyInterface_GetMotionType(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_SetMotionQuality(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_MotionQuality inMotionQuality);
JPC_API JPC_MotionQuality JPC_BodyInterface_GetMotionQuality(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_GetInverseInertia(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Mat44 *outMatrix);
JPC_API void JPC_BodyInterface_SetRestitution(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inRestitution);
JPC_API float JPC_BodyInterface_GetRestitution(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_SetFriction(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inFriction);
JPC_API float JPC_BodyInterface_GetFriction(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_SetGravityFactor(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inGravityFactor);
JPC_API float JPC_BodyInterface_GetGravityFactor(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_SetUseManifoldReduction(JPC_BodyInterface *self, JPC_BodyID inBodyID, bool inUseReduction);
JPC_API bool JPC_BodyInterface_GetUseManifoldReduction(const JPC_BodyInterface *self, JPC_BodyID inBodyID);

// TransformedShape JPC_BodyInterface_GetTransformedShape(const JPC_BodyInterface *self, JPC_BodyID inBodyID);

JPC_API uint64_t JPC_BodyInterface_GetUserData(const JPC_BodyInterface *self, JPC_BodyID inBodyID);
JPC_API void JPC_BodyInterface_SetUserData(const JPC_BodyInterface *self, JPC_BodyID inBodyID, uint64_t inUserData);

// const PhysicsMaterial* JPC_BodyInterface_GetMaterial(const JPC_BodyInterface *self, JPC_BodyID inBodyID, const SubShapeID &inSubShapeID);

JPC_API void JPC_BodyInterface_InvalidateContactCache(JPC_BodyInterface *self, JPC_BodyID inBodyID);

////////////////////////////////////////////////////////////////////////////////
// NarrowPhaseQuery

typedef struct JPC_NarrowPhaseQuery JPC_NarrowPhaseQuery;

typedef struct JPC_NarrowPhaseQuery_CastRayArgs {
	JPC_RRayCast Ray;
	JPC_RayCastResult Result;
	const JPC_BroadPhaseLayerFilter *BroadPhaseLayerFilter;
	const JPC_ObjectLayerFilter *ObjectLayerFilter;
	const JPC_BodyFilter *BodyFilter;
	const JPC_ShapeFilter *ShapeFilter;
} JPC_NarrowPhaseQuery_CastRayArgs;

JPC_API bool JPC_NarrowPhaseQuery_CastRay(const JPC_NarrowPhaseQuery* self, JPC_NarrowPhaseQuery_CastRayArgs* args);

typedef struct JPC_RShapeCast {
	const JPC_Shape *Shape;
	JPC_Vec3 Scale;
	JPC_RMat44 CenterOfMassStart;
	JPC_Vec3 Direction;
	// const JPC_AABox ShapeWorldBounds;
} JPC_RShapeCast;

typedef struct JPC_ShapeCastSettings {
	// JPH::CollideSettingsBase
	// EActiveEdgeMode ActiveEdgeMode;
	// ECollectFacesMode CollectFacesMode;
	float CollisionTolerance;
	float PenetrationTolerance;
	JPC_Vec3 ActiveEdgeMovementDirection;

	// JPH::ShapeCastSettings
	JPC_BackFaceMode BackFaceModeTriangles;
	JPC_BackFaceMode BackFaceModeConvex;
	bool UseShrunkenShapeAndConvexRadius;
	bool ReturnDeepestPoint;
} JPC_ShapeCastSettings;

JPC_API void JPC_ShapeCastSettings_default(JPC_ShapeCastSettings* object);

typedef struct JPC_NarrowPhaseQuery_CastShapeArgs {
	JPC_RShapeCast ShapeCast;
	JPC_ShapeCastSettings Settings;
	JPC_RVec3 BaseOffset;
	JPC_CastShapeCollector *Collector;
	const JPC_BroadPhaseLayerFilter *BroadPhaseLayerFilter;
	const JPC_ObjectLayerFilter *ObjectLayerFilter;
	const JPC_BodyFilter *BodyFilter;
	// const JPC_ShapeFilter *ShapeFilter;
} JPC_NarrowPhaseQuery_CastShapeArgs;

JPC_API void JPC_NarrowPhaseQuery_CastShape(const JPC_NarrowPhaseQuery* self, JPC_NarrowPhaseQuery_CastShapeArgs* args);

////////////////////////////////////////////////////////////////////////////////
// PhysicsSystem

typedef struct JPC_PhysicsSystem JPC_PhysicsSystem;

JPC_API JPC_PhysicsSystem* JPC_PhysicsSystem_new();
JPC_API void JPC_PhysicsSystem_delete(JPC_PhysicsSystem* object);
JPC_API void JPC_PhysicsSystem_Init(
	JPC_PhysicsSystem* self,
	uint inMaxBodies,
	uint inNumBodyMutexes,
	uint inMaxBodyPairs,
	uint inMaxContactConstraints,
	JPC_BroadPhaseLayerInterface* inBroadPhaseLayerInterface,
	JPC_ObjectVsBroadPhaseLayerFilter* inObjectVsBroadPhaseLayerFilter,
	JPC_ObjectLayerPairFilter* inObjectLayerPairFilter);

JPC_API void JPC_PhysicsSystem_OptimizeBroadPhase(JPC_PhysicsSystem* self);

JPC_API JPC_PhysicsUpdateError JPC_PhysicsSystem_Update(
	JPC_PhysicsSystem* self,
	float inDeltaTime,
	int inCollisionSteps,
	JPC_TempAllocatorImpl *inTempAllocator, // FIXME: un-specialize
	JPC_JobSystemThreadPool *inJobSystem); // FIXME: un-specialize

JPC_API JPC_BodyInterface* JPC_PhysicsSystem_GetBodyInterface(JPC_PhysicsSystem* self);

JPC_API const JPC_NarrowPhaseQuery* JPC_PhysicsSystem_GetNarrowPhaseQuery(const JPC_PhysicsSystem* self);

JPC_API void JPC_PhysicsSystem_DrawBodies(
	JPC_PhysicsSystem* self,
	JPC_BodyManager_DrawSettings* inSettings,
	JPC_DebugRendererSimple* inRenderer, // FIXME: un-specialize
	const void* inBodyFilter); // FIXME: BodyDrawFilter

#ifdef __cplusplus
}
#endif