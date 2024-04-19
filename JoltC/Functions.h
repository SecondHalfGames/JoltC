#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>

#ifdef _MSC_VER
	#define JPC_API extern __declspec(dllexport)
#else
	#define JPC_API
#endif

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

typedef struct JPC_Vec3 {
	alignas(16) float x;
	float y;
	float z;
	float _w;
} JPC_Vec3;

ENSURE_SIZE_ALIGN(JPC_Vec3, JPH::Vec3);

typedef struct JPC_DVec3 {
	alignas(32) double x;
	double y;
	double z;
	double _w;
} JPC_DVec3;

ENSURE_SIZE_ALIGN(JPC_DVec3, JPH::DVec3);

typedef struct JPC_Quat {
	alignas(16) float x;
	float y;
	float z;
	float w;
} JPC_Quat;

ENSURE_SIZE_ALIGN(JPC_Quat, JPH::Quat);

#ifdef JPC_DOUBLE_PRECISION
	typedef JPC_DVec3 JPC_RVec3;
#else
	typedef JPC_Vec3 JPC_RVec3;
#endif

ENSURE_SIZE_ALIGN(JPC_RVec3, JPH::RVec3);

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

typedef struct JPC_BroadPhaseLayerInterface {
	const void *self;
	JPC_BroadPhaseLayerInterfaceFns fns;
} JPC_BroadPhaseLayerInterface;

////////////////////////////////////////////////////////////////////////////////
// ObjectVsBroadPhaseLayerFilter

typedef struct JPC_ObjectVsBroadPhaseLayerFilterFns {
	bool (*ShouldCollide)(const void *self, JPC_ObjectLayer inLayer1, JPC_BroadPhaseLayer inLayer2);
} JPC_ObjectVsBroadPhaseLayerFilterFns;

typedef struct JPC_ObjectVsBroadPhaseLayerFilter {
	const void *self;
	JPC_ObjectVsBroadPhaseLayerFilterFns fns;
} JPC_ObjectVsBroadPhaseLayerFilter;

////////////////////////////////////////////////////////////////////////////////
// ObjectLayerPairFilter

typedef struct JPC_ObjectLayerPairFilterFns {
	bool (*ShouldCollide)(const void *self, JPC_ObjectLayer inLayer1, JPC_ObjectLayer inLayer2);
} JPC_ObjectLayerPairFilterFns;

typedef struct JPC_ObjectLayerPairFilter {
	const void *self;
	JPC_ObjectLayerPairFilterFns fns;
} JPC_ObjectLayerPairFilter;

////////////////////////////////////////////////////////////////////////////////
// BodyInterface

typedef struct JPC_BodyInterface JPC_BodyInterface;

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
	JPC_BroadPhaseLayerInterface inBroadPhaseLayerInterface,
	JPC_ObjectVsBroadPhaseLayerFilter inObjectVsBroadPhaseLayerFilter,
	JPC_ObjectLayerPairFilter inObjectLayerPairFilter);

JPC_API JPC_PhysicsUpdateError JPC_PhysicsSystem_Update(
	JPC_PhysicsSystem* self,
	float inDeltaTime,
	int inCollisionSteps,
	JPC_TempAllocatorImpl *inTempAllocator, // FIXME: un-specialize
	JPC_JobSystemThreadPool *inJobSystem); // FIXME: un-specialize

JPC_API const JPC_BodyInterface* JPC_PhysicsSystem_GetBodyInterface(JPC_PhysicsSystem* self);

#ifdef __cplusplus
}
#endif