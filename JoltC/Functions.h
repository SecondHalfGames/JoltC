#pragma once

#define JPC_API extern __declspec(dllexport)

// C-compatible typedefs that match Jolt's internal primitive typedefs.
#define uint unsigned int

#ifdef __cplusplus
extern "C" {
#endif

JPC_API void JPC_RegisterDefaultAllocator();
JPC_API void JPC_FactoryInit();
JPC_API void JPC_RegisterTypes();

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

typedef uint8_t JPC_BroadPhaseLayer;
ENSURE_SIZE_ALIGN(JPC_BroadPhaseLayer, JPH::BroadPhaseLayer)

typedef uint16_t JPC_ObjectLayer; // FIXME: Branch on JPH_OBJECT_LAYER_BITS?
ENSURE_SIZE_ALIGN(JPC_ObjectLayer, JPH::ObjectLayer)

typedef struct JPC_BroadPhaseLayerInterfaceFns {
	uint (*GetNumBroadPhaseLayers)(void *self);
	JPC_BroadPhaseLayer (*GetBroadPhaseLayer)(void *self, JPC_ObjectLayer inLayer);
} JPC_BroadPhaseLayerInterfaceFns;

typedef struct JPC_BroadPhaseLayerInterface {
	JPC_BroadPhaseLayerInterfaceFns *fns;
	void *self;
} JPC_BroadPhaseLayerInterface;

////////////////////////////////////////////////////////////////////////////////
// PhysicsSystem

typedef struct JPC_PhysicsSystem JPC_PhysicsSystem;

JPC_API JPC_PhysicsSystem* JPC_PhysicsSystem_new();
JPC_API void JPC_PhysicsSystem_delete(JPC_PhysicsSystem* object);
JPC_API void JPC_PhysicsSystem_Init(
	uint inMaxBodies,
	uint inNumBodyMutexes,
	uint inMaxBodyPairs,
	uint inMaxContactConstraints,
	JPC_BroadPhaseLayerInterface inBroadPhaseLayerInterface);
	// const ObjectVsBroadPhaseLayerFilter &inObjectVsBroadPhaseLayerFilter,
	// const ObjectLayerPairFilter &inObjectLayerPairFilter);

#ifdef __cplusplus
}
#endif