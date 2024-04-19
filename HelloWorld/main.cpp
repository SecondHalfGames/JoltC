#include <stdio.h>
#include <stdint.h>

#include "JoltC/JoltC.h"

#ifdef _MSC_VER
	#define unreachable() __assume(0)
#else
	#define unreachable() __builtin_unreachable()
#endif

typedef enum Hello_ObjectLayers {
	HELLO_OL_NON_MOVING,
	HELLO_OL_MOVING,
	HELLO_OL_COUNT,
} Hello_ObjectLayers;

typedef enum Hello_BroadPhaseLayers {
	HELLO_BPL_NON_MOVING,
	HELLO_BPL_MOVING,
	HELLO_BPL_COUNT,
} Hello_BroadPhaseLayers;

unsigned int Hello_BPL_GetNumBroadPhaseLayers(void *self) {
	return HELLO_BPL_COUNT;
}

JPC_BroadPhaseLayer Hello_BPL_GetBroadPhaseLayer(void *self, JPC_ObjectLayer inLayer) {
	switch (inLayer) {
	case HELLO_OL_NON_MOVING:
		return HELLO_BPL_NON_MOVING;

	case HELLO_OL_MOVING:
		return HELLO_BPL_MOVING;

	default:
		unreachable();
	}
}

static JPC_BroadPhaseLayerInterfaceFns Hello_BPL = {
	.GetNumBroadPhaseLayers = Hello_BPL_GetNumBroadPhaseLayers,
	.GetBroadPhaseLayer = Hello_BPL_GetBroadPhaseLayer,
};

bool Hello_OVB_ShouldCollide(void *self, JPC_ObjectLayer inLayer1, JPC_BroadPhaseLayer inLayer2) {
	switch (inLayer1) {
	case HELLO_OL_NON_MOVING:
		return inLayer2 == HELLO_BPL_MOVING;

	case HELLO_OL_MOVING:
		return true;

	default:
		unreachable();
	}
}

static JPC_ObjectVsBroadPhaseLayerFilterFns Hello_OVB = {
	.ShouldCollide = Hello_OVB_ShouldCollide,
};

bool Hello_OVO_ShouldCollide(void *self, JPC_ObjectLayer inLayer1, JPC_ObjectLayer inLayer2) {
	switch (inLayer1)
	{
	case HELLO_OL_NON_MOVING:
		return inLayer2 == HELLO_OL_MOVING; // Non moving only collides with moving

	case HELLO_OL_MOVING:
		return true; // Moving collides with everything

	default:
		unreachable();
	}
}

static JPC_ObjectLayerPairFilterFns Hello_OVO = {
	.ShouldCollide = Hello_OVO_ShouldCollide,
};

int main() {
	JPC_RegisterDefaultAllocator();
	JPC_FactoryInit();
	JPC_RegisterTypes();

	JPC_TempAllocatorImpl* temp_allocator = JPC_TempAllocatorImpl_new(10 * 1024 * 1024);

	JPC_JobSystemThreadPool* job_system = JPC_JobSystemThreadPool_new2(JPC_MAX_PHYSICS_JOBS, JPC_MAX_PHYSICS_BARRIERS);

	JPC_BroadPhaseLayerInterface broad_phase_layer_interface = {
		.self = nullptr,
		.fns = Hello_BPL,
	};

	JPC_ObjectVsBroadPhaseLayerFilter object_vs_broad_phase_layer_filter = {
		.self = nullptr,
		.fns = Hello_OVB,
	};

	JPC_ObjectLayerPairFilter object_vs_object_layer_filter = {
		.self = nullptr,
		.fns = Hello_OVO,
	};

	JPC_PhysicsSystem* physics_system = JPC_PhysicsSystem_new();

	const unsigned int cMaxBodies = 1024;
	const unsigned int cNumBodyMutexes = 0;
	const unsigned int cMaxBodyPairs = 1024;
	const unsigned int cMaxContactConstraints = 1024;

	JPC_PhysicsSystem_Init(
		physics_system,
		cMaxBodies,
		cNumBodyMutexes,
		cMaxBodyPairs,
		cMaxContactConstraints,
		broad_phase_layer_interface,
		object_vs_broad_phase_layer_filter,
		object_vs_object_layer_filter);

	// TODO: register body activation listener
	// TODO: register contact listener
	// TODO: body interface
	// TODO: creating bodies
	// TODO: PhysicsSystem::OptimizeBroadPhase

	const float cDeltaTime = 1.0f / 60.0f;
	const int cCollisionSteps = 1;

	// TODO: Update loop
	for (int i = 0; i < 100; i++) {
		JPC_PhysicsSystem_Update(physics_system, cDeltaTime, cCollisionSteps, temp_allocator, job_system);
	}

	// TODO: RemoveBody and DestroyBody

	JPC_PhysicsSystem_delete(physics_system);
	JPC_JobSystemThreadPool_delete(job_system);
	JPC_TempAllocatorImpl_delete(temp_allocator);

	JPC_UnregisterTypes();
	JPC_FactoryDelete();

	printf("Hello, world!\n");
}