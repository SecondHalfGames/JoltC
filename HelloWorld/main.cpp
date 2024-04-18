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

unsigned int Hello_GetNumBroadPhaseLayers(void* self) {
	return HELLO_BPL_COUNT;
}

JPC_BroadPhaseLayer Hello_GetBroadPhaseLayer(void* self, JPC_ObjectLayer inLayer) {
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
	.GetNumBroadPhaseLayers = Hello_GetNumBroadPhaseLayers,
	.GetBroadPhaseLayer = Hello_GetBroadPhaseLayer,
};

bool Hello_ShouldCollide(void* self, JPC_ObjectLayer inLayer1, JPC_BroadPhaseLayer inLayer2) {
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
	.ShouldCollide = Hello_ShouldCollide,
};

int main() {
	JPC_RegisterDefaultAllocator();
	JPC_FactoryInit();
	JPC_RegisterTypes();

	JPC_TempAllocatorImpl* temp_allocator = JPC_TempAllocatorImpl_new(10 * 1024 * 1024);

	JPC_JobSystemThreadPool* job_system = JPC_JobSystemThreadPool_new2(JPC_MAX_PHYSICS_JOBS, JPC_MAX_PHYSICS_BARRIERS);

	JPC_BroadPhaseLayerInterface broad_phase_layer_interface = {
		.fns = &Hello_BPL,
		.self = nullptr,
	};

	JPC_ObjectVsBroadPhaseLayerFilter object_vs_broad_phase_layer_filter = {
		.fns = &Hello_OVB,
		.self = nullptr,
	};

	// create object_vs_object_layer_filter

	JPC_PhysicsSystem* physics_system = JPC_PhysicsSystem_new();

	// JPC_PhysicsSystem_Init();

	JPC_PhysicsSystem_delete(physics_system);
	JPC_JobSystemThreadPool_delete(job_system);
	JPC_TempAllocatorImpl_delete(temp_allocator);

	printf("Hello, world!\n");
}