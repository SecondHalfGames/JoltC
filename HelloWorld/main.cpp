#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

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

unsigned int Hello_BPL_GetNumBroadPhaseLayers(const void *self) {
	return HELLO_BPL_COUNT;
}

JPC_BroadPhaseLayer Hello_BPL_GetBroadPhaseLayer(const void *self, JPC_ObjectLayer inLayer) {
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

bool Hello_OVB_ShouldCollide(const void *self, JPC_ObjectLayer inLayer1, JPC_BroadPhaseLayer inLayer2) {
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

bool Hello_OVO_ShouldCollide(const void *self, JPC_ObjectLayer inLayer1, JPC_ObjectLayer inLayer2) {
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

void Hello_Debug_DrawLine(const void *self, JPC_RVec3 inFrom, JPC_RVec3 inTo, JPC_Color inColor) {
	// printf("Draw line from (%f, %f, %f) to (%f, %f, %f) with color (%d, %d, %d)\n",
	// 	inFrom.x, inFrom.y, inFrom.z, inTo.x, inTo.y, inTo.z, inColor.r, inColor.g, inColor.b);
}

static JPC_DebugRendererSimpleFns Hello_DebugRenderer = {
	.DrawLine = Hello_Debug_DrawLine,
};

int main() {
	JPC_RegisterDefaultAllocator();
	JPC_FactoryInit();
	JPC_RegisterTypes();

	JPC_TempAllocatorImpl* temp_allocator = JPC_TempAllocatorImpl_new(10 * 1024 * 1024);

	JPC_JobSystemThreadPool* job_system = JPC_JobSystemThreadPool_new2(JPC_MAX_PHYSICS_JOBS, JPC_MAX_PHYSICS_BARRIERS);

	JPC_BroadPhaseLayerInterface* broad_phase_layer_interface = JPC_BroadPhaseLayerInterface_new(nullptr, Hello_BPL);
	JPC_ObjectVsBroadPhaseLayerFilter* object_vs_broad_phase_layer_filter = JPC_ObjectVsBroadPhaseLayerFilter_new(nullptr, Hello_OVB);
	JPC_ObjectLayerPairFilter* object_vs_object_layer_filter = JPC_ObjectLayerPairFilter_new(nullptr, Hello_OVO);

	const unsigned int cMaxBodies = 1024;
	const unsigned int cNumBodyMutexes = 0;
	const unsigned int cMaxBodyPairs = 1024;
	const unsigned int cMaxContactConstraints = 1024;

	JPC_PhysicsSystem* physics_system = JPC_PhysicsSystem_new();
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

	JPC_BodyInterface* body_interface = JPC_PhysicsSystem_GetBodyInterface(physics_system);

	JPC_BoxShapeSettings floor_shape_settings;
	JPC_BoxShapeSettings_default(&floor_shape_settings);
	floor_shape_settings.HalfExtent = JPC_Vec3{100.0f, 1.0f, 100.0f};
	floor_shape_settings.Density = 500.0;

	JPC_Shape* floor_shape;
	JPC_String* err;
	if (!JPC_BoxShapeSettings_Create(&floor_shape_settings, &floor_shape, &err)) {
		printf("fatal error: %s\n", JPC_String_c_str(err));

		// the world is ending, but I guess we can still free memory
		JPC_String_delete(err);

		exit(1);
	}

	JPC_BodyCreationSettings floor_settings;
	JPC_BodyCreationSettings_default(&floor_settings);
	floor_settings.Position = JPC_RVec3{0.0, -1.0, 0.0};
	floor_settings.MotionType = JPC_MOTION_TYPE_STATIC;
	floor_settings.ObjectLayer = HELLO_OL_NON_MOVING;
	floor_settings.Shape = floor_shape;

	JPC_Body* floor = JPC_BodyInterface_CreateBody(body_interface, &floor_settings);
	JPC_BodyInterface_AddBody(body_interface, JPC_Body_GetID(floor), JPC_ACTIVATION_DONT_ACTIVATE);

	JPC_SphereShapeSettings sphere_shape_settings;
	JPC_SphereShapeSettings_default(&sphere_shape_settings);
	sphere_shape_settings.Radius = 0.5;

	JPC_Shape* sphere_shape;
	if (!JPC_SphereShapeSettings_Create(&sphere_shape_settings, &sphere_shape, &err)) {
		printf("fatal error: %s\n", JPC_String_c_str(err));

		// the world is ending, but I guess we can still free memory
		JPC_String_delete(err);

		exit(1);
	}

	JPC_BodyCreationSettings sphere_settings;
	JPC_BodyCreationSettings_default(&sphere_settings);
	sphere_settings.Position = JPC_RVec3{0.0, 2.0, 0.0};
	sphere_settings.MotionType = JPC_MOTION_TYPE_DYNAMIC;
	sphere_settings.ObjectLayer = HELLO_OL_MOVING;
	sphere_settings.Shape = sphere_shape;

	JPC_Body* sphere = JPC_BodyInterface_CreateBody(body_interface, &sphere_settings);
	JPC_BodyID sphere_id = JPC_Body_GetID(sphere);
	JPC_BodyInterface_AddBody(body_interface, sphere_id, JPC_ACTIVATION_ACTIVATE);

	JPC_BodyInterface_SetLinearVelocity(body_interface, sphere_id, JPC_Vec3{0.0, -5.0, 0.0});

	JPC_DebugRendererSimple* debug_renderer = JPC_DebugRendererSimple_new(nullptr, Hello_DebugRenderer);
	JPC_BodyManager_DrawSettings draw_settings;
	JPC_BodyManager_DrawSettings_default(&draw_settings);
	JPC_PhysicsSystem_DrawBodies(physics_system, &draw_settings, debug_renderer, nullptr);

	JPC_PhysicsSystem_OptimizeBroadPhase(physics_system);

	const float cDeltaTime = 1.0f / 60.0f;
	const int cCollisionSteps = 1;

	int step = 0;
	while (JPC_BodyInterface_IsActive(body_interface, sphere_id)) {
		++step;

		JPC_RVec3 position = JPC_BodyInterface_GetCenterOfMassPosition(body_interface, sphere_id);
		JPC_Vec3 velocity = JPC_BodyInterface_GetLinearVelocity(body_interface, sphere_id);

		printf("Step %d: Position = (%f, %f, %f), Velocity = (%f, %f, %f)\n", step, position.x, position.y, position.z, velocity.x, velocity.y, velocity.z);

		JPC_PhysicsSystem_Update(physics_system, cDeltaTime, cCollisionSteps, temp_allocator, job_system);
	}

	JPC_BodyInterface_RemoveBody(body_interface, sphere_id);
	JPC_BodyInterface_DestroyBody(body_interface, sphere_id);

	JPC_BodyInterface_RemoveBody(body_interface, JPC_Body_GetID(floor));
	JPC_BodyInterface_DestroyBody(body_interface, JPC_Body_GetID(floor));

	JPC_PhysicsSystem_delete(physics_system);
	JPC_BroadPhaseLayerInterface_delete(broad_phase_layer_interface);
	JPC_ObjectVsBroadPhaseLayerFilter_delete(object_vs_broad_phase_layer_filter);
	JPC_ObjectLayerPairFilter_delete(object_vs_object_layer_filter);

	JPC_JobSystemThreadPool_delete(job_system);
	JPC_TempAllocatorImpl_delete(temp_allocator);

	JPC_UnregisterTypes();
	JPC_FactoryDelete();

	printf("Hello, world!\n");
}