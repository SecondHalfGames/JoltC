#include <assert.h>
#include <stddef.h>
#include <type_traits>

#include "JoltPhysicsC.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/EPhysicsUpdateError.h"
#include "Jolt/Physics/Collision/Shape/Shape.h"

template<typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type 
{
   return static_cast<typename std::underlying_type<E>::type>(e);
}

#define ENSURE_ENUM_EQ(c_const, cpp_enum) \
	static_assert(c_const == to_integral(cpp_enum), #c_const " did not match " #cpp_enum); \
	static_assert(sizeof(c_const) == sizeof(cpp_enum), #c_const " did not have same size as " #cpp_enum)

ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_CONVEX, JPH::EShapeType::Convex);
ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_COMPOUND, JPH::EShapeType::Compound);
ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_DECORATED, JPH::EShapeType::Decorated);
ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_MESH, JPH::EShapeType::Mesh);
ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_HEIGHT_FIELD, JPH::EShapeType::HeightField);
ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_USER1, JPH::EShapeType::User1);
ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_USER2, JPH::EShapeType::User2);
ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_USER3, JPH::EShapeType::User3);
ENSURE_ENUM_EQ(JPC_SHAPE_TYPE_USER4, JPH::EShapeType::User4);

ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_SPHERE, JPH::EShapeSubType::Sphere);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_BOX, JPH::EShapeSubType::Box);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_TRIANGLE, JPH::EShapeSubType::Triangle);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_CAPSULE, JPH::EShapeSubType::Capsule);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_TAPEREDCAPSULE, JPH::EShapeSubType::TaperedCapsule);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_CYLINDER, JPH::EShapeSubType::Cylinder);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_CONVEX_HULL, JPH::EShapeSubType::ConvexHull);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_STATIC_COMPOUND, JPH::EShapeSubType::StaticCompound);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_MUTABLE_COMPOUND, JPH::EShapeSubType::MutableCompound);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_ROTATED_TRANSLATED, JPH::EShapeSubType::RotatedTranslated);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_SCALED, JPH::EShapeSubType::Scaled);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_OFFSET_CENTER_OF_MASS, JPH::EShapeSubType::OffsetCenterOfMass);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_MESH, JPH::EShapeSubType::Mesh);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_HEIGHT_FIELD, JPH::EShapeSubType::HeightField);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_SOFT_BODY, JPH::EShapeSubType::SoftBody);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER1, JPH::EShapeSubType::User1);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER2, JPH::EShapeSubType::User2);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER3, JPH::EShapeSubType::User3);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER4, JPH::EShapeSubType::User4);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER5, JPH::EShapeSubType::User5);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER6, JPH::EShapeSubType::User6);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER7, JPH::EShapeSubType::User7);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER8, JPH::EShapeSubType::User8);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER_CONVEX1, JPH::EShapeSubType::UserConvex1);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER_CONVEX2, JPH::EShapeSubType::UserConvex2);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER_CONVEX3, JPH::EShapeSubType::UserConvex3);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER_CONVEX4, JPH::EShapeSubType::UserConvex4);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER_CONVEX5, JPH::EShapeSubType::UserConvex5);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER_CONVEX6, JPH::EShapeSubType::UserConvex6);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER_CONVEX7, JPH::EShapeSubType::UserConvex7);
ENSURE_ENUM_EQ(JPC_SHAPE_SUB_TYPE_USER_CONVEX8, JPH::EShapeSubType::UserConvex8);

ENSURE_ENUM_EQ(JPC_PHYSICS_UPDATE_ERROR_NONE, JPH::EPhysicsUpdateError::None);
ENSURE_ENUM_EQ(JPC_PHYSICS_UPDATE_ERROR_MANIFOLD_CACHE_FULL, JPH::EPhysicsUpdateError::ManifoldCacheFull);
ENSURE_ENUM_EQ(JPC_PHYSICS_UPDATE_ERROR_BODY_PAIR_CACHE_FULL, JPH::EPhysicsUpdateError::BodyPairCacheFull);
ENSURE_ENUM_EQ(JPC_PHYSICS_UPDATE_ERROR_CONTACT_CONSTRAINTS_FULL, JPH::EPhysicsUpdateError::ContactConstraintsFull);