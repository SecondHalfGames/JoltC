#include <assert.h>
#include <stddef.h>
#include <type_traits>

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Body/BodyType.h"
#include "Jolt/Physics/Body/MotionQuality.h"
#include "Jolt/Physics/Body/MotionType.h"
#include "Jolt/Physics/Character/CharacterBase.h"
#include "Jolt/Physics/Collision/BackFaceMode.h"
#include "Jolt/Physics/Collision/ContactListener.h"
#include "Jolt/Physics/Collision/Shape/Shape.h"
#include "Jolt/Physics/Constraints/Constraint.h"
#include "Jolt/Physics/EActivation.h"
#include "Jolt/Physics/EPhysicsUpdateError.h"
#include "Jolt/Physics/PhysicsSettings.h"
#include "Jolt/Physics/Body/AllowedDOFs.h"

template<typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type 
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

#define ENSURE_TESTS

#define ENSURE_EQUAL(c_const, cpp_const) \
    static_assert(c_const == cpp_const, #c_const " did not match " #cpp_const); \
    static_assert(std::is_same_v<decltype(c_const), decltype(cpp_const)>, "type of " #c_const " did not match type of " #cpp_const);

#define ENSURE_ENUM_EQ(c_const, cpp_enum) \
    static_assert(c_const == to_integral(cpp_enum), #c_const " did not match " #cpp_enum); \
    static_assert(sizeof(c_const) == sizeof(cpp_enum), #c_const " did not have same size as " #cpp_enum);

#define ENSURE_SIZE_ALIGN(type0, type1) \
    static_assert(sizeof(type0) == sizeof(type1)); \
    static_assert(alignof(type0) == alignof(type1));

#define unsafe_offsetof(st, m) ((size_t) ( (char *)&((st *)(0))->m - (char *)0 ))
#define unsafe_fieldtype(st, m) decltype((st *)(0)->m)

#define ENSURE_FIELD(type0, field0, type1, field1) \
    static_assert(unsafe_offsetof(type0, field0) == unsafe_offsetof(type1, field1), \
        #type0 "." #field0 " did not have same offset as " #type1 "." #field1); \
    ENSURE_SIZE_ALIGN(unsafe_fieldtype(type0, field0), unsafe_fieldtype(type1, field1));

#include "JoltC/JoltC.h"