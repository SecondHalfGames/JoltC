#include <assert.h>
#include <stddef.h>
#include <type_traits>

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
#include <Jolt/Physics/Body/BodyManager.h>

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

// Ensures that a C struct we define is compatible in layout with its
// corresponding C++ type, and that the C++ type is safe to represent in C.
#define ENSURE_SIZE_ALIGN(c_type, cpp_type) \
    static_assert(sizeof(c_type) == sizeof(cpp_type), "size of " #c_type " did not match size of " #cpp_type); \
    static_assert(alignof(c_type) == alignof(cpp_type), "align of " #c_type " did not match align of " #cpp_type); \
    static_assert(!std::is_polymorphic_v<cpp_type>, #cpp_type " is polymorphic and cannot be mirrored");

#define unsafe_offsetof(st, m) ((size_t) ( (char *)&((st *)(0))->m - (char *)0 ))
#define unsafe_fieldtype(st, m) decltype((st *)(0)->m)

#define ENSURE_FIELD(type0, field0, type1, field1) \
    static_assert(unsafe_offsetof(type0, field0) == unsafe_offsetof(type1, field1), \
        #type0 "." #field0 " did not have same offset as " #type1 "." #field1); \
    ENSURE_SIZE_ALIGN(unsafe_fieldtype(type0, field0), unsafe_fieldtype(type1, field1));

#include "JoltC/JoltC.h"