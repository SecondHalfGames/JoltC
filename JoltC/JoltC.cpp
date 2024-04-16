#include "JoltC/JoltC.h"

#include "Jolt/Jolt.h"
#include "Jolt/Core/Memory.h"

void JPC_RegisterDefaultAllocator() {
	JPH::RegisterDefaultAllocator();
}