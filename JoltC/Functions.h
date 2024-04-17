#pragma once

#define JPC_API extern __declspec(dllexport)

JPC_API void JPC_RegisterDefaultAllocator();
JPC_API void JPC_FactoryInit();