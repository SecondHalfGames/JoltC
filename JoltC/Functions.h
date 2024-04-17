#pragma once

#define JPC_API extern __declspec(dllexport)

#ifdef __cplusplus
extern "C" {
#endif

JPC_API void JPC_RegisterDefaultAllocator();
JPC_API void JPC_FactoryInit();

#ifdef __cplusplus
}
#endif