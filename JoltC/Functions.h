#pragma once

#define JPC_API extern __declspec(dllexport)

#ifdef __cplusplus
extern "C" {
#endif

JPC_API void JPC_RegisterDefaultAllocator();
JPC_API void JPC_FactoryInit();
JPC_API void JPC_RegisterTypes();

typedef struct JPC_TempAllocatorImpl JPC_TempAllocatorImpl;

JPC_API JPC_TempAllocatorImpl* JPC_TempAllocatorImpl_new(unsigned int size);
JPC_API void JPC_TempAllocatorImpl_delete(JPC_TempAllocatorImpl* object);

#ifdef __cplusplus
}
#endif