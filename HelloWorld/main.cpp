#include <stdio.h>

#include "JoltC/JoltC.h"

int main() {
	JPC_RegisterDefaultAllocator();
	JPC_FactoryInit();
	JPC_RegisterTypes();

	JPC_TempAllocatorImpl* temp_allocator = JPC_TempAllocatorImpl_new(10 * 1024 * 1024);

	JPC_TempAllocatorImpl_delete(temp_allocator);

	printf("Hello, world!\n");
}