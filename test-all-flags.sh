#!/usr/bin/env bash

set -euo pipefail

run_tests() {
	echo "Building with flags $1"

	cmake $1 -B build
	cmake --build build

	./build/Debug/HelloWorld
}

run_tests ""
run_tests "-DDOUBLE_PRECISION=ON"
run_tests "-DOBJECT_LAYER_BITS=32"