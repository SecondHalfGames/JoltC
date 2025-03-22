# JoltC
C wrapper for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.x.

Currently a work in progress. Bindings contain functions that we've needed as part of [our game](https://store.steampowered.com/app/2660180/MEANWHILE_IN_SECTOR_80/) or the Rust bindings we're working on in [jolt-rust](https://github.com/SecondHalfGames/jolt-rust).

The [`JoltC`](./JoltC) directory contains headers for the C interface that you can bind to.

The [`JoltCImpl`](./JoltCImpl) directory contains the C++ implementation of the C interface and serves as the glue between the C++ and C code.

## Goals
1. Sound C wrapper around current version of Jolt Physics
	- We avoid relying on unspecified C++ type layout, which means reflecting those types as opaque handles.
	- We always assert identical layout when creating FFI-compatible versions of C++ types in cases where it makes sense.
2. Headers suitable for usage in automatic binding generation tools (i.e. Rust `bindgen`, LuaJIT FFI)

## Building
Use CMake:

```bash
# Configure the build
cmake -B build

# Optionally, you can enable double precision, or make ObjectLayer use 32 bits
# We aim to support most of the configuration of Jolt's C++ API.
cmake -B build -DDOUBLE_PRECISION=ON -DOBJECT_LAYER_BITS=32

# Build
cmake --build build
```

## Hello, world!
A port of Jolt's "HelloWorld" example is provided in [HelloWorld/main.cpp](HelloWorld/main.cpp).

## Jolt's `RefTarget` Types
Jolt has internal reference counting for a few types along with [good documentation about handling them in C++](https://jrouwe.github.io/JoltPhysicsDocs/5.3.0/index.html#memory-management). This interface to Jolt does not have a generic `Ref`/`RefConst` wrapper, so it's important to be conscious of ref counts when handling these types.

Just like Jolt, this library will return those objects with a reference count of 0. Jolt APIs that accept these types will take ownership of the given value and increment the reference count.

You can use `JPC_{type}_AddRef` and `JPC_{type}_Release`, where `{type}` is the type that directly inherits from `RefTarget<{type}>`, to add or release references. For example, for `FixedConstraint`, use `JPC_Constraint_AddRef` to add a new reference.

## Other C Wrappers
Other C wrappers for Jolt Physics include:
- "JoltC", part of the [zphysics] Zig library started by [Michal Ziulek][michal-ziulek]
- "JoltC", part of the [jolt-rs] Rust library started by [cohaereo] and a fork of the zphysics C wrapper
- "joltc", part of the [JoltPhysicsSharp] C# library started by [Amer Koleci][amerkoleci]

The goal of this project is to be a C wrapper around Jolt Physics that is not part of a larger binding project and to eliminate major sources of undefined behavior. It's intended to be useful for any other language-specific bindings, but naturally these bindings were born out of the goal of creating Rust bindings to Jolt Physics.

## License
Licensed under either of

* Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

[zphysics]: https://github.com/zig-gamedev/zig-gamedev/tree/main/libs/zphysics
[jolt-rs]: https://github.com/cohaereo/jolt-rs
[JoltPhysicsSharp]: https://github.com/amerkoleci/JoltPhysicsSharp
[michal-ziulek]: https://github.com/michal-z
[amerkoleci]: https://github.com/amerkoleci
[cohaereo]: https://github.com/cohaereo