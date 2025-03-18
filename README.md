# JoltC
C wrapper for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.1.0.

Currently a work in progress. Bindings contain functions that we've needed as part of our game or the Rust bindings we're working on in [jolt-rust](https://github.com/SecondHalfGames/jolt-rust).

The [`JoltC`](./JoltC) directory contains headers for the C interface that you can bind to.

The [`JoltCImpl`](./JoltCImpl) directory contains the C++ implementation of the C interface and serves as the glue between the C++ and C code.

## Goals
1. Sound C wrapper around current version of Jolt Physics
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

## Other C Wrappers
Other C wrappers for Jolt Physics include:
- "JoltC", part of the [zphysics] Zig library started by [Michal Ziulek][michal-ziulek]
- "JoltC", part of the [jolt-rs] Rust library started by [cohaereo] and a fork of the zphysics C wrapper
- "joltc", part of the [JoltPhysicsSharp] C# library started by [Amer Koleci][amerkoleci]

The goal of this project is to be the first C wrapper around Jolt Physics that is not part of a larger binding project and to eliminate sources of undefined behavior. It's intended to be useful for any other language-specific bindings and to reduce the need to duplicate work.

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